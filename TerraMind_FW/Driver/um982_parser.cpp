#include "um982_parser.h"

#include <stdlib.h>
#include <string.h>

namespace
{
uint32_t unicore_crc32(const char *data, size_t length)
{
    // Unicore ASCII CRC: initial value 0, reflected polynomial, no final XOR.
    uint32_t crc = 0u;
    for (size_t i = 0u; i < length; ++i)
    {
        crc ^= static_cast<uint8_t>(data[i]);
        for (unsigned bit = 0u; bit < 8u; ++bit)
        {
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
        }
    }
    return crc;
}

int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool read_crc(const char *text, uint32_t &value)
{
    value = 0u;
    for (unsigned i = 0u; i < 8u; ++i)
    {
        const int digit = hex_digit(text[i]);
        if (digit < 0) return false;
        value = (value << 4) | static_cast<uint32_t>(digit);
    }
    return text[8] == '\0';
}

size_t split_fields(char *text, char **fields, size_t capacity)
{
    if (capacity == 0u) return 0u;
    size_t count = 1u;
    fields[0] = text;
    for (char *p = text; *p != '\0'; ++p)
    {
        if (*p == ',')
        {
            if (count == capacity) return 0u;
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    return count;
}

bool parse_unsigned(const char *text, uint32_t &value)
{
    if (*text < '0' || *text > '9') return false;
    char *end = NULL;
    const unsigned long parsed = strtoul(text, &end, 10);
    if (*end != '\0' || parsed > 0xfffffffful) return false;
    value = static_cast<uint32_t>(parsed);
    return true;
}

bool parse_number(const char *text, double &value)
{
    if (*text == '\0') return false;
    char *end = NULL;
    value = strtod(text, &end);
    return *end == '\0' && value == value && value < 1.0e100 && value > -1.0e100;
}

bool parse_float(const char *text, float &value)
{
    double parsed;
    if (!parse_number(text, parsed) || parsed > 3.4e38 || parsed < -3.4e38) return false;
    value = static_cast<float>(parsed);
    return true;
}

bool parse_byte(const char *text, uint8_t &value)
{
    uint32_t parsed;
    if (!parse_unsigned(text, parsed) || parsed > 255u) return false;
    value = static_cast<uint8_t>(parsed);
    return true;
}

Um982FixType fix_type(const char *text)
{
    if (strcmp(text, "NONE") == 0) return UM982_FIX_NONE;
    if (strcmp(text, "SINGLE") == 0) return UM982_FIX_SINGLE;
    if (strcmp(text, "PSRDIFF") == 0) return UM982_FIX_PSRDIFF;
    if (strcmp(text, "NARROW_FLOAT") == 0) return UM982_FIX_RTK_FLOAT;
    if (strcmp(text, "NARROW_INT") == 0) return UM982_FIX_RTK_FIXED;
    return UM982_FIX_OTHER;
}

bool parse_time(char **header, Um982Position &out)
{
    uint32_t week;
    if (!parse_unsigned(header[4], week) || week > 65535u ||
        !parse_unsigned(header[5], out.gps_tow_ms) || out.gps_tow_ms >= 604800000u) return false;
    out.gps_week = static_cast<uint16_t>(week);
    out.time_valid = static_cast<uint8_t>(strcmp(header[3], "FINE") == 0);
    return true;
}

bool parse_time(char **header, Um982Heading &out)
{
    uint32_t week;
    if (!parse_unsigned(header[4], week) || week > 65535u ||
        !parse_unsigned(header[5], out.gps_tow_ms) || out.gps_tow_ms >= 604800000u) return false;
    out.gps_week = static_cast<uint16_t>(week);
    out.time_valid = static_cast<uint8_t>(strcmp(header[3], "FINE") == 0);
    return true;
}

bool parse_bestnav(char **header, char **body, uint32_t received_at_ms,
                   uint32_t sequence, Um982Position &out)
{
    memset(&out, 0, sizeof(out));
    out.sequence = sequence;
    out.received_at_ms = received_at_ms;
    out.fix_type = fix_type(body[1]);
    if (!parse_time(header, out) ||
        !parse_number(body[2], out.latitude_deg) ||
        !parse_number(body[3], out.longitude_deg) ||
        !parse_number(body[4], out.height_msl_m) ||
        !parse_float(body[5], out.undulation_m) ||
        !parse_float(body[7], out.latitude_std_m) ||
        !parse_float(body[8], out.longitude_std_m) ||
        !parse_float(body[9], out.height_std_m) ||
        !parse_float(body[11], out.differential_age_s) ||
        !parse_float(body[12], out.solution_age_s) ||
        !parse_byte(body[13], out.satellites_tracked) ||
        !parse_byte(body[14], out.satellites_used) ||
        !parse_float(body[23], out.velocity_latency_s) ||
        !parse_float(body[25], out.horizontal_speed_mps) ||
        !parse_float(body[26], out.track_deg) ||
        !parse_float(body[27], out.vertical_speed_mps) ||
        !parse_float(body[28], out.vertical_speed_std_mps) ||
        !parse_float(body[29], out.horizontal_speed_std_mps)) return false;
    out.position_valid = static_cast<uint8_t>(out.time_valid &&
        strcmp(body[0], "SOL_COMPUTED") == 0 && out.fix_type != UM982_FIX_NONE &&
        out.fix_type != UM982_FIX_OTHER &&
        out.latitude_deg >= -90.0 && out.latitude_deg <= 90.0 &&
        out.longitude_deg >= -180.0 && out.longitude_deg <= 180.0);
    out.velocity_valid = static_cast<uint8_t>(out.time_valid &&
        strcmp(body[21], "SOL_COMPUTED") == 0 &&
        strcmp(body[22], "DOPPLER_VELOCITY") == 0 &&
        out.horizontal_speed_mps >= 0.0f && out.track_deg >= 0.0f &&
        out.track_deg <= 360.0f);
    return true;
}

bool parse_heading(char **header, char **body, uint32_t received_at_ms,
                   uint32_t sequence, Um982Heading &out)
{
    memset(&out, 0, sizeof(out));
    out.sequence = sequence;
    out.received_at_ms = received_at_ms;
    out.fix_type = fix_type(body[1]);
    if (!parse_time(header, out) ||
        !parse_float(body[2], out.baseline_m) ||
        !parse_float(body[3], out.heading_deg) ||
        !parse_float(body[4], out.pitch_deg) ||
        !parse_float(body[6], out.heading_std_deg) ||
        !parse_float(body[7], out.pitch_std_deg)) return false;
    out.heading_valid = static_cast<uint8_t>(out.time_valid &&
        strcmp(body[0], "SOL_COMPUTED") == 0 &&
        (out.fix_type == UM982_FIX_RTK_FIXED || out.fix_type == UM982_FIX_RTK_FLOAT) &&
        out.baseline_m > 0.0f && out.heading_deg >= 0.0f && out.heading_deg < 360.0f &&
        out.pitch_deg >= -90.0f && out.pitch_deg <= 90.0f);
    return true;
}
}

Um982Parser::Um982Parser()
    : line_length_(0u), has_position_(false), has_heading_(false)
{
    memset(line_, 0, sizeof(line_));
    memset(&position_, 0, sizeof(position_));
    memset(&heading_, 0, sizeof(heading_));
    memset(&stats_, 0, sizeof(stats_));
}

void Um982Parser::feed(const uint8_t *bytes, size_t length, uint32_t received_at_ms)
{
    if (bytes == NULL) return;
    for (size_t i = 0u; i < length; ++i)
    {
        const char byte = static_cast<char>(bytes[i]);
        if (byte == '#')
        {
            line_[0] = byte;
            line_length_ = 1u;
        }
        else if (line_length_ != 0u)
        {
            if (byte == '\n')
            {
                if (line_length_ != 0u && line_[line_length_ - 1u] == '\r') --line_length_;
                line_[line_length_] = '\0';
                parse_line(received_at_ms);
                line_length_ = 0u;
            }
            else if (line_length_ >= LINE_CAPACITY - 1u)
            {
                ++stats_.line_overflows;
                line_length_ = 0u;
            }
            else
            {
                line_[line_length_++] = byte;
            }
        }
    }
}

void Um982Parser::parse_line(uint32_t received_at_ms)
{
    const bool bestnav = strncmp(line_, "#BESTNAVA,", 10u) == 0;
    const bool heading = strncmp(line_, "#UNIHEADINGA,", 13u) == 0;
    if (!bestnav && !heading) return;
    char *star = strrchr(line_, '*');
    uint32_t expected;
    if (star == NULL || strlen(star + 1) != 8u || !read_crc(star + 1, expected))
    {
        ++stats_.format_errors;
        return;
    }
    if (unicore_crc32(line_ + 1, static_cast<size_t>(star - line_ - 1)) != expected)
    {
        ++stats_.crc_errors;
        return;
    }
    *star = '\0';
    char *semicolon = strchr(line_, ';');
    if (semicolon == NULL)
    {
        ++stats_.format_errors;
        return;
    }
    *semicolon = '\0';
    char *header[12];
    char *body[32];
    const size_t header_count = split_fields(line_ + 1, header, 12u);
    const size_t body_count = split_fields(semicolon + 1, body, 32u);
    if (header_count != 10u || body_count != (bestnav ? 30u : 17u))
    {
        ++stats_.format_errors;
        return;
    }
    if (bestnav)
    {
        Um982Position parsed;
        if (!parse_bestnav(header, body, received_at_ms,
                           stats_.bestnav_messages + 1u, parsed))
        {
            ++stats_.format_errors;
            return;
        }
        position_ = parsed;
        has_position_ = true;
        ++stats_.bestnav_messages;
    }
    else
    {
        Um982Heading parsed;
        if (!parse_heading(header, body, received_at_ms,
                           stats_.heading_messages + 1u, parsed))
        {
            ++stats_.format_errors;
            return;
        }
        heading_ = parsed;
        has_heading_ = true;
        ++stats_.heading_messages;
    }
}

bool Um982Parser::get_position(Um982Position &out) const
{
    if (!has_position_) return false;
    out = position_;
    return true;
}

bool Um982Parser::get_heading(Um982Heading &out) const
{
    if (!has_heading_) return false;
    out = heading_;
    return true;
}

Um982ParserStats Um982Parser::get_stats() const
{
    return stats_;
}
