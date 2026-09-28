#ifndef UM982_PARSER_H
#define UM982_PARSER_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    UM982_FIX_NONE = 0,
    UM982_FIX_SINGLE = 1,
    UM982_FIX_PSRDIFF = 2,
    UM982_FIX_RTK_FLOAT = 3,
    UM982_FIX_RTK_FIXED = 4,
    UM982_FIX_OTHER = 255
} Um982FixType;

typedef struct
{
    uint32_t sequence;
    uint32_t received_at_ms; /* UART delivery time, not the GNSS measurement time. */
    uint32_t gps_tow_ms;
    uint16_t gps_week;
    uint8_t time_valid;
    uint8_t position_valid;
    uint8_t velocity_valid;
    Um982FixType fix_type;
    double latitude_deg;
    double longitude_deg;
    double height_msl_m;
    float undulation_m;
    float latitude_std_m;
    float longitude_std_m;
    float height_std_m;
    float differential_age_s;
    float solution_age_s;
    uint8_t satellites_tracked;
    uint8_t satellites_used;
    float velocity_latency_s;
    float horizontal_speed_mps;
    float track_deg; /* Ground track; unreliable as heading at near-zero speed. */
    float vertical_speed_mps; /* Positive upward. */
    float horizontal_speed_std_mps;
    float vertical_speed_std_mps;
} Um982Position;

typedef struct
{
    uint32_t sequence;
    uint32_t received_at_ms;
    uint32_t gps_tow_ms;
    uint16_t gps_week;
    uint8_t time_valid;
    uint8_t heading_valid;
    Um982FixType fix_type;
    float baseline_m;
    float heading_deg; /* True north, clockwise, master antenna to slave. */
    float pitch_deg;
    float heading_std_deg;
    float pitch_std_deg;
} Um982Heading;

typedef struct
{
    uint32_t bestnav_messages;
    uint32_t heading_messages;
    uint32_t crc_errors;
    uint32_t format_errors;
    uint32_t line_overflows;
} Um982ParserStats;

#ifdef __cplusplus
class Um982Parser
{
public:
    Um982Parser();
    void feed(const uint8_t *bytes, size_t length, uint32_t received_at_ms);
    bool get_position(Um982Position &out) const;
    bool get_heading(Um982Heading &out) const;
    Um982ParserStats get_stats() const;

private:
    enum { LINE_CAPACITY = 384 };
    char line_[LINE_CAPACITY];
    size_t line_length_;
    Um982Position position_;
    Um982Heading heading_;
    Um982ParserStats stats_;
    bool has_position_;
    bool has_heading_;

    void parse_line(uint32_t received_at_ms);
};
#endif

#endif
