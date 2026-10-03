#include "g365_imu.h"

#include <string.h>

extern "C"
{
G365Imu::Sample g_g365_debug_sample = {};
}

G365Imu::G365Imu() : mode_(Mode::Raw32), delta_ctrl_(0x0048u),
                     sample_callback_(NULL), callback_context_(NULL)
{
    reset();
}

void G365Imu::reset()
{
    memset(frame_, 0, sizeof(frame_));
    frame_length_ = 0u;
    memset(&latest_, 0, sizeof(latest_));
    memset(&g_g365_debug_sample, 0, sizeof(g_g365_debug_sample));
    memset(&stats_, 0, sizeof(stats_));
    has_sample_ = false;
}

bool G365Imu::set_mode(Mode mode)
{
    if (mode != Mode::Raw32 && mode != Mode::Delta32)
    {
        return false;
    }
    mode_ = mode;
    reset();
    return true;
}

G365Imu::Mode G365Imu::mode() const
{
    return mode_;
}

void G365Imu::set_delta_ctrl(uint16_t value)
{
    if (delta_ctrl_ != value)
    {
        delta_ctrl_ = value;
        reset();
    }
}

uint16_t G365Imu::delta_ctrl() const
{
    return delta_ctrl_;
}

uint16_t G365Imu::read_be16(const uint8_t *data)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

int16_t G365Imu::read_be16_signed(const uint8_t *data)
{
    const uint16_t bits = read_be16(data);
    int16_t value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

int32_t G365Imu::read_be32_signed(const uint8_t *data)
{
    const uint32_t bits = (static_cast<uint32_t>(data[0]) << 24) |
                          (static_cast<uint32_t>(data[1]) << 16) |
                          (static_cast<uint32_t>(data[2]) << 8) |
                          static_cast<uint32_t>(data[3]);
    int32_t value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

uint16_t G365Imu::checksum(const uint8_t *frame)
{
    uint32_t sum = 0u;
    // Excludes address byte 0, checksum bytes 33-34, and CR byte 35.
    for (size_t offset = 1u; offset < FRAME_SIZE - 3u; offset += 2u)
    {
        sum += read_be16(&frame[offset]);
    }
    return static_cast<uint16_t>(sum);
}

void G365Imu::resynchronize()
{
    size_t next = 1u;
    while (next < frame_length_ && frame_[next] != 0x80u)
    {
        ++next;
    }
    if (next == frame_length_)
    {
        frame_length_ = 0u;
        return;
    }
    frame_length_ -= next;
    memmove(frame_, &frame_[next], frame_length_);
}

void G365Imu::feed_byte(uint8_t byte, uint32_t received_at_ms,
                       uint32_t timer_counter)
{
    if (frame_length_ == 0u && byte != 0x80u)
    {
        return;
    }
    frame_[frame_length_++] = byte;
    if (frame_length_ != FRAME_SIZE)
    {
        return;
    }

    if (frame_[FRAME_SIZE - 1u] != 0x0du)
    {
        ++stats_.framing_errors;
        resynchronize();
        return;
    }
    if (checksum(frame_) != read_be16(&frame_[FRAME_SIZE - 3u]))
    {
        ++stats_.checksum_errors;
        resynchronize();
        return;
    }

    Sample sample = {};
    sample.mode = mode_;
    sample.flag = read_be16(&frame_[1]);
    sample.temperature = read_be16_signed(&frame_[3]);
    // G365PDF1 TEMP_HIGH: 2634 counts at 25 C, -0.0037918 C/count.
    sample.temperature_c = 25.0f + static_cast<float>(sample.temperature - 2634) * -0.0037918f;
    for (size_t axis = 0u; axis < 3u; ++axis)
    {
        const int32_t first = read_be32_signed(&frame_[5u + axis * 4u]);
        const int32_t second = read_be32_signed(&frame_[17u + axis * 4u]);
        if (mode_ == Mode::Raw32)
        {
            sample.gyro[axis] = first;
            sample.accel[axis] = second;
        }
        else
        {
            sample.delta_angle[axis] = first;
            sample.delta_velocity[axis] = second;
        }
    }
    const float gyro_scale = 1.0f / (66.0f * 65536.0f);
    // G365PDF1 has a fixed +/-10 g range: 2.5 LSB/mg for 16-bit output.
    const float accel_scale = 1.0f / (2.5f * 1000.0f * 65536.0f);
    const float angle_scale = (1.0f / (66.0f * 2000.0f)) *
                              static_cast<float>(1u << ((delta_ctrl_ >> 4) & 0x0fu)) / 65536.0f;
    const float velocity_scale = (0.4f * 0.001f * 9.80665f / 2000.0f) *
                                 static_cast<float>(1u << (delta_ctrl_ & 0x0fu)) / 65536.0f;
    sample.gyro_x_dps = static_cast<float>(sample.gyro[0]) * gyro_scale;
    sample.gyro_y_dps = static_cast<float>(sample.gyro[1]) * gyro_scale;
    sample.gyro_z_dps = static_cast<float>(sample.gyro[2]) * gyro_scale;
    sample.accel_x_g = static_cast<float>(sample.accel[0]) * accel_scale;
    sample.accel_y_g = static_cast<float>(sample.accel[1]) * accel_scale;
    sample.accel_z_g = static_cast<float>(sample.accel[2]) * accel_scale;
    sample.delta_angle_x_deg = static_cast<float>(sample.delta_angle[0]) * angle_scale;
    sample.delta_angle_y_deg = static_cast<float>(sample.delta_angle[1]) * angle_scale;
    sample.delta_angle_z_deg = static_cast<float>(sample.delta_angle[2]) * angle_scale;
    sample.delta_velocity_x_mps = static_cast<float>(sample.delta_velocity[0]) * velocity_scale;
    sample.delta_velocity_y_mps = static_cast<float>(sample.delta_velocity[1]) * velocity_scale;
    sample.delta_velocity_z_mps = static_cast<float>(sample.delta_velocity[2]) * velocity_scale;
    sample.gpio = read_be16(&frame_[29]);
    sample.count = read_be16(&frame_[31]);
    sample.received_at_ms = received_at_ms;
    sample.uart_end_counter = timer_counter;

    if (has_sample_)
    {
        const uint16_t delta = static_cast<uint16_t>(sample.count - latest_.count);
        if (delta != OBSERVED_COUNT_STEP && delta != OBSERVED_COUNT_STEP + 1u)
        {
            ++stats_.count_jumps;
            if (delta > OBSERVED_COUNT_STEP + 1u)
            {
                const uint32_t elapsed_frames = (static_cast<uint32_t>(delta) * 2u + 312u) / 625u;
                if (elapsed_frames > 1u)
                    stats_.estimated_missing_frames += elapsed_frames - 1u;
            }
        }
    }
    if ((sample.flag & 0x0001u) != 0u)
    {
        ++stats_.sensor_error_frames;
    }
    if ((sample.flag & 0x0100u) != 0u)
    {
        ++stats_.range_over_frames;
    }
    latest_ = sample;
    g_g365_debug_sample = sample;
    has_sample_ = true;
    ++stats_.valid_frames;
    frame_length_ = 0u;
    if (sample_callback_ != NULL) sample_callback_(latest_, callback_context_);
}

void G365Imu::feed(const uint8_t *data, size_t length, uint32_t received_at_ms)
{
    if (data == NULL)
    {
        return;
    }
    for (size_t i = 0u; i < length; ++i)
    {
        feed_byte(data[i], received_at_ms, 0u);
    }
}

void G365Imu::feed_timed(const uint8_t *data,
                         const uint32_t *timer_counters,
                         size_t length, uint32_t received_at_ms)
{
    if (data == NULL || timer_counters == NULL) return;
    for (size_t i = 0u; i < length; ++i)
    {
        feed_byte(data[i], received_at_ms, timer_counters[i]);
    }
}

bool G365Imu::get_latest(Sample &out) const
{
    if (!has_sample_)
    {
        return false;
    }
    out = latest_;
    return true;
}

G365Imu::Stats G365Imu::get_stats() const
{
    return stats_;
}

void G365Imu::set_sample_callback(SampleCallback callback, void *context)
{
    sample_callback_ = callback;
    callback_context_ = context;
}
