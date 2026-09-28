#ifndef G365_IMU_H
#define G365_IMU_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus

static_assert(sizeof(float) == 4u, "G365 requires 32-bit float");

// Both supported UART profiles have the same 36-byte framing.  The six 32-bit
// payload values are ambiguous on the wire, so the selected mode must match
// the IMU's BURST_CTRL/ATTI_CTRL configuration.
class G365Imu
{
public:
    enum class Mode : uint8_t
    {
        Raw32 = 2u,   // F007 / 3000 / ATTI=00
        Delta32 = 3u  // CC07 / 0C00 / ATTI=02
    };

    static const size_t FRAME_SIZE = 36u;
    // The observed 200 Hz output advances the 62.5 kHz counter by 312/313.
    static const uint16_t OBSERVED_COUNT_STEP = 312u;

    struct Sample
    {
        Mode mode;
        uint16_t flag;
        int32_t temperature; // Signed 16-bit register value, widened for callers.
        int32_t gyro[3];
        int32_t accel[3];
        int32_t delta_angle[3];
        int32_t delta_velocity[3];
        float temperature_c;
        float gyro_x_dps;
        float gyro_y_dps;
        float gyro_z_dps;
        float accel_x_g;
        float accel_y_g;
        float accel_z_g;
        float delta_angle_x_deg;
        float delta_angle_y_deg;
        float delta_angle_z_deg;
        float delta_velocity_x_mps;
        float delta_velocity_y_mps;
        float delta_velocity_z_mps;
        uint16_t gpio;
        uint16_t count;
        uint32_t received_at_ms;
        uint32_t uart_end_counter; // TIM2 counter latched by the final UART byte ISR.
    };

    struct Stats
    {
        uint32_t valid_frames;
        uint32_t checksum_errors;
        uint32_t framing_errors;
        uint32_t count_jumps;
        uint32_t estimated_missing_frames;
        uint32_t sensor_error_frames;
        uint32_t range_over_frames;
    };

    typedef void (*SampleCallback)(const Sample &sample, void *context);

    G365Imu();
    void reset();
    bool set_mode(Mode mode);
    Mode mode() const;
    // G365PDF1 DLT_CTRL (W1:0x12); its low byte selects delta scale codes.
    void set_delta_ctrl(uint16_t value);
    uint16_t delta_ctrl() const;
    // Compatibility with the earlier G366-based API name.
    void set_glob_cmd3(uint16_t value) { set_delta_ctrl(value); }
    uint16_t glob_cmd3() const { return delta_ctrl(); }
    void feed(const uint8_t *data, size_t length, uint32_t received_at_ms);
    void feed_timed(const uint8_t *data, const uint32_t *timer_counters,
                    size_t length, uint32_t received_at_ms);
    bool get_latest(Sample &out) const;
    Stats get_stats() const;
    void set_sample_callback(SampleCallback callback, void *context);

private:
    uint8_t frame_[FRAME_SIZE];
    size_t frame_length_;
    Sample latest_;
    Stats stats_;
    bool has_sample_;
    Mode mode_;
    uint16_t delta_ctrl_;
    SampleCallback sample_callback_;
    void *callback_context_;

    void feed_byte(uint8_t byte, uint32_t received_at_ms,
                   uint32_t timer_counter);
    void resynchronize();
    static uint16_t read_be16(const uint8_t *data);
    static int16_t read_be16_signed(const uint8_t *data);
    static int32_t read_be32_signed(const uint8_t *data);
    static uint16_t checksum(const uint8_t *frame);
};

// Direct Keil Watch expression: g_g365_debug_sample.gyro_x_dps
// Updated only after a valid frame; mode 3 fills delta fields instead.
extern "C" G365Imu::Sample g_g365_debug_sample;

#endif /* __cplusplus */
#endif
