#include "../TerraMind_FW/Driver/g365_imu.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static unsigned g_sample_callbacks = 0u;
static void on_sample(const G365Imu::Sample &, void *)
{
    ++g_sample_callbacks;
}

static void put_be16(uint8_t *out, uint16_t value)
{
    out[0] = static_cast<uint8_t>(value >> 8);
    out[1] = static_cast<uint8_t>(value);
}

static bool near(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void put_be32(uint8_t *out, int32_t value)
{
    const uint32_t bits = static_cast<uint32_t>(value);
    out[0] = static_cast<uint8_t>(bits >> 24);
    out[1] = static_cast<uint8_t>(bits >> 16);
    out[2] = static_cast<uint8_t>(bits >> 8);
    out[3] = static_cast<uint8_t>(bits);
}

static void make_frame(uint8_t *out, uint16_t flag, int16_t temperature,
                       const int32_t first[3], const int32_t second[3],
                       uint16_t gpio, uint16_t count)
{
    memset(out, 0, G365Imu::FRAME_SIZE);
    out[0] = 0x80u;
    put_be16(out + 1, flag);
    put_be16(out + 3, static_cast<uint16_t>(temperature));
    for (size_t axis = 0; axis < 3; ++axis)
    {
        put_be32(out + 5 + axis * 4, first[axis]);
        put_be32(out + 17 + axis * 4, second[axis]);
    }
    put_be16(out + 29, gpio);
    put_be16(out + 31, count);

    uint32_t sum = 0;
    for (size_t offset = 1; offset < 33; offset += 2)
    {
        sum += (static_cast<uint16_t>(out[offset]) << 8) | out[offset + 1];
    }
    put_be16(out + 33, static_cast<uint16_t>(sum));
    out[35] = 0x0du;
}

int main()
{
    G365Imu imu;
    assert(imu.delta_ctrl() == 0x0048u);
    // Keep coverage of the previous profile as an explicit, non-default setting.
    imu.set_delta_ctrl(0x00ccu);
    imu.set_sample_callback(on_sample, NULL);
    G365Imu::Sample sample = {};
    assert(imu.mode() == G365Imu::Mode::Raw32);
    assert(!imu.get_latest(sample));
    assert(!imu.set_mode(static_cast<G365Imu::Mode>(4u)));

    const int32_t gyro[3] = {-2147483647 - 1, 0x0000800d, -1234567};
    const int32_t accel[3] = {0, 123456789, -987654321};
    uint8_t raw[G365Imu::FRAME_SIZE];
    make_frame(raw, 0xfefcu, -200, gyro, accel, 0x0100u, 0x1384u);

    // Noise and a split frame; 0x80 and 0x0D inside the payload are data.
    const uint8_t noise[] = {0x55u, 0x0du, 0x7fu};
    imu.feed(noise, sizeof(noise), 0u);
    imu.feed(raw, 9u, 1u);
    assert(!imu.get_latest(sample));
    imu.feed(raw + 9u, sizeof(raw) - 9u, 2u);
    assert(imu.get_latest(sample));
    assert(sample.mode == G365Imu::Mode::Raw32);
    assert(sample.flag == 0xfefcu && sample.temperature == -200);
    assert(sample.gyro[0] == gyro[0] && sample.gyro[1] == gyro[1]);
    assert(sample.gyro[2] == gyro[2] && sample.accel[2] == accel[2]);
    assert(sample.delta_angle[0] == 0 && sample.delta_velocity[0] == 0);
    assert(sample.gpio == 0x0100u && sample.count == 0x1384u);
    assert(sample.received_at_ms == 2u);
    assert(near(sample.temperature_c, 25.0f + (-200.0f - 2634.0f) * -0.0037918f, 0.00001f));
    assert(near(sample.gyro_y_dps, static_cast<float>(gyro[1]) / (66.0f * 65536.0f), 0.000001f));
    assert(near(sample.accel_y_g, static_cast<float>(accel[1]) / (2.5f * 1000.0f * 65536.0f), 0.000001f));
    assert(sample.delta_angle_x_deg == 0.0f);
    assert(near(g_g365_debug_sample.gyro_y_dps, sample.gyro_y_dps, 0.0f));

    uint8_t raw_next[G365Imu::FRAME_SIZE];
    make_frame(raw_next, 0u, 20, gyro, accel, 0u, 0x15f5u);
    imu.feed(raw_next, sizeof(raw_next), 12u);
    assert(imu.get_stats().valid_frames == 2u);
    assert(imu.get_stats().count_jumps == 1u);
    assert(imu.get_stats().estimated_missing_frames == 1u);

    // Mode 3 has the same length and checksum, but the six 32-bit words are
    // delta angle and delta velocity rather than gyro and accelerometer data.
    const int32_t angle[3] = {1000, -2000, 3000};
    const int32_t velocity[3] = {-4000, 5000, -6000};
    uint8_t delta[G365Imu::FRAME_SIZE];
    make_frame(delta, 0x0001u, -30, angle, velocity, 0x1234u, 0xabcdu);
    assert(imu.set_mode(G365Imu::Mode::Delta32));
    assert(!imu.get_latest(sample));
    assert(imu.get_stats().valid_frames == 0u);

    uint8_t damaged[G365Imu::FRAME_SIZE];
    memcpy(damaged, delta, sizeof(damaged));
    damaged[8] ^= 1u;
    imu.feed(damaged, sizeof(damaged), 20u);
    imu.feed(delta, sizeof(delta), 21u);
    assert(imu.get_latest(sample));
    assert(sample.mode == G365Imu::Mode::Delta32);
    assert(sample.temperature == -30 && sample.flag == 0x0001u);
    assert(sample.delta_angle[0] == angle[0] && sample.delta_angle[1] == angle[1]);
    assert(sample.delta_angle[2] == angle[2]);
    assert(sample.delta_velocity[0] == velocity[0]);
    assert(sample.delta_velocity[1] == velocity[1] && sample.delta_velocity[2] == velocity[2]);
    assert(sample.gyro[0] == 0 && sample.accel[0] == 0);
    assert(sample.gpio == 0x1234u && sample.count == 0xabcdu);
    assert(sample.gyro_x_dps == 0.0f && sample.accel_x_g == 0.0f);
    assert(near(sample.delta_angle_x_deg, 0.00047348485f, 0.00000001f));
    assert(near(sample.delta_velocity_x_mps, -0.0004903325f, 0.00000001f));
    assert(imu.get_stats().checksum_errors == 1u);
    assert(imu.get_stats().valid_frames == 1u);

    // Changing profile drops a partial frame and stale latest sample.
    imu.feed(delta, 10u, 22u);
    assert(imu.set_mode(G365Imu::Mode::Raw32));
    assert(!imu.get_latest(sample));
    imu.feed(raw, sizeof(raw), 23u);
    assert(imu.get_latest(sample) && sample.mode == G365Imu::Mode::Raw32);
    imu.set_delta_ctrl(0x0100u);
    assert(!imu.get_latest(sample));
    assert(imu.delta_ctrl() == 0x0100u);
    imu.feed(raw, sizeof(raw), 24u);
    assert(imu.get_latest(sample));
    assert(near(sample.accel_y_g, static_cast<float>(accel[1]) / (2.5f * 1000.0f * 65536.0f), 0.000001f));

    // One G on G365PDF1 is exactly 2.5 * 1000 * 2^16 counts.
    const int32_t zero[3] = {0, 0, 0};
    const int32_t one_g[3] = {0, 0, 163840000};
    uint8_t gravity[G365Imu::FRAME_SIZE];
    make_frame(gravity, 0u, 2634, zero, one_g, 0u, 0u);
    imu.feed(gravity, sizeof(gravity), 25u);
    assert(imu.get_latest(sample));
    assert(near(sample.temperature_c, 25.0f, 0.00001f));
    assert(near(sample.accel_z_g, 1.0f, 0.000001f));
    assert(g_sample_callbacks == 6u);
    // Actual 2026-10-02 capture, with independently read back DLT_CTRL=0x0008.
    const uint8_t captured[] = {
        0x80,0xfe,0xfc,0x07,0x64,0xff,0x05,0xc1,0x25,0x0a,0x50,0x1d,
        0x3c,0xff,0x73,0xc5,0xfe,0x00,0x00,0x06,0x22,0xff,0xfc,0x84,
        0xf1,0xff,0x9d,0x8b,0x09,0x02,0x00,0x73,0x13,0x3e,0x4f,0x0d
    };
    imu.set_mode(G365Imu::Mode::Delta32);
    imu.set_delta_ctrl(0x0008u);
    imu.feed(captured, sizeof(captured), 26u);
    assert(imu.get_latest(sample));
    assert(imu.get_stats().valid_frames == 1u);
    assert(near(sample.delta_angle_y_deg, 0.0200008651f, 1e-8f));
    assert(near(sample.delta_velocity_z_mps, -0.0494352537f, 1e-8f));
    return 0;
}
