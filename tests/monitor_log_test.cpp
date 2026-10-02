#include "../TerraMind_FW/App/monitor_app.h"
#include "../TerraMind_FW/App/app_main.h"
#include "../TerraMind_FW/App/um982_app.h"
#include "../TerraMind_FW/BSP/time_sync_capture.h"

#include <assert.h>
#include <stdio.h>
#include <string>
#include <vector>

UART_HandleTypeDef huart6 = {};
static std::string g_output;
static bool g_calibration_ready = false;
extern "C" uint8_t App_ImuCalibrationReady(void) { return g_calibration_ready; }
extern "C" uint16_t App_ImuGetDeltaCtrl(void) { return 0xCC; }
extern "C" AppImuMode App_ImuGetMode(void) { return APP_IMU_MODE_DELTA32; }
extern "C" void App_ImuGetCalibration(GyroCalibrationResult *result) {
    *result={};result->state=GYRO_CAL_READY;result->samples=600;result->duration_s=3;
}

extern "C" uint32_t HAL_GetTick(void) { return 123u; }
extern "C" uint64_t TimeSync_ExpandCounter(uint32_t counter) { return counter; }
extern "C" uint8_t TimeSync_MatchDrdy(uint32_t, TimeSyncEdge *) { return 0u; }
extern "C" void TimeSync_RecordUnmatched(void) {}
extern "C" uint8_t TimeSync_PopDrdy(TimeSyncEdge *) { return 0u; }
extern "C" uint8_t TimeSync_PopPps(TimeSyncEdge *) { return 0u; }
extern "C" void TimeSync_GetStats(TimeSyncStats *stats) { *stats = {}; }
extern "C" void App_ImuGetStats(AppImuStats *stats) { *stats = {}; }
extern "C" void App_Um982GetStats(Um982AppStats *stats) { *stats = {}; }
extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart,
                                                     uint8_t *data, uint16_t length)
{
    g_output.append(reinterpret_cast<const char *>(data), length);
    MonitorUart6Dma::instance().on_tx_complete(uart);
    return HAL_OK;
}
extern "C" HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *)
{
    return HAL_OK;
}

int main(int argc, char **argv)
{
    assert(argc == 1 || argc == 2);
    huart6.hdmatx = &huart6;
    assert(!Monitor_Init());
    Monitor_Step();
    assert(g_output.empty());
    g_calibration_ready=true;
    assert(Monitor_Init());
    FILE *file = argc == 2 ? fopen(argv[1], "rb") : tmpfile();
    assert(file != NULL);
    if (argc == 1)
    {
        const char input[] = "#BESTNAVA,synthetic*12345678\r\n$GNGGA,synthetic*00\r\n";
        assert(fwrite(input, 1u, sizeof(input) - 1u, file) == sizeof(input) - 1u);
        rewind(file);
    }
    uint8_t bytes[67];
    uint32_t timer_counters[67];
    for (size_t i = 0u; i < 67u; ++i) timer_counters[i] = 100u;
    size_t length;
    while ((length = fread(bytes, 1u, sizeof(bytes), file)) != 0u)
    {
        Monitor_OnGnssBytes(bytes, timer_counters, length, 123u);
        Monitor_Step();
    }
    while (MonitorUart6Dma::instance().stats().queue_bytes != 0u) Monitor_Step();

    const std::string version = "V,2,921600,4000000\r\n";
    assert(g_output.compare(0, version.size(), version) == 0);
    rewind(file);
    FILE *original = file;
    const std::string calibration="B,1,123,3,204,600,3000000,0,0,0,0,0,0,0\r\n";
    assert(g_output.compare(version.size(),calibration.size(),calibration)==0);
    size_t cursor = version.size()+calibration.size();
    char original_line[512];
    unsigned lines = 0u;
    while (fgets(original_line, sizeof(original_line), original) != NULL)
    {
        std::string expected = "N,123,";
        expected += original_line;
        if (expected.size() < 2u || expected.substr(expected.size() - 2u) != "\r\n")
        {
            if (!expected.empty() && expected[expected.size() - 1u] == '\n')
                expected.insert(expected.size() - 1u, 1u, '\r');
            else
                expected += "\r\n";
        }
        assert(g_output.compare(cursor, expected.size(), expected) == 0);
        cursor += expected.size();
        const std::string receive_tick = "R,123,0,100\r\n";
        assert(g_output.compare(cursor, receive_tick.size(), receive_tick) == 0);
        cursor += receive_tick.size();
        ++lines;
    }
    fclose(original);
    assert(lines == (argc == 2 ? 1320u : 2u) && cursor == g_output.size());
    assert(MonitorUart6Dma::instance().stats().records_dropped == 0u);

    G365Imu::Sample sample = {};
    sample.mode = G365Imu::Mode::Raw32;
    sample.received_at_ms = 456u;
    sample.count = 625u;
    sample.gyro[0] = -123;
    sample.accel[2] = 789;
    Monitor_OnImuSample(sample, NULL);
    Monitor_Step();
    assert(g_output.find("I,456,2,625,0,0,0,-123,0,0,0,0,789\r\n") != std::string::npos);
    assert(g_output.find("T,456,625,0,0,0,0,0\r\n") != std::string::npos);
    sample.mode = G365Imu::Mode::Delta32;
    sample.delta_angle[0] = INT32_MIN;
    sample.delta_angle[1] = INT32_MAX;
    sample.delta_velocity[2] = -400000;
    Monitor_OnImuSample(sample, NULL);
    Monitor_Step();
    assert(g_output.find("I,456,3,625,0,0,0,-2147483648,2147483647,0,0,0,-400000\r\n") != std::string::npos);
    TimeSyncClockMap mapping = {};
    mapping.pps_sequence = 3u;
    mapping.gps_week = 2438u;
    mapping.gps_tow_ms = 31000000u;
    mapping.pps_ticks = 12000000u;
    mapping.period_ticks = 4000000u;
    mapping.serial_lag_ticks = 80000u;
    mapping.consecutive = 3u;
    mapping.locked = 1u;
    Monitor_OnSyncAnchor(mapping);
    Monitor_Step();
    assert(g_output.find("A,123,3,2438,31000000,0,12000000,4000000,80000,3,1\r\n") != std::string::npos);
    return 0;
}
