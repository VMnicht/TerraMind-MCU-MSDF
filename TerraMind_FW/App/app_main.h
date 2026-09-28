#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_IMU_MODE_RAW32 = 2,   /* F007 / 3000 / ATTI=00 */
    APP_IMU_MODE_DELTA32 = 3  /* CC07 / 0C00 / ATTI=02 */
} AppImuMode;

typedef struct
{
    AppImuMode mode;
    uint16_t flag;
    int32_t temperature_raw; /* Signed 16-bit temperature, widened to int32_t. */
    int32_t gyro_raw[3];
    int32_t accel_raw[3];
    int32_t delta_angle_raw[3];
    int32_t delta_velocity_raw[3];
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
} AppImuSample;

typedef struct
{
    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint32_t framing_errors;
    uint32_t count_jumps;
    uint32_t estimated_missing_frames;
    uint32_t sensor_error_frames;
    uint32_t range_over_frames;
    uint32_t rx_overflows;
    uint32_t uart_errors;
    uint32_t rx_rearm_errors;
} AppImuStats;

/* 0: not entered, 1: init entered, 2: parser ready, 3: UART armed,
   4: main loop active, 5: first valid frame received. */
extern volatile uint32_t g_imu_debug_stage;

uint8_t App_ImuInit(void);
void App_ImuStep(void);
/* Select the parser profile after configuring the sensor to the same mode.
   The change is applied by the IMU task at its next App_ImuStep() call. */
uint8_t App_ImuSetMode(AppImuMode mode);
/* Set the sensor's actual G365PDF1 DLT_CTRL value for delta float scaling. */
void App_ImuSetDeltaCtrl(uint16_t value);
/* Earlier G366-based name, retained for source compatibility. */
void App_ImuSetGlobCmd3(uint16_t value);
uint8_t App_ImuGetLatest(AppImuSample *sample);
void App_ImuGetStats(AppImuStats *stats);

#ifdef __cplusplus
}
#endif

#endif
