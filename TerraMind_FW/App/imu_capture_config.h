#ifndef TERRAMIND_IMU_CAPTURE_CONFIG_H
#define TERRAMIND_IMU_CAPTURE_CONFIG_H

/* Host interpretation only: these settings do not program the sensor.
 * Confirm the physical G365PDF1 register value before using scaled output.
 * Raw int32 monitor records do not depend on the assumed scale. */
#define IMU_CAPTURE_DEFAULT_MODE APP_IMU_MODE_DELTA32
#define IMU_CAPTURE_DELTA_CTRL 0x0008u /* PC readback 2026-10-02: angle code 0, velocity code 8. */
#define IMU_CAPTURE_SAMPLE_HZ 200u
#define IMU_GYRO_CALIBRATION_SECONDS 3.0

#endif
