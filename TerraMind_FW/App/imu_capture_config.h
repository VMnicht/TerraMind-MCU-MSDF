#ifndef TERRAMIND_IMU_CAPTURE_CONFIG_H
#define TERRAMIND_IMU_CAPTURE_CONFIG_H

/* Host interpretation only: these settings do not program the sensor.
 * Confirm the physical G365PDF1 register value before using scaled output.
 * Raw int32 monitor records do not depend on the assumed scale. */
#define IMU_CAPTURE_DEFAULT_MODE APP_IMU_MODE_DELTA32
#define IMU_CAPTURE_DELTA_CTRL 0x0048u /* Requires sensor configured/read back as 0x0048: angle 4, velocity 8. */
#define IMU_CAPTURE_SAMPLE_HZ 200u
#define IMU_GYRO_CALIBRATION_SECONDS 3.0

#endif
