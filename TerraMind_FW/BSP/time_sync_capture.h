#ifndef TIME_SYNC_CAPTURE_H
#define TIME_SYNC_CAPTURE_H

#include <stdint.h>

/* Fixed log/protocol timebase, independent of the CPU and APB clock rates. */
#define TIME_SYNC_TIMER_HZ 4000000u

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t sequence;
    uint32_t mcu_ms;
    uint64_t ticks;
    uint32_t period_ticks; /* PPS only; zero for first PPS and all DRDY edges. */
} TimeSyncEdge;

typedef struct
{
    uint32_t timer_hz_nominal;
    uint32_t drdy_captures;
    uint32_t pps_captures;
    uint32_t drdy_queue_drops;
    uint32_t pps_queue_drops;
    uint32_t drdy_overcaptures;
    uint32_t pps_overcaptures;
    uint32_t imu_unmatched;
} TimeSyncStats;

typedef struct
{
    uint32_t pps_sequence;
    uint16_t gps_week;
    uint32_t gps_tow_ms; /* Integer GPS second represented by pps_ticks. */
    uint64_t pps_ticks;
    uint32_t period_ticks;
    uint32_t serial_lag_ticks;
    uint8_t consecutive;
    uint8_t locked; /* Conditional: PPS polarity/time reference must be verified. */
} TimeSyncClockMap;

/* Call after MX_TIM2_Init, before starting the scheduler.
 * Rejects any clock/PSC combination that is not exactly TIME_SYNC_TIMER_HZ,
 * or a counter that is not configured for full-range 32-bit up-counting. */
uint8_t TimeSync_Init(void);
/* Called at the beginning of TIM2_IRQHandler, before HAL_TIM_IRQHandler. */
void TimeSync_OnTim2Irq(void);
uint32_t TimeSync_Counter32(void);
uint64_t TimeSync_NowTicks(void);
/* Reconstruct a recent UART ISR counter reading; 0 means too old/unavailable. */
uint64_t TimeSync_ExpandCounter(uint32_t counter);
/* Match the last byte of a validated G365 frame to its preceding DRDY. */
uint8_t TimeSync_MatchDrdy(uint32_t uart_end_counter, TimeSyncEdge *edge);
void TimeSync_RecordUnmatched(void);
uint8_t TimeSync_PopDrdy(TimeSyncEdge *edge);
uint8_t TimeSync_PopPps(TimeSyncEdge *edge);
void TimeSync_GetStats(TimeSyncStats *stats);
/* Call once for a newly parsed FINE BESTNAVA integer-second epoch. */
uint8_t TimeSync_BindGpsSecond(uint16_t week, uint32_t tow_ms,
                               uint64_t observed_ticks,
                               TimeSyncClockMap *mapping);
uint8_t TimeSync_GetClockMap(TimeSyncClockMap *mapping);
uint8_t TimeSync_GpsToTicks(uint16_t week, uint32_t tow_ms,
                            uint64_t *ticks);

#ifdef __cplusplus
}
#endif

#endif
