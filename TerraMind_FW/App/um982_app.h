#ifndef UM982_APP_H
#define UM982_APP_H

#include "../Driver/um982_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    Um982ParserStats parser;
    uint32_t rx_overflows;
    uint32_t uart_errors;
    uint32_t rx_rearm_errors;
} Um982AppStats;

/* Keil Watch flags for the latest parsed messages. */
#define UM982_DEBUG_UART_INIT_OK    0x01u
#define UM982_DEBUG_POSITION_SEEN   0x02u
#define UM982_DEBUG_HEADING_SEEN    0x04u
#define UM982_DEBUG_POSITION_VALID  0x08u
#define UM982_DEBUG_VELOCITY_VALID  0x10u
#define UM982_DEBUG_HEADING_VALID   0x20u
#define UM982_DEBUG_POSITION_FIXED  0x40u
#define UM982_DEBUG_HEADING_FIXED   0x80u

extern Um982Position g_um982_debug_position;
extern Um982Heading g_um982_debug_heading;
extern Um982AppStats g_um982_debug_stats;
extern volatile uint32_t g_um982_debug_flags;

uint8_t App_Um982Init(void);
void App_Um982Step(void);
uint8_t App_Um982GetPosition(Um982Position *position);
uint8_t App_Um982GetHeading(Um982Heading *heading);
void App_Um982GetStats(Um982AppStats *stats);

#ifdef __cplusplus
}
#endif

#endif
