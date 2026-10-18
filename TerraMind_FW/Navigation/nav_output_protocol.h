#ifndef TERRAMIND_NAV_OUTPUT_PROTOCOL_H
#define TERRAMIND_NAV_OUTPUT_PROTOCOL_H
#include "navigation.h"
#ifdef __cplusplus
extern "C" {
#endif
enum { NAV_PACKET_SIZE=80, NAV_WIRE_VALID=1, NAV_WIRE_FAULT=2,
       NAV_WIRE_TIME_LOCKED=4, NAV_WIRE_GNSS_RECENT=8, NAV_WIRE_SIMULATED=16 };
uint16_t NavWire_Crc(const uint8_t *data, size_t size);
// Null output encodes an invalid startup heartbeat. Caller supplies live status,
// sender sequence and current time on the same axis as the measurement.
void NavWire_Encode(uint8_t packet[NAV_PACKET_SIZE], const NavOutput *output,
                    uint32_t sequence, uint8_t status, double now_s);
uint8_t NavWire_StateFinite(const NavOutput *output);
#ifdef __cplusplus
}
#endif
#endif
