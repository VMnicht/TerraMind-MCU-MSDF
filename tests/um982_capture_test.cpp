#include "../TerraMind_FW/Driver/um982_parser.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>


int main(int argc, char **argv)
{
    assert(argc == 2);
    FILE *file = fopen(argv[1], "rb");
    assert(file != NULL);
    Um982Parser parser;
    uint8_t bytes[67];
    size_t length;
    while ((length = fread(bytes, 1u, sizeof(bytes), file)) != 0u)
    {
        parser.feed(bytes, length, 123u);
    }
    fclose(file);

    const Um982ParserStats stats = parser.get_stats();
    assert(stats.bestnav_messages == 330u);
    assert(stats.heading_messages == 330u);
    assert(stats.crc_errors == 0u);
    assert(stats.format_errors == 0u);
    assert(stats.line_overflows == 0u);

    Um982Position position;
    Um982Heading heading;
    assert(parser.get_position(position));
    assert(parser.get_heading(heading));
    assert(position.sequence == 330u && heading.sequence == 330u);
    assert(position.position_valid && position.velocity_valid && heading.heading_valid);
    assert(position.fix_type == UM982_FIX_RTK_FIXED);
    assert(heading.fix_type == UM982_FIX_RTK_FIXED);
    assert(position.gps_week == 2438u && heading.gps_week == 2438u);
    assert(position.gps_tow_ms == 20599500u && heading.gps_tow_ms == 20599500u);
    assert(fabs(position.latitude_deg - 23.04235104078) < 1.0e-9);
    assert(fabs(position.longitude_deg - 113.38977112498) < 1.0e-9);
    assert(fabsf(position.horizontal_speed_mps - 0.0020f) < 1.0e-6f);
    assert(fabsf(heading.heading_deg - 251.4280f) < 1.0e-4f);
    assert(fabsf(heading.baseline_m - 0.3073f) < 1.0e-4f);

    // A valid-looking message with a changed payload must not replace the sample.
    const char damaged[] =
        "#UNIHEADINGA,79,GPS,FINE,2438,20599500,0,0,18,10;"
        "SOL_COMPUTED,NARROW_INT,0.3073,252.4280,-1.1993,0.0000,"
        "1.0598,6.9729,\"999\",23,22,22,20,3,01,3,f3*b19e5986\r\n";
    parser.feed(reinterpret_cast<const uint8_t *>(damaged), sizeof(damaged) - 1u, 124u);
    assert(parser.get_stats().crc_errors == 1u);
    assert(parser.get_heading(heading) && heading.sequence == 330u);
    return 0;
}
