#include "../../TerraMind_FW/Driver/um982_parser.h"
#include <cassert>
#include <string>
#include <cstdio>
static unsigned positions=0,headings=0;
static void callback(const Um982Position *p,const Um982Heading *h,void *) {
    if(p) { assert(p->sequence==++positions && p->position_valid && p->velocity_valid); }
    if(h) { assert(h->sequence==++headings && h->heading_valid); }
}
static std::string frame(const std::string &s) {
    uint32_t crc=0;
    for(unsigned char b:s) { crc^=b;for(unsigned i=0;i<8;++i)crc=(crc>>1)^((crc&1)?0xedb88320u:0); }
    char hex[16];std::snprintf(hex,sizeof(hex),"*%08x\r\n",crc);return "#"+s+hex;
}
int main() {
    Um982Parser parser;parser.set_callback(callback,nullptr);
    const auto p=frame("BESTNAVA,79,GPS,FINE,2438,20599500,0,0,18,10;"
        "SOL_COMPUTED,NARROW_INT,30.5,114,20,0,WGS84,0.02,0.02,0.04,0,0,0,25,20,0,0,0,0,0,0,"
        "SOL_COMPUTED,DOPPLER_VELOCITY,0,0,0.002,0,0,0.05,0.05");
    const auto h=frame("UNIHEADINGA,79,GPS,FINE,2438,20599500,0,0,18,10;"
        "SOL_COMPUTED,NARROW_INT,0.3073,251.4280,-1.1993,0.0000,1.0598,6.9729,\"999\",23,22,22,20,3,01,3,f3");
    const auto batch=p+h+p+h;
    parser.feed(reinterpret_cast<const uint8_t *>(batch.data()),batch.size(),123);
    assert(positions==2 && headings==2 && parser.get_stats().format_errors==0);
    auto bad=h;bad[50]^=1;parser.feed(reinterpret_cast<const uint8_t *>(bad.data()),bad.size(),124);
    assert(headings==2 && parser.get_stats().crc_errors==1);
}
