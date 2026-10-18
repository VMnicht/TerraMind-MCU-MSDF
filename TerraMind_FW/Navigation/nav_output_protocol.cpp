#include "nav_output_protocol.h"
#include <cmath>
#include <cstring>
#include <cfloat>
namespace {
// Explicit little endian writes, independent of struct padding/alignment.
void u16(uint8_t *p,uint16_t v) { p[0]=uint8_t(v);p[1]=uint8_t(v>>8); }
void u32(uint8_t *p,uint32_t v) { for(unsigned i=0;i<4;++i)p[i]=uint8_t(v>>(8*i)); }
void f32(uint8_t *p,double x) { float f=static_cast<float>(x);uint32_t v;std::memcpy(&v,&f,4);u32(p,v); }
void f64(uint8_t *p,double x) { uint64_t v;std::memcpy(&v,&x,8);for(unsigned i=0;i<8;++i)p[i]=uint8_t(v>>(8*i)); }
constexpr uint16_t crcEntry(unsigned i,unsigned n=8) {
    return n ? crcEntry((i&0x8000u)?((i<<1)^0x1021u)&65535u:(i<<1)&65535u,n-1):uint16_t(i);
}
// Generated at compile time, 512 bytes in Flash.
template<unsigned... I> struct Table { static constexpr uint16_t values[sizeof...(I)]={crcEntry(I<<8)...}; };
template<unsigned... I> constexpr uint16_t Table<I...>::values[sizeof...(I)];
template<unsigned N,unsigned... I> struct MakeTable : MakeTable<N-1,N-1,I...> {};
template<unsigned... I> struct MakeTable<0,I...> : Table<I...> {};
}
extern "C" uint16_t NavWire_Crc(const uint8_t *data,size_t size) {
    uint16_t crc=0xffff;
    while(size--)crc=uint16_t((crc<<8)^MakeTable<256>::values[((crc>>8)^*data++)&255]);
    return crc;
}
extern "C" uint8_t NavWire_StateFinite(const NavOutput *o) {
    if(!o || !std::isfinite(o->state_time_s) || o->state_time_s<0)return 0;
    double norm=0;
    for(unsigned i=0;i<3;++i) {
        if(!std::isfinite(o->state.position[i]) || !std::isfinite(o->state.velocity[i]) ||
           !std::isfinite(o->state.attitude[i]) || std::abs(o->state.velocity[i])>FLT_MAX ||
           std::abs(o->state.attitude[i])>FLT_MAX)return 0;
    }
    if(std::abs(o->state.position[0])>1.570796326794897 ||
       std::abs(o->state.position[1])>3.141592653589794 || std::abs(o->state.position[2])>FLT_MAX)return 0;
    for(double q:o->quaternion_bn) { if(!std::isfinite(q))return 0;norm+=q*q; }
    return std::abs(std::sqrt(norm)-1)<=.001;
}
extern "C" void NavWire_Encode(uint8_t *p,const NavOutput *o,uint32_t seq,uint8_t status,double now) {
    std::memset(p,0,NAV_PACKET_SIZE);p[0]=0xaa;p[1]=0x55;p[2]=1;
    const bool finite=NavWire_StateFinite(o)!=0;
    if(!finite || (status&NAV_WIRE_FAULT))status&=uint8_t(~NAV_WIRE_VALID);
    status&=uint8_t(~NAV_WIRE_GNSS_RECENT);
    if((status&NAV_WIRE_VALID) && o->last_gnss_time_s>=0 &&
       std::fmax(0,o->state_time_s-o->last_gnss_time_s)<=.5)status|=NAV_WIRE_GNSS_RECENT;
    p[3]=status;u32(p+4,seq);uint16_t age=65535;
    if(finite) {
        f64(p+8,o->state_time_s);f64(p+16,o->state.position[0]);f64(p+24,o->state.position[1]);
        f32(p+32,o->state.position[2]);
        for(unsigned i=0;i<3;++i) { f32(p+36+4*i,o->state.velocity[i]);f32(p+48+4*i,o->state.attitude[i]); }
        for(unsigned i=0;i<4;++i)f32(p+60+4*i,o->quaternion_bn[i]);
        const double ms=std::floor(std::fmax(0,now-o->state_time_s)*1000+.5);
        if(std::isfinite(now) && std::isfinite(ms) && ms<=65534)age=uint16_t(ms);
    }
    u16(p+76,age);u16(p+78,NavWire_Crc(p+2,76));
}
