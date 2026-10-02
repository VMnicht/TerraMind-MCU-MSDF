#include "../TerraMind_FW/App/gyro_bias_calibration.h"
#include <cassert>
#include <cmath>
#include <limits>
static void feed(GyroBiasCalibration &c,unsigned n,double rate=0.001) {
    const double a[]={rate*0.005,-rate*0.005,0},v[]={0,0,-9.80665*0.005};
    for(unsigned i=0;i<n;++i)c.push(a,v,0.005,25,true,false);
}
int main() {
    GyroBiasCalibration c;
    feed(c,599);assert(c.result().state!=GYRO_CAL_READY);
    feed(c,1);assert(c.result().state==GYRO_CAL_READY);
    assert(c.result().samples==600 && std::abs(c.result().duration_s-3)<1e-10);
    assert(std::abs(c.result().stationary_rate[0]-0.001)<1e-14);
    feed(c,100,0.04);assert(std::abs(c.result().stationary_rate[0]-0.001)<1e-14);
    auto config=GyroBiasCalibration::defaults();assert(c.reset(config));
    feed(c,400);c.invalidate(GYRO_CAL_TIMING);feed(c,599);
    assert(c.result().state!=GYRO_CAL_READY);feed(c,1);assert(c.result().state==GYRO_CAL_READY);
    assert(c.result().restarts==1);
    double a[]={0.000005,0,0},v[]={0,0,-9.80665*0.005};
    c.reset(config);feed(c,100);c.push(a,v,0.01,25,true,false);
    assert(c.result().reason==GYRO_CAL_TIMING && c.result().samples==0);
    c.reset(config);feed(c,100);c.push(a,v,0.005,25,true,true);
    assert(c.result().reason==GYRO_CAL_MOTION);
    c.reset(config);feed(c,100);c.push(a,v,0.005,28,true,false);
    assert(c.result().reason==GYRO_CAL_UNSTABLE);
    c.reset(config);feed(c,100);c.push(a,v,0.005,25,false,false);
    assert(c.result().reason==GYRO_CAL_BAD_SAMPLE);
    c.reset(config);feed(c,100);feed(c,100,0.002);
    assert(c.result().state!=GYRO_CAL_READY && c.result().restarts>0);
    c.reset(config);a[0]=std::numeric_limits<double>::quiet_NaN();
    c.push(a,v,0.005,25,true,false);assert(c.result().samples==0);
    config.duration_s=0.1;assert(!c.reset(config));
    config.duration_s=1;assert(c.reset(config));feed(c,200);
    assert(c.result().state==GYRO_CAL_READY);
}
