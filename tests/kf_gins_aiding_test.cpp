#include "aiding.h"
#include <iostream>
#include <memory>
#include <stdexcept>
#define CHECK(x) do {if(!(x)) throw std::runtime_error(std::string(__FILE__)+":"+std::to_string(__LINE__)+": " #x);} while(0)
using namespace tmnav;
static GINSOptions config() {
    GINSOptions o{};
    o.initstate.pos=Vector3d(.5,1.9,30);o.initstate.vel.setZero();o.initstate.euler.setZero();
    o.initstate.imuerror.gyrbias.setZero();o.initstate.imuerror.accbias.setZero();
    o.initstate.imuerror.gyrscale.setZero();o.initstate.imuerror.accscale.setZero();
    o.initstate_std.pos.setConstant(.1);o.initstate_std.vel.setConstant(.1);
    o.initstate_std.euler.setConstant(2*D2R);
    o.initstate_std.imuerror.gyrbias.setConstant(1e-4);o.initstate_std.imuerror.accbias.setConstant(.01);
    o.initstate_std.imuerror.gyrscale.setConstant(.001);o.initstate_std.imuerror.accscale.setConstant(.001);
    o.imunoise.gyr_arw.setConstant(1e-4);o.imunoise.acc_vrw.setConstant(.005);
    o.imunoise.gyrbias_std.setConstant(1e-4);o.imunoise.accbias_std.setConstant(.01);
    o.imunoise.gyrscale_std.setConstant(.001);o.imunoise.accscale_std.setConstant(.001);
    o.imunoise.corr_time=3600;o.antlever=Vector3d(0,-.15,0);return o;
}
static IMU imu(double t) {
    IMU i{};i.time=t;i.dt=.005;i.dtheta=Vector3d(1e-6,2e-6,.01);i.dvel=Vector3d(.001,0,-.049);
    return i;
}
static void jacobians() {
    const auto c=Rotation::euler2matrix(Vector3d(.24,-.15,2.1));
    Vector3d w(.2,-.3,2),v(1,.5,-.3),l(.8,-.15,.25),earth(6e-5,0,-3e-5),scale(.01,-.02,.03);
    Matrix3x21 h,unused;const auto predicted=OfflineAiding::velocityModel(c,v,w,l,earth,scale,h);
    const double eps=1e-7;
    for(int k: {3,4,5,6,7,8,9,10,11,15,16,17}) {
        auto ct=c;Vector3d vt=v,wt=w,st=scale;
        if(k>=3 && k<6) vt[k-3]-=eps;
        if(k>=6 && k<9) ct=Eigen::AngleAxisd(eps,Vector3d::Unit(k-6)).toRotationMatrix()*c;
        if(k>=9 && k<12) wt[k-9]-=eps/(1+scale[k-9]);
        if(k>=15) {st[k-15]+=eps;wt[k-15]*=(1+scale[k-15])/(1+st[k-15]);}
        Vector3d numeric=(predicted-OfflineAiding::velocityModel(ct,vt,wt,l,earth,st,unused))/eps;
        CHECK((numeric-h.col(k)).norm()<3e-7);
    }
    for(double yaw: {0.,1.5707963267948966,-.8}) {
        double predicted_heading=OfflineAiding::headingModel(c,yaw,h);
        for(int k=0;k<3;++k) {
            Matrix3d ct=Eigen::AngleAxisd(eps,Vector3d::Unit(k)).toRotationMatrix()*c;
            double numeric=OfflineAiding::wrap(predicted_heading-OfflineAiding::headingModel(ct,yaw,unused))/eps;
            CHECK(std::abs(numeric-h(0,k+6))<2e-7);
        }
    }
}
static void unchangedBranches() {
    for(int branch=0;branch<4;++branch) {
        auto o=config();auto reference=std::unique_ptr<GIEngine>(new GIEngine(o));
        auto aided=std::unique_ptr<GIEngine>(new GIEngine(o));AidingStats stats;
        reference->addImuData(imu(100),true);aided->addImuData(imu(100),true);
        AidingObservation a;a.position=true;a.time=branch==1?100:branch==2?100.005:100.0025;
        a.blh=o.initstate.pos;a.position_std.setConstant(.03);
        GNSS g{};g.blh=a.blh;g.time=a.time;g.std=a.position_std;g.isvalid=true;
        if(branch) reference->addGnssData(g);
        reference->addImuData(imu(100.005));aided->addImuData(imu(100.005));
        reference->newImuProcess();OfflineAiding::process(*aided,branch?&a:nullptr,stats);
        CHECK(reference->healthy() && aided->healthy());CHECK(aided->lastBranch()==branch);
        const auto r=reference->getNavState(),n=aided->getNavState();
        CHECK((r.pos-n.pos).norm()<1e-13);CHECK((r.vel-n.vel).norm()<1e-13);
        CHECK((r.euler-n.euler).norm()<1e-13);
        CHECK((reference->getCovariance()-aided->getCovariance()).norm()<1e-13);
    }
}
static void aidingUpdates() {
    auto o=config();o.initstate.euler.z()=359*D2R;
    auto e=std::unique_ptr<GIEngine>(new GIEngine(o));AidingStats s;AidingObservation a;
    a.heading=true;a.heading_rad=0;a.heading_std=.5*D2R;
    auto at=imu(100);OfflineAiding::update(*e,a,at,s);
    CHECK(e->healthy() && s.heading_used==1 && s.heading_rejected==0);
    CHECK(std::abs(OfflineAiding::wrap(e->getNavState().euler.z()))<.1*D2R);
    const auto before=e->getNavState();a.heading_rad=90*D2R;
    OfflineAiding::update(*e,a,at,s);CHECK(s.heading_rejected==1);
    CHECK((e->getNavState().euler-before.euler).norm()<1e-12);
    o.initstate.euler.setZero();e.reset(new GIEngine(o));a=AidingObservation{};a.velocity=true;
    at.dtheta=Vector3d(0,0,.01); // 2 rad/s, 15 cm lateral lever -> +0.30 m/s forward.
    Vector3d earth(WGS84_WIE*std::cos(o.initstate.pos.x()),0,-WGS84_WIE*std::sin(o.initstate.pos.x()));
    a.velocity_ned=Vector3d(.3,0,0)-earth.cross(o.antlever);a.velocity_std.setConstant(.05);
    OfflineAiding::update(*e,a,at,s);CHECK(s.velocity_used==1);
    CHECK(e->getNavState().vel.norm()<1e-12);
    a.velocity_ned=Vector3d(100,100,100);OfflineAiding::update(*e,a,at,s);
    CHECK(s.velocity_rejected==1 && e->healthy());
    Eigen::LDLT<Matrix21> chol(e->getCovariance());CHECK(chol.info()==Eigen::Success);
    CHECK(chol.vectorD().minCoeff()>=0);
    // Added observations exercise every epoch placement and stay finite / PSD.
    for(int branch=1;branch<4;++branch) {
        e.reset(new GIEngine(o));e->addImuData(imu(100),true);e->addImuData(imu(100.005));
        a.time=branch==1?100:branch==2?100.005:100.0025;
        a.velocity_ned=Vector3d(.3,0,0);a.heading=true;a.heading_rad=.005;a.heading_std=D2R;
        AidingStats count;OfflineAiding::process(*e,&a,count);
        CHECK(e->healthy() && e->lastBranch()==branch && count.heading_used==1 && count.velocity_used==1);
        Eigen::LDLT<Matrix21> factor(e->getCovariance());CHECK(factor.vectorD().minCoeff()>=0);
    }
}
int main() {
    try {jacobians();unchangedBranches();aidingUpdates();std::cout<<"Jacobians, gates, wrap, lever, covariance, epoch branches passed\n";}
    catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
