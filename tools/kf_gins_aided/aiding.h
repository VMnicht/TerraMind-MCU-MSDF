// Full KF-GINS error state and mechanization retained. Offline experiment only.
// Derived observation extension: GPL-3.0-or-later; see KF-GINS/LICENSE.
#pragma once
#include "kf-gins/gi_engine.h"
#include "common/earth.h"
#include "common/rotation.h"
#include <cmath>
#include <stdexcept>

namespace tmnav {
struct AidingObservation {
    double time=0;
    bool position=false, velocity=false, heading=false;
    Vector3d blh=Vector3d::Zero(), position_std=Vector3d::Ones();
    Vector3d velocity_ned=Vector3d::Zero(), velocity_std=Vector3d::Ones();
    double heading_rad=0, heading_std=1, baseline_yaw=0;
};
struct AidingStats {
    unsigned velocity_used=0, velocity_rejected=0, heading_used=0, heading_rejected=0;
    double velocity_nis=-1, heading_nis=-1;
};
class OfflineAiding {
public:
    static double wrap(double v) {return std::atan2(std::sin(v),std::cos(v));}
    // Predicted-minus-true error convention agrees with GIEngine::stateFeedback:
    // true attitude = Exp(phi) * estimated attitude; true gyro bias = estimated + db.
    static double headingModel(const Matrix3d &c, double baseline_yaw, Matrix3x21 &h) {
        Vector3d b(std::cos(baseline_yaw),std::sin(baseline_yaw),0);
        Vector3d v=c*b; double q=v.x()*v.x()+v.y()*v.y();
        if(q<1e-6) throw std::runtime_error("Heading baseline is near vertical");
        Eigen::RowVector3d az;az << -v.y()/q,v.x()/q,0;
        h.setZero();h.block<1,3>(0,6)=az*Rotation::skewSymmetric(v);
        return std::atan2(v.y(),v.x());
    }
    static Vector3d velocityModel(const Matrix3d &c,const Vector3d &v,const Vector3d &omega,
                                   const Vector3d &lever,const Vector3d &earth,const Vector3d &scale,
                                   Matrix3x21 &h) {
        Vector3d arm=c*lever, rotational=c*omega.cross(lever);
        Matrix3d sl=Rotation::skewSymmetric(lever), se=Rotation::skewSymmetric(earth);
        h.setZero();h.block<3,3>(0,3).setIdentity();
        h.block<3,3>(0,6)=Rotation::skewSymmetric(rotational)-se*Rotation::skewSymmetric(arm);
        Vector3d inv=(Vector3d::Ones()+scale).cwiseInverse();
        h.block<3,3>(0,9)=-c*sl*inv.asDiagonal();
        h.block<3,3>(0,15)=-c*sl*omega.cwiseProduct(inv).asDiagonal();
        return v+rotational-earth.cross(arm);
    }
    static void update(GIEngine &e,const AidingObservation &o,const IMU &at,AidingStats &stats) {
        if(o.position) {
            GNSS g{};g.time=o.time;g.blh=o.blh;g.std=o.position_std;g.isvalid=true;
            e.gnssUpdate(g);
        }
        if(!e.healthy_) return;
        if(o.velocity) {
            Matrix3x21 h;
            double lat=e.pvacur_.pos[0];
            Vector3d earth(WGS84_WIE*std::cos(lat),0,-WGS84_WIE*std::sin(lat));
            Vector3d expected=velocityModel(e.pvacur_.att.cbn,e.pvacur_.vel,at.dtheta/at.dt,
                e.options_.antlever,earth,e.imuerror_.gyrscale,h);
            Matrix3d r=o.velocity_std.cwiseProduct(o.velocity_std).asDiagonal();
            bool used=gatedUpdate(e,expected-o.velocity_ned,h,r,16.27,stats.velocity_nis);
            used ? ++stats.velocity_used : ++stats.velocity_rejected;
        }
        if(o.heading && e.healthy_) {
            Matrix3x21 h;
            try {
                double expected=headingModel(e.pvacur_.att.cbn,o.baseline_yaw,h);
                Vector3d dz(wrap(expected-o.heading_rad),0,0);
                Matrix3d r=Matrix3d::Identity();r(0,0)=o.heading_std*o.heading_std;
                bool used=gatedUpdate(e,dz,h,r,10.83,stats.heading_nis);
                used ? ++stats.heading_used : ++stats.heading_rejected;
            } catch(const std::runtime_error &) {++stats.heading_rejected;stats.heading_nis=-1;}
        }
        if(e.healthy_) e.stateFeedback();
    }
    static bool gatedUpdate(GIEngine &e,const Vector3d &dz,const Matrix3x21 &h,
                            const Matrix3d &r,double limit,double &nis) {
        Matrix3d s=h*e.Cov_*h.transpose()+r;
        Eigen::LDLT<Matrix3d> solver(s);
        if(solver.info()!=Eigen::Success || !s.allFinite() || (solver.vectorD().array()<=0).any()) {
            e.healthy_=false;return false;
        }
        Vector3d innovation=dz-h*e.dx_;
        nis=innovation.dot(solver.solve(innovation));
        if(!std::isfinite(nis) || nis>limit) return false;
        e.EKFUpdate(dz,h,r);return e.healthy_;
    }
    // Same propagation branches and interpolation as KF-GINS, extra observations
    // applied at their GPS epoch before the original single state feedback.
    static void process(GIEngine &e,const AidingObservation *o,AidingStats &stats) {
        if(!o) {e.newImuProcess();return;}
        e.timestamp_=e.imucur_.time;
        int branch=e.isToUpdate(e.imupre_.time,e.imucur_.time,o->time);
        if(!branch) throw std::runtime_error("Observation outside IMU interval");
        e.last_branch_=branch;
        if(branch==1) {
            update(e,*o,e.imupre_,stats);if(!e.healthy_) return;
            e.pvapre_=e.pvacur_;e.insPropagation(e.imupre_,e.imucur_);
        } else if(branch==2) {
            e.insPropagation(e.imupre_,e.imucur_);update(e,*o,e.imucur_,stats);
        } else {
            IMU mid{};GIEngine::imuInterpolate(e.imupre_,e.imucur_,o->time,mid);
            e.insPropagation(e.imupre_,mid);update(e,*o,mid,stats);if(!e.healthy_) return;
            e.pvapre_=e.pvacur_;e.insPropagation(mid,e.imucur_);
        }
        if(!e.checkCov()) e.healthy_=false;
        e.pvapre_=e.pvacur_;e.imupre_=e.imucur_;
    }
};
}
