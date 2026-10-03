/* Derived from i2Nav-WHU/KF-GINS, commit 8291a93e49de513fe9d21f819500d39082ded611.
 * Copyright (C) 2022 i2Nav Group, Wuhan University. GPL-3.0-or-later.
 * See KF-GINS/LICENSE and Navigation/UPSTREAM.md.
 */
#ifndef TMNAV_GI_ENGINE_H
#define TMNAV_GI_ENGINE_H
#include "common/types.h"
#include "kf_gins_types.h"

namespace tmnav {
using Matrix21 = Eigen::Matrix<double, 21, 21>;
using Matrix21x18 = Eigen::Matrix<double, 21, 18>;
using Matrix3x21 = Eigen::Matrix<double, 3, 21>;
using Matrix21x3 = Eigen::Matrix<double, 21, 3>;

class GIEngine {
public:
    explicit GIEngine(GINSOptions &options);
    void addImuData(const IMU &imu, bool compensate = false) {
        imupre_ = imucur_;
        imucur_ = imu;
        if (compensate) {
            imuCompensate(imucur_);
            timestamp_ = imu.time;
        }
    }
    void addGnssData(const GNSS &gnss) { gnssdata_ = gnss; gnssdata_.isvalid = true; }
    void newImuProcess();
    double timestamp() const { return timestamp_; }
    NavState getNavState();
    const Matrix21 &getCovariance() const { return Cov_; }
    bool healthy() const { return healthy_; }
    int lastBranch() const { return last_branch_; }
    int updateBranchFor(double next_time, double gnss_time) const {
        return isToUpdate(timestamp_, next_time, gnss_time);
    }
    static void imuInterpolate(const IMU &imu1, IMU &imu2, double time, IMU &mid) {
        const double lambda = (time - imu1.time) / (imu2.time - imu1.time);
        mid = IMU{};
        mid.time = time;
        mid.dtheta = imu2.dtheta * lambda;
        mid.dvel = imu2.dvel * lambda;
        mid.dt = time - imu1.time;
        imu2.dtheta -= mid.dtheta;
        imu2.dvel -= mid.dvel;
        imu2.dt -= mid.dt;
    }
private:
    // Offline-only observation experiments reuse this full 21-state engine.
    // The friend has no definition in firmware targets and adds no state/code there.
    friend class OfflineAiding;
    void initialize(const NavState &, const NavState &);
    void imuCompensate(IMU &);
    int isToUpdate(double, double, double) const;
    void insPropagation(IMU &, IMU &);
    void gnssUpdate(GNSS &);
    void EKFPredict(const Matrix21 &, const Matrix21 &);
    void EKFUpdate(const Eigen::Vector3d &, const Matrix3x21 &, const Eigen::Matrix3d &);
    void stateFeedback();
    bool checkCov() const {
        return Cov_.allFinite() && (Cov_.diagonal().array() >= 0).all() &&
               pvacur_.pos.allFinite() && pvacur_.vel.allFinite() &&
               pvacur_.att.qbn.coeffs().allFinite() && pvacur_.att.euler.allFinite() &&
               imuerror_.gyrbias.allFinite() && imuerror_.accbias.allFinite() &&
               imuerror_.gyrscale.allFinite() && imuerror_.accscale.allFinite();
    }
    GINSOptions options_;
    double timestamp_;
    static constexpr double TIME_ALIGN_ERR = 0.001;
    static constexpr int RANK = 21, NOISERANK = 18;
    IMU imupre_, imucur_;
    GNSS gnssdata_;
    PVA pvacur_, pvapre_;
    ImuError imuerror_;
    Matrix21 Cov_, Phi_, F_, Qd_, work_a_, work_b_, joseph_;
    Eigen::Matrix<double, 18, 18> Qc_;
    Eigen::Matrix<double, 21, 1> dx_;
    Matrix21x18 G_, noise_tmp_;
    Matrix21x3 pht_, gain_;
    bool healthy_;
    int last_branch_;
    enum StateID { P_ID=0, V_ID=3, PHI_ID=6, BG_ID=9, BA_ID=12, SG_ID=15, SA_ID=18 };
    enum NoiseID { VRW_ID=0, ARW_ID=3, BGSTD_ID=6, BASTD_ID=9, SGSTD_ID=12, SASTD_ID=15 };
};
}
#endif
