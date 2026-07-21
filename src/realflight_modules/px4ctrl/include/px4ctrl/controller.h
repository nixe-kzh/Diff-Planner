/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef PX4CTRL_CONTROLLER_H_
#define PX4CTRL_CONTROLLER_H_

#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>

#include <px4ctrl/input_data.h>
#include <Eigen/Dense>

namespace px4ctrl {

struct DesiredState {
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Quaterniond q;
    double yaw;
    double yaw_rate;

    DesiredState() {};

    DesiredState(OdometryData &odom)
        : p(odom.p_),
          v(Eigen::Vector3d::Zero()),
          a(Eigen::Vector3d::Zero()),
          j(Eigen::Vector3d::Zero()),
          q(odom.q_),
          yaw(uav_utils::get_yaw_from_quaternion(odom.q_)),
          yaw_rate(0) {};
};

struct ControllerOutput {

    // Orientation of the body frame with respect to the world frame
    Eigen::Quaterniond q;

    // Body rates in body frame
    Eigen::Vector3d bodyrates; // [rad/s]

    // Collective mass normalized thrust
    double thrust;

    //Eigen::Vector3d des_v_real;
};

class Controller {
  public:
    Parameters &parameters_;

    Eigen::Vector3d kp_;
    Eigen::Vector3d kv_;
    Eigen::Vector3d kvi_;
    Eigen::Vector3d kvd_;
    Eigen::Vector3d k_ang_;
    Eigen::Vector3d integral_velocity_error_;
    Eigen::Vector3d gravity_;
    std::queue<std::pair<ros::Time, double>> timed_thrust_;
    ros::Time last_ctrl_timestamp_{ros::Time(0)};
    Eigen::Vector3d last_bodyrate_{Eigen::Vector3d(0, 0, 0)};

    quadrotor_msgs::Px4ctrlDebug debug_; //debug_

    // Thrust-accel mapping params
    double thrust_scale_compensation_;
    const double kRho2 = 0.998; // do not change
    double thrust_to_acceleration_;
    double p_;

    Controller(Parameters &);

    /* Algorithm0 from  Zhepei Wang*/
    quadrotor_msgs::Px4ctrlDebug UpdateAlg0(
        const DesiredState &des,
        const OdometryData &odom,
        const ImuData &imu,
        ControllerOutput &u,
        double voltage);

    Eigen::Vector3d ComputeLimitedTotalAccFromThrustForce(
        const Eigen::Vector3d &thrust_force,
        const double &mass) const;

    bool FlatnessWithDrag(const Eigen::Vector3d &vel,
                          const Eigen::Vector3d &acc,
                          const Eigen::Vector3d &jer,
                          const double &psi,
                          const double &dpsi,
                          double &thr,
                          Eigen::Vector4d &quat,
                          Eigen::Vector3d &omg,
                          const double &mass,
                          const double &grav,
                          const double &dh,
                          const double &dv,
                          const double &cp,
                          const double &veps) const;

    void MinimumSingularityFlatWithDrag(const double mass,
                                        const double grav,
                                        const Eigen::Vector3d &vel,
                                        const Eigen::Vector3d &acc,
                                        const Eigen::Vector3d &jer,
                                        const double &yaw,
                                        const double &yawd,
                                        const Eigen::Quaterniond &att_est,
                                        Eigen::Quaterniond &att,
                                        Eigen::Vector3d &omg,
                                        double &thrust) const;

    /* Algorithm1 from  Zhepei Wang*/
    quadrotor_msgs::Px4ctrlDebug UpdateAlg1(
        const DesiredState &des,
        const OdometryData &odom,
        const ImuData &imu,
        ControllerOutput &u,
        double voltage);

    void NormalizeWithGrad(
        const Eigen::Vector3d &x,
        const Eigen::Vector3d &xd,
        Eigen::Vector3d &x_norm,
        Eigen::Vector3d &x_norm_dot) const;

    void ComputeFlatInput(
        const Eigen::Vector3d &thr_acc,
        const Eigen::Vector3d &jer,
        const double &yaw,
        const double &yawd,
        const Eigen::Quaterniond &att_est,
        Eigen::Quaterniond &att,
        Eigen::Vector3d &omg) const;

    /*Algorithm from the rotor-drag paper*/
    void UpdateAlg2(
        const DesiredState &des,
        const OdometryData &odom,
        const ImuData &imu,
        ControllerOutput &u,
        double voltage);

    void ComputeAeroCompensatedReferenceInputs(
        const DesiredState &des,
        const OdometryData &odom, const Parameters &parameters,
        ControllerOutput *u, Eigen::Vector3d *drag_acc) const;

    Eigen::Quaterniond ComputeDesiredAttitude(
        const Eigen::Vector3d &des_acc, const double reference_heading,
        const Eigen::Quaterniond &est_q) const;

    Eigen::Vector3d ComputeRobustBodyXAxis(
        const Eigen::Vector3d &x_b_prototype, const Eigen::Vector3d &x_c,
        const Eigen::Vector3d &y_c,
        const Eigen::Quaterniond &est_q) const;

    /* Shared functions*/
    Eigen::Vector3d ComputePidErrorAcc(
        const OdometryData &odom, const DesiredState &des,
        const Parameters &parameters);

    Eigen::Vector3d ComputeLimitedTotalAcc(
        const Eigen::Vector3d &pid_error_acc,
        const Eigen::Vector3d &ref_acc,
        const Eigen::Vector3d &drag_acc = Eigen::Vector3d::Zero()) const;

    Eigen::Vector3d ComputeLimitedAngularAcc(
        const Eigen::Vector3d candidate_bodyrate);

    double ComputeDesiredCollectiveThrustSignal(
        const Eigen::Quaterniond &est_q,
        const Eigen::Vector3d &est_v,
        const Eigen::Vector3d &des_acc,
        const Parameters &parameters,
        double voltage);

    double AccurateThrustAccMapping(
        const double des_acc_z,
        double voltage,
        const Parameters &parameters) const;

    Eigen::Vector3d ComputeFeedbackControlBodyrates(
        const Eigen::Quaterniond &des_q,
        const Eigen::Quaterniond &est_q,
        const Parameters &parameters);

    bool EstimateThrustModel(
        const Eigen::Vector3d &est_a,
        const double voltage,
        const Eigen::Vector3d &est_v,
        const Parameters &parameters);

    bool AlmostZero(const double value) const;

    bool AlmostZeroThrust(const double thrust_value) const;

    void ResetThrustMapping(void);

  private:
    static constexpr double kMinNormalizedCollectiveAcc = 3;
    static constexpr double kAlmostZeroValueThreshold = 0.001;
    static constexpr double kAlmostZeroThrustThreshold = 0.01;
    static constexpr double kMaxBodyratesFeedback = 4;
    static constexpr double kMaxAngularAcc = 60;
};

}  // namespace px4ctrl

#endif
