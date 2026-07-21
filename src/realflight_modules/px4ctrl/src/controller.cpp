#include <px4ctrl/controller.h>
#include <rosfmt/rosfmt.h>

using namespace std;

namespace px4ctrl {

Controller::Controller(Parameters &parameters) : parameters_(parameters) {
    integral_velocity_error_.setZero();
    kp_(0) = parameters_.gain_.kp0;
    kp_(1) = parameters_.gain_.kp1;
    kp_(2) = parameters_.gain_.kp2;
    kv_(0) = parameters_.gain_.kv0;
    kv_(1) = parameters_.gain_.kv1;
    kv_(2) = parameters_.gain_.kv2;
    kvi_(0) = parameters_.gain_.kvi0;
    kvi_(1) = parameters_.gain_.kvi1;
    kvi_(2) = parameters_.gain_.kvi2;
    kvd_(0) = parameters_.gain_.kvd0;
    kvd_(1) = parameters_.gain_.kvd1;
    kvd_(2) = parameters_.gain_.kvd2;
    k_ang_(0) = parameters_.gain_.k_ang_r;
    k_ang_(1) = parameters_.gain_.k_ang_p;
    k_ang_(2) = parameters_.gain_.k_ang_y;

    ResetThrustMapping();
    gravity_ = Eigen::Vector3d(0.0, 0.0, -parameters_.gra_);
}

/************* Algorithm0 from the Zhepei Wang, start ***************/

quadrotor_msgs::Px4ctrlDebug Controller::UpdateAlg0(
    const DesiredState &des,
    const OdometryData &odom,
    const ImuData &imu,
    ControllerOutput &u,
    double voltage) {
    // Check the given velocity is valid.
    if (des.v(2) < -3.0)
        ROSFMT_WARN("Unsafe desired vertical speed: {:.3f} m/s", des.v(2));

    // Compute desired control commands
    const Eigen::Vector3d pid_error_accelerations = ComputePidErrorAcc(odom, des, parameters_);
    Eigen::Vector3d translational_acc = pid_error_accelerations + des.a;
    Eigen::Quaterniond desired_attitude, idel_att;
    Eigen::Vector3d omega;
    double thrust, debug_thrust;
    translational_acc = (gravity_ + ComputeLimitedTotalAccFromThrustForce(translational_acc - gravity_, 1.0)).eval();

    // wmywmy
    MinimumSingularityFlatWithDrag(parameters_.mass_, parameters_.gra_,
                                   des.v, des.a, des.j, des.yaw, des.yaw_rate,
                                   odom.q_, idel_att, omega, debug_thrust);
    // wmywmy

    MinimumSingularityFlatWithDrag(parameters_.mass_, parameters_.gra_,
                                   des.v, translational_acc, des.j, des.yaw, des.yaw_rate,
                                   odom.q_, desired_attitude, u.bodyrates, thrust);

    Eigen::Vector3d thrust_force = desired_attitude * (thrust * Eigen::Vector3d::UnitZ());
    Eigen::Vector3d total_des_acc = ComputeLimitedTotalAccFromThrustForce(thrust_force, parameters_.mass_);

    u.thrust = ComputeDesiredCollectiveThrustSignal(odom.q_, odom.v_, total_des_acc, parameters_, voltage);

    const Eigen::Vector3d feedback_bodyrates = ComputeFeedbackControlBodyrates(desired_attitude, odom.q_, parameters_);

    // Compute the error quaternion wmywmy
    const Eigen::Quaterniond q_e = idel_att.inverse() * desired_attitude;

    Eigen::AngleAxisd rotation_vector(q_e); //debug_
    Eigen::Vector3d axis = rotation_vector.axis();
    debug_.fb_axisang_x = axis(0);
    debug_.fb_axisang_y = axis(1);
    debug_.fb_axisang_z = axis(2);
    debug_.fb_axisang_ang = rotation_vector.angle();
    // wmywmy

    debug_.fb_a_x = pid_error_accelerations(0); //debug_
    debug_.fb_a_y = pid_error_accelerations(1);
    debug_.fb_a_z = pid_error_accelerations(2);
    debug_.des_a_x = total_des_acc(0);
    debug_.des_a_y = total_des_acc(1);
    debug_.des_a_z = total_des_acc(2);
    debug_.des_q_w = desired_attitude.w(); //debug_
    debug_.des_q_x = desired_attitude.x();
    debug_.des_q_y = desired_attitude.y();
    debug_.des_q_z = desired_attitude.z();

    u.q = imu.q_ * odom.q_.inverse() * desired_attitude; // Align with FCU frame
    const Eigen::Vector3d bodyrate_candidate = u.bodyrates + feedback_bodyrates;

    // limit the angular acceleration
    u.bodyrates = ComputeLimitedAngularAcc(bodyrate_candidate);
    // u.bodyrates += feedback_bodyrates;

    // Used for thrust-accel mapping estimation
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrust_.size() > 100)
        timed_thrust_.pop();

    return debug_; //debug_
};

Eigen::Vector3d Controller::ComputeLimitedTotalAccFromThrustForce(
    const Eigen::Vector3d &thrust_force,
    const double &mass) const {
    Eigen::Vector3d total_acc = thrust_force / mass;
    double total_acc_norm = total_acc.norm();

    // Limit magnitude
    if (total_acc_norm < kAlmostZeroValueThreshold) {
        // The thrust direction is undefined for a zero vector. Use the upright
        // direction instead of normalizing the zero vector and generating NaNs.
        total_acc = kMinNormalizedCollectiveAcc * Eigen::Vector3d::UnitZ();
        total_acc_norm = kMinNormalizedCollectiveAcc;
    } else if (total_acc_norm < kMinNormalizedCollectiveAcc) {
        total_acc *= kMinNormalizedCollectiveAcc / total_acc_norm;
        total_acc_norm = kMinNormalizedCollectiveAcc;
    }

    // Limit angle
    if (parameters_.max_angle_ > 0) {
        double z_acc = total_acc.dot(Eigen::Vector3d::UnitZ());
        const Eigen::Vector3d z_b = total_acc / total_acc_norm;
        if (z_acc < kMinNormalizedCollectiveAcc) {
            z_acc = kMinNormalizedCollectiveAcc; // Not allow too small z-force when angle limit is enabled.
        }
        const double cos_rot_ang = std::max(-1.0, std::min(1.0, Eigen::Vector3d::UnitZ().dot(z_b)));
        const double rot_ang = std::acos(cos_rot_ang);
        if (rot_ang > parameters_.max_angle_) { // Exceed the angle limit
            Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ().cross(z_b);
            const double rot_axis_norm = rot_axis.norm();
            if (rot_axis_norm < kAlmostZeroValueThreshold) {
                // z_b is antiparallel to UnitZ. The limiting rotation axis is not
                // unique, so choose a deterministic horizontal axis.
                rot_axis = Eigen::Vector3d::UnitX();
            } else {
                rot_axis /= rot_axis_norm;
            }
            Eigen::Vector3d limited_z_b = Eigen::AngleAxisd(parameters_.max_angle_, rot_axis) * Eigen::Vector3d::UnitZ();
            total_acc = z_acc / std::cos(parameters_.max_angle_) * limited_z_b;
        }
    }

    return total_acc;
}

bool Controller::FlatnessWithDrag(const Eigen::Vector3d &vel,
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
                                  const double &veps) const {
    const double almost_zero = 1.0e-6;

    double w0, w1, w2, dw0, dw1, dw2;
    double v0, v1, v2, a0, a1, a2, v_dot_a;
    double z0, z1, z2, dz0, dz1, dz2;
    double cp_term, w_term, dh_over_m;
    double zu_sqr_norm, zu_norm, zu0, zu1, zu2;
    double zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
    double ng00, ng01, ng02, ng11, ng12, ng22, ng_den;
    double dw_term, dz_term0, dz_term1, dz_term2, f_term0, f_term1, f_term2;
    double tilt_den, tilt0, tilt1, tilt2, c_half_psi, s_half_psi;
    double c_psi, s_psi, omg_den, omg_term;

    v0 = vel(0);
    v1 = vel(1);
    v2 = vel(2);
    a0 = acc(0);
    a1 = acc(1);
    a2 = acc(2);
    cp_term = sqrt(v0 * v0 + v1 * v1 + v2 * v2 + veps);
    w_term = 1.0 + cp * cp_term;
    w0 = w_term * v0;
    w1 = w_term * v1;
    w2 = w_term * v2;
    dh_over_m = dh / mass;
    zu0 = a0 + dh_over_m * w0;
    zu1 = a1 + dh_over_m * w1;
    zu2 = a2 + dh_over_m * w2 + grav;
    zu_sqr0 = zu0 * zu0;
    zu_sqr1 = zu1 * zu1;
    zu_sqr2 = zu2 * zu2;
    zu01 = zu0 * zu1;
    zu12 = zu1 * zu2;
    zu02 = zu0 * zu2;
    zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
    zu_norm = sqrt(zu_sqr_norm);
    if (zu_norm < almost_zero) {
        return false;
    }
    z0 = zu0 / zu_norm;
    z1 = zu1 / zu_norm;
    z2 = zu2 / zu_norm;
    ng_den = zu_sqr_norm * zu_norm;
    ng00 = (zu_sqr1 + zu_sqr2) / ng_den;
    ng01 = -zu01 / ng_den;
    ng02 = -zu02 / ng_den;
    ng11 = (zu_sqr0 + zu_sqr2) / ng_den;
    ng12 = -zu12 / ng_den;
    ng22 = (zu_sqr0 + zu_sqr1) / ng_den;
    v_dot_a = v0 * a0 + v1 * a1 + v2 * a2;
    dw_term = cp * v_dot_a / cp_term;
    dw0 = w_term * a0 + dw_term * v0;
    dw1 = w_term * a1 + dw_term * v1;
    dw2 = w_term * a2 + dw_term * v2;
    dz_term0 = jer(0) + dh_over_m * dw0;
    dz_term1 = jer(1) + dh_over_m * dw1;
    dz_term2 = jer(2) + dh_over_m * dw2;
    dz0 = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
    dz1 = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
    dz2 = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;
    f_term0 = mass * a0 + dv * w0;
    f_term1 = mass * a1 + dv * w1;
    f_term2 = mass * (a2 + grav) + dv * w2;
    thr = z0 * f_term0 + z1 * f_term1 + z2 * f_term2;
    if (1.0 + z2 < almost_zero) {
        return false;
    }
    tilt_den = sqrt(2.0 * (1.0 + z2));
    tilt0 = 0.5 * tilt_den;
    tilt1 = -z1 / tilt_den;
    tilt2 = z0 / tilt_den;
    c_half_psi = cos(0.5 * psi);
    s_half_psi = sin(0.5 * psi);
    quat(0) = tilt0 * c_half_psi;
    quat(1) = tilt1 * c_half_psi + tilt2 * s_half_psi;
    quat(2) = tilt2 * c_half_psi - tilt1 * s_half_psi;
    quat(3) = tilt0 * s_half_psi;
    c_psi = cos(psi);
    s_psi = sin(psi);
    omg_den = z2 + 1.0;
    omg_term = dz2 / omg_den;
    omg(0) = dz0 * s_psi - dz1 * c_psi -
             (z0 * s_psi - z1 * c_psi) * omg_term;
    omg(1) = dz0 * c_psi + dz1 * s_psi -
             (z0 * c_psi + z1 * s_psi) * omg_term;
    omg(2) = (z1 * dz0 - z0 * dz1) / omg_den + dpsi;

    return true;
}

// grav is the gravitional acceleration
// the coordinate should have upward z-axis
void Controller::MinimumSingularityFlatWithDrag(const double mass,
        const double grav,
        const Eigen::Vector3d &vel,
        const Eigen::Vector3d &acc,
        const Eigen::Vector3d &jer,
        const double &yaw,
        const double &yawd,
        const Eigen::Quaterniond &att_est,
        Eigen::Quaterniond &att,
        Eigen::Vector3d &omg,
        double &thrust) const {

    // Drag effect parameters (Drag may cause larger tracking error in aggressive flight during our tests)
    // dv >= dh is required
    // dv is the rotor drag effect in vertical direction, typical value is 0.35
    // dh is the rotor drag effect in horizontal direction, typical value is 0.25
    // cp is the second-order drag effect, typical valye is 0.01
    // const double dh = 0.10;
    // const double dv = 0.23;
    // const double cp = 0.01;
    const double dh = 0.00;
    const double dv = 0.00;
    const double cp = 0.00;

    // veps is a smnoothing constant, do not change it
    const double veps = 0.02; //ms^-s

    static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);
    static double thrust_old = mass * (acc + grav * Eigen::Vector3d::UnitZ()).norm();

    Eigen::Vector4d quat;
    if (FlatnessWithDrag(vel, acc, jer, yaw, yawd,
                         thrust, quat, omg,
                         mass, grav, dh, dv, cp, veps)) {
        att = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3));
        omg_old = omg;
        thrust_old = thrust;
    } else {
        ROSFMT_WARN("Attitude singularity: inverted or unactuated flight");
        att = att_est;
        omg = omg_old;
        thrust = thrust_old;
    }

    return;
}
/************* Algorithm0 from Zhepei Wang, end ***************/

/************* Algorithm1 from the Zhepei Wang, start ***************/
quadrotor_msgs::Px4ctrlDebug Controller::UpdateAlg1(
    const DesiredState &des,
    const OdometryData &odom,
    const ImuData &imu,
    ControllerOutput &u,
    double voltage) {
    // Check the given velocity is valid.
    if (des.v(2) < -3.0)
        ROSFMT_WARN("Unsafe desired vertical speed: {:.3f} m/s", des.v(2));

    // Compute desired control commands
    const Eigen::Vector3d pid_error_accelerations = ComputePidErrorAcc(odom, des, parameters_);
    Eigen::Vector3d total_des_acc = ComputeLimitedTotalAcc(pid_error_accelerations, des.a);

    debug_.fb_a_x = pid_error_accelerations(0); //debug_
    debug_.fb_a_y = pid_error_accelerations(1);
    debug_.fb_a_z = pid_error_accelerations(2);
    debug_.des_a_x = total_des_acc(0);
    debug_.des_a_y = total_des_acc(1);
    debug_.des_a_z = total_des_acc(2);

    u.thrust = ComputeDesiredCollectiveThrustSignal(odom.q_, odom.v_, total_des_acc, parameters_, voltage);

    Eigen::Quaterniond desired_attitude;
    ComputeFlatInput(total_des_acc, des.j, des.yaw, des.yaw_rate, odom.q_, desired_attitude, u.bodyrates);
    const Eigen::Vector3d feedback_bodyrates = ComputeFeedbackControlBodyrates(desired_attitude, odom.q_, parameters_);

    debug_.des_q_w = desired_attitude.w(); //debug_
    debug_.des_q_x = desired_attitude.x();
    debug_.des_q_y = desired_attitude.y();
    debug_.des_q_z = desired_attitude.z();

    u.q = imu.q_ * odom.q_.inverse() * desired_attitude; // Align with FCU frame
    u.bodyrates += feedback_bodyrates;

    // Used for thrust-accel mapping estimation
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrust_.size() > 100)
        timed_thrust_.pop();

    return debug_; //debug_
};

void Controller::NormalizeWithGrad(const Eigen::Vector3d &x,
                                   const Eigen::Vector3d &xd,
                                   Eigen::Vector3d &x_norm,
                                   Eigen::Vector3d &x_norm_dot) const {
    const double x_squared_norm = x.squaredNorm();
    const double x_norm_value = sqrt(x_squared_norm);
    x_norm = x / x_norm_value;
    x_norm_dot = (xd - x * (x.dot(xd) / x_squared_norm)) / x_norm_value;
    return;
}

// grav is the gravitional acceleration
// the coordinate should have upward z-axis
void Controller::ComputeFlatInput(const Eigen::Vector3d &thr_acc,
                                  const Eigen::Vector3d &jer,
                                  const double &yaw,
                                  const double &yawd,
                                  const Eigen::Quaterniond &att_est,
                                  Eigen::Quaterniond &att,
                                  Eigen::Vector3d &omg) const {
    static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);

    if (thr_acc.norm() < kMinNormalizedCollectiveAcc) {
        att = att_est;
        omg.setConstant(0.0);
        // ROSFMT_WARN("Thrust acceleration too small: {:.3f}", thr_acc.norm());
        return;
    } else {
        Eigen::Vector3d zb, zbd;
        NormalizeWithGrad(thr_acc, jer, zb, zbd);
        double syaw = sin(yaw);
        double cyaw = cos(yaw);
        Eigen::Vector3d xc(cyaw, syaw, 0.0);
        Eigen::Vector3d xcd(-syaw * yawd, cyaw * yawd, 0.0);
        Eigen::Vector3d yc = zb.cross(xc);
        if (yc.norm() < kAlmostZeroValueThreshold) {
            ROSFMT_WARN("Attitude singularity: pitch near 90 deg");
            att = att_est;
            omg = omg_old;
        } else {
            Eigen::Vector3d ycd = zbd.cross(xc) + zb.cross(xcd);
            Eigen::Vector3d yb, ybd;
            NormalizeWithGrad(yc, ycd, yb, ybd);
            Eigen::Vector3d xb = yb.cross(zb);
            Eigen::Vector3d xbd = ybd.cross(zb) + yb.cross(zbd);
            omg(0) = (zb.dot(ybd) - yb.dot(zbd)) / 2.0;
            omg(1) = (xb.dot(zbd) - zb.dot(xbd)) / 2.0;
            omg(2) = (yb.dot(xbd) - xb.dot(ybd)) / 2.0;
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix << xb, yb, zb;
            att = Eigen::Quaterniond(rotation_matrix);
            omg_old = omg;
        }
    }
    return;
}
/************* Algorithm1 from Zhepei Wang, end ***************/

/************* Algorithm from the rotor-drag paper, start ***************/
void Controller::UpdateAlg2(
    const DesiredState &des,
    const OdometryData &odom,
    const ImuData &imu,
    ControllerOutput &u,
    double voltage) {
    // Check the given velocity is valid.
    if (des.v(2) < -3.0)
        ROSFMT_WARN("Unsafe desired vertical speed: {:.3f} m/s", des.v(2));

    // Compute reference inputs that compensate for aerodynamic drag
    Eigen::Vector3d drag_acc = Eigen::Vector3d::Zero();
    ComputeAeroCompensatedReferenceInputs(des, odom, parameters_, &u, &drag_acc);

    // Compute desired control commands
    const Eigen::Vector3d pid_error_accelerations = ComputePidErrorAcc(odom, des, parameters_);
    Eigen::Vector3d total_des_acc = ComputeLimitedTotalAcc(pid_error_accelerations, des.a, drag_acc);

    u.thrust = ComputeDesiredCollectiveThrustSignal(odom.q_, odom.v_, total_des_acc, parameters_, voltage);

    const Eigen::Quaterniond desired_attitude = ComputeDesiredAttitude(total_des_acc, des.yaw, odom.q_);
    const Eigen::Vector3d feedback_bodyrates = ComputeFeedbackControlBodyrates(desired_attitude, odom.q_, parameters_);

    u.q = imu.q_ * odom.q_.inverse() * desired_attitude; // Align with FCU frame

    // Used for thrust-accel mapping estimation
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrust_.size() > 100)
        timed_thrust_.pop();
};

void Controller::ComputeAeroCompensatedReferenceInputs(
    const DesiredState &des,
    const OdometryData &odom,
    const Parameters &parameters,
    ControllerOutput *outputs,
    Eigen::Vector3d *drag_acc) const {

    const double dx = parameters.rotor_drag_.x;
    const double dy = parameters.rotor_drag_.y;
    const double dz = parameters.rotor_drag_.z;

    const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
            Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()));

    const Eigen::Vector3d x_c = q_heading * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d y_c = q_heading * Eigen::Vector3d::UnitY();

    const Eigen::Vector3d alpha =
        des.a - gravity_ + dx * des.v;
    const Eigen::Vector3d beta =
        des.a - gravity_ + dy * des.v;
    const Eigen::Vector3d gamma =
        des.a - gravity_ + dz * des.v;

    // Reference attitude
    const Eigen::Vector3d x_b_prototype = y_c.cross(alpha);
    const Eigen::Vector3d x_b = ComputeRobustBodyXAxis(x_b_prototype, x_c, y_c, odom.q_);

    Eigen::Vector3d y_b = beta.cross(x_b);
    if (AlmostZero(y_b.norm())) {
        const Eigen::Vector3d z_b_estimated =
            odom.q_ * Eigen::Vector3d::UnitZ();
        y_b = z_b_estimated.cross(x_b);
        if (AlmostZero(y_b.norm())) {
            y_b = y_c;
        } else {
            y_b.normalize();
        }
    } else {
        y_b.normalize();
    }

    const Eigen::Vector3d z_b = x_b.cross(y_b);

    const Eigen::Matrix3d r_w_b_ref(
        (Eigen::Matrix3d() << x_b, y_b, z_b).finished());

    outputs->q = Eigen::Quaterniond(r_w_b_ref);

    // Reference thrust
    outputs->thrust = z_b.dot(gamma);

    // Rotor drag matrix
    const Eigen::Matrix3d d = Eigen::Vector3d(dx, dy, dz).asDiagonal();

    // Reference body rates
    const double b1 = outputs->thrust -
                      (dz - dx) * z_b.dot(des.v);
    const double c1 = -(dx - dy) * y_b.dot(des.v);
    const double d1 = x_b.dot(des.j) +
                      dx * x_b.dot(des.a);
    const double a2 = outputs->thrust +
                      (dy - dz) * z_b.dot(des.v);
    const double c2 = (dx - dy) * x_b.dot(des.v);
    const double d2 = -y_b.dot(des.j) -
                      dy * y_b.dot(des.a);
    const double b3 = -y_c.dot(z_b);
    const double c3 = (y_c.cross(z_b)).norm();
    const double d3 = des.yaw_rate * x_c.dot(x_b);

    const double denominator = b1 * c3 - b3 * c1;

    if (AlmostZero(denominator)) {
        outputs->bodyrates = Eigen::Vector3d::Zero();
    } else {
        // Compute body rates
        if (AlmostZero(a2)) {
            outputs->bodyrates.x() = 0.0;
        } else {
            outputs->bodyrates.x() =
                (-b1 * c2 * d3 + b1 * c3 * d2 - b3 * c1 * d2 + b3 * c2 * d1) /
                (a2 * denominator);
        }
        outputs->bodyrates.y() = (-c1 * d3 + c3 * d1) / denominator;
        outputs->bodyrates.z() = (b1 * d3 - b3 * d1) / denominator;
    }

    // Transform reference rates and derivatives into estimated body frame
    const Eigen::Matrix3d r_trans =
        odom.q_.toRotationMatrix().transpose() * r_w_b_ref;
    const Eigen::Vector3d bodyrates_ref = outputs->bodyrates;

    outputs->bodyrates = r_trans * bodyrates_ref;

    // Drag accelerations
    *drag_acc = -1.0 * (r_w_b_ref * (d * (r_w_b_ref.transpose() * des.v)));
}

Eigen::Quaterniond Controller::ComputeDesiredAttitude(
    const Eigen::Vector3d &des_acc, const double reference_heading,
    const Eigen::Quaterniond &est_q) const {
    const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
            Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));

    // Compute desired orientation
    const Eigen::Vector3d x_c = q_heading * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d y_c = q_heading * Eigen::Vector3d::UnitY();

    // Eigen::Vector3d des_acc2(-1.0,-1.0,-1.0);

    Eigen::Vector3d z_b;
    if (AlmostZero(des_acc.norm())) {
        // In case of free fall we keep the thrust direction to be the estimated one
        // This only works assuming that we are in this condition for a very short
        // time (otherwise attitude drifts)
        z_b = est_q * Eigen::Vector3d::UnitZ();
    } else {
        z_b = des_acc.normalized();
    }

    const Eigen::Vector3d x_b_prototype = y_c.cross(z_b);
    const Eigen::Vector3d x_b = ComputeRobustBodyXAxis(x_b_prototype, x_c, y_c, est_q);

    const Eigen::Vector3d y_b = (z_b.cross(x_b)).normalized();

    // From the computed desired body axes we can now compose a desired attitude
    const Eigen::Matrix3d r_w_b((Eigen::Matrix3d() << x_b, y_b, z_b).finished());

    const Eigen::Quaterniond desired_attitude(r_w_b);

    return desired_attitude;
}

Eigen::Vector3d Controller::ComputeRobustBodyXAxis(
    const Eigen::Vector3d &x_b_prototype, const Eigen::Vector3d &x_c,
    const Eigen::Vector3d &y_c,
    const Eigen::Quaterniond &est_q) const {
    Eigen::Vector3d x_b = x_b_prototype;

    // cout << "x_b.norm()=" << x_b.norm() << endl;

    if (AlmostZero(x_b.norm())) {
        // if cross(y_c, z_b) == 0, they are collinear =>
        // every x_b lies automatically in the x_c - z_c plane

        // Project estimated body x-axis into the x_c - z_c plane
        const Eigen::Vector3d x_b_estimated =
            est_q * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d x_b_projected =
            x_b_estimated - (x_b_estimated.dot(y_c)) * y_c;
        if (AlmostZero(x_b_projected.norm())) {
            // Not too much intelligent stuff we can do in this case but it should
            // basically never occur
            x_b = x_c;
        } else {
            x_b = x_b_projected.normalized();
        }
    } else {
        x_b.normalize();
    }

    // if the quad is upside down, x_b will point in the "opposite" direction
    // of x_c => flip x_b (unfortunately also not the solution for our problems)
    if (x_b.dot(x_c) < 0.0) {
        x_b = -x_b;
        // std::cout << "CCCCCCCCCCCCCC" << std::endl;
    }

    return x_b;
}
/************* Algorithm from the rotor-drag paper, end ***************/

Eigen::Vector3d Controller::ComputeFeedbackControlBodyrates(
    const Eigen::Quaterniond &des_q,
    const Eigen::Quaterniond &est_q,
    const Parameters &parameters) {
    // Compute the error quaternion
    const Eigen::Quaterniond q_e = est_q.inverse() * des_q;

    Eigen::AngleAxisd rotation_vector(q_e); //debug_
    Eigen::Vector3d axis = rotation_vector.axis();
    debug_.exec_err_axisang_x = axis(0);
    debug_.exec_err_axisang_y = axis(1);
    debug_.exec_err_axisang_z = axis(2);
    debug_.exec_err_axisang_ang = rotation_vector.angle();

    // Compute desired body rates from control error
    Eigen::Vector3d bodyrates;

    if (q_e.w() >= 0) {
        bodyrates.x() = 2.0 * k_ang_(0) * q_e.x();
        bodyrates.y() = 2.0 * k_ang_(1) * q_e.y();
        bodyrates.z() = 2.0 * k_ang_(2) * q_e.z();
    } else {
        bodyrates.x() = -2.0 * k_ang_(0) * q_e.x();
        bodyrates.y() = -2.0 * k_ang_(1) * q_e.y();
        bodyrates.z() = -2.0 * k_ang_(2) * q_e.z();
    }

    if (bodyrates.x() > kMaxBodyratesFeedback)
        bodyrates.x() = kMaxBodyratesFeedback;
    if (bodyrates.x() < -kMaxBodyratesFeedback)
        bodyrates.x() = -kMaxBodyratesFeedback;
    if (bodyrates.y() > kMaxBodyratesFeedback)
        bodyrates.y() = kMaxBodyratesFeedback;
    if (bodyrates.y() < -kMaxBodyratesFeedback)
        bodyrates.y() = -kMaxBodyratesFeedback;
    if (bodyrates.z() > kMaxBodyratesFeedback)
        bodyrates.z() = kMaxBodyratesFeedback;
    if (bodyrates.z() < -kMaxBodyratesFeedback)
        bodyrates.z() = -kMaxBodyratesFeedback;

    //debug_
    debug_.fb_rate_x = bodyrates.x();
    debug_.fb_rate_y = bodyrates.y();
    debug_.fb_rate_z = bodyrates.z();

    return bodyrates;
}

Eigen::Vector3d Controller::ComputePidErrorAcc(
    const OdometryData &odom,
    const DesiredState &des,
    const Parameters &parameters) {
    // Compute the desired accelerations due to control errors in world frame
    // with a PID controller
    Eigen::Vector3d acc_error;

    // x acceleration
    double x_pos_error = std::isnan(des.p(0)) ? 0.0 : std::max(std::min(des.p(0) - odom.p_(0), 1.0), -1.0);
    double x_vel_error = std::max(std::min((des.v(0) + kp_(0) * x_pos_error) - odom.v_(0), 1.0), -1.0);
    acc_error(0) = kv_(0) * x_vel_error;

    // y acceleration
    double y_pos_error = std::isnan(des.p(1)) ? 0.0 : std::max(std::min(des.p(1) - odom.p_(1), 1.0), -1.0);
    double y_vel_error = std::max(std::min((des.v(1) + kp_(1) * y_pos_error) - odom.v_(1), 1.0), -1.0);
    acc_error(1) = kv_(1) * y_vel_error;

    // z acceleration
    double z_pos_error = std::isnan(des.p(2)) ? 0.0 : std::max(std::min(des.p(2) - odom.p_(2), 1.0), -1.0);
    double z_vel_error = std::max(std::min((des.v(2) + kp_(2) * z_pos_error) - odom.v_(2), 1.0), -1.0);
    acc_error(2) = kv_(2) * z_vel_error;

    debug_.des_v_x = (des.v(0) + kp_(0) * x_pos_error); //debug_
    debug_.des_v_y = (des.v(1) + kp_(1) * y_pos_error);
    debug_.des_v_z = (des.v(2) + kp_(2) * z_pos_error);

    return acc_error;
}

Eigen::Vector3d Controller::ComputeLimitedTotalAcc(
    const Eigen::Vector3d &pid_error_acc,
    const Eigen::Vector3d &ref_acc,
    const Eigen::Vector3d &drag_acc /*default = Eigen::Vector3d::Zero() */) const {
    Eigen::Vector3d total_acc;
    total_acc = pid_error_acc + ref_acc - gravity_ - drag_acc;

    // Limit magnitude
    if (total_acc.norm() < kMinNormalizedCollectiveAcc) {
        total_acc = total_acc.normalized() * kMinNormalizedCollectiveAcc;
    }

    // Limit angle
    if (parameters_.max_angle_ > 0) {
        double z_acc = total_acc.dot(Eigen::Vector3d::UnitZ());
        Eigen::Vector3d z_b = total_acc.normalized();
        if (z_acc < kMinNormalizedCollectiveAcc) {
            z_acc = kMinNormalizedCollectiveAcc; // Not allow too small z-force when angle limit is enabled.
        }
        Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ().cross(z_b).normalized();
        double rot_ang = std::acos(Eigen::Vector3d::UnitZ().dot(z_b) / (1 * 1));
        if (rot_ang > parameters_.max_angle_) { // Exceed the angle limit
            Eigen::Vector3d limited_z_b = Eigen::AngleAxisd(parameters_.max_angle_, rot_axis) * Eigen::Vector3d::UnitZ();
            total_acc = z_acc / std::cos(parameters_.max_angle_) * limited_z_b;
        }
    }

    return total_acc;
}

Eigen::Vector3d Controller::ComputeLimitedAngularAcc(const Eigen::Vector3d candidate_bodyrate) {
    ros::Time t_now = ros::Time::now();
    if (last_ctrl_timestamp_ != ros::Time(0)) {
        double dura = (t_now - last_ctrl_timestamp_).toSec();
        double max_delta_bodyrate = kMaxAngularAcc * dura;
        Eigen::Vector3d bodyrate_out;

        if ((candidate_bodyrate(0) - last_bodyrate_(0)) > max_delta_bodyrate) {
            bodyrate_out(0) = last_bodyrate_(0) + max_delta_bodyrate;
        } else if ((candidate_bodyrate(0) - last_bodyrate_(0)) < -max_delta_bodyrate) {
            bodyrate_out(0) = last_bodyrate_(0) - max_delta_bodyrate;
        } else {
            bodyrate_out(0) = candidate_bodyrate(0);
        }

        if ((candidate_bodyrate(1) - last_bodyrate_(1)) > max_delta_bodyrate) {
            bodyrate_out(1) = last_bodyrate_(1) + max_delta_bodyrate;
        } else if ((candidate_bodyrate(1) - last_bodyrate_(1)) < -max_delta_bodyrate) {
            bodyrate_out(1) = last_bodyrate_(1) - max_delta_bodyrate;
        } else {
            bodyrate_out(1) = candidate_bodyrate(1);
        }

        if ((candidate_bodyrate(2) - last_bodyrate_(2)) > max_delta_bodyrate) {
            bodyrate_out(2) = last_bodyrate_(2) + max_delta_bodyrate;
        } else if ((candidate_bodyrate(2) - last_bodyrate_(2)) < -max_delta_bodyrate) {
            bodyrate_out(2) = last_bodyrate_(2) - max_delta_bodyrate;
        } else {
            bodyrate_out(2) = candidate_bodyrate(2);
        }

        last_ctrl_timestamp_ = t_now;
        last_bodyrate_ = bodyrate_out;

        return bodyrate_out;
    } else {
        last_ctrl_timestamp_ = t_now;
        last_bodyrate_ = candidate_bodyrate;
        return candidate_bodyrate;
    }
}

double Controller::ComputeDesiredCollectiveThrustSignal(
    const Eigen::Quaterniond &est_q,
    const Eigen::Vector3d &est_v,
    const Eigen::Vector3d &des_acc,
    const Parameters &parameters,
    double voltage) {

    double normalized_thrust;
    const Eigen::Vector3d body_z_axis = est_q * Eigen::Vector3d::UnitZ();
    double des_acc_norm = des_acc.dot(body_z_axis);
    // double des_acc_norm = des_acc.norm();
    if (des_acc_norm < kMinNormalizedCollectiveAcc) {
        des_acc_norm = kMinNormalizedCollectiveAcc;
    }

    // This compensates for an acceleration component in thrust direction due
    // to the square of the body-horizontal velocity.
    des_acc_norm -= parameters.rotor_drag_.k_thrust_horz * (pow(est_v.x(), 2.0) + pow(est_v.y(), 2.0));

    debug_.des_thr = des_acc_norm; //debug_

    if (parameters.thrust_mapping_.accurate_thrust_model) {
        normalized_thrust = thrust_scale_compensation_ * AccurateThrustAccMapping(des_acc_norm, voltage, parameters);
    } else {
        normalized_thrust = des_acc_norm / thrust_to_acceleration_;
    }

    return normalized_thrust;
}

double Controller::AccurateThrustAccMapping(
    const double des_acc_z,
    double voltage,
    const Parameters &parameters) const {
    if (voltage < parameters.low_voltage_) {
        voltage = parameters.low_voltage_;
        ROSFMT_ERROR("Battery voltage below model minimum: {:.2f} V", parameters.low_voltage_);
    }
    if (voltage > 1.5 * parameters.low_voltage_) {
        voltage = 1.5 * parameters.low_voltage_;
    }

    // F=k1*Voltage^k2*(k3*u^2+(1-k3)*u)
    double a = parameters.thrust_mapping_.k3;
    double b = 1 - parameters.thrust_mapping_.k3;
    double c = -(parameters.mass_ * des_acc_z) / (parameters.thrust_mapping_.k1 * pow(voltage, parameters.thrust_mapping_.k2));
    double b2_4ac = pow(b, 2) - 4 * a * c;
    if (b2_4ac <= 0)
        b2_4ac = 0;
    double thrust = (-b + sqrt(b2_4ac)) / (2 * a);
    // if (thrust <= 0) thrust = 0; // This should be avoided before calling this function
    return thrust;
}

bool Controller::AlmostZero(const double value) const {
    return fabs(value) < kAlmostZeroValueThreshold;
}

bool Controller::AlmostZeroThrust(const double thrust_value) const {
    return fabs(thrust_value) < kAlmostZeroThrustThreshold;
}

bool Controller::EstimateThrustModel(
    const Eigen::Vector3d &est_a,
    const double voltage,
    const Eigen::Vector3d &est_v,
    const Parameters &parameters) {

    ros::Time t_now = ros::Time::now();
    while (timed_thrust_.size() >= 1) {
        // Choose data before 35~45ms ago
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        double time_passed = (t_now - t_t.first).toSec();
        if (time_passed > 0.045) { // 45ms
            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            continue;
        }
        if (time_passed < 0.035) { // 35ms
            // printf("skip, time_passed=%f\n", time_passed);
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thr = t_t.second;
        timed_thrust_.pop();
        if (parameters.thrust_mapping_.accurate_thrust_model) {
            /**************************************************************************/
            /* Model: thr = thrust_scale_compensation_ * AccurateThrustAccMapping(est_a(2)) */
            /**************************************************************************/
            double thr_fb = AccurateThrustAccMapping(est_a(2), voltage, parameters);
            double gamma = 1 / (kRho2 + thr_fb * p_ * thr_fb);
            double gain = gamma * p_ * thr_fb;
            thrust_scale_compensation_ = thrust_scale_compensation_ + gain * (thr - thr_fb * thrust_scale_compensation_);
            p_ = (1 - gain * thr_fb) * p_ / kRho2;
            // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thrust_scale_compensation_, gamma, K, p_);
            // fflush(stdout);

            if (thrust_scale_compensation_ > 1.15 || thrust_scale_compensation_ < 0.85) {
                ROSFMT_ERROR("Thrust scale out of range: {:.3f}; recalibrate model",
                             thrust_scale_compensation_);
                thrust_scale_compensation_ = thrust_scale_compensation_ > 1.15 ? 1.15 : thrust_scale_compensation_;
                thrust_scale_compensation_ = thrust_scale_compensation_ < 0.85 ? 0.85 : thrust_scale_compensation_;
            }

            debug_.thr_scale_compensate = thrust_scale_compensation_; //debug_
            debug_.voltage = voltage;
            if (parameters.thrust_mapping_.print_val) {
                ROSFMT_WARN("Thrust scale: {:.3f}", thrust_scale_compensation_);
            }
        } else {
            /***********************************/
            /* Model: est_a(2) = thrust_to_acceleration_ * thr */
            /***********************************/
            if (!parameters.thrust_mapping_.noisy_imu) {
                double gamma = 1 / (kRho2 + thr * p_ * thr);
                double gain = gamma * p_ * thr;
                thrust_to_acceleration_ = thrust_to_acceleration_ + gain * (est_a(2) - thr * thrust_to_acceleration_);
                p_ = (1 - gain * thr) * p_ / kRho2;
                //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thrust_to_acceleration_, gamma, K, p_);
                //fflush(stdout);
            } else { // Strongly not recommended to use!!!
                double gain = 10 / parameters.ctrl_freq_max_; // thrust_to_acceleration_ changes 10 every second when est_v(2) - des_v(2) = 1 m/s
                thrust_to_acceleration_ = thrust_to_acceleration_ + gain * (est_v(2) - debug_.des_v_z);
                // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thrust_to_acceleration_, K, est_v(2), debug_.des_v_z);
                // fflush(stdout);
            }
            const double hover_percentage = parameters.gra_ / thrust_to_acceleration_;
            if (hover_percentage > 0.8 || hover_percentage < 0.1) {
                ROSFMT_ERROR("Hover throttle out of range: {:.3f}; check IMU vibration", hover_percentage);
                thrust_to_acceleration_ = hover_percentage > 0.8 ? parameters.gra_ / 0.8 : thrust_to_acceleration_;
                thrust_to_acceleration_ = hover_percentage < 0.1 ? parameters.gra_ / 0.1 : thrust_to_acceleration_;
            }
            debug_.hover_percentage = hover_percentage; // debug_
            if (parameters.thrust_mapping_.print_val) {
                ROSFMT_WARN("Hover throttle: {:.3f}", debug_.hover_percentage);
            }
        }

        return true;
    }

    return false;
}

void Controller::ResetThrustMapping(void) {
    thrust_to_acceleration_ = parameters_.gra_ / parameters_.thrust_mapping_.hover_percentage;
    thrust_scale_compensation_ = 1.0;
    p_ = 1e6;
}

}  // namespace px4ctrl
