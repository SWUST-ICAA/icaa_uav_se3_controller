#include "se3_controller.h"
#include "common.h"
#include "geometry_utils.h"

using namespace std;

double SE3Controller::fromQuaternion2yaw(Eigen::Quaterniond q)
{
    double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    return yaw;
}

Eigen::Matrix3d SE3Controller::quat2RotMatrix(const Eigen::Quaterniond &q)
{
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    return R;
}

SE3Controller::SE3Controller(Parameter_t &param) : param_(param)
{
    resetThrustMapping();
    resetIntegrals();
    uav_mass_difference_ = 0.0;
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
icaa_uav_se3_controller::Px4ctrlDebug
SE3Controller::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
    // Get SE3 gains
    Eigen::Vector3d Kp, Kv, Ka;
    Kp << param_.se3_gains.Kp_xy, param_.se3_gains.Kp_xy, param_.se3_gains.Kp_z;
    Kv << param_.se3_gains.Kv_xy, param_.se3_gains.Kv_xy, param_.se3_gains.Kv_z;
    Ka << param_.se3_gains.Ka_xy, param_.se3_gains.Ka_xy, param_.se3_gains.Ka_z;

    // Position and velocity errors
    Eigen::Vector3d ep = des.p - odom.p;
    Eigen::Vector3d ev = des.v - odom.v;

    // Get current rotation matrix
    Eigen::Matrix3d R = quat2RotMatrix(odom.q);
    
    // World frame integral (position error)
    Iw_w_ += param_.se3_gains.Ki_xy_world * ep.head<2>() * 0.01; // dt = 0.01
    
    // Saturate world integrals
    for (int i = 0; i < 2; i++) {
        if (Iw_w_(i) > param_.se3_gains.Ki_xy_world_lim) {
            Iw_w_(i) = param_.se3_gains.Ki_xy_world_lim;
        } else if (Iw_w_(i) < -param_.se3_gains.Ki_xy_world_lim) {
            Iw_w_(i) = -param_.se3_gains.Ki_xy_world_lim;
        }
    }

    // Mass estimation
    uav_mass_difference_ += param_.se3_gains.Km * ep(2) * 0.01;
    if (uav_mass_difference_ > param_.se3_gains.Km_lim) {
        uav_mass_difference_ = param_.se3_gains.Km_lim;
    } else if (uav_mass_difference_ < -param_.se3_gains.Km_lim) {
        uav_mass_difference_ = -param_.se3_gains.Km_lim;
    }

    double total_mass = param_.mass + uav_mass_difference_;

    // Compute desired force
    Eigen::Vector3d F_des = Kp.asDiagonal() * ep + 
                            Kv.asDiagonal() * ev + 
                            total_mass * Ka.asDiagonal() * des.a +
                            total_mass * Eigen::Vector3d(0, 0, param_.gra);
    
    // Add world integral feedback
    F_des(0) += Iw_w_(0);
    F_des(1) += Iw_w_(1);

    // Body frame integral (using fcu frame)
    // Transform position error to body frame
    Eigen::Vector3d ep_body = R.transpose() * ep;
    Ib_b_ += param_.se3_gains.Ki_xy_body * ep_body.head<2>() * 0.01;
    
    // Saturate body integrals  
    for (int i = 0; i < 2; i++) {
        if (Ib_b_(i) > param_.se3_gains.Ki_xy_body_lim) {
            Ib_b_(i) = param_.se3_gains.Ki_xy_body_lim;
        } else if (Ib_b_(i) < -param_.se3_gains.Ki_xy_body_lim) {
            Ib_b_(i) = -param_.se3_gains.Ki_xy_body_lim;
        }
    }
    
    // Transform body integral back to world frame and add to force
    Eigen::Vector3d Ib_w = R * Eigen::Vector3d(Ib_b_(0), Ib_b_(1), 0);
    F_des += Ib_w;

    // Limit downward acceleration to prevent flip
    if (F_des(2) < 0.5 * total_mass * param_.gra) {
        ROS_WARN_THROTTLE(1.0, "[SE3Controller]: limiting downward acceleration");
        F_des(2) = 0.5 * total_mass * param_.gra;
    }

    // Normalize and check tilt angle
    auto f_result = icaa_common::sanitizeDesiredForce(
        F_des.normalized(), 
        param_.constraints.tilt_angle_failsafe,
        param_.max_angle > 0 ? param_.max_angle : M_PI,
        "SE3Controller"
    );

    if (!f_result.first) {
        // Force sanitization failed, output zero control
        u.thrust = 0;
        u.q = imu.q;
        u.bodyrates = Eigen::Vector3d::Zero();
        return debug_msg_;
    }

    Eigen::Vector3d f_normalized = f_result.second;

    // Compute desired orientation
    double yaw = des.yaw;
    Eigen::Vector3d b1d(cos(yaw), sin(yaw), 0);
    
    Eigen::Matrix3d Rd = icaa_common::so3transform(f_normalized, b1d, false);

    // Compute attitude error
    Eigen::Vector3d e_R = icaa_common::orientationError(R, Rd);

    // Attitude control gains
    Eigen::Vector3d Kq;
    Kq << param_.se3_gains.Kq_roll_pitch, param_.se3_gains.Kq_roll_pitch, param_.se3_gains.Kq_yaw;
    
    // Compute desired angular velocity
    Eigen::Vector3d w_des = Kq.asDiagonal() * e_R;

    // Add yaw rate feedforward if available
    if (des.yaw_rate != 0) {
        w_des(2) += des.yaw_rate;
    }

    // Compute thrust
    double thrust_force = F_des.norm();
    u.thrust = computeDesiredCollectiveThrustSignal(Eigen::Vector3d(0, 0, thrust_force));

    // Saturate thrust
    if (u.thrust > param_.constraints.throttle_saturation) {
        u.thrust = param_.constraints.throttle_saturation;
    } else if (u.thrust < 0.0) {
        u.thrust = 0.0;
    }

    // Set desired orientation
    u.q = Eigen::Quaterniond(Rd);

    // Set body rates
    u.bodyrates = w_des;

    // Fill debug message
    debug_msg_.header.stamp = ros::Time::now();
    
    debug_msg_.des_v_x = des.v(0);
    debug_msg_.des_v_y = des.v(1);
    debug_msg_.des_v_z = des.v(2);
    
    debug_msg_.des_a_x = des.a(0);
    debug_msg_.des_a_y = des.a(1);
    debug_msg_.des_a_z = des.a(2);
    
    debug_msg_.fb_a_x = F_des(0) / total_mass;
    debug_msg_.fb_a_y = F_des(1) / total_mass;
    debug_msg_.fb_a_z = F_des(2) / total_mass - param_.gra;
    
    debug_msg_.des_q_x = u.q.x();
    debug_msg_.des_q_y = u.q.y();
    debug_msg_.des_q_z = u.q.z();
    debug_msg_.des_q_w = u.q.w();
    
    debug_msg_.des_thr = u.thrust;
    debug_msg_.thr_scale_compensate = thr2acc_;
    
    // Store thrust for thrust model estimation
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrust_.size() > 100) {
        timed_thrust_.pop();
    }
    
    return debug_msg_;
}

/*
  compute throttle percentage 
*/
double 
SE3Controller::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_f)
{
    double throttle_percentage = 0.0;
    
    if (param_.thr_map.accurate_thrust_model) {
        // TODO: implement accurate thrust model
        throttle_percentage = des_f(2) / thr2acc_;
    } else {
        // Simple thrust model
        throttle_percentage = icaa_common::throttle_model::forceToThrottle(
            des_f(2), 
            param_.thr_map.hover_percentage,
            param_.mass + uav_mass_difference_,
            param_.gra
        );
    }

    return throttle_percentage;
}

bool 
SE3Controller::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
    ros::Time t_now = ros::Time::now();
    while (timed_thrust_.size() >= 1)
    {
        // Choose data before 35~45ms ago
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        double time_passed = (t_now - t_t.first).toSec();
        if (time_passed > 0.045) // 45ms
        {
            timed_thrust_.pop();
            continue;
        }
        if (time_passed < 0.035) // 35ms
        {
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thr = t_t.second;
        timed_thrust_.pop();
        
        /***********************************/
        /* Model: est_a(2) = thr2acc_ * thr */
        /***********************************/
        double gamma = 1 / (rho2_ + thr * P_ * thr);
        double K = gamma * P_ * thr;
        thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
        P_ = (1 - K * thr) * P_ / rho2_;
        
        if (param.thr_map.print_val) {
            ROS_INFO_THROTTLE(1.0, "thr2acc = %.3f", thr2acc_);
        }
        
        debug_msg_.thr_scale_compensate = thr2acc_;
        return true;
    }
    return false;
}

void 
SE3Controller::resetThrustMapping(void)
{
    thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
    P_ = 1e6;
}

void 
SE3Controller::resetIntegrals(void)
{
    Ib_b_ = Eigen::Vector2d::Zero();
    Iw_w_ = Eigen::Vector2d::Zero();
}

void 
SE3Controller::setIntegralLimits(double xy_limit, double z_limit)
{
    // Not used in current implementation
}

Eigen::Vector3d 
SE3Controller::computeRobustBodyXAxis(
    const Eigen::Vector3d &x_B_des, 
    const Eigen::Vector3d &x_C, 
    const Eigen::Vector3d &y_C,
    const Eigen::Quaterniond &q_mb)
{
    // Not used in current implementation, kept for compatibility
    return x_B_des;
}