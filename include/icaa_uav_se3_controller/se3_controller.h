#ifndef __SE3_CONTROLLER_H
#define __SE3_CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <icaa_uav_se3_controller/Px4ctrlDebug.h>
#include <nav_msgs/Path.h>
#include <queue>

#include "input.h"
#include <Eigen/Dense>

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t(){};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};

struct Controller_Output_t
{
	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;
};

class SE3Controller
{
public:
	SE3Controller(Parameter_t &);
	
	icaa_uav_se3_controller::Px4ctrlDebug calculateControl(
		const Desired_State_t &des,
		const Odom_Data_t &odom,
		const Imu_Data_t &imu, 
		Controller_Output_t &u);
		
	bool estimateThrustModel(
		const Eigen::Vector3d &est_a,
		const Parameter_t &param);
		
	void resetThrustMapping(void);
	void resetIntegrals(void);
	void setIntegralLimits(double xy_limit, double z_limit);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	Parameter_t param_;
	icaa_uav_se3_controller::Px4ctrlDebug debug_msg_;
	std::queue<std::pair<ros::Time, double>> timed_thrust_;

	// Thrust-accel mapping params
	const double rho2_ = 0.998; 
	double thr2acc_;
	double P_;

	// Integral terms
	Eigen::Vector2d Ib_b_;  // body frame integral
	Eigen::Vector2d Iw_w_;  // world frame integral
	
	// Mass difference estimation
	double uav_mass_difference_;

	double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_f);
	double fromQuaternion2yaw(Eigen::Quaterniond q);
	Eigen::Matrix3d quat2RotMatrix(const Eigen::Quaterniond &q);
	Eigen::Vector3d computeRobustBodyXAxis(
		const Eigen::Vector3d &x_B_des, 
		const Eigen::Vector3d &x_C, 
		const Eigen::Vector3d &y_C,
		const Eigen::Quaterniond &q_mb);
};

#endif