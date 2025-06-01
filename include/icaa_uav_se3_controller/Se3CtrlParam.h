#ifndef __SE3CTRLPARAM_H
#define __SE3CTRLPARAM_H

#include <ros/ros.h>

class Parameter_t
{
public:
	struct SE3Gains
	{
		// Position gains
		double Kp_xy, Kp_z;
		// Velocity gains
		double Kv_xy, Kv_z;
		// Acceleration gains
		double Ka_xy, Ka_z;
		// Attitude gains
		double Kq_roll_pitch, Kq_yaw;
		// Attitude rate gains
		double Kw_roll_pitch, Kw_yaw;
		// Integral gains
		double Ki_xy_world, Ki_xy_body;
		double Ki_xy_world_lim, Ki_xy_body_lim;
		// Mass estimator
		double Km, Km_lim;
	};

	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
		double imu;
		double bat;
	};

	struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
	};

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	};

	struct AutoTakeoffLand
	{
		bool enable;
		bool enable_auto_arm;
		bool no_RC;
		double height;
		double speed;
	};

	struct Constraints
	{
		double tilt_angle_failsafe;
		double throttle_saturation;
	};

	SE3Gains se3_gains;
	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	ThrustMapping thr_map;
	AutoTakeoffLand takeoff_land;
	Constraints constraints;

	double mass;
	double gra;
	double max_angle;
	double ctrl_freq_max;
	double max_manual_vel;
	double low_voltage;

	bool use_bodyrate_ctrl;

	Parameter_t();
	void config_from_ros_handle(const ros::NodeHandle &nh);

private:
	template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};
};

#endif