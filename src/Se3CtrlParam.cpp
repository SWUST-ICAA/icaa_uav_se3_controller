#include "Se3CtrlParam.h"

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
	// Basic parameters
	read_essential_param(nh, "mass", mass);
	read_essential_param(nh, "gra", gra);
	read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
	read_essential_param(nh, "use_bodyrate_ctrl", use_bodyrate_ctrl);
	read_essential_param(nh, "max_manual_vel", max_manual_vel);
	read_essential_param(nh, "max_angle", max_angle);
	read_essential_param(nh, "low_voltage", low_voltage);

	// RC reverse settings
	read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
	read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
	read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
	read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

	// Auto takeoff/land settings
	read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
	read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
	read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
	read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
	read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);

	// Thrust model parameters
	read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
	read_essential_param(nh, "thrust_model/K1", thr_map.K1);
	read_essential_param(nh, "thrust_model/K2", thr_map.K2);
	read_essential_param(nh, "thrust_model/K3", thr_map.K3);
	read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
	read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);

	// SE3 Controller gains
	read_essential_param(nh, "se3_controller/position_gains/kp_xy", se3_gains.Kp_xy);
	read_essential_param(nh, "se3_controller/position_gains/kp_z", se3_gains.Kp_z);
	
	read_essential_param(nh, "se3_controller/velocity_gains/kv_xy", se3_gains.Kv_xy);
	read_essential_param(nh, "se3_controller/velocity_gains/kv_z", se3_gains.Kv_z);
	
	read_essential_param(nh, "se3_controller/acceleration_gains/ka_xy", se3_gains.Ka_xy);
	read_essential_param(nh, "se3_controller/acceleration_gains/ka_z", se3_gains.Ka_z);
	
	read_essential_param(nh, "se3_controller/attitude_gains/kq_roll_pitch", se3_gains.Kq_roll_pitch);
	read_essential_param(nh, "se3_controller/attitude_gains/kq_yaw", se3_gains.Kq_yaw);
	
	read_essential_param(nh, "se3_controller/attitude_rate_gains/kw_roll_pitch", se3_gains.Kw_roll_pitch);
	read_essential_param(nh, "se3_controller/attitude_rate_gains/kw_yaw", se3_gains.Kw_yaw);
	
	read_essential_param(nh, "se3_controller/integral_gains/ki_xy_world", se3_gains.Ki_xy_world);
	read_essential_param(nh, "se3_controller/integral_gains/ki_xy_body", se3_gains.Ki_xy_body);
	read_essential_param(nh, "se3_controller/integral_gains/ki_xy_world_lim", se3_gains.Ki_xy_world_lim);
	read_essential_param(nh, "se3_controller/integral_gains/ki_xy_body_lim", se3_gains.Ki_xy_body_lim);
	
	read_essential_param(nh, "se3_controller/mass_estimator/km", se3_gains.Km);
	read_essential_param(nh, "se3_controller/mass_estimator/km_lim", se3_gains.Km_lim);
	
	read_essential_param(nh, "se3_controller/constraints/tilt_angle_failsafe", constraints.tilt_angle_failsafe);
	read_essential_param(nh, "se3_controller/constraints/throttle_saturation", constraints.throttle_saturation);

	// Message timeouts
	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
	read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
	read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

	// Convert max_angle from degrees to radians
	max_angle /= (180.0 / M_PI);
	constraints.tilt_angle_failsafe /= (180.0 / M_PI);

	// Validation checks
	if (takeoff_land.enable_auto_arm && !takeoff_land.enable)
	{
		takeoff_land.enable_auto_arm = false;
		ROS_ERROR("\"enable_auto_arm\" is only allowed with \"auto_takeoff_land\" enabled.");
	}
	if (takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable))
	{
		takeoff_land.no_RC = false;
		ROS_ERROR("\"no_RC\" is only allowed with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
	}

	if (thr_map.print_val)
	{
		ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
	}
}