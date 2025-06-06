#ifndef __SE3CTRLFSM_H
#define __SE3CTRLFSM_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>

#include "input.h"
#include "se3_controller.h"

struct AutoTakeoffLand_t
{
	bool landed{true};
	ros::Time toggle_takeoff_land_time;
	std::pair<bool, ros::Time> delay_trigger{std::pair<bool, ros::Time>(false, ros::Time(0))};
	Eigen::Vector4d start_pose;
	
	static constexpr double MOTORS_SPEEDUP_TIME = 3.0; // motors idle running for 3 seconds before takeoff
	static constexpr double DELAY_TRIGGER_TIME = 2.0;  // Time to be delayed when reach at target height
};

class SE3CtrlFSM
{
public:
	Parameter_t &param;

	RC_Data_t rc_data;
	State_Data_t state_data;
	ExtendedState_Data_t extended_state_data;
	Odom_Data_t odom_data;
	Imu_Data_t imu_data;
	Battery_Data_t bat_data;
	Takeoff_Land_Data_t takeoff_land_data;

	// Path tracking related
	nav_msgs::Path trajectory_path;
	int current_waypoint_index;
	ros::Time path_recv_time;
	bool path_initialized;

	SE3Controller &controller;

	ros::Publisher traj_start_trigger_pub;
	ros::Publisher ctrl_FCU_pub;
	ros::Publisher debug_pub;
	ros::ServiceClient set_FCU_mode_srv;
	ros::ServiceClient arming_client_srv;
	ros::ServiceClient reboot_FCU_srv;

	icaa_uav_se3_controller::Px4ctrlDebug debug_msg;

	Eigen::Vector4d hover_pose;
	ros::Time last_set_hover_pose_time;

	enum State_t
	{
		MANUAL_CTRL = 1,
		AUTO_HOVER,
		CMD_CTRL,
		AUTO_TAKEOFF,
		AUTO_LAND
	};

	SE3CtrlFSM(Parameter_t &, SE3Controller &);
	void process();
	bool rc_is_received(const ros::Time &now_time);
	bool cmd_is_received(const ros::Time &now_time);
	bool odom_is_received(const ros::Time &now_time);
	bool imu_is_received(const ros::Time &now_time);
	bool bat_is_received(const ros::Time &now_time);
	bool recv_new_odom();
	State_t get_state() { return state; }
	bool get_landed() { return takeoff_land.landed; }

	// Path tracking methods
	void updateTrajectoryPath(const nav_msgs::PathConstPtr &msg);
	bool getDesiredStateFromPath(const ros::Time &now_time, Desired_State_t &des_state);
	double getPathProgress() const;

private:
	State_t state;
	AutoTakeoffLand_t takeoff_land;

	// Control related
	Desired_State_t get_hover_des();
	Desired_State_t get_cmd_des();

	// Auto takeoff/land
	void motors_idling(const Imu_Data_t &imu, Controller_Output_t &u);
	void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom);
	void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
	Desired_State_t get_rotor_speed_up_des(const ros::Time now);
	Desired_State_t get_takeoff_land_des(const double speed);

	// Tools
	void set_hov_with_odom();
	void set_hov_with_rc();

	bool toggle_offboard_mode(bool on_off);
	bool toggle_arm_disarm(bool arm);
	void reboot_FCU();

	void publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_trigger(const nav_msgs::Odometry &odom_msg);

	// Path utilities
	int findClosestWaypoint(const Eigen::Vector3d &current_pos);
	Eigen::Vector3d interpolatePosition(const geometry_msgs::Point &p1, 
	                                   const geometry_msgs::Point &p2, 
	                                   double t);
};

#endif