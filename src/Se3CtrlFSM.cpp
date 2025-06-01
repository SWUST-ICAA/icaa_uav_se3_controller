#include "Se3CtrlFSM.h"
#include "converters.h"
#include "geometry_utils.h"

using namespace std;
using namespace uav_utils;

SE3CtrlFSM::SE3CtrlFSM(Parameter_t &param_, SE3Controller &controller_) : param(param_), controller(controller_)
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
	current_waypoint_index = 0;
	path_initialized = false;
}

void SE3CtrlFSM::process()
{
	ros::Time now_time = ros::Time::now();
	Controller_Output_t u;
	Desired_State_t des(odom_data);
	bool rotor_low_speed_during_land = false;

	// STEP1: state machine runs
	switch (state)
	{
	case MANUAL_CTRL:
	{
		if (rc_data.enter_hover_mode) // Try to jump to AUTO_HOVER
		{
			if (!odom_is_received(now_time))
			{
				ROS_ERROR("[SE3CtrlFSM] Reject AUTO_HOVER(L2). No odom!");
				break;
			}
			if (path_initialized && cmd_is_received(now_time))
			{
				ROS_ERROR("[SE3CtrlFSM] Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER!");
				break;
			}
			if (odom_data.v.norm() > 3.0)
			{
				ROS_ERROR("[SE3CtrlFSM] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems wrong!", odom_data.v.norm());
				break;
			}

			state = AUTO_HOVER;
			controller.resetThrustMapping();
			controller.resetIntegrals();
			set_hov_with_odom();
			toggle_offboard_mode(true);

			ROS_INFO("\033[32m[SE3CtrlFSM] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
		}
		else if (param.takeoff_land.enable && takeoff_land_data.triggered && 
		         takeoff_land_data.takeoff_land_cmd == icaa_uav_se3_controller::TakeoffLand::TAKEOFF)
		{
			if (!odom_is_received(now_time))
			{
				ROS_ERROR("[SE3CtrlFSM] Reject AUTO_TAKEOFF. No odom!");
				break;
			}
			if (path_initialized && cmd_is_received(now_time))
			{
				ROS_ERROR("[SE3CtrlFSM] Reject AUTO_TAKEOFF. Stop sending commands now!");
				break;
			}
			if (odom_data.v.norm() > 0.1)
			{
				ROS_ERROR("[SE3CtrlFSM] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff not allowed!", odom_data.v.norm());
				break;
			}
			if (!get_landed())
			{
				ROS_ERROR("[SE3CtrlFSM] Reject AUTO_TAKEOFF. Drone is not landed!");
				break;
			}
			
			// For no_RC mode, skip RC checks
			if (!param.takeoff_land.no_RC && rc_is_received(now_time))
			{
				if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered())
				{
					ROS_ERROR("[SE3CtrlFSM] Reject AUTO_TAKEOFF. Check RC switches and sticks!");
					while (ros::ok())
					{
						ros::Duration(0.01).sleep();
						ros::spinOnce();
						if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered())
						{
							ROS_INFO("\033[32m[SE3CtrlFSM] OK, you can takeoff again.\033[32m");
							break;
						}
					}
					break;
				}
			}

			state = AUTO_TAKEOFF;
			controller.resetThrustMapping();
			controller.resetIntegrals();
			set_start_pose_for_takeoff_land(odom_data);
			toggle_offboard_mode(true);
			for (int i = 0; i < 10 && ros::ok(); ++i)
			{
				ros::Duration(0.01).sleep();
				ros::spinOnce();
			}
			if (param.takeoff_land.enable_auto_arm)
			{
				toggle_arm_disarm(true);
			}
			takeoff_land.toggle_takeoff_land_time = now_time;

			ROS_INFO("\033[32m[SE3CtrlFSM] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
		}

		if (rc_data.toggle_reboot)
		{
			if (state_data.current_state.armed)
			{
				ROS_ERROR("[SE3CtrlFSM] Reject reboot! Disarm the drone first!");
				break;
			}
			reboot_FCU();
		}

		break;
	}

	case AUTO_HOVER:
	{
		if ((!param.takeoff_land.no_RC && !rc_data.is_hover_mode) || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);
			ROS_WARN("[SE3CtrlFSM] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
		}
		else if ((param.takeoff_land.no_RC || rc_data.is_command_mode) && path_initialized && cmd_is_received(now_time))
		{
			if (state_data.current_state.mode == "OFFBOARD")
			{
				state = CMD_CTRL;
				// Get desired state from path
				if (!getDesiredStateFromPath(now_time, des))
				{
					des = get_hover_des();
				}
				ROS_INFO("\033[32m[SE3CtrlFSM] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
			}
		}
		else if (takeoff_land_data.triggered && 
		         takeoff_land_data.takeoff_land_cmd == icaa_uav_se3_controller::TakeoffLand::LAND)
		{
			state = AUTO_LAND;
			set_start_pose_for_takeoff_land(odom_data);
			ROS_INFO("\033[32m[SE3CtrlFSM] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
		}
		else
		{
			if (!param.takeoff_land.no_RC)
			{
				set_hov_with_rc();
			}
			des = get_hover_des();
			if ((rc_data.enter_command_mode) ||
				(takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second))
			{
				takeoff_land.delay_trigger.first = false;
				publish_trigger(odom_data.msg);
				ROS_INFO("\033[32m[SE3CtrlFSM] TRIGGER sent, allow user command.\033[32m");
			}
		}

		break;
	}

	case CMD_CTRL:
	{
		if ((!param.takeoff_land.no_RC && !rc_data.is_hover_mode) || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);
			ROS_WARN("[SE3CtrlFSM] CMD_CTRL(L3) --> MANUAL_CTRL(L1)!");
		}
		else if ((!param.takeoff_land.no_RC && !rc_data.is_command_mode) || !cmd_is_received(now_time))
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_INFO("[SE3CtrlFSM] CMD_CTRL(L3) --> AUTO_HOVER(L2)!");
		}
		else
		{
			// Get desired state from path
			if (!getDesiredStateFromPath(now_time, des))
			{
				// If no valid path, use last hover position
				des = get_hover_des();
			}
		}

		if (takeoff_land_data.triggered && 
		    takeoff_land_data.takeoff_land_cmd == icaa_uav_se3_controller::TakeoffLand::LAND)
		{
			ROS_ERROR("[SE3CtrlFSM] Reject AUTO_LAND in CMD_CTRL. Return to AUTO_HOVER first!");
		}

		break;
	}

	case AUTO_TAKEOFF:
	{
		if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME)
		{
			des = get_rotor_speed_up_des(now_time);
		}
		else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height))
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			ROS_INFO("\033[32m[SE3CtrlFSM] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");

			takeoff_land.delay_trigger.first = true;
			takeoff_land.delay_trigger.second = now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
		}
		else
		{
			des = get_takeoff_land_des(param.takeoff_land.speed);
		}

		break;
	}

	case AUTO_LAND:
	{
		if ((!param.takeoff_land.no_RC && !rc_data.is_hover_mode) || !odom_is_received(now_time))
		{
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);
			ROS_WARN("[SE3CtrlFSM] AUTO_LAND --> MANUAL_CTRL(L1)!");
		}
		else if (!param.takeoff_land.no_RC && !rc_data.is_command_mode)
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_INFO("[SE3CtrlFSM] AUTO_LAND --> AUTO_HOVER(L2)!");
		}
		else if (!get_landed())
		{
			des = get_takeoff_land_des(-param.takeoff_land.speed);
		}
		else
		{
			rotor_low_speed_during_land = true;

			static bool print_once_flag = true;
			if (print_once_flag)
			{
				ROS_INFO("\033[32m[SE3CtrlFSM] Wait for about 10s to let the drone disarm.\033[32m");
				print_once_flag = false;
			}

			if (extended_state_data.current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
			{
				static double last_trial_time = 0;
				if (now_time.toSec() - last_trial_time > 1.0)
				{
					if (toggle_arm_disarm(false))
					{
						print_once_flag = true;
						state = MANUAL_CTRL;
						toggle_offboard_mode(false);
						ROS_INFO("\033[32m[SE3CtrlFSM] AUTO_LAND --> MANUAL_CTRL(L1)\033[32m");
					}

					last_trial_time = now_time.toSec();
				}
			}
		}

		break;
	}

	default:
		break;
	}

	// STEP2: estimate thrust model
	if (state == AUTO_HOVER || state == CMD_CTRL)
	{
		controller.estimateThrustModel(imu_data.a, param);
	}

	// STEP3: solve and update new control commands
	if (rotor_low_speed_during_land)
	{
		motors_idling(imu_data, u);
	}
	else
	{
		debug_msg = controller.calculateControl(des, odom_data, imu_data, u);
		debug_msg.header.stamp = now_time;
		debug_pub.publish(debug_msg);
	}

	// STEP4: publish control commands to mavros
	if (param.use_bodyrate_ctrl)
	{
		publish_bodyrate_ctrl(u, now_time);
	}
	else
	{
		publish_attitude_ctrl(u, now_time);
	}

	// STEP5: Detect if the drone has landed
	land_detector(state, des, odom_data);

	// STEP6: Clear flags beyond their lifetime
	rc_data.enter_hover_mode = false;
	rc_data.enter_command_mode = false;
	rc_data.toggle_reboot = false;
	takeoff_land_data.triggered = false;
}

void SE3CtrlFSM::updateTrajectoryPath(const nav_msgs::PathConstPtr &msg)
{
	trajectory_path = *msg;
	path_recv_time = ros::Time::now();
	path_initialized = true;
	current_waypoint_index = 0;
	
	ROS_INFO("[SE3CtrlFSM] Received path with %zu waypoints", trajectory_path.poses.size());
}

bool SE3CtrlFSM::getDesiredStateFromPath(const ros::Time &now_time, Desired_State_t &des_state)
{
	if (!path_initialized || trajectory_path.poses.empty())
	{
		return false;
	}

	// Find closest waypoint to current position
	current_waypoint_index = findClosestWaypoint(odom_data.p);
	
	if (current_waypoint_index >= (int)trajectory_path.poses.size() - 1)
	{
		// Reached end of path, hover at last waypoint
		const auto& last_pose = trajectory_path.poses.back();
		des_state.p << last_pose.pose.position.x, last_pose.pose.position.y, last_pose.pose.position.z;
		des_state.v = Eigen::Vector3d::Zero();
		des_state.a = Eigen::Vector3d::Zero();
		des_state.yaw = uav_utils::get_yaw_from_quaternion(
			Eigen::Quaterniond(last_pose.pose.orientation.w, last_pose.pose.orientation.x,
			                   last_pose.pose.orientation.y, last_pose.pose.orientation.z));
		des_state.yaw_rate = 0;
		return true;
	}

	// Get current and next waypoint
	const auto& current_pose = trajectory_path.poses[current_waypoint_index];
	const auto& next_pose = trajectory_path.poses[current_waypoint_index + 1];

	// Simple position interpolation (can be improved with velocity/acceleration planning)
	des_state.p << next_pose.pose.position.x, next_pose.pose.position.y, next_pose.pose.position.z;
	
	// Compute velocity towards next waypoint
	Eigen::Vector3d direction = des_state.p - odom_data.p;
	double distance = direction.norm();
	if (distance > 0.1)
	{
		direction.normalize();
		des_state.v = direction * std::min(distance * 2.0, 1.0); // Simple P controller for velocity
	}
	else
	{
		des_state.v = Eigen::Vector3d::Zero();
	}

	des_state.a = Eigen::Vector3d::Zero();
	des_state.yaw = uav_utils::get_yaw_from_quaternion(
		Eigen::Quaterniond(next_pose.pose.orientation.w, next_pose.pose.orientation.x,
		                   next_pose.pose.orientation.y, next_pose.pose.orientation.z));
	des_state.yaw_rate = 0;

	return true;
}

int SE3CtrlFSM::findClosestWaypoint(const Eigen::Vector3d &current_pos)
{
	if (trajectory_path.poses.empty())
		return 0;

	double min_dist = std::numeric_limits<double>::max();
	int closest_idx = current_waypoint_index;

	// Search around current index for efficiency
	int search_start = std::max(0, current_waypoint_index - 5);
	int search_end = std::min((int)trajectory_path.poses.size() - 1, current_waypoint_index + 10);

	for (int i = search_start; i <= search_end; i++)
	{
		Eigen::Vector3d waypoint_pos(trajectory_path.poses[i].pose.position.x,
		                            trajectory_path.poses[i].pose.position.y,
		                            trajectory_path.poses[i].pose.position.z);
		double dist = (waypoint_pos - current_pos).norm();
		if (dist < min_dist)
		{
			min_dist = dist;
			closest_idx = i;
		}
	}

	// If within threshold of waypoint, move to next
	if (min_dist < 0.3 && closest_idx < (int)trajectory_path.poses.size() - 1)
	{
		closest_idx++;
	}

	return closest_idx;
}

double SE3CtrlFSM::getPathProgress() const
{
	if (!path_initialized || trajectory_path.poses.empty())
		return 0.0;
		
	return (double)current_waypoint_index / (double)(trajectory_path.poses.size() - 1);
}

void SE3CtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u)
{
	u.q = imu.q;
	u.bodyrates = Eigen::Vector3d::Zero();
	u.thrust = 0.04;
}

void SE3CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
	static State_t last_state = State_t::MANUAL_CTRL;
	if (last_state == State_t::MANUAL_CTRL && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF))
	{
		takeoff_land.landed = false;
	}
	last_state = state;

	if (state == State_t::MANUAL_CTRL && !state_data.current_state.armed)
	{
		takeoff_land.landed = true;
		return;
	}

	// land_detector parameters
	constexpr double POSITION_DEVIATION_C = -0.5;
	constexpr double VELOCITY_THR_C = 0.1;
	constexpr double TIME_KEEP_C = 3.0;

	static ros::Time time_C12_reached;
	static bool is_last_C12_satisfy;
	if (takeoff_land.landed)
	{
		time_C12_reached = ros::Time::now();
		is_last_C12_satisfy = false;
	}
	else
	{
		bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
		if (C12_satisfy && !is_last_C12_satisfy)
		{
			time_C12_reached = ros::Time::now();
		}
		else if (C12_satisfy && is_last_C12_satisfy)
		{
			if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C)
			{
				takeoff_land.landed = true;
			}
		}

		is_last_C12_satisfy = C12_satisfy;
	}
}

Desired_State_t SE3CtrlFSM::get_hover_des()
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t SE3CtrlFSM::get_rotor_speed_up_des(const ros::Time now)
{
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec();
	double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0;
	if (des_a_z > 0.1)
	{
		ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
		des_a_z = 0.0;
	}

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d(0, 0, des_a_z);
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t SE3CtrlFSM::get_takeoff_land_des(const double speed)
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec() - 
	                 (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0);

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
	des.v = Eigen::Vector3d(0, 0, speed);
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

void SE3CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

	last_set_hover_pose_time = ros::Time::now();
}

void SE3CtrlFSM::set_hov_with_rc()
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - last_set_hover_pose_time).toSec();
	last_set_hover_pose_time = now;

	hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1 : -1);
	hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);
	hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? 1 : -1);
	hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);

	if (hover_pose(2) < -0.3)
		hover_pose(2) = -0.3;
}

void SE3CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
	takeoff_land.start_pose.head<3>() = odom_data.p;
	takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom_data.q);

	takeoff_land.toggle_takeoff_land_time = ros::Time::now();
}

bool SE3CtrlFSM::rc_is_received(const ros::Time &now_time)
{
	return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool SE3CtrlFSM::cmd_is_received(const ros::Time &now_time)
{
	return path_initialized && 
	       (now_time - path_recv_time).toSec() < param.msg_timeout.cmd;
}

bool SE3CtrlFSM::odom_is_received(const ros::Time &now_time)
{
	return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool SE3CtrlFSM::imu_is_received(const ros::Time &now_time)
{
	return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

bool SE3CtrlFSM::bat_is_received(const ros::Time &now_time)
{
	return (now_time - bat_data.rcv_stamp).toSec() < param.msg_timeout.bat;
}

bool SE3CtrlFSM::recv_new_odom()
{
	if (odom_data.recv_new_msg)
	{
		odom_data.recv_new_msg = false;
		return true;
	}

	return false;
}

void SE3CtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

	msg.body_rate.x = u.bodyrates.x();
	msg.body_rate.y = u.bodyrates.y();
	msg.body_rate.z = u.bodyrates.z();

	msg.thrust = u.thrust;

	ctrl_FCU_pub.publish(msg);
}

void SE3CtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	msg.orientation.x = u.q.x();
	msg.orientation.y = u.q.y();
	msg.orientation.z = u.q.z();
	msg.orientation.w = u.q.w();

	msg.thrust = u.thrust;

	ctrl_FCU_pub.publish(msg);
}

void SE3CtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub.publish(msg);
}

bool SE3CtrlFSM::toggle_offboard_mode(bool on_off)
{
	mavros_msgs::SetMode offb_set_mode;

	if (on_off)
	{
		state_data.state_before_offboard = state_data.current_state;
		if (state_data.state_before_offboard.mode == "OFFBOARD")
			state_data.state_before_offboard.mode = "MANUAL";

		offb_set_mode.request.custom_mode = "OFFBOARD";
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Enter OFFBOARD rejected by PX4!");
			return false;
		}
	}
	else
	{
		offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Exit OFFBOARD rejected by PX4!");
			return false;
		}
	}

	return true;
}

bool SE3CtrlFSM::toggle_arm_disarm(bool arm)
{
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;
	if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success))
	{
		if (arm)
			ROS_ERROR("ARM rejected by PX4!");
		else
			ROS_ERROR("DISARM rejected by PX4!");

		return false;
	}

	return true;
}

void SE3CtrlFSM::reboot_FCU()
{
	mavros_msgs::CommandLong reboot_srv;
	reboot_srv.request.broadcast = false;
	reboot_srv.request.command = 246;
	reboot_srv.request.param1 = 1;
	reboot_srv.request.param2 = 0;
	reboot_srv.request.confirmation = true;

	reboot_FCU_srv.call(reboot_srv);

	ROS_INFO("Reboot FCU");
}