#ifndef ICAA_UAV_SE3_CONTROLLER_COMMON_H
#define ICAA_UAV_SE3_CONTROLLER_COMMON_H

#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>

namespace icaa_se3_controller
{

namespace common
{

// Orientation error calculation
Eigen::Vector3d orientationError(const Eigen::Matrix3d& R, const Eigen::Matrix3d& Rd);

// Sanitize desired force to ensure safe tilt angles
std::optional<Eigen::Vector3d> sanitizeDesiredForce(const Eigen::Vector3d& desired_force, 
                                                    const double& tilt_over_limit, 
                                                    const double& tilt_saturation,
                                                    const std::string& node_name);

// SO(3) transformation for desired orientation
Eigen::Matrix3d so3transform(const Eigen::Vector3d& body_z, 
                            const Eigen::Vector3d& heading, 
                            const bool& preserve_heading);

// Path following utilities
struct PathPoint {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    double yaw;
    double yaw_rate;
};

// Get target point from path based on current position and lookahead distance
PathPoint getTargetFromPath(const nav_msgs::Path& path, 
                           const Eigen::Vector3d& current_position,
                           const double& lookahead_distance,
                           const double& desired_speed);

// Convert path to desired state
void pathToDesiredState(const nav_msgs::Path& path,
                       const Eigen::Vector3d& current_position,
                       const double& lookahead_distance,
                       const double& desired_speed,
                       Eigen::Vector3d& des_pos,
                       Eigen::Vector3d& des_vel,
                       Eigen::Vector3d& des_acc,
                       double& des_yaw,
                       double& des_yaw_rate);

} // namespace common

} // namespace icaa_se3_controller

#endif // ICAA_UAV_SE3_CONTROLLER_COMMON_H