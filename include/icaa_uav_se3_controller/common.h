#ifndef ICAA_UAV_SE3_CONTROLLER_COMMON_H
#define ICAA_UAV_SE3_CONTROLLER_COMMON_H

#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>

namespace icaa_common
{

// Rotation matrix utilities
Eigen::Matrix3d quat2RotMatrix(const Eigen::Quaterniond &q);
Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d &R);

// Orientation error calculation
Eigen::Vector3d orientationError(const Eigen::Matrix3d& R, const Eigen::Matrix3d& Rd);

// SO(3) transformation
Eigen::Matrix3d so3transform(const Eigen::Vector3d& body_z, const Eigen::Vector3d& heading, const bool& preserve_heading);

// Sanitize desired force
std::pair<bool, Eigen::Vector3d> sanitizeDesiredForce(
    const Eigen::Vector3d& f_des, 
    const double& tilt_safety_limit, 
    const double& tilt_saturation,
    const std::string& node_name);

// Throttle model utilities
namespace throttle_model
{
    double forceToThrottle(double force, double hover_percentage, double mass, double g);
    double throttleToForce(double throttle, double hover_percentage, double mass, double g);
}

}  // namespace icaa_common

#endif  // ICAA_UAV_SE3_CONTROLLER_COMMON_H