#include <common.h>

namespace icaa_common
{

/* quat2RotMatrix() //{ */
Eigen::Matrix3d quat2RotMatrix(const Eigen::Quaterniond &q)
{
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    return R;
}
//}

/* rotationMatrixToEulerAngles() //{ */
Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d &R)
{
    double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    bool singular = sy < 1e-6;
    
    double x, y, z;
    if (!singular)
    {
        x = atan2(R(2, 1), R(2, 2));
        y = atan2(-R(2, 0), sy);
        z = atan2(R(1, 0), R(0, 0));
    }
    else
    {
        x = atan2(-R(1, 2), R(1, 1));
        y = atan2(-R(2, 0), sy);
        z = 0;
    }
    return Eigen::Vector3d(x, y, z);
}
//}

/* orientationError() //{ */
Eigen::Vector3d orientationError(const Eigen::Matrix3d& R, const Eigen::Matrix3d& Rd)
{
    // orientation error
    Eigen::Matrix3d R_error = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

    // vectorize the orientation error
    Eigen::Vector3d R_error_vec;
    R_error_vec << (R_error(1, 2) - R_error(2, 1)) / 2.0,
                   (R_error(2, 0) - R_error(0, 2)) / 2.0,
                   (R_error(0, 1) - R_error(1, 0)) / 2.0;

    return R_error_vec;
}
//}

/* so3transform() //{ */
Eigen::Matrix3d so3transform(const Eigen::Vector3d& body_z, const Eigen::Vector3d& heading, const bool& preserve_heading)
{
    Eigen::Vector3d body_z_normed = body_z.normalized();
    Eigen::Matrix3d Rd;

    if (preserve_heading)
    {
        // Using Baca's method for heading preservation
        // body z
        Rd.col(2) = body_z_normed;

        // body x
        // construct the oblique projection
        Eigen::Matrix3d projector_body_z_compl = (Eigen::Matrix3d::Identity(3, 3) - body_z_normed * body_z_normed.transpose());

        // create a basis of the body-z complement subspace
        Eigen::MatrixXd A = Eigen::MatrixXd(3, 2);
        A.col(0) = projector_body_z_compl.col(0);
        A.col(1) = projector_body_z_compl.col(1);

        // create the basis of the projection null-space complement
        Eigen::MatrixXd B = Eigen::MatrixXd(3, 2);
        B.col(0) = Eigen::Vector3d(1, 0, 0);
        B.col(1) = Eigen::Vector3d(0, 1, 0);

        // oblique projector to <range_basis>
        Eigen::MatrixXd Bt_A = B.transpose() * A;
        Eigen::MatrixXd Bt_A_pseudoinverse = ((Bt_A.transpose() * Bt_A).inverse()) * Bt_A.transpose();
        Eigen::MatrixXd oblique_projector = A * Bt_A_pseudoinverse * B.transpose();

        Rd.col(0) = oblique_projector * heading;
        Rd.col(0).normalize();

        // body y
        Rd.col(1) = Rd.col(2).cross(Rd.col(0));
        Rd.col(1).normalize();
    }
    else
    {
        // Using Lee's method
        Rd.col(2) = body_z_normed;
        Rd.col(1) = Rd.col(2).cross(heading);
        Rd.col(1).normalize();
        Rd.col(0) = Rd.col(1).cross(Rd.col(2));
        Rd.col(0).normalize();
    }

    return Rd;
}
//}

/* sanitizeDesiredForce() //{ */
std::pair<bool, Eigen::Vector3d> sanitizeDesiredForce(
    const Eigen::Vector3d& f_des, 
    const double& tilt_safety_limit, 
    const double& tilt_saturation,
    const std::string& node_name)
{
    Eigen::Vector3d f_norm = f_des.normalized();
    
    // calculate the force in spherical coordinates
    double theta = acos(f_norm(2));
    double phi = atan2(f_norm(1), f_norm(0));

    // check for the failsafe limit
    if (!std::isfinite(theta))
    {
        ROS_ERROR("[%s]: sanitizeDesiredForce(): NaN detected in variable 'theta'", node_name.c_str());
        return std::make_pair(false, Eigen::Vector3d::Zero());
    }

    if (tilt_safety_limit > 1e-3 && std::abs(theta) > tilt_safety_limit)
    {
        ROS_ERROR("[%s]: the produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg)", 
                  node_name.c_str(), (180.0 / M_PI) * theta, (180.0 / M_PI) * tilt_safety_limit);
        ROS_ERROR_STREAM("[" << node_name << "]: f = [" << f_des.transpose() << "]");
        return std::make_pair(false, Eigen::Vector3d::Zero());
    }

    // saturate the angle
    if (tilt_saturation > 1e-3 && std::abs(theta) > tilt_saturation)
    {
        ROS_WARN_THROTTLE(1.0, "[%s]: tilt is being saturated, desired: %.2f deg, saturated %.2f deg", 
                          node_name.c_str(), (theta / M_PI) * 180.0, (tilt_saturation / M_PI) * 180.0);
        theta = tilt_saturation;
    }

    // reconstruct the force vector back out of the spherical coordinates
    Eigen::Vector3d output(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));

    return std::make_pair(true, output);
}
//}

namespace throttle_model
{

/* forceToThrottle() //{ */
double forceToThrottle(double force, double hover_percentage, double mass, double g)
{
    double hover_force = mass * g;
    double throttle = force / hover_force * hover_percentage;
    
    // Saturate throttle
    if (throttle > 1.0) throttle = 1.0;
    if (throttle < 0.0) throttle = 0.0;
    
    return throttle;
}
//}

/* throttleToForce() //{ */
double throttleToForce(double throttle, double hover_percentage, double mass, double g)
{
    double hover_force = mass * g;
    double force = throttle * hover_force / hover_percentage;
    return force;
}
//}

} // namespace throttle_model

} // namespace icaa_common