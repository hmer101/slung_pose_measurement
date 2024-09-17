#ifndef UTILS_H
#define UTILS_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <string>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include "std_msgs/msg/string.hpp"

#include "slung_pose_measurement/State.h"


namespace utils {
    // TRANSFORMS
    void broadcast_tf(const rclcpp::Time &time, const std::string &frame_parent, const std::string &frame_child, const Eigen::Vector3d &pos, const Eigen::Quaterniond &att, tf2_ros::TransformBroadcaster &broadcaster);
    void broadcast_tf(const rclcpp::Time &time, const std::string &frame_parent, const std::string &frame_child, const Eigen::Vector3d &pos, const Eigen::Quaterniond &att, tf2_ros::StaticTransformBroadcaster &broadcaster);
    std::optional<geometry_msgs::msg::TransformStamped> lookup_tf(const std::string &target_frame, const std::string &source_frame, tf2_ros::Buffer &tfBuffer, const rclcpp::Time &time, rclcpp::Logger logger);

    Eigen::Vector3d transform_position(const Eigen::Vector3d& p_BA, const Eigen::Vector3d& p_CB, const tf2::Quaternion& q_CB);
    tf2::Quaternion transform_orientation(const tf2::Quaternion& q_BA, const tf2::Quaternion& q_CB);
    //std::shared_ptr<State> transform_frames(const State& state, const std::string& frame2_name, tf2_ros::Buffer& tf_buffer, rclcpp::Logger logger);
    std::shared_ptr<droneState::State> transform_frames(const droneState::State &state, const std::string &frame2_name, tf2_ros::Buffer &tf_buffer, rclcpp::Logger logger, droneState::CS_type cs_out_type = droneState::CS_type::XYZ);

    // MATH
    float getTrace(const tf2::Matrix3x3 &matrix);

    // STRING HANDLING
    int extract_id_from_name(const std::string &input);
    std::vector<float> splitAndConvert(const std::string &s, char delimiter);

    // CONVERSIONS
    Eigen::Vector3d convert_vec_floats_to_eigen(const std::vector<float> &float_vector);  
    geometry_msgs::msg::Pose convert_state_to_pose_msg(const droneState::State &state);
    droneState::State convert_tf_stamped_msg_to_state(const geometry_msgs::msg::TransformStamped &pose_msg, std::string frame, droneState::CS_type cs_type, Eigen::Vector3d vel = Eigen::Vector3d(0.0, 0.0, 0.0));
    tf2::Quaternion convert_rvec_to_quaternion(const cv::Vec3d &rvec);
    Eigen::Matrix3d convert_rvec_to_rotmat(const Eigen::Vector3d &rvec);
}

#endif // UTILS_H
