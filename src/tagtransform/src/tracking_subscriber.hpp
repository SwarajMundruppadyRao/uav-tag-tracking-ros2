#ifndef TRACKING_SUBSCRIBER_HPP_
#define TRACKING_SUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include "rclcpp/qos.hpp"
#include <cmath>

#define X_DIST 1.0
#define Y_DIST 0.0
#define HEIGHT -2.5

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class TrackingSubscriber : public rclcpp::Node
{
public:
    TrackingSubscriber();

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publish_transform();
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void lookup_transform();
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;
    float base_x = 0.0;
    float base_y = 0.0;
    float base_z = 0.0f;                
    float april_x;
    float april_y;
    float april_z;
    float diff_x = 0.0;
    float diff_y = 0.0;
    float diff_z = 0.0;
    float reference_z_          = 0.0f; 
    bool  reference_z_captured_ = false;
    bool  last_offboard_state_  = false;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::TimerBase::SharedPtr timer3_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_command_listener_;
    uint64_t offboard_setpoint_counter_;
    VehicleControlMode c_mode;
};

#endif  // TRACKING_SUBSCRIBER_HPP_
