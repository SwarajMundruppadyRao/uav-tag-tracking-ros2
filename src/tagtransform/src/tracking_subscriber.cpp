#include "tracking_subscriber.hpp"


TrackingSubscriber::TrackingSubscriber() : Node("tracking_subscriber")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/tag_detections", qos, std::bind(&TrackingSubscriber::pose_callback, this, std::placeholders::_1));

    // For Simulation as topic is /tf
    // subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/tf", qos, std::bind(&TrackingSubscriber::pose_callback, this, std::placeholders::_1));


    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&TrackingSubscriber::odometry_callback, this, std::placeholders::_1));
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_   = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    vehicle_command_listener_ = this->create_subscription<VehicleControlMode>("/fmu/out/vehicle_control_mode", qos2,[this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg) { c_mode = *msg; });

    offboard_setpoint_counter_ = 0;
    reference_z_captured_ = false;
    last_offboard_state_  = false; 

    auto timer_callback = [this]() -> void {
        publish_transform();
        lookup_transform();
        double intpart;
        if (modf(static_cast<double>(offboard_setpoint_counter_) / 2, &intpart) == 0.0) {
            publish_offboard_control_mode();
        }
        publish_trajectory_setpoint();
    };
}


void TrackingSubscriber::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "tracking_camera";
    transform.child_frame_id  = "apriltag";
    transform.transform.translation.x = msg->pose.position.x;
    transform.transform.translation.y = msg->pose.position.y;
    transform.transform.translation.z = msg->pose.position.z;
    transform.transform.rotation.w = msg->pose.orientation.x;
    transform.transform.rotation.x = msg->pose.orientation.y;
    transform.transform.rotation.y = msg->pose.orientation.z;
    transform.transform.rotation.z = msg->pose.orientation.w;
    tf_broadcaster_->sendTransform(transform);
}


void TrackingSubscriber::publish_transform()
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "map";
    transform.child_frame_id  = "ned";
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 1.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 0.0;
    tf_broadcaster_->sendTransform(transform);

    geometry_msgs::msg::TransformStamped transform2;
    transform2.header.stamp = this->now();
    transform2.header.frame_id = "base_link";
    transform2.child_frame_id  = "tracking_camera";
    transform2.transform.translation.x = 0.120;
    transform2.transform.translation.y = -0.03;
    transform2.transform.translation.z = -0.242;
    transform2.transform.rotation.x = 0.2706;
    transform2.transform.rotation.y = 0.2706;
    transform2.transform.rotation.z = 0.6533;
    transform2.transform.rotation.w = 0.6533;
    tf_broadcaster_->sendTransform(transform2);
}


void TrackingSubscriber::odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "ned";
    transform.child_frame_id  = "base_link";
    transform.transform.translation.x = msg->position[0];
    transform.transform.translation.y = msg->position[1];
    transform.transform.translation.z = msg->position[2];
    transform.transform.rotation.w = msg->q[0];
    transform.transform.rotation.x = msg->q[1];
    transform.transform.rotation.y = msg->q[2];
    transform.transform.rotation.z = msg->q[3];
    tf_broadcaster_->sendTransform(transform);

    base_x = msg->position[0];
    base_y = msg->position[1];
    base_z = msg->position[2];
}

void TrackingSubscriber::lookup_transform()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer_->lookupTransform(
            "base_link", "apriltag", tf2::TimePointZero);
        RCLCPP_INFO(this->get_logger(), "Transform from map to april tag:");
        RCLCPP_INFO(this->get_logger(), "Translation: x=%.2f, y=%.2f, z=%.2f",
                    transformStamped.transform.translation.x,
                    transformStamped.transform.translation.y,
                    transformStamped.transform.translation.z);
        april_x = transformStamped.transform.translation.x;
        april_y = transformStamped.transform.translation.y;
        april_z = transformStamped.transform.translation.z;

        diff_x = april_x - X_DIST;
        diff_y = april_y - Y_DIST;
    } catch (tf2::TransformException &ex) {
        diff_x = 0.0;
        diff_y = 0.0;
        RCLCPP_WARN(this->get_logger(), "Could not transform map to test: %s", ex.what());
    }
}

void TrackingSubscriber::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position     = true;
    msg.velocity     = false;
    msg.acceleration = false;
    msg.attitude     = false;
    msg.body_rate    = false;
    msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}


void TrackingSubscriber::publish_trajectory_setpoint()
{
    offboard_setpoint_counter_++;
    bool offboard_now = (c_mode.flag_control_offboard_enabled == 1);
    if (offboard_now && !last_offboard_state_) {
        reference_z_ = base_z;
        reference_z_captured_= true;
        RCLCPP_INFO(this->get_logger(),"Latched reference Z (NED) = %.2f m", reference_z_);}
    last_offboard_state_=offboard_now;
    auto msg=px4_msgs::msg::TrajectorySetpoint();
    msg.timestamp=this->get_clock()->now().nanoseconds() / 1000;
    float z_setpoint=reference_z_captured_ ? reference_z_ : HEIGHT;
    msg.position= {base_x + diff_x, base_y + diff_y, z_setpoint};
    msg.yaw= 0.0;
    RCLCPP_INFO(this->get_logger(), "Publishing Trajectory Translation: x=%.2f, y=%.2f, z=%.2f",base_x + diff_x, base_y + diff_y, z_setpoint);
    if (offboard_now && offboard_setpoint_counter_ >= 30) {trajectory_setpoint_publisher_->publish(msg);RCLCPP_INFO(this->get_logger(), "Published TrajectorySetpoint");offboard_setpoint_counter_ = 0;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingSubscriber>());
    rclcpp::shutdown();
    return 0;
}
