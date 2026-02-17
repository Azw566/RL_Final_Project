#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

class DynamicTfPublisher : public rclcpp::Node {
public:
    DynamicTfPublisher() : Node("dynamic_tf_publisher") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/drone1/odom", 10,
            std::bind(&DynamicTfPublisher::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Dynamic TF publisher initialized (drone1).");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "drone1/odom";
        transform_stamped.child_frame_id = "base_footprint";

        transform_stamped.transform.translation.x = x;
        transform_stamped.transform.translation.y = y;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform_stamped);

        RCLCPP_DEBUG(this->get_logger(),
                     "Published transform: [frame: drone1/odom -> base_footprint] [x: %.2f, y: %.2f, yaw: %.2f]",
                     x, y, yaw);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicTfPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
