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

        RCLCPP_INFO(this->get_logger(), "Dynamic TF publisher initialized (drone1) with Time Offset.");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        geometry_msgs::msg::TransformStamped transform_stamped;
        
        // --- CORRECTION : AJOUT D'UN OFFSET DE 50ms ---
        // On utilise le stamp du message + une petite marge pour éviter les rejets du cache TF
        rclcpp::Time stamp = rclcpp::Time(msg->header.stamp) + rclcpp::Duration::from_seconds(0.05);
        transform_stamped.header.stamp = stamp;
        // ----------------------------------------------

        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_footprint";

        transform_stamped.transform.translation.x = x;
        transform_stamped.transform.translation.y = y;
        transform_stamped.transform.translation.z = 0.0;
        
        transform_stamped.transform.rotation.x = msg->pose.pose.orientation.x;
        transform_stamped.transform.rotation.y = msg->pose.pose.orientation.y;
        transform_stamped.transform.rotation.z = msg->pose.pose.orientation.z;
        transform_stamped.transform.rotation.w = msg->pose.pose.orientation.w;

        tf_broadcaster_->sendTransform(transform_stamped);
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
