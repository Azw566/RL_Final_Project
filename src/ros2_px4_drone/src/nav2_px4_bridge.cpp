#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/qos.hpp>
#include <cmath>

class Nav2PX4Bridge : public rclcpp::Node {
public:
    Nav2PX4Bridge() : Node("nav2_px4_bridge") {
        // Parameters
        this->declare_parameter<std::string>("namespace", "drone1");
        this->declare_parameter<double>("target_altitude", 1.0);
        this->declare_parameter<double>("max_horizontal_velocity", 2.0);
        this->declare_parameter<double>("max_vertical_velocity", 0.5);
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter<double>("geofence_min_x", -9.0);
        this->declare_parameter<double>("geofence_max_x",  9.0);
        this->declare_parameter<double>("geofence_min_y", -1.0);
        this->declare_parameter<double>("geofence_max_y", 17.0);

        namespace_ = this->get_parameter("namespace").as_string();
        target_altitude_ = this->get_parameter("target_altitude").as_double();
        max_horizontal_vel_ = this->get_parameter("max_horizontal_velocity").as_double();
        max_vertical_vel_ = this->get_parameter("max_vertical_velocity").as_double();
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        geofence_min_x_ = this->get_parameter("geofence_min_x").as_double();
        geofence_max_x_ = this->get_parameter("geofence_max_x").as_double();
        geofence_min_y_ = this->get_parameter("geofence_min_y").as_double();
        geofence_max_y_ = this->get_parameter("geofence_max_y").as_double();

        // Publishers
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/" + namespace_ + "/fmu/in/trajectory_setpoint", 10);
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/" + namespace_ + "/fmu/in/offboard_control_mode", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/" + namespace_ + "/fmu/in/vehicle_command", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/" + namespace_ + "/odom", 10);

        // QoS for PX4 topics (BEST_EFFORT required for uXRCE-DDS)
        rclcpp::QoS px4_qos(10);
        px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

        // Subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 10,
            std::bind(&Nav2PX4Bridge::cmdVelCallback, this, std::placeholders::_1));
        local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/" + namespace_ + "/fmu/out/vehicle_local_position", px4_qos,
            std::bind(&Nav2PX4Bridge::localPositionCallback, this, std::placeholders::_1));
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/" + namespace_ + "/fmu/out/vehicle_status", px4_qos,
            std::bind(&Nav2PX4Bridge::vehicleStatusCallback, this, std::placeholders::_1));

        // Single unified control loop — publishes offboard heartbeat + combined setpoint
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz
            std::bind(&Nav2PX4Bridge::controlLoop, this));

        current_altitude_ = 0.0;
        current_x_ = 0.0;
        current_y_ = 0.0;
        cmd_vx_ = 0.0;
        cmd_vy_ = 0.0;
        cmd_yawspeed_ = 0.0;
        armed_ = false;
        offboard_ = false;
        offboard_setpoint_count_ = 0;

        RCLCPP_INFO(this->get_logger(), "Nav2-PX4 Bridge initialized for namespace: %s", namespace_.c_str());
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Store latest Nav2 velocity commands (ENU frame)
        cmd_vx_ = std::clamp(msg->linear.x, -max_horizontal_vel_, max_horizontal_vel_);
        cmd_vy_ = std::clamp(msg->linear.y, -max_horizontal_vel_, max_horizontal_vel_);
        cmd_yawspeed_ = msg->angular.z;

        // Geofence: suppress velocity pushing drone outside maze bounds
        if (current_x_ <= geofence_min_x_ && cmd_vx_ < 0) cmd_vx_ = 0.0;
        if (current_x_ >= geofence_max_x_ && cmd_vx_ > 0) cmd_vx_ = 0.0;
        if (current_y_ <= geofence_min_y_ && cmd_vy_ < 0) cmd_vy_ = 0.0;
        if (current_y_ >= geofence_max_y_ && cmd_vy_ > 0) cmd_vy_ = 0.0;
    }

    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        current_altitude_ = -msg->z;
        current_x_ = msg->y;   // ENU X = NED Y
        current_y_ = msg->x;   // ENU Y = NED X

        // Publish Odometry for Nav2 and TF publisher
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = namespace_ + "/odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = msg->y;
        odom.pose.pose.position.y = msg->x;
        odom.pose.pose.position.z = -msg->z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, -msg->heading + M_PI / 2.0);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = msg->vy;
        odom.twist.twist.linear.y = msg->vx;
        odom.twist.twist.linear.z = -msg->vz;

        odom_pub_->publish(odom);
    }

    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        offboard_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    }

    void controlLoop() {
        // 1. Publish offboard heartbeat (required every cycle)
        px4_msgs::msg::OffboardControlMode mode;
        mode.timestamp = this->now().nanoseconds() / 1000;
        mode.position = false;
        mode.velocity = true;
        mode.acceleration = false;
        mode.attitude = false;
        mode.body_rate = false;
        offboard_mode_pub_->publish(mode);

        offboard_setpoint_count_++;

        // 2. Auto arm+offboard after 1s of heartbeats
        if (offboard_setpoint_count_ == 20) {  // 20 cycles at 20Hz = 1s
            sendOffboardCommand();
            sendArmCommand();
        }

        // 3. Compute altitude control (P controller)
        double altitude_error = target_altitude_ - current_altitude_;
        double vz = std::clamp(1.0 * altitude_error, -max_vertical_vel_, max_vertical_vel_);

        // 4. Publish ONE combined setpoint: horizontal (from Nav2) + vertical (altitude)
        px4_msgs::msg::TrajectorySetpoint setpoint;
        setpoint.position[0] = NAN;
        setpoint.position[1] = NAN;
        setpoint.position[2] = NAN;

        // Convert ENU velocities to NED
        setpoint.velocity[0] = cmd_vy_;    // NED X (North) = ENU Y
        setpoint.velocity[1] = cmd_vx_;    // NED Y (East) = ENU X
        setpoint.velocity[2] = -vz;        // NED Z (Down) = -ENU Z

        setpoint.yawspeed = -cmd_yawspeed_;  // NED yaw = -ENU yaw
        setpoint.timestamp = this->now().nanoseconds() / 1000;

        trajectory_setpoint_pub_->publish(setpoint);
    }

    void sendOffboardCommand() {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp = this->now().nanoseconds() / 1000;
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1 = 1.0;
        cmd.param2 = 6.0;  // offboard
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        vehicle_command_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Offboard mode requested");
    }

    void sendArmCommand() {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp = this->now().nanoseconds() / 1000;
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.param1 = 1.0;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        vehicle_command_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }

    std::string namespace_;
    double target_altitude_;
    double max_horizontal_vel_;
    double max_vertical_vel_;
    double current_altitude_;
    double current_x_, current_y_;
    double cmd_vx_, cmd_vy_, cmd_yawspeed_;
    double geofence_min_x_, geofence_max_x_;
    double geofence_min_y_, geofence_max_y_;
    bool armed_;
    bool offboard_;
    int offboard_setpoint_count_;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2PX4Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
