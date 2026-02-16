#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class Nav2PX4Bridge : public rclcpp::Node {
public:
    Nav2PX4Bridge() : Node("nav2_px4_bridge") {
        // Parameters
        this->declare_parameter<std::string>("namespace", "px4_drone");
        this->declare_parameter<double>("target_altitude", 2.0);
        this->declare_parameter<double>("max_horizontal_velocity", 2.0);
        this->declare_parameter<double>("max_vertical_velocity", 1.0);
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        
        namespace_ = this->get_parameter("namespace").as_string();
        target_altitude_ = this->get_parameter("target_altitude").as_double();
        max_horizontal_vel_ = this->get_parameter("max_horizontal_velocity").as_double();
        max_vertical_vel_ = this->get_parameter("max_vertical_velocity").as_double();
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        
        // Publishers
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/" + namespace_ + "/fmu/in/trajectory_setpoint", 10);
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/" + namespace_ + "/fmu/in/offboard_control_mode", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/" + namespace_ + "/fmu/in/vehicle_command", 10);
        
        // Subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 10,
            std::bind(&Nav2PX4Bridge::cmdVelCallback, this, std::placeholders::_1));
        local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/" + namespace_ + "/fmu/out/vehicle_local_position", 10,
            std::bind(&Nav2PX4Bridge::localPositionCallback, this, std::placeholders::_1));
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/" + namespace_ + "/fmu/out/vehicle_status", 10,
            std::bind(&Nav2PX4Bridge::vehicleStatusCallback, this, std::placeholders::_1));
        
        // Timer to publish offboard mode continuously (required by PX4)
        offboard_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&Nav2PX4Bridge::publishOffboardMode, this));
        
        // Timer for altitude control
        altitude_control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz
            std::bind(&Nav2PX4Bridge::altitudeControlLoop, this));
        
        current_altitude_ = 0.0;
        armed_ = false;
        offboard_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Nav2-PX4 Bridge initialized for namespace: %s", namespace_.c_str());
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Convert Nav2 cmd_vel (ENU) to PX4 TrajectorySetpoint (NED)
        px4_msgs::msg::TrajectorySetpoint setpoint;
        
        // PX4 uses NED frame: X=N, Y=E, Z=-D
        // Nav2 uses ENU: X=E, Y=N, Z=U
        // Conversion: NED_x = ENU_y, NED_y = ENU_x, NED_z = -ENU_z
        
        // Limit velocities
        double vx = std::max(-max_horizontal_vel_, std::min(max_horizontal_vel_, msg->linear.x));
        double vy = std::max(-max_horizontal_vel_, std::min(max_horizontal_vel_, msg->linear.y));
        
        // Convert ENU to NED
        setpoint.velocity[0] = vy;   // NED X (North) = ENU Y
        setpoint.velocity[1] = vx;    // NED Y (East) = ENU X
        setpoint.velocity[2] = 0.0;  // Will be controlled by altitude controller
        
        // Yaw rate conversion (ENU yaw = NED yaw, but direction might differ)
        setpoint.yawspeed = -msg->angular.z;  // NED uses opposite yaw direction
        
        // Set altitude to NaN to use velocity control in Z
        setpoint.position[0] = NAN;
        setpoint.position[1] = NAN;
        setpoint.position[2] = NAN;
        
        setpoint.timestamp = this->now().nanoseconds() / 1000;  // microseconds
        
        trajectory_setpoint_pub_->publish(setpoint);
        
        RCLCPP_DEBUG(this->get_logger(), 
                     "Published setpoint: vx=%.2f, vy=%.2f, yawspeed=%.2f",
                     vx, vy, msg->angular.z);
    }
    
    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        // PX4 position is in NED frame, z is negative (down)
        // Convert to altitude (positive up)
        current_altitude_ = -msg->z;
    }
    
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        offboard_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    }
    
    void publishOffboardMode() {
        px4_msgs::msg::OffboardControlMode mode;
        mode.timestamp = this->now().nanoseconds() / 1000;
        mode.position = false;
        mode.velocity = true;  // We're using velocity control
        mode.acceleration = false;
        mode.attitude = false;
        mode.body_rate = false;
        
        offboard_mode_pub_->publish(mode);
    }
    
    void altitudeControlLoop() {
        // PID control for altitude
        double altitude_error = target_altitude_ - current_altitude_;
        
        // Simple P controller (can be extended to PID)
        double kp = 1.0;
        double vertical_velocity = kp * altitude_error;
        
        // Limit vertical velocity
        vertical_velocity = std::max(-max_vertical_vel_, std::min(max_vertical_vel_, vertical_velocity));
        
        // Publish altitude control as part of trajectory setpoint
        // This will be merged with horizontal velocity commands
        px4_msgs::msg::TrajectorySetpoint setpoint;
        setpoint.position[0] = NAN;
        setpoint.position[1] = NAN;
        setpoint.position[2] = NAN;
        setpoint.velocity[0] = NAN;
        setpoint.velocity[1] = NAN;
        setpoint.velocity[2] = -vertical_velocity;  // NED: negative is up
        setpoint.yawspeed = NAN;
        setpoint.timestamp = this->now().nanoseconds() / 1000;
        
        // Only publish if we have valid altitude data
        if (current_altitude_ > 0.0) {
            trajectory_setpoint_pub_->publish(setpoint);
        }
    }
    
    std::string namespace_;
    double target_altitude_;
    double max_horizontal_vel_;
    double max_vertical_vel_;
    double current_altitude_;
    bool armed_;
    bool offboard_;
    
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    
    rclcpp::TimerBase::SharedPtr offboard_timer_;
    rclcpp::TimerBase::SharedPtr altitude_control_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2PX4Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
