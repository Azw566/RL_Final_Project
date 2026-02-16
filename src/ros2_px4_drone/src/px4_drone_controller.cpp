#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class PX4DroneController : public rclcpp::Node {
public:
    PX4DroneController() : Node("px4_drone_controller") {
        // Parameters
        this->declare_parameter<std::string>("namespace", "px4_drone");
        this->declare_parameter<double>("target_altitude", 2.0);
        this->declare_parameter<double>("takeoff_altitude", 2.0);
        this->declare_parameter<double>("max_horizontal_velocity", 2.0);
        this->declare_parameter<double>("max_vertical_velocity", 1.0);
        
        namespace_ = this->get_parameter("namespace").as_string();
        target_altitude_ = this->get_parameter("target_altitude").as_double();
        takeoff_altitude_ = this->get_parameter("takeoff_altitude").as_double();
        max_horizontal_vel_ = this->get_parameter("max_horizontal_velocity").as_double();
        max_vertical_vel_ = this->get_parameter("max_vertical_velocity").as_double();
        
        // Publishers
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/" + namespace_ + "/fmu/in/trajectory_setpoint", 10);
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/" + namespace_ + "/fmu/in/offboard_control_mode", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/" + namespace_ + "/fmu/in/vehicle_command", 10);
        
        // Subscribers
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/" + namespace_ + "/navigate_to_pose", 10,
            std::bind(&PX4DroneController::goalPoseCallback, this, std::placeholders::_1));
        local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/" + namespace_ + "/fmu/out/vehicle_local_position", 10,
            std::bind(&PX4DroneController::localPositionCallback, this, std::placeholders::_1));
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/" + namespace_ + "/fmu/out/vehicle_status", 10,
            std::bind(&PX4DroneController::vehicleStatusCallback, this, std::placeholders::_1));
        
        // Timer to publish offboard mode continuously
        offboard_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PX4DroneController::publishOffboardMode, this));
        
        // Control timer
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PX4DroneController::controlLoop, this));
        
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_z_ = 0.0;
        current_yaw_ = 0.0;
        armed_ = false;
        offboard_ = false;
        state_ = INIT;
        
        RCLCPP_INFO(this->get_logger(), "PX4 Drone Controller initialized for namespace: %s", namespace_.c_str());
    }

private:
    enum State {
        INIT,
        ARMING,
        TAKEOFF,
        HOVERING,
        NAVIGATING,
        LANDING
    };
    
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Store goal position (in map frame, ENU)
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_z_ = target_altitude_;  // Always use target altitude
        
        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        goal_yaw_ = yaw;
        
        if (state_ == HOVERING || state_ == NAVIGATING) {
            state_ = NAVIGATING;
            RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f, %.2f)", goal_x_, goal_y_, goal_z_);
        }
    }
    
    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        // PX4 position is in NED frame
        // Convert to ENU for internal use
        current_x_ = msg->y;   // ENU X = NED Y
        current_y_ = msg->x;   // ENU Y = NED X
        current_z_ = -msg->z;  // ENU Z = -NED Z
        
        // Yaw conversion (NED to ENU)
        current_yaw_ = msg->heading;
    }
    
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        offboard_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        
        // State machine transitions
        if (state_ == INIT && armed_ && offboard_) {
            state_ = TAKEOFF;
            RCLCPP_INFO(this->get_logger(), "Armed and in offboard mode, starting takeoff");
        }
    }
    
    void publishOffboardMode() {
        px4_msgs::msg::OffboardControlMode mode;
        mode.timestamp = this->now().nanoseconds() / 1000;
        mode.position = true;   // Using position control
        mode.velocity = false;
        mode.acceleration = false;
        mode.attitude = false;
        mode.body_rate = false;
        
        offboard_mode_pub_->publish(mode);
        
        // Also send arm command if not armed
        if (!armed_ && state_ == INIT) {
            sendArmCommand();
        }
    }
    
    void sendArmCommand() {
        px4_msgs::msg::VehicleCommand cmd;
        cmd.timestamp = this->now().nanoseconds() / 1000;
        cmd.param1 = 1.0;  // arm
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        vehicle_command_pub_->publish(cmd);
    }
    
    void controlLoop() {
        px4_msgs::msg::TrajectorySetpoint setpoint;
        setpoint.timestamp = this->now().nanoseconds() / 1000;
        
        switch (state_) {
            case INIT:
                // Wait for arming
                break;
                
            case TAKEOFF:
                // Set target position to current position but at takeoff altitude
                setpoint.position[0] = current_y_;  // NED X = ENU Y
                setpoint.position[1] = current_x_;  // NED Y = ENU X
                setpoint.position[2] = -takeoff_altitude_;  // NED Z (negative is up)
                setpoint.yaw = current_yaw_;
                
                trajectory_setpoint_pub_->publish(setpoint);
                
                // Check if reached takeoff altitude
                if (std::abs(current_z_ - takeoff_altitude_) < 0.2) {
                    state_ = HOVERING;
                    RCLCPP_INFO(this->get_logger(), "Reached takeoff altitude, hovering");
                }
                break;
                
            case HOVERING:
                // Maintain current position and altitude
                setpoint.position[0] = current_y_;
                setpoint.position[1] = current_x_;
                setpoint.position[2] = -target_altitude_;
                setpoint.yaw = current_yaw_;
                
                trajectory_setpoint_pub_->publish(setpoint);
                break;
                
            case NAVIGATING:
                // Navigate to goal position
                setpoint.position[0] = goal_y_;  // NED X = ENU Y
                setpoint.position[1] = goal_x_;  // NED Y = ENU X
                setpoint.position[2] = -goal_z_;  // NED Z
                setpoint.yaw = goal_yaw_;
                
                trajectory_setpoint_pub_->publish(setpoint);
                
                // Check if reached goal
                double dx = goal_x_ - current_x_;
                double dy = goal_y_ - current_y_;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                if (distance < 0.5) {  // 0.5m tolerance
                    state_ = HOVERING;
                    RCLCPP_INFO(this->get_logger(), "Reached goal, hovering");
                }
                break;
                
            case LANDING:
                // Land at current position
                setpoint.position[0] = current_y_;
                setpoint.position[1] = current_x_;
                setpoint.position[2] = 0.0;  // Ground level
                setpoint.yaw = current_yaw_;
                
                trajectory_setpoint_pub_->publish(setpoint);
                break;
        }
    }
    
    std::string namespace_;
    double target_altitude_;
    double takeoff_altitude_;
    double max_horizontal_vel_;
    double max_vertical_vel_;
    
    double current_x_, current_y_, current_z_, current_yaw_;
    double goal_x_, goal_y_, goal_z_, goal_yaw_;
    bool armed_;
    bool offboard_;
    State state_;
    
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    
    rclcpp::TimerBase::SharedPtr offboard_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4DroneController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
