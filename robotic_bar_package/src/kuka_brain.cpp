#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class KukaBrain : public rclcpp::Node {
public:
// --- CONSTRUCTOR ---
  KukaBrain() : Node("kuka_brain"), state_("IDLE") {

    // Useful topics
    pub_traj_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/iiwa/iiwa_arm_trajectory_controller/joint_trajectory", 10);
    pub_attach_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/bar/gripper/attach", 10);
    pub_detach_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/bar/gripper/detach", 10);

    // Topic to get orders
    sub_order_ = this->create_subscription<std_msgs::msg::String>(
      "/bar/new_order", 10, std::bind(&KukaBrain::order_callback, this, _1));

    // Topic to get Fra2mo status to pickup
    sub_fra2mo_ = this->create_subscription<std_msgs::msg::Empty>(
      "/fra2mo/ready", 10, std::bind(&KukaBrain::fra2mo_callback, this, _1));

    joint_names_ = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"};

    // Initial Move to Home
    std::vector<std::vector<double>> path = {HOME_CONF};
    move_through_waypoints(path, 4.0); 

    RCLCPP_INFO(this->get_logger(), "🧠 KUKA BRAIN ACTIVE!");
  }

private:
  // Angle home position
  const std::vector<double> HOME_CONF = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  // 1. Approaching Pose before Pick
  const std::vector<double> PICK_APPROACH_CONF =  {-0.02, -0.747881, 0.0, 0.712028, 0.0, -0.152515, 0.0}; 

  // 2. Pick Pose
  const std::vector<double> PICK_CONF = {-0.02, -0.78960, 0.0, 0.712028, 0.0, 0.056905, 0.0}; 

  // 3. Intermediate Pose before Handover
  const std::vector<double> HANDOVER_INTERMEDIATE_CONF = {-1.602072, -0.795796, 0.0, 0.712028, 0.0, 0.056905, 0.0};

  // 4. Handover Pose
  const std::vector<double> HANDOVER_CONF = {-1.602072, -2.0, 0.0, -0.041884, 0.0, 0.293188, 0.0};

  // 5. Returning Home Pose
  const std::vector<double> RETURNING_HOME_CONF = {-1.602072,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  // Safety function: In any case, it won't be possible to issue new orders while shipping not completed
  void order_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (state_ != "IDLE") {
      RCLCPP_WARN(this->get_logger(), "⚠️  I am busy.");
      return;
    }

    std::string target_name = msg->data;
    RCLCPP_INFO(this->get_logger(), "🔔 ORDER RECEIVED FROM: %s", target_name.c_str());

    // 1. Picking Phase
    state_ = "PICKING";
    RCLCPP_INFO(this->get_logger(), "💪 Picking the object...");

    // Moving
    std::vector<std::vector<double>> pick_path = {PICK_APPROACH_CONF, PICK_CONF};
    move_through_waypoints(pick_path, 6.0);

    // Wait a bit to stabilize
    std::this_thread::sleep_for(6500ms); 

    // 2. Attach Phase
    RCLCPP_INFO(this->get_logger(), "🧲 Attaching to the object...");

    // Publishing empty message just to trigger attach 
    auto empty_msg = std_msgs::msg::Empty();
    pub_attach_->publish(empty_msg);

    std::this_thread::sleep_for(500ms);

    // 3. Handover Phase
    RCLCPP_INFO(this->get_logger(), "🚚 Transporting the object...");
    
    // Moving
    std::vector<std::vector<double>> handover_path = {
        HANDOVER_INTERMEDIATE_CONF, 
        HANDOVER_CONF
    };
    move_through_waypoints(handover_path, 5.0);
    std::this_thread::sleep_for(6s);

    // 4. Waiting for Fra2mo
    state_ = "WAITING_HANDOVER";
    RCLCPP_INFO(this->get_logger(), "⏳ Waiting for Fra2mo...");
  }

  void fra2mo_callback(const std_msgs::msg::Empty::SharedPtr msg) {
    (void)msg;
    if (state_ != "WAITING_HANDOVER") return;

    RCLCPP_INFO(this->get_logger(), "📩 Fra2mo arrived!");

    // 5. Detach Phase
    RCLCPP_INFO(this->get_logger(), "🔓 Detaching the can...");

    // Publishing empty message just to trigger detach
    pub_detach_->publish(std_msgs::msg::Empty());

    state_ = "IDLE";
    
    RCLCPP_INFO(this->get_logger(), "✅ Returning to Home...");
    // Wait a bit to Fra2mo go away
    std::this_thread::sleep_for(4s); 
    
    // Moving
    std::vector<std::vector<double>> home_path = {
      RETURNING_HOME_CONF,
      HOME_CONF
  };
  move_through_waypoints(home_path, 6.0);
  }

  // Function to move through a series of waypoints in given total duration specifying joint trajectory
  void move_through_waypoints(const std::vector<std::vector<double>>& waypoints, double total_duration_sec) {
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.joint_names = joint_names_;
  
    double time_per_segment = total_duration_sec / waypoints.size();
    double current_time = 0.0;
  
    for (const auto& pose : waypoints) {
      current_time += time_per_segment;
      
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = pose;
      
      point.velocities.resize(joint_names_.size(), 0.0);
      point.accelerations.resize(joint_names_.size(), 0.0);
  
      point.time_from_start.sec = (int)current_time;
      point.time_from_start.nanosec = (current_time - (int)current_time) * 1e9;
  
      traj_msg.points.push_back(point);
    }
  
    pub_traj_->publish(traj_msg);
  }

  // --- MEMBERS ---
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_traj_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_attach_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_detach_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_order_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_fra2mo_;
  std::vector<std::string> joint_names_;
  std::string state_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KukaBrain>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}