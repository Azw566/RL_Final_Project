#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <thread>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Fra2moBrain : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Fra2moBrain() : Node("fra2mo_brain"), state_("IDLE"), tag_found_(false) {

    // --- POSES TO REACH (bar will be reached through a blind docking)---
    poses_db_["home"] = {0.0, 0.0, 0.0};

    poses_db_["table_1"] = {1.936, 7.751, -2.9};
    poses_db_["table_2"] = {-1.394, 7.751, -2.9};
    poses_db_["table_3"] = {-4.744, 7.751, -2.9};

    // --- TF SETUP ---
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subs
    sub_order_ = this->create_subscription<std_msgs::msg::String>(
      "/bar/new_order", 10, std::bind(&Fra2moBrain::order_callback, this, _1));

    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    sub_aruco_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aruco_single/pose", qos_profile, std::bind(&Fra2moBrain::aruco_callback, this, _1));

    // Pubs
    pub_ready_ = this->create_publisher<std_msgs::msg::Empty>("/fra2mo/ready", 10);
    pub_delivered_ = this->create_publisher<std_msgs::msg::String>("/delivered", 10);
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/fra2mo/cmd_vel", 10);
    pub_attach_ = this->create_publisher<std_msgs::msg::Empty>("/fra2mo/bar/gripper/attach", 10);
    pub_detach_ = this->create_publisher<std_msgs::msg::Empty>("/fra2mo/bar/gripper/detach", 10);

    // Nav2 Action Client
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Docking Control Loop Timer
    docking_timer_ = this->create_wall_timer(
      50ms, std::bind(&Fra2moBrain::control_loop, this));
    docking_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "🧠 FRA2MO BRAIN ACTIVE!");
  }

private:
// --- MEMBERS ---

  // Bar Docking coordinates 
  const std::vector<double> BLIND_BAR_TARGET = {-0.559, -0.371, 1.604};
  const double BAR_ROTATION = -(M_PI/2) - 0.3;
  const double BAR_REVERSE  = 0.405;
  const double BAR_STOP     = 1.4;

  // Table Docking parameters
  const double T1_ROTATION = (M_PI/2) + 1.0;
  const double T1_REVERSE  = 1.1;
  const double T2_ROTATION = (M_PI/2) + 1.0;
  const double T2_REVERSE  = 1.1;
  const double T3_ROTATION = (M_PI/2) + 1.0;
  const double T3_REVERSE  = 1.1;
  const double TABLE_STOP_DIST = 1.50;

  std::string state_;
  bool tag_found_;
  geometry_msgs::msg::PoseStamped last_aruco_pose_;
  rclcpp::Time last_aruco_received_time_;
  rclcpp::Time search_timer_start_;

  std::string current_table_name_;
  int current_target_table_ = 1; // Initialization
  bool targeting_bar_ = false; // True when going to bar, false when going to table

  // Variables used for docking
  double start_yaw_;
  double target_yaw_;             
  bool rotation_initialized_ = false;
  geometry_msgs::msg::PoseStamped start_reverse_pose_;
  bool reverse_initialized_ = false;
  geometry_msgs::msg::PoseStamped start_linear_pose_;
  bool linear_initialized_ = false;
  double cur_stop_dist_;
  double cur_reverse_dist_;
  double cur_rotation_ang_;


  std::map<std::string, std::vector<double>> poses_db_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::vector<std::string> order_queue_;
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_order_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_aruco_pose_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_ready_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_delivered_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_attach_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_detach_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr docking_timer_;

  // ---------------- HELPERS ----------------
  void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    last_aruco_pose_ = *msg;
    last_aruco_received_time_ = this->now();
    tag_found_ = true;
  }


  void order_callback(const std_msgs::msg::String::SharedPtr msg) {
    // If coming back home from previous order, queue the new one
    if (state_ != "IDLE") {
        RCLCPP_INFO(this->get_logger(), "⏳ Busy. Order queued: %s", msg->data.c_str());
        order_queue_.push_back(msg->data);
        return;
    }
    // Else process immediately
    process_order(msg->data);
  }


  void process_order(std::string order_data) {
    current_table_name_ = order_data;
    
    if (current_table_name_.find("table1") != std::string::npos) current_target_table_ = 1;
    else if (current_table_name_.find("table2") != std::string::npos) current_target_table_ = 2;
    else if (current_table_name_.find("table3") != std::string::npos) current_target_table_ = 3;
    else current_target_table_ = 1;
    RCLCPP_INFO(this->get_logger(), "🔔 ORDER RECEIVED FROM TABLE %d.", current_target_table_);
    targeting_bar_ = true; 
    cur_stop_dist_    = BAR_STOP;
    cur_reverse_dist_ = BAR_REVERSE;
    cur_rotation_ang_ = BAR_ROTATION;
    RCLCPP_INFO(this->get_logger(), "🚚 Moving to Bar...");
    state_ = "BLIND_TURN_TO_BAR";
    docking_timer_->reset();
  }


  // --- HELPER FUNCTIONS ---
  void configure_table_docking(int table_id) {
      cur_stop_dist_ = TABLE_STOP_DIST;
      if (table_id == 1) {
          cur_rotation_ang_ = T1_ROTATION; cur_reverse_dist_ = T1_REVERSE;
      } else if (table_id == 2) {
          cur_rotation_ang_ = T2_ROTATION; cur_reverse_dist_ = T2_REVERSE;
      } else {
          cur_rotation_ang_ = T3_ROTATION; cur_reverse_dist_ = T3_REVERSE;
      }
      RCLCPP_INFO(this->get_logger(), "📐 Parametri Tavolo %d settati.", table_id);
  }

  // Obtain yaw of a frame relative to base_footprint
  bool get_yaw_in_map_frame(double &yaw_out) {
      try {
          auto t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
          tf2::Quaternion q(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w);
          q.normalize();
          double roll, pitch, yaw;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
          yaw_out = yaw;
          return true;
      } catch (tf2::TransformException &ex) {
          (void)ex;
          return false;
      } catch (...) {
          return false;
      }
  }


  bool get_map_pose(geometry_msgs::msg::PoseStamped &pose) {
      try {
          auto t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
          pose.pose.position.x = t.transform.translation.x;
          pose.pose.position.y = t.transform.translation.y;
          pose.pose.orientation = t.transform.rotation;
          return true;
      } catch (...) { return false; }
  }

  // Helper function to normalize angles between -PI and PI
  double normalize_angle(double angle) {
      while (angle > M_PI) angle -= 2.0 * M_PI;
      while (angle < -M_PI) angle += 2.0 * M_PI;
      return angle;
  }

  // ---------- CONTROL LOOP ----------
  void control_loop() {
    geometry_msgs::msg::Twist cmd;

    // SAFE TF
    geometry_msgs::msg::PoseStamped map_pose;
    double yaw_map = 0.0;
    // Obtain map pose and yaw
    bool has_map = get_map_pose(map_pose);
    if(has_map) yaw_map = tf2::getYaw(map_pose.pose.orientation);


    // If going from table to home it is needed to realign the yaw with a small error in order to guarantee repeatability
    if (state_ == "ALIGN_HOME") {
        double yaw_map = 0.0;
        if (!get_yaw_in_map_frame(yaw_map)) return;
        double err_yaw = normalize_angle(0.0 - yaw_map);
        // Very small tolerance: 0.057 degrees
        if (std::abs(err_yaw) < 0.0001) {
            pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
            if (!order_queue_.empty()) {
                std::string next_order = order_queue_.front();
                order_queue_.erase(order_queue_.begin());
                RCLCPP_INFO(this->get_logger(), "🔄 Queued order found. Executing...");
                process_order(next_order); 
            } else {
                state_ = "IDLE";
                RCLCPP_INFO(this->get_logger(), "🏠 Realigned to Home. READY.");
            }
            return;
        }
        // Command to rotate with saturation
        cmd.angular.z = std::clamp(2.0 * err_yaw, -0.2, 0.2);
        pub_cmd_vel_->publish(cmd);
        return;
    }
    // Rotate and point towards the docking
    else if (state_ == "BLIND_TURN_TO_BAR") {
        if (!has_map) return;
        double dx = BLIND_BAR_TARGET[0] - map_pose.pose.position.x;
        double dy = BLIND_BAR_TARGET[1] - map_pose.pose.position.y;
        double target_yaw = std::atan2(dy, dx);
        double err_yaw = normalize_angle(target_yaw - yaw_map);

        if (std::abs(err_yaw) < 0.02) {
            pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
            state_ = "BLIND_MOVE_TO_BAR";
            return;
        }
        cmd.angular.z = std::clamp(1.5 * err_yaw, -0.5, 0.5);
        // If angular speed is too low, apply a minimum
        if(std::abs(cmd.angular.z) < 0.1) cmd.angular.z = (cmd.angular.z > 0 ? 0.1 : -0.1);
        pub_cmd_vel_->publish(cmd);
    }
    // Move linearly to the bar
    else if (state_ == "BLIND_MOVE_TO_BAR") {
        if (!has_map) return;
        double dx = BLIND_BAR_TARGET[0] - map_pose.pose.position.x;
        double dy = BLIND_BAR_TARGET[1] - map_pose.pose.position.y;
        double dist = std::hypot(dx, dy);

        if (dist < 0.02) {
            pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
            state_ = "BLIND_ALIGN_BAR";
            return;
        }
        double target_yaw = std::atan2(dy, dx);
        double err_yaw = normalize_angle(target_yaw - yaw_map);

        cmd.linear.x = 0.3;
        cmd.angular.z = std::clamp(1.5 * err_yaw, -0.4, 0.4);
        pub_cmd_vel_->publish(cmd);
    }
    // Align to the tag orientation
    else if (state_ == "BLIND_ALIGN_BAR") {
        if (!has_map) return;
        double err_yaw = normalize_angle(BLIND_BAR_TARGET[2] - yaw_map);

        if (std::abs(err_yaw) < 0.02) {
            pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
            state_ = "DOCKING";
            search_timer_start_ = this->now();
            RCLCPP_INFO(this->get_logger(), "📍 Bar reached. Starting visual docking...");
            return;
        }
        cmd.angular.z = std::clamp(1.0 * err_yaw, -0.5, 0.5);
        if(std::abs(cmd.angular.z) < 0.1) cmd.angular.z = (cmd.angular.z > 0 ? 0.1 : -0.1);
        pub_cmd_vel_->publish(cmd);
    }

    // Checking tag for tables
    else if (state_ == "CHECK_TAG") {
        bool is_visible = tag_found_ && (this->now() - last_aruco_received_time_).seconds() < 0.5;

        if (is_visible) {
            state_ = "DOCKING";
            RCLCPP_INFO(this->get_logger(), "👁️ Tag seen! Starting docking...");
        } 
        // Sometimes nav2 arrives but it could place with a wrong orientation and it is just sufficient to rotate.
        else {
            if ((this->now() - search_timer_start_).seconds() > 5.0) {
                state_ = "SEARCHING_TAG";
                RCLCPP_WARN(this->get_logger(), "⚠️ Tag not seen. Searching...");
            }
        }
        // In both cases, stop the robot
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        return;
    }
    else if (state_ == "SEARCHING_TAG") {
        if (tag_found_ && (this->now() - last_aruco_received_time_).seconds() < 0.5) {
            pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
            state_ = "DOCKING";
            RCLCPP_INFO(this->get_logger(), "✅ Tag found!");
            return;
        }
        cmd.angular.z = 0.3;
        pub_cmd_vel_->publish(cmd);
        return;
    }
    // Visual Docking Phase
    else if (state_ == "DOCKING") {
        // If tag is not found, stop and search for it
        if (!tag_found_ || (this->now() - last_aruco_received_time_).seconds() > 1.0) {
             if(!targeting_bar_) { state_ = "CHECK_TAG"; search_timer_start_ = this->now(); }
             pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
             return;
        }
        // Transform tag pose to robot base frame
        geometry_msgs::msg::PoseStamped tag_pose_robot_frame;
        try {
            tf_buffer_->transform(last_aruco_pose_, tag_pose_robot_frame, "base_footprint", std::chrono::milliseconds(100));
        } catch (...) { return; }

        double x = tag_pose_robot_frame.pose.position.x;
        double y = tag_pose_robot_frame.pose.position.y;
        double err_x = x - cur_stop_dist_;

        if (std::abs(err_x) < 0.02 && std::abs(y) < 0.02) {
            pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
            state_ = "ROTATING";
            rotation_initialized_ = false;
            RCLCPP_INFO(this->get_logger(), "📍 Target Docking Reached!");
            return;
        }

        double lin = 0.4 * err_x;
        if (std::abs(lin) < 0.02) lin = (lin > 0) ? 0.02 : -0.02; 

        cmd.linear.x = std::clamp(lin, -0.15, 0.15);
        cmd.angular.z = std::clamp(1.5 * std::atan2(y, x), -0.4, 0.4);
        pub_cmd_vel_->publish(cmd);
    }
    // Rotation Phase
    else if (state_ == "ROTATING") {
      double current_yaw = 0.0;
      // Get the current yaw from map frame
      if (!get_yaw_in_map_frame(current_yaw)) return;

      if (!rotation_initialized_) {
        // Initialize rotation the first time
          start_yaw_ = current_yaw;
          // Final target yaw
          target_yaw_ = normalize_angle(start_yaw_ + cur_rotation_ang_);
          rotation_initialized_ = true;
          RCLCPP_INFO(this->get_logger(), "↻ Starting rotation: start=%.3f target=%.3f (delta=%.3f)",
                      start_yaw_, target_yaw_, cur_rotation_ang_);
      }

      double error = normalize_angle(target_yaw_ - current_yaw);

      if (std::abs(error) < 0.02) {
          pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
          std::this_thread::sleep_for(200ms);
          state_ = "REVERSING";
          reverse_initialized_ = false;
          RCLCPP_INFO(this->get_logger(), "🔄 Rotation completed!");
          return;
      }
      double speed = std::clamp(1.2 * error, -0.6, 0.6);
      cmd.angular.z = speed;
      pub_cmd_vel_->publish(cmd);
    }
    // Reverse Phase to attach/detach
    else if (state_ == "REVERSING") {
        geometry_msgs::msg::PoseStamped current_pose;
        if (!get_map_pose(current_pose)) return;

        if (!reverse_initialized_) {
            start_reverse_pose_ = current_pose;
            reverse_initialized_ = true;
        }
        // Compute distance traveled in reverse
        double dist = std::hypot(
            current_pose.pose.position.x - start_reverse_pose_.pose.position.x,
            current_pose.pose.position.y - start_reverse_pose_.pose.position.y
        );

        if (cur_reverse_dist_ - dist <= 0) {
            pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
            if (targeting_bar_) handle_bar_arrival();
            else handle_table_arrival();
            return;
        }
        cmd.linear.x = -0.15;
        cmd.angular.z = 0.0;
        pub_cmd_vel_->publish(cmd);
    }
    // Go forward of the same distance as reverse to leave the table
    else if (state_ == "FORWARDING_FROM_TABLE") {
        geometry_msgs::msg::PoseStamped current_pose;
        if (!get_map_pose(current_pose)) return;

        if (!linear_initialized_) { start_linear_pose_ = current_pose; linear_initialized_ = true; }

        double dist = std::hypot(current_pose.pose.position.x - start_linear_pose_.pose.position.x,
                                 current_pose.pose.position.y - start_linear_pose_.pose.position.y);

        if (dist >= cur_reverse_dist_) {
            pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
            docking_timer_->cancel();

                RCLCPP_INFO(this->get_logger(), "📩 ORDER DELIVERED!");
                std_msgs::msg::String msg; msg.data = current_table_name_;
                pub_delivered_->publish(msg);

                state_ = "MOVING_HOME";

                go_to_pose(poses_db_["home"]);
                return;
        }
        cmd.linear.x = 0.15;
        pub_cmd_vel_->publish(cmd);
    }
  }

  void handle_bar_arrival() {
      docking_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "🍻 Arrived at bar. Attaching...");
      pub_attach_->publish(std_msgs::msg::Empty());
      std::this_thread::sleep_for(500ms);
      pub_ready_->publish(std_msgs::msg::Empty());
      state_ = "WAITING_LOAD";



      std::thread([this](){
          std::this_thread::sleep_for(1500ms);
          geometry_msgs::msg::Twist fwd; fwd.linear.x = 0.2;
          for(int i=0; i<20; i++) { pub_cmd_vel_->publish(fwd); std::this_thread::sleep_for(50ms); }
          pub_cmd_vel_->publish(geometry_msgs::msg::Twist());

          targeting_bar_ = false;
          state_ = "MOVING_TO_TABLE";

          std::string target_key = "table_" + std::to_string(this->current_target_table_);
          if (poses_db_.count(target_key)) {
              configure_table_docking(this->current_target_table_);
              go_to_pose(poses_db_[target_key]);
          } else {
              RCLCPP_ERROR(this->get_logger(), "ERROR DB: %s non found. Going Home...", target_key.c_str());
              go_to_pose(poses_db_["home"]);
          }
      }).detach();
  }

  void handle_table_arrival() {
      RCLCPP_INFO(this->get_logger(), "🔓 Arrived to table. Detaching...");
      pub_detach_->publish(std_msgs::msg::Empty());
      std::this_thread::sleep_for(500ms);
      state_ = "FORWARDING_FROM_TABLE";
      linear_initialized_ = false;
  }

  // Navigate to a given pose in the map frame using Nav2
  void go_to_pose(const std::vector<double>& pose) {
    if (!nav_client_->wait_for_action_server(2s)) {
        RCLCPP_WARN(this->get_logger(), "Nav2 action server not available");
        return;
    }
    auto goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = "map"; // Reference frame
    goal.pose.header.stamp = this->now(); 
    goal.pose.pose.position.x = pose[0];
    goal.pose.pose.position.y = pose[1];
    tf2::Quaternion q; q.setRPY(0, 0, pose[2]); // Create quaternion from yaw
    q.normalize();
    goal.pose.pose.orientation = tf2::toMsg(q); // Convert to geometry_msgs

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    // Set the result callback
    opts.result_callback = std::bind(&Fra2moBrain::nav_result_callback, this, std::placeholders::_1);
    nav_client_->async_send_goal(goal, opts);
  }

  void nav_result_callback(const GoalHandleNav::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        if (state_ == "MOVING_TO_TABLE") {
            state_ = "CHECK_TAG";
            // Start search timer
            search_timer_start_ = this->now();
            docking_timer_->reset();
            RCLCPP_INFO(this->get_logger(), "🏁 Table reached. Checking Tag...");
        } else if (state_ == "MOVING_HOME") {
            state_ = "ALIGN_HOME";
            docking_timer_->reset();
            RCLCPP_INFO(this->get_logger(), "🏠 Going Home. Alignment finished...");
        } else {
            RCLCPP_INFO(this->get_logger(), "Nav2: goal reached in state %s", state_.c_str());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Nav2: goal not succeeded or canceled");
    }
  }

}; // end class

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fra2moBrain>());
  rclcpp::shutdown();
  return 0;
}