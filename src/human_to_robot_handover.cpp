#include <memory>
#include <functional>
#include <string>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <Eigen/Geometry>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class HumanToRobotHandover : public rclcpp::Node{
    public:
        HumanToRobotHandover():Node("human_to_robot_handover"){
            
            // client
            left_preaction_client_ = this->create_client<std_srvs::srv::Trigger>("/left_preaction_server/move_to_state");
            right_preaction_client_ = this->create_client<std_srvs::srv::Trigger>("/right_preaction_server/move_to_state");
            prepare_left_pose_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/left_pose_tracker/prepare_tracker");
            unprepare_left_pose_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/left_pose_tracker/unprepare_tracker");
            start_left_pose_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/left_pose_tracker/start_tracker");
            stop_left_pose_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/left_pose_tracker/stop_tracker");

            // service
            human_to_robot_handover_ = this->create_service<std_srvs::srv::Trigger>("~/handover",handover_callback_);

            // subscription
            object_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/object0_filtered_pose",10,
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
                    current_setpoint_pose_ = std::make_shared<geometry_msgs::msg::Pose>();
                                
                    // ---------------- Linear transform ----------------
                    Eigen::Vector3d p_in(
                        msg->pose.position.x,
                        msg->pose.position.y,
                        msg->pose.position.z
                    );
                
                    Eigen::Vector3d p_out = p_in + object_to_grasp_linear_transform_;
                
                    current_setpoint_pose_->position.x = p_out.x();
                    current_setpoint_pose_->position.y = p_out.y();
                    current_setpoint_pose_->position.z = p_out.z();
                
                    // ---------------- Orientation transform ----------------
                    // Input quaternion
                    Eigen::Quaterniond q_in(
                        msg->pose.orientation.w,
                        msg->pose.orientation.x,
                        msg->pose.orientation.y,
                        msg->pose.orientation.z
                    );
                
                    // Convert Euler transform (RPY) into quaternion
                    Eigen::AngleAxisd rollAngle(object_to_grasp_euler_transform_.x(), Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd pitchAngle(object_to_grasp_euler_transform_.y(), Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd yawAngle(object_to_grasp_euler_transform_.z(), Eigen::Vector3d::UnitZ());
                
                    Eigen::Quaterniond q_offset = yawAngle * pitchAngle * rollAngle;
                
                    // Compose orientation: q_out = q_in * q_offset
                    Eigen::Quaterniond q_out = q_in * q_offset;
                    q_out.normalize();
                
                    current_setpoint_pose_->orientation.w = q_out.w();
                    current_setpoint_pose_->orientation.x = q_out.x();
                    current_setpoint_pose_->orientation.y = q_out.y();
                    current_setpoint_pose_->orientation.z = q_out.z();

                    // transform this from right_camera_color_optical_frame to world frame and store in the same setpoint
                }
            );

            linear_error_subscription_ = this->create_subscription<std_msgs::msg::Float32>("/left_pose_tracker/linear_error",10,
                [this](const std_msgs::msg::Float32::SharedPtr msg){
                    linear_error_ = msg->data;
                }
            );

            angular_error_subscription_ = this->create_subscription<std_msgs::msg::Float32>("/left_pose_tracker/angular_error",10,
                [this](const std_msgs::msg::Float32::SharedPtr msg){
                    angular_error_ = msg->data;
                }
            );
            
            // publisher
            setpoint_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/left_pose_tracker/target_pose",10);

        }

        void handover_callback_(const std_srvs::srv::Trigger_Request::SharedPtr request, std_srvs::srv::Trigger_Response::SharedPtr response){
            /*
            - [ ] move to preactions
            - [ ] wait for an object detection
            - [ ] wait till object move z_threshold up
            - [ ] setpoint - 7cm in left_tool0 z axis, until error < some threshold
            - [ ] then setpoint = setpoint
            - [ ] wait for sometime
            - [ ] move back in the z direction
            */
        }
    
    private:
        // clients
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_preaction_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_preaction_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr prepare_left_pose_tracker_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr unprepare_left_pose_tracker_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_left_pose_tracker_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_left_pose_tracker_client_;

        // servers
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr human_to_robot_handover_;

        // subscriptions
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr linear_error_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angular_error_subscription_;

        // publishers
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr setpoint_pose_publisher_;

        // other internal variables
        geometry_msgs::msg::Pose::SharedPtr current_setpoint_pose_; // this should be wrt world, update this in the object pose subscriber callback
        double handover_z_threshold=0.15; //m
        Eigen::Vector3d object_to_grasp_linear_transform_ = {0.0,0.0,0.0};// initialize here
        Eigen::Vector3d object_to_grasp_euler_transform_ = {0.0,0.0,0.0};//initialize here
        double blind_grasp_distance_ = 0.07; //m
        double move_back_distance_ = 0.1; //m
        double linear_error_ = 100.0; //m
        double angular_error_ = 3.14; //rad
};

int main(){
    return 0;
}