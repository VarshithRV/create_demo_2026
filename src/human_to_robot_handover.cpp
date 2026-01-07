#include <memory>
#include <functional>
#include <string>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <vector>
#include <cmath>
#include <future>
#include <math.h>
#include <numbers>

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
        HumanToRobotHandover(): Node("human_to_robot_handover"),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_){
            callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
          
            left_preaction_client_ = this->create_client<std_srvs::srv::Trigger>("/left_preaction_server/move_to_state",   rmw_qos_profile_services_default,callback_group_);
            right_preaction_client_ = this->create_client<std_srvs::srv::Trigger>("/right_preaction_server/move_to_state",rmw_qos_profile_services_default,callback_group_);
            prepare_left_pose_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/left_pose_tracker/prepare_tracker",rmw_qos_profile_services_default,callback_group_);
            unprepare_left_pose_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/left_pose_tracker/unprepare_tracker",rmw_qos_profile_services_default,callback_group_);
            start_left_pose_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/left_pose_tracker/start_tracker",rmw_qos_profile_services_default,callback_group_);
            stop_left_pose_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/left_pose_tracker/stop_tracker",rmw_qos_profile_services_default,callback_group_);
          
            using std::placeholders::_1;
            using std::placeholders::_2;
          
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_publisher_;
            grasp_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/grasp_pose",10);

            human_to_robot_handover_ = this->create_service<std_srvs::srv::Trigger>("~/handover",std::bind(&HumanToRobotHandover::handover_callback_, this, _1, _2),rmw_qos_profile_services_default,callback_group_);
            object_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/apriltag_grid_detector/object0_filtered_pose",10,
                [this,grasp_pose_publisher_](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
                    geometry_msgs::msg::PoseStamped pose_cam = *msg;
                  
                    // this is in the objects coordinates and not in camera coordinates
                    Eigen::Vector3d p_in(pose_cam.pose.position.x,pose_cam.pose.position.y,pose_cam.pose.position.z);
                    Eigen::Quaterniond q_in(pose_cam.pose.orientation.w,pose_cam.pose.orientation.x,pose_cam.pose.orientation.y,pose_cam.pose.orientation.z);
                    
                    // transform object_to_grasp_linear_transform_ to object frame
                    Eigen::Quaterniond offset_in_object_frame = q_in * Eigen::Quaterniond{0.0,object_to_grasp_linear_transform_.x(),object_to_grasp_linear_transform_.y(),object_to_grasp_linear_transform_.z()} * q_in.inverse();
                  
                    Eigen::Vector3d p_out = p_in + Eigen::Vector3d{offset_in_object_frame.x(),offset_in_object_frame.y(),offset_in_object_frame.z()};
                  
                    pose_cam.pose.position.x = p_out.x();
                    pose_cam.pose.position.y = p_out.y();
                    pose_cam.pose.position.z = p_out.z();
                  
                    q_in.normalize();
                  
                    Eigen::AngleAxisd rollAngle(object_to_grasp_euler_transform_.x(), Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd pitchAngle(object_to_grasp_euler_transform_.y(), Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd yawAngle(object_to_grasp_euler_transform_.z(), Eigen::Vector3d::UnitZ());
                    Eigen::Quaterniond q_offset = yawAngle * pitchAngle * rollAngle;
                  
                    Eigen::Quaterniond q_out = q_in * q_offset;
                    q_out.normalize();
                    
                    pose_cam.pose.orientation.w = q_out.w();
                    pose_cam.pose.orientation.x = q_out.x();
                    pose_cam.pose.orientation.y = q_out.y();
                    pose_cam.pose.orientation.z = q_out.z();
                  
                    // Transform to world
                    geometry_msgs::msg::TransformStamped tf_world_cam;
                    try
                    {
                        tf_world_cam = tf_buffer_.lookupTransform(
                            world_frame_,
                            pose_cam.header.frame_id,
                            tf2::TimePointZero,// latest transform
                            tf2::durationFromSec(0.05)
                        );
                    }
                    catch (const tf2::TransformException &ex){
                        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                        return;
                    }
    
                    geometry_msgs::msg::PoseStamped pose_world;
                    tf2::doTransform(pose_cam, pose_world, tf_world_cam);
                    
                    latest_object_z_ = pose_world.pose.position.z;
                    object_seen_ = true;
                    
                    current_setpoint_pose_ = std::make_shared<geometry_msgs::msg::Pose>();
                    *current_setpoint_pose_ = pose_world.pose;
                    grasp_pose_publisher_->publish(pose_world);
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
          
            setpoint_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/left_pose_tracker/target_pose",10);
      }

    private:
        // call Trigger client synchronously (no nested spin)
        bool call_trigger_client(
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
        const std::string &name)
        {
            using namespace std::chrono_literals;
            if (!client->wait_for_service(5s)){
                RCLCPP_ERROR(this->get_logger(), "Service %s not available", name.c_str());
                return false;
            }

            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future  = client->async_send_request(request);

            // Because we are in a Reentrant callback group and using a MultiThreadedExecutor,
            // another thread can execute the response callback while we wait here.
            if (future.wait_for(10s) != std::future_status::ready){
              RCLCPP_ERROR(this->get_logger(), "Service %s call timed out", name.c_str());
              return false;
            }

            auto response = future.get();
            if (!response->success){
              RCLCPP_ERROR(
                this->get_logger(),
                "Service %s responded with failure: %s",
                name.c_str(),
                response->message.c_str());
              return false;
            }

            RCLCPP_INFO(
              this->get_logger(),
              "Service %s succeeded: %s",
              name.c_str(),
              response->message.c_str());
            return true;
        }

        // publish pose at 50hz
        void publish_pose_repeated(const geometry_msgs::msg::Pose &pose,int count){
          rclcpp::Rate rate(50.0);
          for (int i = 0; rclcpp::ok() && i < count; ++i){
            setpoint_pose_publisher_->publish(pose);
            rate.sleep();
          }
        }

        // compute blind approach pose (offset along -tool z)
        geometry_msgs::msg::Pose compute_blind_approach_pose(const geometry_msgs::msg::Pose &grasp_pose){
            geometry_msgs::msg::Pose result = grasp_pose;

            Eigen::Quaterniond q(
              grasp_pose.orientation.w,
              grasp_pose.orientation.x,
              grasp_pose.orientation.y,
              grasp_pose.orientation.z);
            q.normalize();
            
            Eigen::Vector3d z_axis = q * Eigen::Vector3d::UnitZ();
            
            Eigen::Vector3d p(
              grasp_pose.position.x,
              grasp_pose.position.y,
              grasp_pose.position.z);
            
            Eigen::Vector3d p_approach = p - blind_grasp_distance_ * z_axis;
            
            result.position.x = p_approach.x();
            result.position.y = p_approach.y();
            result.position.z = p_approach.z();
            
            return result;
        }

        // compute move-back pose (along tool z from grasp pose)
        geometry_msgs::msg::Pose compute_move_back_pose(const geometry_msgs::msg::Pose &grasp_pose){
            geometry_msgs::msg::Pose result = grasp_pose;

            Eigen::Quaterniond q(
              grasp_pose.orientation.w,
              grasp_pose.orientation.x,
              grasp_pose.orientation.y,
              grasp_pose.orientation.z);
            q.normalize();
            
            Eigen::Vector3d z_axis = q * Eigen::Vector3d::UnitZ();
            
            Eigen::Vector3d p(
              grasp_pose.position.x,
              grasp_pose.position.y,
              grasp_pose.position.z);
            
            Eigen::Vector3d p_back = p - move_back_distance_ * z_axis;
            
            result.position.x = p_back.x();
            result.position.y = p_back.y();
            result.position.z = p_back.z();
            
            return result;
        }

        // full handover workflow
        void handover_callback_(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,std_srvs::srv::Trigger::Response::SharedPtr response){
            RCLCPP_INFO(this->get_logger(), "Handover sequence started");
            // move to preaction
            if (!call_trigger_client(left_preaction_client_, "left_preaction")){
              response->success = false;
              response->message = "Left preaction failed";
              return;
            }

            if (!call_trigger_client(right_preaction_client_, "right_preaction")){
              response->success = false;
              response->message = "Right preaction failed";
              return;
            }

            // Wait for an object detection
            RCLCPP_INFO(this->get_logger(), "Waiting for object detection...");
            object_seen_ = false;
            linear_error_ = 100.0;
            angular_error_ = 3.14;

            rclcpp::Rate wait_rate(50.0);
            while (rclcpp::ok() && !object_seen_){
                wait_rate.sleep();
            }
        
            if (!rclcpp::ok()){
                response->success = false;
                response->message = "Interrupted while waiting for object";
                return;
            }
        
            double initial_object_z = latest_object_z_;
            RCLCPP_INFO(this->get_logger(), "Object detected at z = %.3f", initial_object_z);
        
            // Wait till object moves up by handover_z_threshold_
            RCLCPP_INFO(this->get_logger(), "Waiting for object to move up by %.3f m", handover_z_threshold_);
            while (rclcpp::ok() && latest_object_z_ < initial_object_z + handover_z_threshold_){
                wait_rate.sleep();
            }
        
            if (!rclcpp::ok()){
                response->success = false;
                response->message = "Interrupted while waiting for object to move up";
                return;
            }
        
            RCLCPP_INFO(this->get_logger(),"Object moved up to z = %.3f (threshold: %.3f)",latest_object_z_, initial_object_z + handover_z_threshold_);
        
            if (!current_setpoint_pose_){
                RCLCPP_ERROR(this->get_logger(), "No valid current_setpoint_pose_ available");
                response->success = false;
                response->message = "No valid setpoint pose";
                return;
            }
        
            geometry_msgs::msg::Pose grasp_pose = *current_setpoint_pose_;
            geometry_msgs::msg::Pose approach_pose = compute_blind_approach_pose(grasp_pose);
        
            // Prepare tracker
            if (!call_trigger_client(prepare_left_pose_tracker_client_, "prepare_left_pose_tracker")){
                response->success = false;
                response->message = "prepare_tracker failed";
                return;
            }
        
            // Pre-feed tracker
            RCLCPP_INFO(this->get_logger(), "Pre-feeding tracker with approach pose (20 messages at 50 Hz)");
            publish_pose_repeated(approach_pose, 20);
        
            // Start tracker
            if (!call_trigger_client(start_left_pose_tracker_client_, "start_left_pose_tracker")){
                response->success = false;
                response->message = "start_tracker failed";
                return;
            }
        
            rclcpp::Rate track_rate(50.0);
        
            // Track approach pose
            RCLCPP_INFO(this->get_logger(), "Tracking approach pose until error < thresholds");
            auto start_time = this->now();
            while (rclcpp::ok()){
                setpoint_pose_publisher_->publish(approach_pose);
                RCLCPP_INFO(this->get_logger(),"Linear error : %f",linear_error_);
                bool linear_ok = (linear_error_ < linear_convergence_threshold_ + 0.05); // hardcoding some stuff
                bool angular_ok = (angular_error_ < angular_convergence_threshold_);
                // if (linear_ok && angular_ok)
                if (linear_ok){
                    RCLCPP_INFO(this->get_logger(),"Reached approach pose: linear_error=%.4f, angular_error=%.4f",linear_error_, angular_error_);
                    break;
                }
                if ((this->now() - start_time).seconds() > 20.0){
                    RCLCPP_WARN(this->get_logger(), "Timeout while tracking approach pose");
                    break;
                }
                track_rate.sleep();
            }
        
            if (!rclcpp::ok()){
              response->success = false;
              response->message = "Interrupted while tracking approach pose";
              return;
            }
        
            // Track grasp pose
            RCLCPP_INFO(this->get_logger(), "Tracking final grasp pose");
            start_time = this->now();
            while (rclcpp::ok()){
                setpoint_pose_publisher_->publish(grasp_pose);
                bool linear_ok = (linear_error_ < linear_convergence_threshold_);
                bool angular_ok = (angular_error_ < angular_convergence_threshold_);
                if (linear_ok && angular_ok){
                    RCLCPP_INFO(this->get_logger(),"Reached grasp pose: linear_error=%.4f, angular_error=%.4f",linear_error_, angular_error_);
                    break;
                }
              
                if ((this->now() - start_time).seconds() > 20.0){
                    RCLCPP_WARN(this->get_logger(), "Timeout while tracking grasp pose");
                    break;
                }
              
                track_rate.sleep();
            }
        
            if (!rclcpp::ok()){
                response->success = false;
                response->message = "Interrupted while tracking grasp pose";
                return;
            }
        
            // Hold grasp pose
            RCLCPP_INFO(this->get_logger(), "Holding grasp pose for 1 second");
            publish_pose_repeated(grasp_pose, 50);
        
            // Move back along left_tool0 z
            geometry_msgs::msg::Pose move_back_pose = compute_move_back_pose(grasp_pose);
            RCLCPP_INFO(this->get_logger(), "Moving back along left_tool0 z axis");
            publish_pose_repeated(move_back_pose, 50);
        
            // Stop and unprepare tracker
            if (!call_trigger_client(stop_left_pose_tracker_client_, "stop_left_pose_tracker")){
                response->success = false;
                response->message = "stop_tracker failed";
                return;
            }
        
            if (!call_trigger_client(unprepare_left_pose_tracker_client_, "unprepare_left_pose_tracker")){
                response->success = false;
                response->message = "unprepare_tracker failed";
                return;
            }
        
            RCLCPP_INFO(this->get_logger(), "Handover sequence completed");
            response->success = true;
            response->message = "Handover completed successfully";
        }     

        // callback group
        rclcpp::CallbackGroup::SharedPtr callback_group_;   
        
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
        
        // publisher
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr setpoint_pose_publisher_;    
        
        // tf
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        const std::string world_frame_ = "world";   
        
        // state/ parameters
        geometry_msgs::msg::Pose::SharedPtr current_setpoint_pose_;  // world frame
        bool object_seen_ = false;
        double latest_object_z_ = 0.0;  
        double handover_z_threshold_ = 0.15;     // m
        Eigen::Vector3d object_to_grasp_linear_transform_{0.04, 0.04, -0.015};
        Eigen::Vector3d object_to_grasp_euler_transform_{0.0, -M_PI/2, 0.0};
        double blind_grasp_distance_ = 0.07;     // m
        double move_back_distance_ = 0.10;       // m   
        double linear_error_ = 100.0;            // m
        double angular_error_ = 3.14;            // rad 
        double linear_convergence_threshold_ = 0.05;   // m
        double angular_convergence_threshold_ = 0.1;  // rad (~3 deg)
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HumanToRobotHandover>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
