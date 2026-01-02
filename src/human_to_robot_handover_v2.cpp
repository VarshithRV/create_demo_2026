/*
This is a full client for human robot handover
1. looks at the apritag grid detector output /object0_pose
2. Starts handover automatically when the object is raised to a threshold height
3. Tracks the object right until right before grabbing it
4. Blindly grasps the last part and backs up a little bit
*/

/*
Algorithm : 

The following servers :
1. human robot handover : callback should trigger the routine from outside

The following clients : 
1. left and right preaction servers
2. left pose tracker prepare, start, stop, unprepare

The following subscribers : 
1. apriltag grid detector output pose, callback should transform the pose to a equivalent grasp target pose 
2. left pose tracker linear and angular error
3. current end effector pose

The following publishers : 
1. left pose tracker target pose

workflow : 
-   block until object pose is received
-   wait until object pose z > handover_threshold, threshold parameter
-   use the target grasp pose and the current ee pose to interpolate the published left_pose_tracker/target_pose 
    (start with lerp, but better to use something like a spline with control points based on the orientation of the object)
    interpolation is done using a velocity parameter.
-   when the linear error is less than a threshold, do a blind grasp for the last distance. blind grasp linear error parameter
-   grasp
-   back up 10 cm in the opposite z direction
*/

#define THRESHOLD_HEIGHT 0.2

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

using namespace std::chrono_literals;

class HumanRobotHandover : public rclcpp::Node{
    public:
    HumanRobotHandover():Node("human_robot_handover_node"),threshold_height_(THRESHOLD_HEIGHT){        
        
        // subscription for object pose stamped
        object_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/apriltag_grid_detector/object0_filtered_pose",10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
                // need to learn rotation theory and Eigen C++ implementation
                // tf to planning frame which is world
                // then rotate wrt its own frame
                target_grasping_pose_ = msg;
            }
        );

        // handover server
        handover_server_ = this->create_service<std_srvs::srv::Trigger>("~/handover",
            [this](const std_srvs::srv::Trigger_Request::SharedPtr req, std_srvs::srv::Trigger_Response::SharedPtr res){
                if(target_grasping_pose_ == nullptr){
                    RCLCPP_INFO(this->get_logger(),"Object pose not recieved yet");
                    return;
                }


            }
        );

    }
    
    private:
        double threshold_height_;
        geometry_msgs::msg::PoseStamped::SharedPtr target_grasping_pose_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_subscription_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handover_server_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanRobotHandover>());
    rclcpp::shutdown();
    return 0;
}