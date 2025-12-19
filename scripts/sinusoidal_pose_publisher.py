from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
import math
import time

if __name__ == "__main__":
    rclpy.init()
    node = Node("sinusoidal_pose_publisher")
    pose_publisher = node.create_publisher(Pose,"/left_pose_tracker/target_pose",10)
    pose_stamped_publiser = node.create_publisher(PoseStamped,"target_pose",10)
    
    msg = Pose()
    msg_2 = PoseStamped()
    msg_2.header.frame_id = 'world'
    
    x = 0.0
    y = 0.0
    z = 0.0
    
    # roll pitch yaw is {pi/2,0,0}

    qx = 0.707
    qy = 0.0
    qz = 0.0
    qw = 0.707


    amplititude_x = 0.05
    amplititude_y = 0.05
    amplititude_y = 0.0
    amplititude_z = 0.05

    center_x = -0.251
    center_y = -0.249
    center_z = 0.270

    i = 0 

    while(rclpy.ok()):
        x = center_x + (amplititude_x * math.sin(i/50 + math.pi/2))
        y = center_y + (amplititude_y * math.sin(i/50))
        z = center_z + (amplititude_z * math.sin(i/50))
        
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg_2.header.stamp = node.get_clock().now().to_msg()
        msg_2.pose.position.x = x
        msg_2.pose.position.y = y
        msg_2.pose.position.z = z
        msg_2.pose.orientation.x = qx
        msg_2.pose.orientation.y = qy
        msg_2.pose.orientation.z = qz
        msg_2.pose.orientation.w = qw

        i+=1

        pose_publisher.publish(msg)
        pose_stamped_publiser.publish(msg_2)
        time.sleep(0.02)