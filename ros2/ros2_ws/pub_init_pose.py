#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

def main():
    rclpy.init()
    node = rclpy.create_node('init_pose_script')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    time.sleep(1.0)
    
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    # Small initial covariance
    msg.pose.covariance[0] = 0.25  # x var
    msg.pose.covariance[7] = 0.25  # y var
    msg.pose.covariance[35] = 0.068 # yaw var (~15 deg)
    
    for _ in range(5):
        pub.publish(msg)
        time.sleep(0.2)
        
    node.get_logger().info('Published initial pose [0, 0, 0] to /initialpose successfully.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
