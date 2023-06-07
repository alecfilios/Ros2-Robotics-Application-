#!/usr/bin/env python3


# Lifecycle
import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn


# Subscribe 
from sensor_msgs.msg import PointCloud2
import numpy as np
import math
import ros2_numpy as rnp

# Broadcast
import tf2_ros
from geometry_msgs.msg import TransformStamped

class BallFinderNode(Node):

  def __init__(self, node_name, **kwargs):
    super().__init__(node_name, **kwargs)

    self.ball_detected = False  # Flag to keep track of ball detection status
    self.ball_subscriber = None  # Subscriber for ball detection topic
    self.file_written = False # Initialize a boolean flag variable to track whether the file has already been written to
  
  def on_configure(self, state: State) -> TransitionCallbackReturn:  
    self.get_logger().info("on_configure() is called.")
    return TransitionCallbackReturn.SUCCESS

  def on_activate(self, state: State) -> TransitionCallbackReturn:
    self.get_logger().info("on_activate() is called.")
    
    # Transform
    self.transform_stamped = TransformStamped()
    self.transform_stamped.header.frame_id = "head_front_camera_rgb_optical_frame"
    self.transform_stamped.child_frame_id = "tt_ball_center_link"

    # Subscriber
    self.ball_subscriber = self.create_subscription(
      PointCloud2,   # Use sensor_msgs/PointCloud2 as message type for ball detection
      '/head_front_camera/depth_registered/points',  # The topic's name
      self.ball_detection_callback,
      10)
         
    # Broadcaster
    self.br = tf2_ros.TransformBroadcaster(self)

    return super().on_activate(state)
  
  def on_deactivate(self, state: State) -> TransitionCallbackReturn:
    self.get_logger().info("on_deactivate() is called.")
    return super().on_deactivate(state)

  def on_cleanup(self, state: State) -> TransitionCallbackReturn:
    self.get_logger().info('on_cleanup() is called.')
    return TransitionCallbackReturn.SUCCESS

  def on_shutdown(self, state: State) -> TransitionCallbackReturn:  
    self.get_logger().info('on_shutdown() is called.')
    return TransitionCallbackReturn.SUCCESS
  

  def ball_detection_callback(self, msg: PointCloud2):
    # Extract the necessary information from the message
    height = msg.height
    width = msg.width
    point_step = msg.point_step
    row_step = msg.row_step
    data = msg.data

    # Load the data 
    pc = rnp.numpify(msg) 
    pc = rnp.point_cloud2.split_rgb_field(pc)

    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']


    rgb=np.zeros((pc.shape[0],3))
    rgb[:,0]=pc['r']
    rgb[:,1]=pc['g']
    rgb[:,2]=pc['b']

    # Find the white points
    white_indices = np.where((rgb[:, 0] == 255) & (rgb[:, 1] == 255) & (rgb[:, 2] == 255))[0]
    white_points = points[white_indices]

    # Calculate the center of the white points
    if len(white_points) > 0:
        white_center = np.mean(white_points, axis=0)
    else:
        white_center = np.zeros(3)  # Default center if no white points are found

    # Transform
    self.transform_stamped.header.stamp = msg.header.stamp

    self.transform_stamped.transform.translation.x = float(white_center[0])
    self.transform_stamped.transform.translation.y = float(white_center[1])
    self.transform_stamped.transform.translation.z = float(white_center[2])

    self.transform_stamped.transform.rotation.x = float(1)
    self.transform_stamped.transform.rotation.y = float(1)
    self.transform_stamped.transform.rotation.z = float(1)
    self.transform_stamped.transform.rotation.w = float(1)

    # Broadcaster
    self.br.sendTransform(self.transform_stamped)

def main():
  rclpy.init()

  executor = rclpy.executors.SingleThreadedExecutor()
  ball_finder_node = BallFinderNode('ball_finder_node')
  executor.add_node(ball_finder_node)
  try:
    # Configure the Node
    #ball_finder_node.trigger_configure()

    # Activate the Node
    #ball_finder_node.trigger_activate()

    executor.spin()

  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    ball_finder_node.destroy_node()

if __name__ == '__main__':
  main()