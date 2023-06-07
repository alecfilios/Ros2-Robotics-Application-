#!/usr/bin/env python3
import rclpy

# Lifecycles
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
# Request 
from std_srvs.srv import Empty
# Tools
from threading import Thread
import time
# Movement
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult, SmoothPath
from geometry_msgs.msg import PoseStamped 
# Poses
'''
import tf2_ros
from rclpy.time import Time
'''


class TableExplorerNode(Node):
    
    # ----------------------------------------------------------------
    #                       INIT
    # ----------------------------------------------------------------
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.client = None

        # ---------------------------------------
        #               Client
        # ---------------------------------------
        # Create an empty request
        self.request = Empty.Request()
        # Start a new thread for initializing sending the request
        self.client_thread = Thread(target=self.init_client_thread_callback)
        self.client_thread.start()

        # ---------------------------------------
        #               Poses
        # ---------------------------------------
        '''
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Start a new thread for to get the poses
        self.poses_thread = Thread(target=self.get_poses_thread_callback)
        self.poses_thread.start()        
        '''


        # ---------------------------------------
        #               Nav
        # ---------------------------------------
        # Launch the ROS 2 Navigation Stack
        self.navigator = BasicNavigator()
        # Wait for navigation to fully activate. Use this line if autostart is set to true.
        self.navigator.waitUntilNav2Active()
        self.navigation_thread = Thread(target=self.reach_all_goals_thread_callback)

    # ----------------------------------------------------------------
    #                       LIFECYCLES
    # ----------------------------------------------------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")

        # Here starts the fun :)
        self.navigation_thread.start()

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS
    
    # ----------------------------------------------------------------
    #                       THREADS
    # ----------------------------------------------------------------
    def init_client_thread_callback(self):   
        # Create a client for the service
        self.client = self.create_client(Empty, '/tt_umpire/assignment2/i_feel_confident')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info('Success! Service is available..!')
    '''
    def get_poses_thread_callback(self):
        while(not self.tf_buffer.can_transform("map", "tt_table_boundary_1_link", Time())):
            self.get_logger().info('Waiting for the Goal Transforms to become available...')
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.poses = []
        for i in range(1,5):
            pose = self.tf_buffer.lookup_transform("map", "tt_table_boundary_" + str(i) + "_link", Time())
            self.poses.append(pose)
            self.get_logger().info(f'Received Goal {str(i)} succesfully..!')    
    '''


    def reach_all_goals_thread_callback(self):  

        # Helper 1
        self.reach_goal(0.0, 1.75, 0.0, 0.0, 0.0, 0.0, 1.0, False)

        # Goal 1 (-90 degrees turn)
        self.reach_goal(2.64, 1.5, 0.0, 0.0, 0.0, -0.707, 0.707, True)

        # Helper 2
        self.reach_goal(4.4, 1.5, 0.0, 0.0, 0.0, -0.707, 0.707, False)

        # Helper 3
        self.reach_goal(4.4, -1.25, 0.0, 0.0, 0.0, 1.0, 0.0, False)

        # Goal 2 (90 degrees turn)
        self.reach_goal(2.64, -1.5, 0.0, 0.0, 0.0, 0.707, 0.707, True)

        # Helper 4
        self.reach_goal(4.3, -1.25, 0.0, 0.0, 0.0, 0.0, 1.0, False)

        # Goal 3 (180 degrees turn)
        self.reach_goal(4.4, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, True)

        # Helper 5 (same as 3)
        self.reach_goal(4.4, -1.25, 0.0, 0.0, 0.0, 1.0, 0.0, False)

        # Helper 6
        self.reach_goal(0.5, 1.65, 0.0, 0.0, 0.0, 1.0, 0.0, False)

        # Goal 4 (0 degrees turn)
        self.reach_goal(0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, True)

    # ----------------------------------------------------------------
    #                       MOVEMENT/REQUESTS
    # ----------------------------------------------------------------
    def reach_goal(self, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w, isGoal):

        result = self.move_robot_to_goal(pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w)

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            # Send the request to get the sweet points
            if(isGoal):
                self.send_request()
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
        else:
            self.get_logger().info('Goal has an invalid return status!')
        #time.sleep(1.0)

    def send_request(self):
        self.get_logger().info('Sending a request...')
        # Call the service with the empty request
        self.future = self.client.call_async(self.request)
        self.get_logger().info('Request sent!')

    def move_robot_to_goal(self, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w):

        # Set the robot's goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(pos_x)
        goal_pose.pose.position.y = float(pos_y)
        goal_pose.pose.position.z = float(pos_z)
        goal_pose.pose.orientation.x = float(orient_x)
        goal_pose.pose.orientation.y = float(orient_y)
        goal_pose.pose.orientation.z = float(orient_z)
        goal_pose.pose.orientation.w = float(orient_w)
        
        # Go to the goal pose
        self.navigator.goToPose(goal_pose)

        # Keep doing stuff as long as the robot is moving towards the goal
        while not self.navigator.isTaskComplete():
            time. sleep(0.1)

        # Get the result of the movement
        result = self.navigator.getResult()
        return result

def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    table_explorer_node = TableExplorerNode('table_explorer_node')
    executor.add_node(table_explorer_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        table_explorer_node.destroy_node()
   
    
if __name__ == '__main__':
    main()