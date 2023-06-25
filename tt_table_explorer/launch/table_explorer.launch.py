#!/usr/bin/env python3
import os
import launch
from launch_ros.actions import Node

def generate_launch_description():

    # Your launch configuration here

    return launch.LaunchDescription([
        Node(
            package='tt_table_explorer',           
            executable='tt_table_explorer.py',
            name='table_explorer_node'
        )
    ])