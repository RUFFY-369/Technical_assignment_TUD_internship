
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    gazebo_ros2_control_demos_path = os.path.join(
        get_package_share_directory('robotic_arm_1'))

    xacro_file = os.path.join(gazebo_ros2_control_demos_path,
                              'urdf',
                              'rrbot2_1.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters = [params]

    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'rrbot'],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    #load_joint_effort_controller = ExecuteProcess(
     #   cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
      #       'effort_controllers'],
      #  output='screen'
    #)
    
    load_joint_position_controllers = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'position_controllers'],
        output='screen'
    )
    #load_forward_command_controller = ExecuteProcess(
    #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
     #        'forward_command_controller'],
     #   output='screen'
    #)
    
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_position_controllers],
            )
        ),
        #RegisterEventHandler(
         #   event_handler=OnProcessExit(
          #      target_action=load_joint_state_controller,
          #      on_exit=[load_joint_effort_controller],
           # )
        #),
        
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])
