# Copyright 2021 Open Robotics (2021)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch.actions import ExecuteProcess

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Configure robot_description
    robot_description_config = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare("tf_resources_ur_description"), "urdf/ur5_robotiq.urdf.xacro"]
            )
        ]
    )
    robot_description = {'robot_description': robot_description_config}

    # Robot semantics SRDF
    robot_description_semantic_config = load_file(
        "tf_resources_ur_moveit_config", "config/ur5_robotiq.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Kinematics
    kinematics_yaml = load_yaml("tf_resources_ur_moveit_config", "config/kinematics.yaml")
    
    # Update group name
    # kinematics_yaml["{}_arm".format(model)] = kinematics_yaml["group_name"]
    # del kinematics_yaml["group_name"]

    # # Joint limits
    # robot_description_planning = {
    #     "robot_description_planning": PathJoinSubstitution(
    #         [
    #             FindPackageShare("rrbot_moveit_config"),
    #             "config/joint_limits.yml"
    #         ]
    #     )
    # }

    # Planning
    ompl_yaml = load_yaml("tf_resources_ur_moveit_config", "config/ompl_planning.yaml")

    planning = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1
        }
    }

    # Trajectory execution
    trajectory_execution = {"allow_trajectory_execution": True,
                            "moveit_manage_controllers": True}

    # Controllers
    controllers_yaml = load_yaml('tf_resources_ur_moveit_config',
                                 os.path.join('config', 'ur5_robotiq_controllers.yaml'))
    moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml,
                          'moveit_controller_manager':
                          'moveit_simple_controller_manager/MoveItSimpleControllerManager'
                          }

    # Planning scene
    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                                         "publish_geometry_updates": True,
                                         "publish_state_updates": True,
                                         "publish_transforms_updates": True}

    # Time configuration
    use_sim_time = {"use_sim_time": True}

    # Prepare move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        arguments=["--ros-args"],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            # robot_description_planning,
            ompl_yaml,
            planning,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            use_sim_time
        ]
    )

    # RViz
    rviz_config = PathJoinSubstitution([FindPackageShare("tf_resources_ur_moveit_config"), "config/ur5.rviz"])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning,
            use_sim_time
        ],
        arguments=[
            '-d', rviz_config
        ]
    )
    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("tf_resources_ur_moveit_config"),
        "config",
        "ur5_robotiq_ros_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'joint_trajectory_controller'],
        output='screen'
    )

    load_joint_trajectory_controller2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'joint_trajectory_controller2'],
        output='screen'
    )    
    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    return LaunchDescription([ 
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(get_package_share_directory('tf_resources_ur_gazebo'), 'launch'), '/ur5_robotiq_world.launch.py'
        #     ]),
        # ),
        rviz,
        static_tf,
        move_group_node, 
        robot_state_publisher,
        ros2_control_node,
        load_joint_state_controller,
        load_joint_trajectory_controller,
        load_joint_trajectory_controller2,
    ])
