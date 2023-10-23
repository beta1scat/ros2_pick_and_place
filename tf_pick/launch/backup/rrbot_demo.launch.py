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
    # # moveit_cpp.yaml is passed by filename for now since it's node specific
    # moveit_cpp_yaml_file_name = os.path.join(
    #     get_package_share_directory('rrbot_moveit_demo_nodes'),
    #     'config',
    #     'moveit_cpp.yaml')

    # rrbot_description_path = os.path.join(
    #     get_package_share_directory('rrbot_description'))

    # xacro_file = os.path.join(rrbot_description_path,
    #                           'urdf',
    #                           'rrbot.xacro')

    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # robot_description_config = doc.toxml()
    # robot_description = {'robot_description': robot_description_config}

    # robot_description_semantic_config = load_file('rrbot_moveit_config',
    #                                               os.path.join('config', 'rrbot.srdf'))
    # robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # kinematics_yaml = load_yaml('rrbot_moveit_config',
    #                             os.path.join('config', 'kinematics.yaml'))

    # controllers_yaml = load_yaml('rrbot_moveit_demo_nodes',
    #                              os.path.join('config', 'controllers.yaml'))
    # moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml,
    #                       'moveit_controller_manager':
    #                       'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    #                       }

    # ompl_planning_pipeline_config = {'ompl': {
    #     'planning_plugin': 'ompl_interface/OMPLPlanner',
    #     'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
    #     'start_state_max_bounds_error': 0.1}}
    # ompl_planning_yaml = load_yaml('rrbot_moveit_config',
    #                                os.path.join('config', 'ompl_planning.yaml'))
    # ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # # MoveItCpp demo executable
    # run_moveit_cpp_node = Node(name='run_moveit_cpp',
    #                            package='rrbot_moveit_demo_nodes',
    #                            # TODO(henningkayser): add debug argument
    #                            # prefix='xterm -e gdb --args',
    #                            executable='run_moveit_cpp',
    #                            output='screen',
    #                            parameters=[moveit_cpp_yaml_file_name,
    #                                        robot_description,
    #                                        robot_description_semantic,
    #                                        kinematics_yaml,
    #                                        ompl_planning_pipeline_config,
    #                                        moveit_controllers])

    # # RViz
    # rviz_config_file = os.path.join(
    #     get_package_share_directory('rrbot_moveit_config'), 'config', 'rviz_rrbot.rviz')
    # rviz_node = Node(package='rviz2',
    #                  executable='rviz2',
    #                  name='rviz2',
    #                  output='log',
    #                  arguments=['-d', rviz_config_file],
    #                  parameters=[robot_description,
    #                              robot_description_semantic])

    # Evaluate frequently used variables
    model = "rrbot"

    # Configure robot_description
    # rrbot_description_path = os.path.join(
    #     get_package_share_directory('rrbot_description'))

    # xacro_file = os.path.join(rrbot_description_path,
    #                           'urdf',
    #                           'rrbot.xacro')

    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # robot_description_config = doc.toxml()
    # robot_description = {'robot_description': robot_description_config}

    robot_description_config = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare("rrbot_description"), "urdf/{}.xacro".format(model)]
            )
        ]
    )
    robot_description = {'robot_description': robot_description_config}

    # Robot semantics SRDF
    robot_description_semantic = {
        "robot_description_semantic": load_file("rrbot_moveit_config", "config/{}.srdf".format(model))
    }

    # Kinematics
    kinematics_yaml = load_yaml("rrbot_moveit_config", "config/kinematics.yaml")
    
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
    ompl_yaml = load_yaml("rrbot_moveit_config", "config/ompl_planning.yaml")

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
    controllers_yaml = load_yaml('rrbot_moveit_demo_nodes',
                                 os.path.join('config', 'controllers.yaml'))
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
    rviz_config = PathJoinSubstitution([FindPackageShare("rrbot_moveit_config"), "config/rviz_rrbot.rviz"])

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

    return LaunchDescription([
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rrbot_gazebo'), 'launch'), '/rrbot_world.launch.py'
        ]),
      ),
      move_group_node,
      rviz,
    ])
