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
   
    # Evaluate frequently used variables
    robot_description_config = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare("tf_resources_ur_description"), "urdf/ur5_joint_limited_robot.urdf.xacro"]
            )
        ]
    )
    robot_description = {'robot_description': robot_description_config}

    # Robot semantics SRDF
    robot_description_semantic = {
        "robot_description_semantic": load_file("tf_resources_ur_moveit_config", "config/ur5.srdf")
    }

    # Kinematics
    kinematics_yaml = load_yaml("tf_resources_ur_moveit_config", "config/kinematics.yaml")
    print("kinematics_yaml", '%#x'%id(kinematics_yaml))

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
    print("ompl_yaml", '%#x'%id(ompl_yaml))

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
                                 os.path.join('config', 'ur5_controllers.yaml'))
    print("controllers_yaml", '%#x'%id(controllers_yaml))
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

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_yaml,
            planning,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )
    # RViz
    # rviz_config = PathJoinSubstitution([FindPackageShare("tf_resources_ur_moveit_config"), "launch/moveit.rviz"])

    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         kinematics_yaml,
    #         planning,
    #         use_sim_time
    #     ],
    #     arguments=[
    #         '-d', rviz_config
    #     ]
    # )

    return LaunchDescription([
    #   IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('tf_resources_ur_gazebo'), 'launch'), '/ur5_world.launch.py'
    #     ]),
    #   ),
    #   move_group_node,
    run_move_group_node,
    #   rviz,
    ])
