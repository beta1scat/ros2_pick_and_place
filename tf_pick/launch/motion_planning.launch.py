import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("tf_resources_ur_description"),
            "urdf",
            "ur5_robotiq.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "tf_resources_ur_moveit_config", "config/ur5_robotiq.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "tf_resources_ur_moveit_config", "config/kinematics.yaml"
    )

    planning_yaml = load_yaml(
        "tf_resources_ur_moveit_config", "config/ompl_planning.yaml"
    )

    planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}

    return LaunchDescription(
        [
            Node(
                package="tf_pick",
                executable="motion_planning",
                name="motion_planning",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    planning_yaml,
                    planning_plugin,
                ],
            )
        ]
    )
