import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro


from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

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

    # Command-line arguments
    tutorial_arg = DeclareLaunchArgument(
        "rviz_tutorial", default_value="False", description="Tutorial flag"
    )

    # planning_context
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
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "tf_resources_ur_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "tf_resources_ur_moveit_config", "config/ur5_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    tutorial_mode = LaunchConfiguration("rviz_tutorial")
    rviz_base = os.path.join(get_package_share_directory("tf_resources_ur_moveit_config"), "config")
    rviz_full_config = os.path.join(rviz_base, "ur5.rviz")
    # rviz_empty_config = os.path.join(rviz_base, "panda_moveit_config_demo_empty.rviz")
    rviz_node_tutorial = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", rviz_empty_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=IfCondition(tutorial_mode),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=UnlessCondition(tutorial_mode),
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

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("tf_resources_ur_moveit_config"),
        "config",
        "ur5_robotiq_ros_controllers_effort.yaml",
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

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "joint_trajectory_controller",
        "joint_trajectory_controller2",
    ]:
        load_controllers += [
            ExecuteProcess(
                # cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                # shell=True,
                # output="screen",
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
                    controller],
                output='screen'
            )
        ]

    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'joint_trajectory_controller'],
    #     output='screen'
    # )


    # Warehouse mongodb server
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
    )
    ur_world = os.path.join(
        get_package_share_directory('tf_resources_ur_gazebo'),
        'worlds',
        'ur5_pick_place.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"), 
                "launch", 
                "gazebo.launch.py"
            ])
        ),
        # launch_arguments={"-s": "libgazebo_ros_init.so", "-s": "libgazebo_ros_factory.so"}.items()
        launch_arguments={"-s": "libgazebo_ros_init.so", "-s": "libgazebo_ros_factory.so",
        'world': ur_world}.items()
    )
    # gazebo = Node(package='gazebo_ros', executable='/launch/gazebo.launch.py',
    #                     arguments=['-s libgazebo_ros_factory.so'],
    #                     output='screen')

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ur5_robotiq',
                                   '-z', '1.2'],
                        output='screen')

    # moveit_cpp.yaml is passed by filename for now since it's node specific
    moveit_cpp_yaml_file_name = (
        get_package_share_directory("tf_pick") + "/config/moveit_cpp.yaml"
    )
    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="moveit_cpp",
        package="tf_pick",
        executable="moveit_cpp",
        output="screen",
        parameters=[
            moveit_cpp_yaml_file_name,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            moveit_controllers,
        ],
    )

    gripper_init = Node(
        name="gripper_init",
        package="tf_pick",
        executable="example_gripper",
        output="screen",
        parameters=[],
    )

    return LaunchDescription(
        [
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([
            #         os.path.join(get_package_share_directory('tf_resources_ur_gazebo'), 'launch'), '/ur5_robotiq_world.launch.py'
            #     ]),
            # ),
            gazebo,
            spawn_entity,
            tutorial_arg,
            rviz_node,
            # rviz_node_tutorial,
            static_tf,
            robot_state_publisher,
            moveit_cpp_node,
            run_move_group_node,
            ros2_control_node,
            mongodb_server_node,
            gripper_init,
        ]
        + load_controllers
    )
