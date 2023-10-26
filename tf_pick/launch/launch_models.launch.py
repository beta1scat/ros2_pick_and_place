from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.launch_context import LaunchContext


def launch_setup(context, *args, **kwargs):
    load_models = []
    model_num_min = LaunchConfiguration("min").perform(context)
    model_num_max = LaunchConfiguration("max").perform(context)
    print(model_num_min)
    print(model_num_max)
    for model in range(int(model_num_min), int(model_num_max)):
        load_models += [
            ExecuteProcess(
                cmd=["ros2 run gazebo_ros spawn_entity.py -entity {} -database cube -x 0.35 -y -0.3 -z 0.5".format(str(model))],
                shell=True,
                output="screen"
            )
        ]
    return load_models
def generate_launch_description():
    
    return LaunchDescription(
        [
            DeclareLaunchArgument(name = "min"),
            DeclareLaunchArgument(name = "max"),
        ]
        + [
            OpaqueFunction(function=launch_setup)
        ]
        # + load_models
    )
