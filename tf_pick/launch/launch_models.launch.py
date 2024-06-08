
import random
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.launch_context import LaunchContext

# total number is 28, 0-27
models = [
    # Cone
    "coke_can",
    "wooden_peg",
    "beer",
    # Hollow Cone
    "tube_2_25cm",
    "round_tin_base",
    "round_tin_top",
    "plastic_cup",
    "bowl",
    "wood_cube_5cm",
    "hollow_cube_s10_t2cm",
    "hollow_cube_s10_t1cm",
    "robocup_spl_ball",
    "robocup_3Dsim_ball",
    "cricket_ball",
    # YCB
    "YCB_apple",
    "YCB_bowl",
    "YCB_chips_can",
    "YCB_coffee_can",
    "YCB_cracker_box",
    "YCB_golfball",
    "YCB_large_marker",
    "YCB_mini_soccer_ball",
    "YCB_pear_800",
    "YCB_plate",
    "YCB_pudding_box",
    "YCB_small_marker",
    "YCB_softball",
    "YCB_tuna_can",
]
def get_random_model():
    return random.choice(models)


def launch_setup(context, *args, **kwargs):
    load_models = []
    min_arg = int(LaunchConfiguration("min").perform(context))
    max_arg = int(LaunchConfiguration("max").perform(context))
    model_num_min = 27 if min_arg > 27 else min_arg
    model_num_max = 27 if max_arg > 27 else max_arg
    if model_num_min >= model_num_max:
        return []
    print(model_num_min)
    print(model_num_max)
    for i in range(model_num_min, model_num_max):
        load_models += [
        ExecuteProcess(
            cmd=[f"ros2 run gazebo_ros spawn_entity.py -entity {models[i] + '_' + str(random.randint(0, 9))} -database '{models[i]}' -x 0.35 -y 0.3 -z 0.5"],
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
