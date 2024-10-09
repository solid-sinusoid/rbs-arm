from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
# from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def create_spawner(controller_name, namespace, condition=None, inactive=False):
    args = [controller_name, "-c", f"{namespace}/controller_manager"]
    if inactive:
        args.append("--inactive")
    return Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=args,
        condition=condition
    )

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    control_strategy = LaunchConfiguration("control_strategy").perform(context)
    control_space = LaunchConfiguration("control_space").perform(context)
    has_gripper = LaunchConfiguration("has_gripper").perform(context)

    controllers_config = {
        "joint_state_broadcaster": True,
        "gripper_controller": has_gripper == "true",
        "joint_trajectory_controller": control_strategy == "position" and control_space == "joint",
        "cartesian_motion_controller": control_strategy == "position" and control_space == "task",
        "cartesian_force_controller": control_strategy == "effort" and control_space == "task",
        "joint_effort_controller": control_strategy == "effort" and control_space == "joint",
        "force_torque_sensor_broadcaster": control_strategy == "effort" and control_space == "task"
    }

    nodes_to_start = []
    for controller_name, should_start in controllers_config.items():
        if should_start:
            nodes_to_start.append(create_spawner(controller_name, namespace))

    return nodes_to_start

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="A robot's namespace"
        ),
        DeclareLaunchArgument(
            "control_space",
            default_value="task",
            choices=["task", "joint"],
            description="Control type: 'task' for Cartesian, 'joint' for joint space control."
        ),
        DeclareLaunchArgument(
            "control_strategy",
            default_value="position",
            choices=["position", "velocity", "effort"],
            description="Control strategy: 'position', 'velocity', or 'effort'."
        ),
        DeclareLaunchArgument(
            "has_gripper",
            default_value="true",
            description="Whether to activate the gripper_controller."
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
