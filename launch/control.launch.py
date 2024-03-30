from launch.descriptions import executable
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

def launch_setup(context, *args, **kwargs):
    robot_description_content = LaunchConfiguration("robot_description")
    controllers_file = LaunchConfiguration("controllers_file")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    with_gripper_condition = LaunchConfiguration("with_gripper")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    cartesian_controllers = LaunchConfiguration("cartesian_controllers")
    namespace = LaunchConfiguration("namespace")
    
    namespace = namespace.perform(context)
    initial_joint_controllers_file_path = PathJoinSubstitution(
        [FindPackageShare("rba_arm"), "config", controllers_file]
    )
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, initial_joint_controllers_file_path],
        output="both",
        condition=IfCondition(cartesian_controllers)
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "-c", namespace + "/controller_manager",
        ],
    )
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            initial_joint_controller,
            "-c", namespace + "/controller_manager",
        ],
        condition=IfCondition(start_joint_controller) and UnlessCondition(cartesian_controllers),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            initial_joint_controller,
            "-c", namespace + "/controller_manager", "--inactive",
        ],
        condition=UnlessCondition(start_joint_controller) and IfCondition(cartesian_controllers),
    )
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "gripper_controller",
            "-c", namespace + "/controller_manager",
        ],
        condition=IfCondition(with_gripper_condition)
    )
    cartesian_motion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "cartesian_motion_controller",
            "-c", namespace + "/controller_manager",
        ],
        condition=IfCondition(cartesian_controllers)
    )

    joint_effort_controller = Node(
        package="controller_manager",
        executable = "spawner",
        namespace=namespace,
        arguments=[
            "joint_effort_controller",
            "-c", namespace + "/controller_manager", "--inactive"
        ]
    )
   
    # force_torque_sensor_broadcaster = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["force_torque_sensor_broadcaster", "-c", "/controller_manager"]
    # )
    
    nodes_to_start = [
        # control_node,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_started,
        initial_joint_controller_spawner_stopped,
        gripper_controller,
        cartesian_motion_controller_spawner,
        joint_effort_controller
    ]
    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "with_gripper",
            default_value="false",
            description="With gripper or not?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description",
            default_value="",
            description="robot description param",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="rbs_arm_gazebosim.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cartesian_controllers",
            default_value="true",
            description="Launch cartesian controllres",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="A robot's namespace",
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
