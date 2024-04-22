from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from rbs_arm import RbsBuilder

def generate_launch_description():
    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="rbs_arm",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="rbs_arm_modular.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='',
            description="tf_prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware",
            default_value='',
            # choices=["mock", "gazebo"],
            description="The name of hardware", 
        )
    )
    tf_prefix = LaunchConfiguration("tf_prefix")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    hardware = LaunchConfiguration("hardware")

    

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "view_robot.rviz"]
    )
    # initial_joint_controllers = PathJoinSubstitution(
    #     [FindPackageShare("rbs_arm"), "config", "rbs_arm_controllers_gazebosim.yaml"]
    # )
    namespace = "/arm0"
    robot = RbsBuilder(ndof=6, robot_name="arm0", parent="world", gripper_package="rbs_gripper")
    robot.base()
    robot.gripper()
    robot.ros2_control("gazebo")
    robot.simulation()
    robot.moveit()
    robot_description_content = robot.robot.urdf().urdf()
    # urdf = open("/home/bill-finger/rbs_ws/src/rbs_arm/rbs_arm0_6.urdf").read()
    # print(urdf)
    robot_description = {"robot_description": robot_description_content}

    # with open(f"rbs_g_arm0_6.urdf", 'w') as xfile:
    #     xfile.write(robot_description_content)
    #     xfile.close()

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "-c", namespace + "/controller_manager",
        ],
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "joint_trajectory_controller",
            "-c", namespace + "/controller_manager",
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="/arm0",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                            'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r ', "empty.sdf"])],
    )
    # Spawn robot
    gazebo_spawn_robot = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-name', "arm0",
            '-x', '0.0',
            '-z', '0.0',
            '-y', '0.0',
            '-topic', namespace + "/robot_description"
        ],
        output='screen'
    )

    nodes_to_start = [
        joint_state_broadcaster_spawner,
        joint_trajectory_controller,
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        gazebo_spawn_robot
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
