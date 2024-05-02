from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from rbs_launch_utils.launch_common import load_yaml
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "ndof",
            default_value="6",
            description="ndof"
        )
    )
    tf_prefix = LaunchConfiguration("tf_prefix")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    hardware = LaunchConfiguration("hardware")
    ndof = LaunchConfiguration("ndof")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "arm0_moveit_gazebo.rviz"]
    )
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("rbs_arm"), "config", "rbs_arm_controllers_gazebosim.yaml"]
    )

    ndof = 6
    robot_name = "arm0"
    # namespace = "/" + robot_name
    namespace = ""
    pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    rbs_gen = RbsBuilder(ndof=ndof, robot_name=robot_name,pose=pose, parent="world", gripper_package="rbs_gripper")
    rbs_gen.base()
    rbs_gen.gripper()
    rbs_gen.ros2_control("gazebo")
    rbs_gen.simulation()
    rbs_gen.moveit()
    robot_description_content = rbs_gen.robot.urdf().urdf()
    robot_description_semantic_content = rbs_gen.robot.srdf().urdf()


    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    # robot_description_kinematics = PathJoinSubstitution(
    #     [FindPackageShare("rbs_arm"), "config", "kinematics.yaml"]
    # )


    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    kinematics_yaml = load_yaml("rbs_arm", "config/kinematics.yaml")

    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(description_package),
                    'launch',
                    'moveit_arm0.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_description': robot_description_content,
                'robot_description_semantic': robot_description_semantic_content,
                'tf_prefix': tf_prefix,
                'namespace': namespace
            }.items())


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "-c", namespace + "/controller_manager",
        ],
    )

    joint_trajectory_controller_spawner = Node(
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
        namespace=namespace,
        remappings=remappings,
        output="both",
        parameters=[robot_description],
    )


    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("rbs_arm", "config/moveit/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # namespace=namespace,
        remappings=[
        ],
        arguments=["-d", rviz_config_file],
        parameters=[
            ompl_planning_pipeline_config,
            kinematics_yaml,
            robot_description_semantic,
            robot_description,
            {"use_sim_time": True}
        ],

    )


    world_config_file = PathJoinSubstitution(
        [FindPackageShare("rbs_simulation"), "worlds", "asm2.sdf"]
    )

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                            'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r ', world_config_file])],
    )
    # Spawn robot
    gazebo_spawn_robot = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-name', robot_name,
            '-x', '0.0',
            '-z', '0.0',
            '-y', '0.0',
            '-string', robot_description_content
        ],
        output='screen'
    )

    nodes_to_start = [
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        gazebo_spawn_robot,
        moveit
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
