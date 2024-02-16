import os
import xacro
import launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    initial_joint_controllers_file_path = os.path.join(
        get_package_share_directory('rbs_arm'), 'config', 'rbs_arm_controllers_gazebosim.yaml'
    )

    doc = xacro.process_file(os.path.join(get_package_share_directory("rbs_arm"), 'urdf', 'rbs_arm_modular.xacro'), mappings={
        "gripper_name": "rbs-gripper",
        "prefix": "",
        "hardware": "gazebo",
        "simulation_controllers": initial_joint_controllers_file_path
    })

    robot_desc = doc.toprettyxml(indent='  ')
    part1, part2 = robot_desc.split('?>')
    m_encoding = 'UTF-8'
    with open("current.urdf", 'w') as xfile:
        xfile.write(part1 + 'encoding=\"{}\"?>\n'.format(m_encoding) + part2)
        xfile.close()
    return launch.LaunchDescription()
