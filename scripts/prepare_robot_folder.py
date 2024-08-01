import argparse
from rbs_arm import RbsBuilder
import shutil


def main(argv):
    rbs_gen = RbsBuilder(
        ndof=argv.ndof,
        robot_name=argv.robot_name,
        parent=argv.parent,
        pose=argv.pose,
        save_path=argv.save_path
    )
    
    rbs_gen.base()
    
    if argv.gripper_name:
        rbs_gen.gripper(gripper_package=argv.gripper_name,
                        save_geometry_path=argv.save_path)

    urdf = rbs_gen.robot.urdf().urdf()
    urdf_filename = "rbs_arm.urdf"

    if argv.save_path:
        visual, collision = rbs_gen.robot.get_geometry(True, True)
        for vis in visual:
            shutil.copy(vis, argv.save_path)
        for coll in collision:
            shutil.copy(coll, argv.save_path)
        
    with open(f"{argv.save_path}/{urdf_filename}", "w") as file:
        part1, part2 = urdf.split('?>')
        m_encoding = 'UTF-8'
        file.write(part1 + f'encoding="{m_encoding}"?>\n{part2}')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', type=str, default='arm0', help='Specify the robot name')
    parser.add_argument('--gripper-name', type=str, default="rbs_gripper", help="Gripper name to find the gripper ROS2 package")
    parser.add_argument('--parent', type=str, default='world', help='Parent link of robot')
    parser.add_argument('--ndof', type=int, default=6, help='Number Degree of Freedom of robot arm')
    parser.add_argument('--save-path', type=str, default="", help='Absolute path to store robot data')
    parser.add_argument('--pose', type=list, default=[0,0,0,0,0,0], help="List of [x, y, z, th, fi, psi]")

    args, _ = parser.parse_known_args()
    main(args)
