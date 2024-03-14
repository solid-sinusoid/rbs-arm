from rbs_robot import RobotBase
import argparse

def main(arg1):
    robot_description = RobotBase(ndof=arg1, robot_name="rbs_arm", parent="world").get_urdf()
    
    file_name = f"robot_{arg1}_dof.urdf"
    
    with open(file_name, 'w') as urdf_file:
        urdf_file.write(str(robot_description))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="generate robot description")
    
    parser.add_argument("ndof", type=int, help="ndof")
    args = parser.parse_args()
    main(args.ndof)
