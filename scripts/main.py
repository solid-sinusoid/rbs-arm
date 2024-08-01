import argparse
from typing import Dict

def main(args: Dict):
        rbs_gen = RbsBuilder(ndof=args["ndof"], robot_name=args["robot-name"], parent=args["parent"], pose=pose, gripper_package=gripper_name)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    # Environment and its parameters
    parser.add_argument(
        "--path", type=str, default="", help="path to the robot directory in server DB"
    )
    parser.add_argument(
        "--robot_name", type=str, default="arm0"
    )
    parser.add_argument()

    args, unknown = parser.parse_known_args()
    main(args=args)
