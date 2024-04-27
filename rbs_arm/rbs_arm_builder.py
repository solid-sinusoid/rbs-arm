from typing import Optional
from robot_builder.robot_builder import RobotBuilderABC
from robot_builder.robot import RobotConfig
from odio_urdf import *
from robot_builder.components import JointNode, LinkNode
import importlib

class RbsBuilder(RobotBuilderABC):

    def __init__(self, ndof: int, 
                 robot_name: str, 
                 parent: str,
                 gripper_package: Optional[str] = None) -> None:
        self.ndof = ndof
        self.robot_name = robot_name
        self.robot_type = "rbs_arm"
        self.gripper_package = gripper_package
        self.hardware = ""
        self.reset(robot_name, parent)

    def reset(self, robot_prefix: str, parent: str) -> None:
        self._rc = RobotConfig(robot_name=robot_prefix, robot_type=self.robot_type, parent=parent)

    @property
    def robot(self) -> RobotConfig:
        return self._rc

    def base(self) -> None:
        if self.ndof % 2 != 0:
            print("The number of degrees of freedom of the robot manipulator must be even")
            return
        if self._rc.parent == "world":
            link00 = LinkNode(N=None, robot_name=self.robot_name,
                              link_config={}, link_name="world",
                              package_path=self._rc.robot_package_abs_path)
            self._rc.add(link00)
        link0 = LinkNode(0, self.robot_name, 
                         self._rc.robot_config["start_link"], "start_link", self._rc.robot_package_abs_path)
        joint0 = JointNode(0, self._rc.parent, link0.name, joint_config={"type": "fixed"})
        self._rc.add(joint0)
        self._rc.add(link0)



        if self.ndof <= 2:
            link_name = "fork_link"
            linkr = LinkNode(self.ndof - 1, self.robot_name, 
                             self._rc.robot_config[link_name]["link"], 
                             link_name, self._rc.robot_package_abs_path)

            jointr = JointNode(self.ndof - 1, self._rc.links[-1].name, 
                               linkr.name, joint_config=self._rc.robot_config[link_name]["joint_base"])
            self._rc.add(jointr)
            self._rc.add(linkr)
        else:
            for i in range(1, self.ndof):
                joint = "joint_base" if i == 1 else "joint"
                link_name = "fork_link" if i % 2 != 0 else "main_link"
                linkr = LinkNode(i, self.robot_name, self._rc.robot_config[link_name]["link"], 
                                 link_name, self._rc.robot_package_abs_path)
                jointr = JointNode(i, self._rc.links[-1].name, 
                                   linkr.name, joint_config=self._rc.robot_config[link_name][joint])
                self._rc.add(jointr)
                self._rc.add(linkr)

        linkr = LinkNode(self.ndof, self.robot_name,
                         self._rc.robot_config["ee_link"]["link"], "ee_link",
                         self._rc.robot_package_abs_path)
        jointr = JointNode(self.ndof, self._rc.links[-1].name,
                           linkr.name, joint_config=self._rc.robot_config["ee_link"]["joint"])

        self._rc.add(jointr)
        self._rc.add(linkr)

        tool0 = LinkNode(N=None, 
                         robot_name=self.robot_name, 
                         link_config={}, 
                         link_name="tool0", 
                         package_path=self._rc.robot_package_abs_path)
        tool0_joint = JointNode(N=None, parent=linkr.name,
                                child=tool0.name,
                                joint_config={
                                "type": "fixed",
                                "origin": [0.0, 0.0, 0.11, 0.0, 0.0, 0.0]})
        self._rc.add(tool0_joint)
        self._rc.add(tool0)
        self.tool0 = tool0.name
        self._rc.add_part("RobotBase")

    def gripper(self) -> None:
        if self.gripper_package is not None:
            try:
                module = importlib.import_module(self.gripper_package)
                gripper = module.RbsGrippperBuilder(self.robot_name, self.tool0)
                gripper.base()
                self._rc.add(gripper.robot)
                self._rc.merge_children()
                self._rc.add_part("Gripper")

            except ImportError:
                print(f"Module not found")
                return None
        else:
            raise RuntimeError("Set the gripper_package first")

    def ros2_control(self, hardware: str) -> None:
        self.hardware = hardware
        self._rc.add_interface(hardware)
        is_gripper: bool = True if "Gripper" in self._rc.parts else False
        self._rc.add_controller_manager(self.robot_name, config={
            'joint_trajectory_controller': True,
            'cartesian_motion_controller': True,
            'joint_effort_controller': True,
            'gripper_controller': is_gripper
        })
        self._rc.add_part("ros2_control")

    def moveit(self) -> None:
        self._rc.generate_moveit_config(update=True)
        self._rc.add_part("MoveIt2")

    def simulation(self, simulator: Optional[str] = None) -> None:
        if simulator is None:
            if "ros2_control" in self._rc.parts:
                simulator = self.hardware
        self._rc.add_simulation(simulator)

