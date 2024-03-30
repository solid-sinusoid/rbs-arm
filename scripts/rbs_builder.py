from odio_urdf import Joint
from robot import RobotConfig
from robot_builder import RobotBuilderABC
from odio_urdf import *

class RbsBuilder(RobotBuilderABC):

    def __init__(self, ndof:int, 
                 robot_name:str, 
                 parent:str) -> None:
        self.ndof = ndof
        self.robot_name = robot_name
        self.robot_type = "rbs_arm"
        self.reset(robot_name, parent)

    def reset(self, robot_name, parent) -> None:
        self._config = RobotConfig(robot_name=robot_name, robot_type=self.robot_type, parent=parent)

    @property
    def robot(self) -> RobotConfig:
        return self._config

    def base(self) -> None:
        if self.ndof % 2 != 0:
            print("The number of degrees of freedom of the robot manipulator must be even")
            return
        self._config.add_joint(
            Joint(
                Origin(xyz="0 0 0", rpy="0 0 0"),
                Parent(link=self._config.parent),
                Child(link=f"{self.robot_name}_link_0"),
                type="fixed",
                name=f"{self._config.parent}_{self.robot_name}_joint"
            ))
        self._config.add_link(0, self._config.robot_config["start_link"], "start_link")
        for i in range(1, self.ndof):
            joint = "joint_base" if i == 1 else "joint"
            link_type = "fork_link" if i % 2 != 0 else "main_link"
            self._config.add_joint(i, 
                                   self._config.robot_config[link_type][joint],
                                   link_type)
            self._config.add_link(i,
                                  self._config.robot_config[link_type]["link"],
                                  link_type)
        if self.ndof <= 2:
            link_type = "fork_link"
            self._config.add_joint(self.ndof - 1, 
                                   self._config.robot_config[link_type]["joint_base"],
                                   link_type)
            self._config.add_link(self.ndof - 1, 
                                  self._config.robot_config[link_type]["link"], 
                                  link_type)
        self._config.add_part("RobotBase")

    def gripper(self) -> None:
        return super().gripper()

    def ros2_control(self, hardware: str) -> None:
        self._config.add_interface(hardware)
        self._config.add_controller_manager(self.robot_name)
        self._config.add_part("ros2_control")

    def moveit(self) -> None:
        # self._config.add_srdf()
        self._config.add_part("MoveIt2")

