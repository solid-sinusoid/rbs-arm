from robot_builder import Robot, Link, Joint
from robot_builder.robot_builder import RobotBuilderABC


class RbsArmBuilder(RobotBuilderABC):
    def __init__(self, name, ndof, position, config: dict):
        self.name = name
        self.ndof = ndof
        self.position = position
        self.config = config

        self.robot_config: Robot = None

    def robot(self) -> Robot:
        return self.robot_config

    def base(self) -> None:
        pass


    def gripper(self) -> None:
        return super().gripper()

