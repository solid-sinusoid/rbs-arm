import enum
from ament_index_python.packages import get_package_share_directory
from odio_urdf import *
import yaml
from utils import *
import numpy as np

class RobotBase:
    def __init__(self, ndof:int, robot_name:str, parent:str, include_ros2_control = True):
        self.ndof = ndof
        self.robot_name = robot_name
        self.robot_package_abs_path = get_package_share_directory(robot_name)
        self.parent = parent
        self.joint_list = []
        self.link_list = []
        self.link_connections = {}
        self.moveit_config_files = {}
        self.robot_config = load_yaml_abs(self.robot_package_abs_path + "/config/robot_config.yaml")

        if include_ros2_control == True:
            self.urdf = self._genetate_robot_with_interface()
        else:
            self.urdf = self._generate_robot_base()

        print(self._update_moveit_config())

    def _update_ros2_control_config(self):
        pass

    def _update_moveit_config(self):
        self._set_moveit_config_files()
        srdf = self._generate_srdf()
        self._update_joint_limits()
        # self._update_controllers()
        return srdf

    def _update_joint_limits(self):
        joint_limits = load_yaml_abs(self.moveit_config_files["joint_limits"]) 
        if 'joint_limits' in joint_limits:
            joints_old = list(joint_limits["joint_limits"].keys())
            if len(joints_old) == len(self.joint_list):
                for i, joint in enumerate(joints_old):
                    joint_limits["joint_limits"][self.joint_list[i]] = joint_limits["joint_limits"].pop(joint)
            else:
        write_yaml_abs(joint_limits, self.moveit_config_files["joint_limits"])

    def build_joint_limits_config(self, link_name:str):
        has_vel_limit = True if self.robot_config[link_name]["joint"]["max_velocity"] in self.robot_config[link_name]["joint"].items() else False
        config = {
            "has_velocity_limits": has_vel_limit,
            "max_velocity": 0.52000000000000002,
            "has_acceleration_limits": False,
            "max_acceleration": 0,
        }
        return config


    def _update_controllers(self):
        moveit_controllers = load_yaml_abs(self.moveit_config_files["controllers"])
        if 'moveit_simple_controller_manager' in moveit_controllers:
            controller_manager = moveit_controllers['moveit_simple_controller_manager']
            if 'joint_trajectory_controller' in controller_manager:
                controller = controller_manager['joint_trajectory_controller']
                if 'joints' in controller:
                    controller['joints'] = self.joint_list
        write_yaml_abs(moveit_controllers, self.moveit_config_files["controllers"])

    def _set_moveit_config_files(self):
        self.moveit_config_files = {
            "ompl": self.robot_package_abs_path + "/config/moveit/ompl_planning.yaml",
            "controllers": self.robot_package_abs_path + "/config/moveit/moveit_controllers.yaml",
            "joint_limits": self.robot_package_abs_path + "/config/moveit/joint_limits.yaml",
            "initial_positons": self.robot_package_abs_path + "/config/moveit/initial_positons.yaml",
            "srdf": self.robot_package_abs_path + "/config/moveit/" + self.robot_name + ".srdf"
        }

    def _generate_srdf(self):
        adjm = get_adjacency_matrix(self.link_connections, self.link_list)
        srdf = Robot(name=self.robot_name)
        plgr = PlanningGroup(
            Chain(base_link=self.link_list[0], tip_link=self.link_list[-1]),
            name = self.robot_name
        )
        srdf.extend([plgr])
        for i, parent_link in enumerate(self.link_list):
            if parent_link in self.link_connections:
                for child_link in self.link_connections[parent_link]:
                    j = self.link_list.index(child_link)
                    if adjm[i,j] == 1:
                        srdf.extend([DisableCollision(link1=parent_link, link2=child_link)])
        return srdf
        

    def _link(self, N:int, cm:list, mass:float, I:list, mesh_name:str):
        link_name = self.robot_name + "_link_" + str(N)
        self.link_list.append(link_name)
        ret = Link(
            Inertial(
                Mass(mass),
                Inertia(I),
                Origin(cm)
            ),
            Visual(
                Geometry(
                    Mesh(filename=self.robot_package_abs_path + "/meshes/visual/" + mesh_name + ".dae")
                ),
            ),
            Collision(
                Geometry(
                    Mesh(filename=self.robot_package_abs_path + "/meshes/collision/" + mesh_name + ".stl")
                )
            ),
            name= link_name
        )
        return ret

    def _joint(self, N:int, origin:list, limit:list, max_effort:float, max_velocity:float, type:str, axis:list):
        parent = self.robot_name + "_link_" + str(N - 1)
        child = self.robot_name + "_link_" + str(N)
        if parent not in self.link_connections:
            self.link_connections[parent] = []
        self.link_connections[parent].append(child)
        joint_name = self.robot_name + "_joint_" + str(N)
        self.joint_list.append(joint_name)
        ret = Joint(
            Parent(link=parent),
            Child(link=child),
            Origin(origin),
            Axis(xyz=axis),
            Limit(lower=limit[0], upper=limit[1], effort=max_effort, velocity=max_velocity),
            type=type,
            name=joint_name)
        return ret

    def _generate_robot_base_group(self):
        if self.ndof % 2 != 0:
            print_err("The number of degrees of freedom of the robot manipulator must be even")
            return Group()
        ret = Group(
            Joint(
                Origin(xyz="0 0 0", rpy="0 0 0"),
                Parent(link=self.parent), Child(link=self.robot_name + "_link_0"), 
                type="fixed",
                name=self.parent + "_" + self.robot_name + "_joint"),
            self._link(N=0, **self.robot_config["start_link"]),
        )
        if self.ndof > 2:
            for i in range(1, self.ndof):
                joint = "joint_base" if i == 1 else "joint"
                if i % 2 > 0:
                    link_config = self.robot_config["fork_link"]
                else:
                    link_config = self.robot_config["main_link"]
                ret.extend([
                    self._joint(N=i, **link_config[joint]),
                    self._link(N=i, **link_config["link"])
                ])
        else:
            link_config = self.robot_config["fork_link"]
            ret.extend([
                self._joint(N=self.ndof-1, **link_config["joint_base"]),
                self._link(N=self.ndof-1, **link_config["link"]),
            ])
        ret.extend([
            self._joint(N=self.ndof, **self.robot_config["ee_link"]["joint"]),
            self._link(N=self.ndof, **self.robot_config["ee_link"]["link"])
        ])
        return ret
    def _generate_ros2_control(self):
        ret = Group()
        for joint in self.joint_list:
            ros2_control = Ros2Control(
                    Hardware(xmltext="ign_ros2_control/IgnitionSystem"),
                    JointInterface(
                        CommandInterface(
                            Param(name="max", xmltext="1"), 
                            Param(name="min", xmltext="-1"),
                            name="position"),
                        CommandInterface(
                            Param(name="max", xmltext="1"), 
                            Param(name="min", xmltext="-1"),
                            name="velocity"),
                        StateInterface(name="position"),
                        StateInterface(name="velocity"),
                        StateInterface(name="effort"),
                        name = joint
                    ),
                    name = joint,
                    type = "actuator"
                )
            ret.extend([ros2_control])
        return ret

    def _genetate_robot_with_interface(self):
        rbs_arm = self._generate_robot_base_group()
        rbs_ros2_control = self._generate_ros2_control()
        robot_base = Robot(
            Link(self.parent),
            rbs_arm,
            rbs_ros2_control,
            name=self.robot_name,
        )
        return robot_base

    def _generate_robot_base(self):
        rbs_arm = self._generate_robot_base_group()
        robot_base = Robot(
            Link(self.parent),
            rbs_arm,
            name=self.robot_name,
        )
        return robot_base

    def get_urdf(self):
        return self.urdf
