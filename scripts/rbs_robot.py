from ament_index_python.packages import get_package_share_directory
from odio_urdf import *
from utils import *
from moveit import MoveitReconfigurator

class RobotBase:
    def __init__(self, ndof:int, robot_name:str, parent:str, has_ros2_control=True, has_moveit=True, update=False):
        self.ndof = ndof
        self.robot_name = robot_name
        self.robot_package_abs_path = get_package_share_directory(robot_name)
        self.parent = parent
        self.joint_list = []
        self.link_list = []
        self.link_connections = {}
        self.moveit_config_files = {}
        self.robot_config = load_yaml_abs(f"{self.robot_package_abs_path}/config/robot_config.yaml")
        self.group_link_joint = {key: [] for key in self.robot_config.keys()}
        self.has_moveit = has_moveit
        self.allowed_controller_names = self.robot_config["ros2_control"]["allowed_controller_names"]
        self.ros2_control_config = f"{self.robot_package_abs_path}/config/rbs_arm_controllers.yaml"

        if has_ros2_control:
            self.urdf = self._generate_robot_with_interface()
            self._update_ros2_controllers()
        else:
            self.urdf = self._generate_robot_base()

        if self.has_moveit:
            self.moveit_config = MoveitReconfigurator(self, update)
            # print(self.moveit_config.get_srdf())
            # print( self.moveit_config.get_joint_limits() )
            # print("====================================")
            # print(self.moveit_config.get_moveit_controllers())

    def _update_ros2_controllers(self):
        ros2_control_config = load_yaml_abs(self.ros2_control_config)
        ros2_control_config = {
            self.robot_name: ros2_control_config
        }
        for section in ros2_control_config[self.robot_name]:
            if "ros__parameters" in ros2_control_config[self.robot_name][section]:
                params = ros2_control_config[self.robot_name][section]["ros__parameters"]
                if "joints" in params:
                    params["joints"] = self.joint_list

        for section in ros2_control_config[self.robot_name]:
            if "ros__parameters" in ros2_control_config[self.robot_name][section]:
                params = ros2_control_config[self.robot_name][section]["ros__parameters"]
                if "end_effector_link" in params:
                    params["end_effector_link"] = self.link_list[-1]
                if "robot_base_link" in params:
                    params["robot_base_link"] = self.link_list[0]
                if "ft_sensor_ref_link" in params:
                    params["ft_sensor_ref_link"] = f"{self.robot_name}_link_tool0"
        write_yaml_abs(ros2_control_config, self.ros2_control_config)

    def _link(self, N: int, link_config: dict, link_type: str):
        link_name = f"{self.robot_name}_link_{N}"
        self.link_list.append(link_name)
        self.group_link_joint[link_type].append(link_name)
        ret = Link(
            Inertial(
                Mass(link_config.get("mass", 0)),
                Inertia(link_config.get("I", [])),
                Origin(link_config.get("cm", []))
            ),
            Visual(
                Geometry(
                    Mesh(filename=f"{self.robot_package_abs_path}/meshes/visual/{link_config['mesh_name']}.dae")
                ),
            ),
            Collision(
                Geometry(
                    Mesh(filename=f"{self.robot_package_abs_path}/meshes/collision/{link_config['mesh_name']}.stl")
                )
            ),
            name=link_name
        )
        return ret

    def _joint(self, N: int, joint_config: dict, link_type: str):
        parent = f"{self.robot_name}_link_{N - 1}"
        child = f"{self.robot_name}_link_{N}"
        if parent not in self.link_connections:
            self.link_connections[parent] = []
        self.link_connections[parent].append(child)
        joint_name = f"{self.robot_name}_joint_{N}"
        self.joint_list.append(joint_name)
        self.group_link_joint[link_type].append(joint_name)
        ret = Joint(
            Parent(link=parent),
            Child(link=child),
            Origin(joint_config.get("origin", [])),
            Axis(xyz=joint_config.get("axis", [])),
            Limit(
                lower=joint_config.get("limit", [0, 0])[0],
                upper=joint_config.get("limit", [0, 0])[1],
                effort=joint_config.get("max_effort", 0),
                velocity=joint_config.get("max_velocity", 0)
            ),
            type=joint_config.get("type", ""),
            name=joint_name)
        return ret

    def _generate_robot_base_group(self):
        if self.ndof % 2 != 0:
            print("The number of degrees of freedom of the robot manipulator must be even")
            return Group()
        
        ret = Group(
            Joint(
                Origin(xyz="0 0 0", rpy="0 0 0"),
                Parent(link=self.parent),
                Child(link=f"{self.robot_name}_link_0"),
                type="fixed",
                name=f"{self.parent}_{self.robot_name}_joint"
            ),
            self._link(N=0, link_config=self.robot_config["start_link"], link_type="start_link")
        )

        for i in range(1, self.ndof):
            joint = "joint_base" if i == 1 else "joint"
            link_type = "fork_link" if i % 2 != 0 else "main_link"
            ret.extend([
                self._joint(N=i, joint_config=self.robot_config[link_type][joint], link_type=link_type),
                self._link(N=i, link_config=self.robot_config[link_type]["link"], link_type=link_type)
            ])

        if self.ndof <= 2:
            link_type = "fork_link"
            ret.extend([
                self._joint(N=self.ndof - 1, joint_config=self.robot_config[link_type]["joint_base"],
                            link_type=link_type),
                self._link(N=self.ndof - 1, link_config=self.robot_config[link_type]["link"], link_type=link_type),
            ])

        link_type = "ee_link"
        tool0_link_name = f"{self.robot_name}_link_tool0"
        tool0_joint_name = f"{self.robot_name}_joint_tool0"
        tool0_parent = f"{self.robot_name}_link_{self.ndof}"
        tool0_child = f"{self.robot_name}_tool0"
        ret.extend([
            self._joint(N=self.ndof, joint_config=self.robot_config[link_type]["joint"], link_type=link_type),
            self._link(N=self.ndof, link_config=self.robot_config[link_type]["link"], link_type=link_type),
            Joint(
                Origin(xyz="0 0 0.11", rpy="0 0 0"),
                Parent(tool0_parent),
                Child(tool0_child),
                type="fixed",
                name=tool0_joint_name
            ),
            Link(name=tool0_link_name)
        ])
        self.link_list.append(tool0_link_name)
        return ret

    def _generate_ros2_control(self):
        ret = Group()
        for joint in self.joint_list:
            ros2_control = Ros2Control(
                Hardware(xmltext="ign_ros2_control/IgnitionSystem"),
                JointInterface(
                    CommandInterface(
                        # Param(name="max", xmltext="1"),
                        # Param(name="min", xmltext="-1"),
                        name="position"),
                    CommandInterface(
                        # Param(name="max", xmltext="1"),
                        # Param(name="min", xmltext="-1"),
                        name="velocity"),
                    StateInterface(name="position"),
                    StateInterface(name="velocity"),
                    StateInterface(name="effort"),
                    name=joint
                ),
                name=joint,
                type="actuator"
            )
            ret.extend([ros2_control])
        return ret

    def _generate_robot_with_interface(self):
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
