from odio_urdf import *
import numpy as np
import yaml

class Ros2Control(Element):
    allowed_elements = ["Hardware", "JointInterface", "Param", "Plugin"]
    element_name = "ros2_control"
    required_attributes = ["name", "type"]

class Hardware(Element):
    allowed_elements = ["Plugin"]
    element_name = "hardware"

class JointInterface(Element):
    required_attributes = ["name"]
    element_name = "joint"
    allowed_elements = ["CommandInterface", "StateInterface"]

class CommandInterface(Element):
    required_attributes = ["name"]
    allowed_elements = ["Param"]
    element_name = "command_interface"

class StateInterface(Element):
    required_attributes = ["name"]
    allowed_elements = ["Param"]
    element_name = "state_interface"

class Param(Element):
    element_name = "param"
    required_attributes = ["name"]

# SRDF parameters
class DisableCollision(Element):
    required_attributes = ["link1", "link2"]
    allowed_attributes = ["reason"]
    element_name = "disable_collisions"

class PlanningGroup(Element):
    element_name = "group"
    required_attributes = ["name"]
    allowed_elements = ["Chain", "Link", "JointSRDF", "PlanningGroup"]

class PlanningGroupState(Element):
    required_attributes = ["name", "group"]
    allowed_elements = ["JointSRDF"]

class Chain(Element):
    required_attributes = ["base_link", "tip_link"]
    element_name = "chain"

class JointSRDF(Element):
    element_name = "joint"
    required_attributes = ["name"]
    allowed_attributes = ["value"]

class VirtualJoint(Element):
    element_name = "virtual_joint"
    required_attributes = ["name", "type", "parent_frame", "child_link"]

    def __init__(self, *args, **kwargs):

        if not 'type' in kwargs:
            kwargs['type'] = 'fixed'

        Joint_types = ['planar', 'floating', 'fixed']
        if kwargs['type'] not in Joint_types:
            raise Exception('Virtual joint type not correct')

        super(VirtualJoint, self).__init__(*args,**kwargs)

Group.allowed_elements += [ "Ros2Control", "VirtualJoint", "PlanningGroup", "PlanningGroupState", "DisableCollision" ]
Robot.allowed_elements += [ "Ros2Control", "VirtualJoint", "PlanningGroup", "PlanningGroupState", "DisableCollision" ]

# Group.allowed_elements += ["Ros2Control"]
# Robot.allowed_elements += ["Ros2Control"]


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def print_err(msg):
    print(bcolors.HEADER+"[ERROR]: "+msg+bcolors.ENDC)

def get_adjacency_matrix(link_connections: dict, link_names:list):
    adjacency_matrix = np.zeros((len(link_names), len(link_names)), dtype=int)
    for i, parent_link in enumerate(link_names):
        if parent_link in link_connections:
            for child_link in link_connections[parent_link]:
                j = link_names.index(child_link)
                adjacency_matrix[i, j] = 1
    return adjacency_matrix

def load_yaml_abs(filepath:str):
    try:
        with open((filepath), 'r') as file:
           yamlfile = yaml.safe_load(file)
    except FileNotFoundError as e:
        print_err(f"Error: {e}. Make sure the robot configuration file exists.")
        return {}
    except yaml.YAMLError as e:
        print_err(f"Error: {e}. Invalid YAML format in the robot configuration file.")
        return {}
    return yamlfile

def write_yaml_abs(data, file_path):
    try:
        with open(file_path, 'w') as file:
            yaml.dump(data, file)
        print(f"Data has been successfully written to the file: {file_path}")
    except IOError as e:
        print(f"IOError occurred while writing the file: {e}")
    except Exception as e:
        print(f"An error occurred while writing the file: {e}")

class MoveitReconfigurator:
    def __init__(self, robot_package_path, robot_name):
        self.robot_name = robot_name
        self.robot_package_abs_path = robot_package_path
        self.moveit_config_files = {
            "ompl": self.robot_package_abs_path + "/config/moveit/ompl_planning.yaml",
            "controllers": self.robot_package_abs_path + "/config/moveit/moveit_controllers.yaml",
            "joint_limits": self.robot_package_abs_path + "/config/moveit/joint_limits.yaml",
            "initial_positons": self.robot_package_abs_path + "/config/moveit/initial_positons.yaml",
            "srdf": self.robot_package_abs_path + "/config/moveit/" + self.robot_name + ".srdf"
        }
