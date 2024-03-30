
from rbs_builder import RbsBuilder


rbs_gen = RbsBuilder(ndof=6, robot_name="arm0", parent="world")
rbs_gen.base()
rbs_gen.ros2_control("gazebo")
rbs_gen.moveit()
print(rbs_gen.robot.urdf())
print(rbs_gen.robot.srdf())
rbs_gen.robot.list_parts()
rbs_gen.robot.list_links()

