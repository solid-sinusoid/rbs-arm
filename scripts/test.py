from rbs_arm import RbsBuilder

ndof = 6
robot_name = "arm0"

rbs_gen = RbsBuilder(ndof=6, robot_name="arm0", parent="world", gripper_package="rbs_gripper")
rbs_gen.base()
rbs_gen.gripper()
rbs_gen.ros2_control("gazebo")
rbs_gen.simulation()
rbs_gen.moveit()
urdf = rbs_gen.robot.urdf()
srdf = rbs_gen.robot.srdf()
print(srdf)

with open(f"rbs_{robot_name}_{ndof}.urdf", 'w') as xfile:
    xfile.write(urdf.urdf())
    xfile.close()
