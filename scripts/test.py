from rbs_arm import RbsBuilder

ndofs = [2, 4, 6]
gripper_names = ["rbs_gripper", None]

for gripper_name in gripper_names:
    for ndof in ndofs:
        robot_name = "arm0"
        pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        rbs_gen = RbsBuilder(ndof=ndof, robot_name="arm0", parent="world", pose=pose, gripper_package=gripper_name)
        rbs_gen.base()
        if gripper_name:
            rbs_gen.gripper()
        rbs_gen.ros2_control("gazebo")
        rbs_gen.simulation()
        rbs_gen.moveit()
        urdf = rbs_gen.robot.urdf()
        srdf = rbs_gen.robot.srdf()
        print(srdf)
        filename = ""

        if "Gripper" in rbs_gen.robot.parts:
            filename = f"rbs_{robot_name}_{ndof}_G.urdf"
            filename_srdf = f"rbs_{robot_name}_{ndof}_G.srdf"
        else:
            filename = f"rbs_{robot_name}_{ndof}.urdf"
            filename_srdf = f"rbs_{robot_name}_{ndof}.srdf"

        with open(filename, 'w') as xfile:
            xfile.write(urdf.urdf())

        with open(filename_srdf, 'w') as xfile:
            xfile.write(srdf.urdf())
