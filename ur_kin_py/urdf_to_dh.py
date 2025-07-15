#!/usr/bin/env python3

from ur_kin_py.offset_wrist_dh_params import UR5e_PARAMS

# PYTHON
import time
import numpy as np
import pinocchio as pin
from xacrodoc import XacroDoc
from pin_helper import dh_translation

mesh_dir = "/home/bam/python_ws/ur_kin_py/" # should be parent directory for mesh file name
urdf_model_path = "/home/bam/python_ws/ur_kin_py/ur_description/urdf/ur5e.xacro" 

doc = XacroDoc.from_file(urdf_model_path)
with doc.temp_urdf_file_path() as path:

    model, collision_model, visual_model = pin.buildModelsFromUrdf(path, mesh_dir)

data = model.createData()

q = pin.neutral(model) 
print("Joint Positions: ", q)
pin.forwardKinematics(model, data, q) # set starting position
print(("[i] {:<24} :  Position xyz".format( "Joint Name")))
for i in range(model.njoints):
    print(("[{}] {:<24} : {: .4f} {: .4f} {: .4f}".format( i, model.names[i], *data.oMi[i].translation.T.flat )))
print("")
print(f"[FrameId] frame_name - frame_type - parent_joint")
for i, frame in enumerate(model.frames):
    print(f"[{i}] {frame.name} - {frame.type} - {frame.parentJoint}")


pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

# check these in Rviz I guess... 
d1 = dh_translation(model, data, "base", "upper_arm_link", "z")
a2 = dh_translation(model, data, "upper_arm_link", "forearm_link", "x")
a3 = dh_translation(model, data, "forearm_link", "wrist_1_link", "x")
d4 = dh_translation(model, data, "forearm_link", "wrist_1_link", "z")
d5 = -1 * dh_translation(model, data, "wrist_1_link", "wrist_2_link", "y")
d6 = dh_translation(model, data, "wrist_2_link", "wrist_3_link", "y")

robot_name = "ur5e"

dh_params = {
    "d1": float(f"{d1:.6f}"),
    "a2": float(f"{a2:.6f}"),
    "a3": float(f"{a3:.6f}"),
    "d4": float(f"{d4:.6f}"),
    "d5": float(f"{d5:.6f}"),
    "d6": float(f"{d6:.6f}")
}
print("")
print(f"{robot_name}:")
for k, v in dh_params.items():
    print(f"  {k}: {v:.6f}")

print("")
print(f"{robot_name.upper()}_PARAMS = {{")
for k, v in dh_params.items():
    print(f'    "{k}": {v:.6f},')
print("}")


print(UR5e_PARAMS)

# # --- Step 2: Initialize TF buffer client ---
# rclpy.init()
# node = Node('ur_to_ur_dh')
# # node = Node('ur_to_ur_dh', namespace=f'bam_{os.environ.get('ROBOT_ID')}')
# tf_client = TFClient(node, spin=True)
# time.sleep(1) # let system set up 

# # --- Step 3: Extract Transforms ---

# if "ur" in robot_name:
#     print("Using UR link names")
#     tf = tf_client.get_transform("base", "upper_arm_link")
#     d1 = tf.transform.translation.z

#     tf = tf_client.get_transform("upper_arm_link", "forearm_link")
#     a2 = tf.transform.translation.x

#     tf = tf_client.get_transform("forearm_link", "wrist_1_link")
#     a3 = tf.transform.translation.x

#     tf = tf_client.get_transform("forearm_link", "wrist_1_link")
#     d4 = tf.transform.translation.z

#     tf = tf_client.get_transform("wrist_1_link", "wrist_2_link")
#     d5 = -1 * tf.transform.translation.y

#     tf = tf_client.get_transform("wrist_2_link", "wrist_3_link")
#     d6 = tf.transform.translation.y

# else:
#     print("Using BAM link names")
#     tf = tf_client.get_transform(prefix+"base_link_1", prefix+"L1_link_3")
#     d1 = tf.transform.translation.z

#     tf = tf_client.get_transform(prefix+"L1_link_3", prefix+"L2_link_4")
#     a2 = -tf.transform.translation.y

#     tf = tf_client.get_transform(prefix+"L2_link_4", prefix+"wrist_2_link_5")
#     a3 = -np.sqrt(tf.transform.translation.x**2 + tf.transform.translation.y**2)

#     tf = tf_client.get_transform(prefix+"base_link_1", prefix+"wrist_3_link_6")
#     d4 = -tf.transform.translation.y

#     tf = tf_client.get_transform(prefix+"wrist_2_link_5", prefix+"eef_frame")
#     d5 = tf.transform.translation.y

#     tf = tf_client.get_transform(prefix+"wrist_3_link_6", prefix+"eef_frame")
#     d6 = tf.transform.translation.y

#     # Verify Elbow is Straight
#     tf = tf_client.get_transform(prefix+"L1_link_3", prefix+"wrist_2_link_5")
#     is_straight = np.abs(tf.transform.translation.x) < 1e-6
#     print(f"Verify Straight Elbow Straight")
#     print(f"    straight: {is_straight}")
#     print(f"    xyz: {to_numpy(tf.transform.translation)}")
#     print(f"    rpy: {euler_from_quaternion(to_numpy(tf.transform.rotation))}")

# # --- Step 4: Save YAML ---

# dh_params = {
#     "d1": float(f"{d1:.6f}"),
#     "a2": float(f"{a2:.6f}"),
#     "a3": float(f"{a3:.6f}"),
#     "d4": float(f"{d4:.6f}"),
#     "d5": float(f"{d5:.6f}"),
#     "d6": float(f"{d6:.6f}")
# }
# # print("")
# # print(f"{robot_name}:")
# # for k, v in dh_params.items():
# #     print(f"  {k}: {v:.6f}")

# print("")
# print(f"{robot_name.upper()}_PARAMS = {{")
# for k, v in dh_params.items():
#     print(f'    "{k}": {v:.6f},')
# print("}")


