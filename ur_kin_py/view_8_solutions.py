#!/usr/bin/env python3



# PYTHON
import numpy as np
import time

# BAM
from xacrodoc import XacroDoc
import pinocchio as pin
from ur_kin_py.offset_wrist_kinematics import OffsetWristKinematics, IK_SOLUTION_DESCRIPTION, solution_description_match
from ur_kin_py.offset_wrist_dh_params import UR5e_PARAMS, UR10_PARAMS
from pinocchio.visualize import MeshcatVisualizer
from ur_kin_py.pin_helper import random_q_config
from pin_utils.meshcat_client import MeshcatClient

mesh_dir = "/home/bam/python_ws/ur_kin_py/" # should be parent directory for mesh file name
xacro_path = "/home/bam/python_ws/ur_kin_py/ur_description/urdf/ur5e.xacro" 

meshcat_client = MeshcatClient.from_xacro(xacro_path, mesh_dir)

time.sleep(1)

# frame_id = model.getFrameId('ee_link')
# viz.displayFrames(True, [frame_id], axis_length=0.2, axis_width=3)
# viz.updateFrames()

dh_params = [UR5e_PARAMS["d1"], UR5e_PARAMS["a2"], UR5e_PARAMS["a3"], UR5e_PARAMS["d4"], UR5e_PARAMS["d5"], UR5e_PARAMS["d6"]]
K = OffsetWristKinematics(dh_params, use_tool0=True, verbose=False)



# # q_list = [
# # [np.pi/4, 0, 0, 0, 0, 0],
# # [0, np.pi/4, 0, 0, 0, 0],
# # [0, 0, np.pi/4, 0, 0, 0],
# # [0, 0, 0, np.pi/4, 0, 0],
# # [0, 0, 0, 0, np.pi/4, 0],
# # [0, 0, 0, 0, 0, np.pi/4],
# # ]

# # for q in q_list:
# #     dummy_meshcat.display(np.array(K_dh.reflect(q)))
# #     meshcat_client.display(np.array(q))  
# #     input("Press Enter to Continue")

for i in range(10):
    if i == 0:
        q = pin.neutral(model)
        # q_random = K_urdf.random_config()
        q = np.array([np.pi/4, np.pi/4, np.pi/4, 0, 0, 0])

    else:
        q = random_q_config(model)


    # Verify FK and IK are the same
    fk_success, fk_sol = K.fk(q.tolist())
    success_list, ik_sol_list = K.ik(fk_sol)

    display_only = {"Shoulder": "Left",  "Wrist": "Up",  "Elbow": "Up"}
    display_only = {"Shoulder": "Left", "Elbow": "Up"}
    display_only = {}

    print("")
    print(f"Config: {str(np.round(q, 4))}")
    for i, ik_sol in enumerate(ik_sol_list):
        print(f"    Solution {i}: {ik_sol}")
        if ik_sol is not None:
            if solution_description_match(i, display_only):
                print(IK_SOLUTION_DESCRIPTION[i])
                viz.display(np.array(ik_sol)) 
                # time.sleep(0.5)
                input("Press Enter to Continue")
        else:
            print(f"    Solution {i}: None")
