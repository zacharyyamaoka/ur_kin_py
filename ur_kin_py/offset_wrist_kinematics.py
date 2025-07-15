#!/usr/bin/env python3

from ur_kin_py.offset_wrist_math import forward_kinematics_offset_wrist, inverse_kinematics_offset_wrist, IK_SOLUTION_DESCRIPTION, solution_description_match
from ur_kin_py.math_utils import xyzrpy_to_matrix

# PYTHON
import numpy as np
import copy
from typing import Tuple, List, Union


class OffsetWristKinematics():

    def __init__(self, dh_params: list,
                 lower_limits=[-3.14]*6,
                 upper_limits=[3.14]*6,
                 joint_reflect=[],
                 use_tool0=True,
                 verbose=False):
        
        self.dh_params = dh_params
        self.upper_limits = np.asarray(upper_limits)
        self.lower_limits = np.asarray(lower_limits)

        assert len(dh_params) == 6
        assert len(lower_limits) == 6
        assert len(upper_limits) == 6

        self.joint_reflect = joint_reflect
        self.use_tool0 = use_tool0
        self.verbose = verbose

        self.T_ee_link_tool0 = xyzrpy_to_matrix([0, 0, 0], [-np.pi/2, 0, -np.pi/2])
        self.T_tool0_ee_link = xyzrpy_to_matrix([0, 0, 0], [np.pi/2, -np.pi/2, 0])


    def print_v(self, val):
        if self.verbose:
            print(val)

    def check_joint_limits(self, q: list[float]) -> bool:
        q = np.array(q) # type: ignore
        return bool(np.all((q >= self.lower_limits) & (q <= self.upper_limits)))
    
    def fk(self, q: list[float]) -> tuple[bool, np.ndarray]:

        if not self.check_joint_limits(q):
            return False, np.eye(4)

        T_base_link_ee_link = forward_kinematics_offset_wrist(q, self.dh_params)

        if self.use_tool0:
            T = T_ee_link_tool0 = T_base_link_ee_link @ self.T_ee_link_tool0
        else:
            T = T_base_link_ee_link

        return True, T
    
    def ik(self, pose_matrix: np.ndarray, q6_des=0.0) -> Tuple[List[bool], List[List[float]]]:


        if self.use_tool0:
            T_base_link_tool0 = pose_matrix
            T_base_link_ee_link = T_base_link_tool0 @ self.T_tool0_ee_link
            pose_matrix = T_base_link_ee_link

        sol_list = inverse_kinematics_offset_wrist(pose_matrix, self.dh_params, q6_des)
        wrapped_sol_list = copy.copy(sol_list)

        success_list = [False]*8

        """
        - See wrap_angle() for simple way to wrap.
        - This method is inspired by the ur_kinematics.cpp. 
        - Its more complicated, but it works for non symetrical joint angles, different joint angles on different joints, etc.
        """
        
        num_valid = sum(s is not None for s in sol_list)
        self.print_v(f"Analytic IK returned {num_valid} raw solutions:")

        for i, sol in enumerate(sol_list):

            if sol is None:
                self.print_v(f" Solution {i}: None")
                pass
                # success_list[i] = False
            else:
                self.print_v(f" Solution {i}: {np.round(np.array(sol),5)}")
                sol_wrapped = []
                valid = True

                for j, angle in enumerate(sol):
         

                    ok, angle_wrapped = self.try_wrap_within_limits(angle, self.lower_limits[j], self.upper_limits[j])
                    # print(f"Trying to wrap: {angle}, {self.upper_limits[j]}, {self.lower_limits[j]}")
                    # print(f"Output: {ok}, {angle_wrapped}")
                    if not ok:
                        valid = False
                        break
                    sol_wrapped.append(angle_wrapped)
                
                if valid:
                    wrapped_sol_list[i] = sol_wrapped
                    success_list[i] = True

        return success_list, wrapped_sol_list

    def wrap_angle(self, theta: float) -> float:
        """Wrap angle to [-π, π]. OLD WAY of doing it""" 
        return (theta + np.pi) % (2 * np.pi) - np.pi

    def within_limits(self, angle: float, lower: float, upper: float) -> bool:
        return lower <= angle <= upper

    def try_wrap_within_limits(self, angle: float, lower: float, upper: float) -> Tuple[bool, float]:
        if self.within_limits(angle, lower, upper):
            return True, angle
        elif self.within_limits(angle - 2 * np.pi, lower, upper):
            return True, angle - 2 * np.pi
        elif self.within_limits(angle + 2 * np.pi, lower, upper):
            return True, angle + 2 * np.pi
        return False, angle


    def select_sol_by_id(self, success_list: List[bool], sol_list: List[List[float]], id: int) -> Tuple[int, List[float]]:
        # TODO handel case that its not succesful....

        if success_list[id]:
            return True, sol_list[id]

        for i, success in enumerate(success_list):
            if success:
                return True, sol_list[i]
    
        return False, None

    def select_sol_by_config(self, success_list: List[bool], sol_list: List[List[float]], config: dict) -> Tuple[int, List[float]]:
        """
        Selects the IK solution that matches a desired shoulder/wrist/elbow configuration.

        Args:
            success_list: List of booleans indicating valid IK solutions.
            sol_list: List of joint configurations.
            config: Partial or Full configuration dictionary. Example: 
                - {"Shoulder": "Left", "Wrist": "Up"}
                - {"Shoulder": "Left",  "Wrist": "Up", "Elbow": "Up"}
        Returns:
            Tuple of (solution_index, joint_angles), or (None, None) if no match found.
        """

        for i, valid in enumerate(success_list):
            if not valid:
                continue
            description = IK_SOLUTION_DESCRIPTION.get(i, {})
            if all(description.get(k) == v for k, v in config.items()):
                return i, sol_list[i]

        return None, None



if __name__ == "__main__":

    upper_limits = [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]
    lower_limits = [-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi]

    UR10_PARAMS = {
        "d1": 0.1273,
        "a2": -0.612,
        "a3": -0.5723,
        "d4": 0.163941,
        "d5": 0.1157,
        "d6": 0.0922,
    }

    dh_params = [UR10_PARAMS["d1"], UR10_PARAMS["a2"], UR10_PARAMS["a3"], UR10_PARAMS["d4"], UR10_PARAMS["d5"], UR10_PARAMS["d6"]]

    K = OffsetWristKinematics(dh_params, lower_limits, upper_limits, verbose=True)

    # from Universal_Robots_ROS2_Description/config/initial_positions.yaml
    # shoulder_pan_joint: 0.0
    # shoulder_lift_joint: -1.57
    # elbow_joint: 0.0
    # wrist_1_joint: -1.57
    # wrist_2_joint: 0.0
    # wrist_3_joint: 0.0

    q1 = [0, 0, 0, 0, 0, 0]
    q2 = [0, -1.57, 0, -1.57, 0, 0]
    q3 = [0.2, -0.78, 1.4, 1.3, 2.6, -2.6] # go outside [-pi,pi]

    config = {"Shoulder": "Left", "Wrist": "Up"}
    for q in [q1,q2,q3]:
        fk_success, fk_sol = K.fk(q)
        success_list, ik_sol_list = K.ik(fk_sol)

        sol_id, ik_sol = K.select_sol_by_config(success_list, ik_sol_list, config)

        assert np.allclose(np.array(q), np.array(ik_sol_list[0]))
        # print("Num Solutions: ", len(q_list))
        # for q in q_list:
        #     print(q)
    print("All tests succesfully complete")