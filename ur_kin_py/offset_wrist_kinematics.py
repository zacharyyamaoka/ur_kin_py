#!/usr/bin/env python3


# ROS
from geometry_msgs.msg import PoseStamped

# BAM
from bam_ros_utils.msgs.geometry_converter import matrix_to_pose_stamped, pose_stamped_to_matrix
from bam_kinematics_dynamics.six_dof.offset_wrist_math import forward_kinematics_offset_wrist, inverse_kinematics_offset_wrist, IK_SOLUTION_DESCRIPTION
from bam_ros_utils.math.geometry import apply_offset

# PYTHON
import numpy as np
import copy
from typing import Tuple, List, Union

"""

Checked on Jun 2 2025

(Python) Analytic IK returned 8 raw solutions:
 Solution 0: [0.      4.93891 0.82615 1.49012 1.4753  3.14159]
 Solution 1: [0.      5.76506 5.45703 2.31627 1.4753  3.14159]
 Solution 2: [0.      4.32189 1.753   4.32189 4.80789 0.     ]
 Solution 3: [0.      6.07489 4.53019 6.07489 4.80789 0.     ]
 Solution 4: [4.02534 3.38522 1.7287  3.22151 1.18822 0.75925]
 Solution 5: [4.02534 5.11392 4.55449 4.95021 1.18822 0.75925]
 Solution 6: [4.02534 3.62578 0.85825 0.70981 5.09497 3.90084]
 Solution 7: [4.02534 4.48403 5.42494 1.56806 5.09497 3.90084]

(C++) Analytic IK returned 8 raw solutions:
  Solution 0: [0.00000, 4.93891, 0.82615, 1.49012, 1.47530, 3.14159]
  Solution 1: [0.00000, 5.76506, 5.45703, 2.31627, 1.47530, 3.14159]
  Solution 2: [0.00000, 4.32189, 1.75300, 4.32189, 4.80789, 0.00000]
  Solution 3: [0.00000, 6.07489, 4.53019, 6.07489, 4.80789, 0.00000]
  Solution 4: [4.02534, 3.38522, 1.72870, 3.22151, 1.18822, 0.75925]
  Solution 5: [4.02534, 5.11392, 4.55449, 4.95021, 1.18822, 0.75925]
  Solution 6: [4.02534, 3.62578, 0.85825, 0.70981, 5.09497, 3.90084]
  Solution 7: [4.02534, 4.48403, 5.42494, 1.56806, 5.09497, 3.90084]

"""
class OffsetWristKinematics():

    def __init__(self, dh_params: list,
                 base_link: str,
                 lower_limits:list,
                 upper_limits:list,
                 joint_reflect=[],
                 use_tool0=True,
                 verbose=False):
        
        self.dh_params = dh_params
        self.base_link = base_link
        self.upper_limits = upper_limits
        self.lower_limits = lower_limits
        self.joint_reflect = joint_reflect
        self.use_tool0 = use_tool0
        self.verbose = verbose

    def print_v(self, val):
        if self.verbose:
            print(val)

    def reflect(self, q: list) -> list:
        q_copy = copy.copy(q)
        for idx in self.joint_reflect:
            q_copy[idx] *= -1 
        return q_copy
    
    def fk(self, q: list) -> PoseStamped:

        # TODO check if q is within joint limits...
        q_reflect = self.reflect(q) # careful not to override q

        T = forward_kinematics_offset_wrist(q_reflect, self.dh_params)

        pose = matrix_to_pose_stamped(T, self.base_link)

        if self.use_tool0:
            # Transform pose from ee_link to tool0 
            pose = apply_offset(pose, rpy=[-np.pi/2, 0, -np.pi/2], local=True)

        return pose
    
    def ik(self, pose: PoseStamped, q6_des=0.0) -> Tuple[List[bool], List[List[float]]]:

        assert pose.header.frame_id == self.base_link

        if self.use_tool0:
            # Transform pose from tool0 (z sticks out of joint 6) to ee_link
            pose = apply_offset(pose, rpy=[np.pi/2, -np.pi/2, 0], local=True)

        T = pose_stamped_to_matrix(pose)

        sol_list = inverse_kinematics_offset_wrist(T, self.dh_params, q6_des)
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
                    sol_wrapped = self.reflect(sol_wrapped)
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
    from bam_descriptions import get_robot_params

    rp = get_robot_params('ur')

    K = OffsetWristKinematics(rp.dh_list, rp.base_link, rp.lower_limits, rp.upper_limits, verbose=True)

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
        pose = K.fk(q)
        success_list, ik_sol_list = K.ik(pose)

        sol_id, ik_sol = K.select_sol_by_config(success_list, ik_sol_list, config)

        assert np.allclose(np.array(q), np.array(ik_sol_list[0]))
        # print("Num Solutions: ", len(q_list))
        # for q in q_list:
        #     print(q)
    print("All tests succesfully complete")