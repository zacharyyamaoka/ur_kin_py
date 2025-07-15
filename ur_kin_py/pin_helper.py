import numpy as np
import pinocchio as pin


def random_q_config(model: pin.Model) -> np.ndarray:
    margin = 1e-3  # avoid values right on the limit

    lower = model.lowerPositionLimit.copy()
    upper = model.upperPositionLimit.copy()

    # Shrink the joint limits slightly
    lower += margin
    upper -= margin

    # Sample within the reduced bounds
    q = pin.randomConfiguration(model, lower, upper)
    q_wrapped = (q + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]
    return q_wrapped

def transform_between_frames(model, data, frame_a: str, frame_b: str) -> np.ndarray:
    " Assumes that forward kinematics has been called and frames updated"

    # Get frame indices
    idx_a = model.getFrameId(frame_a)
    idx_b = model.getFrameId(frame_b)

    # Get transforms
    T_world_a = data.oMf[idx_a]
    T_world_b = data.oMf[idx_b]

    # Transform from a to b
    T_a_b = T_world_a.inverse() * T_world_b

    T = np.eye(4)
    T[:3, :3] = T_a_b.rotation
    T[:3, 3] = T_a_b.translation

    return T

def axis_translation(axis: str, T: np.ndarray) -> float:
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    return T[axis_map[axis], 3]

def dh_translation(model, data, frame_a, frame_b, axis='z') -> float:
    T = transform_between_frames(model, data, frame_a, frame_b)
    return axis_translation(axis, T)

# def transform_between_frames(model, data, q: np.ndarray, frame_a: str, frame_b: str) -> np.ndarray:
#     # Update all frame placements
#     pin.framesForwardKinematics(model, data, q)

#     # Get frame IDs
#     id_a = model.getFrameId(frame_a)
#     id_b = model.getFrameId(frame_b)

#     # World (i.e., model root) to frame transforms
#     frame_a = data.oMf[id_a]
#     frame_b = data.oMf[id_b]

#     T_world_a = np.eye(4)
#     T_world_a[:3, :3] = frame_a.rotation
#     T_world_a[:3, 3] = frame_a.translation

#     T_world_b = np.eye(4)
#     T_world_b[:3, :3] = frame_b.rotation
#     T_world_b[:3, 3] = frame_b.translation

#     # Transform from A to B: T_a_b = inv(T_world_a) @ T_world_b
#     T_a_b = np.linalg.inv(T_world_a) @ T_world_b

#     return T_a_b