
# COPIED FROM: https://github.com/compas-dev/compas_fab/blob/main/src/compas_fab/backends/kinematics/solvers/offset_wrist_kinematics.py

# The following parameters for UR robots are taken from the following website:
# https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

def dh_dict_to_list(dh_dict: dict) -> list:
    return [dh_dict[key] for key in ["d1", "a2", "a3", "d4", "d5", "d6"]]


UR10_PARAMS = {
    "d1": 0.1273,
    "a2": -0.612,
    "a3": -0.5723,
    "d4": 0.163941,
    "d5": 0.1157,
    "d6": 0.0922,
}

UR10e_PARAMS = {
    "d1": 0.1807,
    "a2": -0.6127,
    "a3": -0.57155,
    "d4": 0.17415,
    "d5": 0.11985,
    "d6": 0.11655,
}

UR5_PARAMS = {
    "d1": 0.089159,
    "a2": -0.42500,
    "a3": -0.39225,
    "d4": 0.10915,
    "d5": 0.09465,
    "d6": 0.0823,
}

UR5e_PARAMS = {
    "d1": 0.1625,
    "a2": -0.425,
    "a3": -0.3922,
    "d4": 0.1333,
    "d5": 0.0997,
    "d6": 0.0996,
}


UR3_PARAMS = {
    "d1": 0.1519,
    "a2": -0.24365,
    "a3": -0.21325,
    "d4": 0.11235,
    "d5": 0.08535,
    "d6": 0.0819,
}

UR3e_PARAMS = {
    "d1": 0.15185,
    "a2": -0.24355,
    "a3": -0.2132,
    "d4": 0.13105,
    "d5": 0.08535,
    "d6": 0.0921,
}
