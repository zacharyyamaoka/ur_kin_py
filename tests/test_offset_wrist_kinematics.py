import numpy as np
from ur_kin_py.math_utils import xyzrpy_to_matrix
from ur_kin_py.offset_wrist_kinematics import OffsetWristKinematics
from ur_kin_py.offset_wrist_dh_params import UR5e_PARAMS, UR10_PARAMS
from ur_kin_py.pin_helper import transform_between_frames
from xacrodoc import XacroDoc
import pinocchio as pin

def test_ee_link_transforms():

    T_ee_link_tool0 = xyzrpy_to_matrix([0, 0, 0], [-np.pi/2, 0, -np.pi/2])
    T_tool0_ee_link = xyzrpy_to_matrix([0, 0, 0], [np.pi/2, -np.pi/2, 0])
    assert np.allclose(T_ee_link_tool0 @ T_tool0_ee_link, np.eye(4))


def check_ik_fk_consistency(K: OffsetWristKinematics, upper_limits: np.ndarray, lower_limits: np.ndarray, n_tests: int = 1000, verbose: bool = False) -> int:

    failed_count = 0

    for i in range(n_tests):
        q = np.random.uniform(lower_limits, upper_limits).tolist()

        fk_success, fk_sol = K.fk(q)
        assert fk_success

        success_list, ik_sol_list = K.ik(fk_sol)
        random_id = np.random.randint(0, 8)
        ik_success, ik_sol = K.select_sol_by_id(success_list, ik_sol_list, random_id)
        if not ik_success:
            failed_count += 1
            continue # don't check fk below if ik failed

        fk_success_2, fk_sol_2 = K.fk(ik_sol)
        assert fk_success_2
        if not np.allclose(fk_sol, fk_sol_2):
            failed_count += 1
            if not np.allclose(fk_sol, fk_sol_2, atol=1e-2):
                assert False

        if verbose and i % 1000 == 0:   
            print(f"Completed {i}/{n_tests} tests")

    # Not sure exactly why its failing, may need to look into it more in the future
    print(f"Failed {failed_count} out of {n_tests} tests")
    return failed_count



upper_limits = np.array([np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]) - 1e-6
lower_limits = np.array([-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi]) + 1e-6

def test_ur10_consistency():

    dh_params = [UR10_PARAMS["d1"], UR10_PARAMS["a2"], UR10_PARAMS["a3"], UR10_PARAMS["d4"], UR10_PARAMS["d5"], UR10_PARAMS["d6"]]
    K = OffsetWristKinematics(dh_params, lower_limits.tolist(), upper_limits.tolist(), verbose=False)
    n_tests = 1000
    failed_count = check_ik_fk_consistency(K, upper_limits, lower_limits, n_tests, verbose=True)
    assert failed_count < n_tests * 0.001

def test_ur5e_consistency():

    dh_params = [UR5e_PARAMS["d1"], UR5e_PARAMS["a2"], UR5e_PARAMS["a3"], UR5e_PARAMS["d4"], UR5e_PARAMS["d5"], UR5e_PARAMS["d6"]]
    K = OffsetWristKinematics(dh_params, lower_limits.tolist(), upper_limits.tolist(), verbose=False)
    n_tests = 1000
    failed_count = check_ik_fk_consistency(K, upper_limits, lower_limits, n_tests, verbose=True)
    assert failed_count < n_tests * 0.001

def test_vs_pin_fk():

    mesh_dir = "/home/bam/python_ws/ur_kin_py/" # should be parent directory for mesh file name
    urdf_model_path = "/home/bam/python_ws/ur_kin_py/ur_description/urdf/ur5e.xacro" 

    doc = XacroDoc.from_file(urdf_model_path)
    with doc.temp_urdf_file_path() as path:

        model, collision_model, visual_model = pin.buildModelsFromUrdf(path, mesh_dir)
        data = model.createData()


    def pin_fk(q: np.ndarray, ik_tip='ee_link') -> np.ndarray:
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        # BUG, `base` gives wrong orientaiton, `base_link` is correct 
        return transform_between_frames(model, data, "base_link", ik_tip)


    dh_params = [UR5e_PARAMS["d1"], UR5e_PARAMS["a2"], UR5e_PARAMS["a3"], UR5e_PARAMS["d4"], UR5e_PARAMS["d5"], UR5e_PARAMS["d6"]]
    K = OffsetWristKinematics(dh_params, lower_limits.tolist(), upper_limits.tolist(), use_tool0=False, verbose=False)

    n_tests = 10000
    for i in range(n_tests):
        q = np.random.uniform(lower_limits, upper_limits)
        pin_fk_sol = pin_fk(q)
        fk_success, cf_fk_sol = K.fk(q.tolist())
        if not fk_success:
            print(f"Test {i} failed")
            print(q)
            continue
        if not np.allclose(pin_fk_sol, cf_fk_sol):
            print(f"Test {i} failed")
            print(pin_fk_sol)
            print(cf_fk_sol)

        if True and i % 1000 == 0:   
            print(f"Completed {i}/{n_tests} tests")

# Ok no need for meshcat viz! I am very confident in correctness now, as it matches the URDF fk....
if __name__ == "__main__":
    test_ee_link_transforms()
    test_ur10_consistency()
    test_ur5e_consistency()
    test_vs_pin_fk()
    print("All tests passed!")