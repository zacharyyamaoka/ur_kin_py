import pinocchio as pin
import numpy as np
from pinocchio.visualize import MeshcatVisualizer
from xacrodoc import XacroDoc
import time
import meshcat.geometry as g
import meshcat.transformations as tf

print(pin.__version__)

mesh_dir = "/home/bam/python_ws/ur_kin_py/" # should be parent directory for mesh file name
urdf_model_path = "/home/bam/python_ws/ur_kin_py/ur_description/urdf/ur5e.xacro" 

doc = XacroDoc.from_file(urdf_model_path)
with doc.temp_urdf_file_path() as path:

    model, collision_model, visual_model = pin.buildModelsFromUrdf(path, mesh_dir)

viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()
viz.display(pin.neutral(model))


frame_id = model.getFrameId('ee_link')
viz.displayFrames(True, [frame_id], axis_length=0.2, axis_width=3)
viz.updateFrames()

label = "Right Gripper"
placement = viz.data.oMf[frame_id]  # SE3 object

label_pos = placement.translation + np.array([0.05, 0.05, 0.1])  # Offset text from frame

viz.viewer["labels/" + 'ee_link'].set_object(
    g.StringIO(label),
    g.MeshLambertMaterial(color=0xff0000)
)
viz.viewer["labels/" + 'ee_link'].set_transform(
    tf.translation_matrix(label_pos)
)


time.sleep(1)


def inspect_pin_model(model):
    print(model.names)
    print(model.njoints)
    print(model.nframes)
    print(model.nframes)
    data = model.createData()

    q = pin.neutral(model) 
    print("Joint Positions: ", q)
    pin.forwardKinematics(model, data, q) # set starting position
    print(("[i] {:<24} :  Position xyz".format( "Joint Name")))
    for i in range(model.njoints):
        print(("[{}] {:<24} : {: .4f} {: .4f} {: .4f}".format( i, model.names[i], *data.oMi[i].translation.T.flat )))

    print(f"[FrameId] frame_name - frame_type - parent_joint")
    for i, frame in enumerate(model.frames):
        print(f"[{i}] {frame.name} - {frame.type} - {frame.parentJoint}")


inspect_pin_model(model)