# ur_kin_py


## DH Params

The kinematics is expected params defined with the following frame:
    https://github.com/compas-dev/compas_fab/blob/main/src/compas_fab/backends/kinematics/solvers/offset_wrist_kinematics.py


These come from the website:
    https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

Example: 

    UR10_PARAMS = {
        "d1": 0.1273,
        "a2": -0.612,
        "a3": -0.5723,
        "d4": 0.163941,
        "d5": 0.1157,
        "d6": 0.0922,
    }


## UR Repos

ROS industrial indigo branch (has simple urdfs, but the meshes on UR5 don't work!)
https://github.com/ros-industrial/universal_robot/tree/indigo-devel/ur_description/urdf

ROS Industrial noetic branch (actively maintained, we are refercing the ur_kinematics packaging for moveit)
https://github.com/ros-industrial/universal_robot/tree/noetic-devel/ur_description/urdf

ROS2 Driver (Using for gz_sim, etc.)
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

RO1 Driver (Currently not using)
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver


Patch by adding ``ee_link` back into `ur_description/urdf/inc/ur_macro.xacro`

```xml
  <!-- https://github.com/ros-industrial/universal_robot/pull/506/files -->
  <!-- Link from original ur_kinematics, see notes in the ur_ros1 package-->
  <link name="${tf_prefix}ee_link"/>
  <joint name="${tf_prefix}ee_fixed_joint" type="fixed">
    <parent link="${tf_prefix}wrist_3_link" />
    <child link="${tf_prefix}ee_link" />
    <origin xyz="0 0 0" rpy="0.0 ${-pi/2.0} ${pi/2.0}" />
  </joint>
```


## verifying dh params

Kinematics are done with respect to DH params, but URDF is made with transforms..

For perfectly aligned robots, you can verify the DH params by measuring the corresponding transforms.

For calibratied robots though, there are issues!

https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/457

As mentioned in the `README.md` of `ur_kinematics`
```
**NOTE**: This does not use the calibrated robot information but the hardcoded uncalibrated robot
model which is why it is not suggested to use this.
```

## ee_link

The ik_tip of the offset_wrist kinematics is with respect to a strange frame called ee_link

Typically you would expect the z_axis to stick in/out of joint6, but the ee_link z_axis points upwards and the x_axis sticks out of joint 6.

See Notes from: https://github.com/ros-industrial/universal_robot/blob/noetic-devel/ur_kinematics/include/ur_kinematics/ur_kin.h

```cpp
// These kinematics find the tranfrom from the base link to the end effector.
// Though the raw D-H parameters specify a transform from the 0th link to the 6th link,
// offset transforms are specified in this formulation.
// To work with the raw D-H kinematics, use the inverses of the transforms below.

// Transform from base link to 0th link
// -1,  0,  0,  0
//  0, -1,  0,  0
//  0,  0,  1,  0
//  0,  0,  0,  1

// Transform from 6th link to end effector
//  0, -1,  0,  0
//  0,  0, -1,  0
//  1,  0,  0,  0
//  0,  0,  0,  1
```


**Warning** Most of the UR urdfs don't work with the closed form kinematics beacuse the ee_link was removed in https://github.com/ros-industrial/universal_robot/pull/506/files


See Change Log Notes:

```
* description: add ROS-I base and tool0 frames. Fix `#49 <https://github.com/ros-industrial/universal_robot/issues/49>`_ and `#95 <https://github.com/ros-industrial/universal_robot/issues/95>`_.
  Note that 'base' is essentially 'base_link' but rotated by 180
  degrees over the Z-axis. This is necessary as the visual and
  collision geometries appear to also have their origins rotated
  180 degrees wrt the real robot.
  'tool0' is similar to 'ee_link', but with its orientation such
  that it coincides with an all-zeros TCP setting on the UR
  controller. Users are expected to attach their own TCP frames
  to this frame, instead of updating it (see also [1]).
  [1] http://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages#Standardised_links\_.2BAC8_frames
```

You do want to support all eef tip conventions:

```xml
    <!-- https://github.com/ros-industrial/universal_robot/pull/506/files -->
    <link name="${prefix}ee_link"/>
    <joint name="${prefix}ee_fixed_joint" type="fixed">
      <parent link="${prefix}wrist_3_link" />
      <child link="${prefix}ee_link" />
      <origin xyz="0 0 0" rpy="0.0 ${-pi/2.0} ${pi/2.0}" />
    </joint>
    
    <!-- ROS-Industrial 'base' frame: base_link to UR 'Base' Coordinates transform -->
    <link name="${prefix}base"/>
    <joint name="${prefix}base_link-base_fixed_joint" type="fixed">
      <!-- Note the rotation over Z of pi radians: as base_link is REP-103
           aligned (ie: has X+ forward, Y+ left and Z+ up), this is needed
           to correctly align 'base' with the 'Base' coordinate system of
           the UR controller.
      -->
      <origin xyz="0 0 0" rpy="0 0 ${pi}"/>
      <parent link="${prefix}base_link"/>
      <child link="${prefix}base"/>
    </joint>

    <!-- ROS-Industrial 'flange' frame: attachment point for EEF models -->
    <link name="${prefix}flange" />
    <joint name="${prefix}wrist_3-flange" type="fixed">
      <parent link="${prefix}wrist_3_link" />
      <child link="${prefix}flange" />
      <origin xyz="0 0 0" rpy="0 ${-pi/2.0} ${-pi/2.0}" />
    </joint>

    <!-- ROS-Industrial 'tool0' frame: all-zeros tool frame -->
    <link name="${prefix}tool0"/>
    <joint name="${prefix}flange-tool0" type="fixed">
      <!-- default toolframe: X+ left, Y+ up, Z+ front -->
      <origin xyz="0 0 0" rpy="${pi/2.0} 0 ${pi/2.0}"/>
      <parent link="${prefix}flange"/>
      <child link="${prefix}tool0"/>
    </joint>
```

## Xacro

Xacro is provided by ros.
```python
import xacro

"/opt/ros/jazzy/lib/python3.12/site-packages/xacro/__init__.py"
```

xacrodoc is alternative that doesn't require ros!

https://github.com/adamheins/xacrodoc

## Package Naming

`ur_kinematics` is already standard ros package: https://wiki.ros.org/ur_kinematics

`ur_kin_py` is actually name for old ros1 package: https://wiki.ros.org/ur_kin_py
