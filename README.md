# gz_realsense

A ROS 2 repository containing a custom Gazebo plugin to simulate real-world like emitter patterns for the RealSense™ camera. This work is inspired by my research on reconstructing a scene populated with emitter patterns and you can see the relevant paper here: [Clarity Enhanced Active Reconstruction of Infrared Imagery for low-light enhancement](http://arxiv.org/abs/2510.04883)



https://github.com/user-attachments/assets/a8f81fb9-3d02-4354-aab1-d3d90f5e4b35


## Build Status

| Distribution | Status |
| :--- | :--- |
| **ROS 2 Humble** | [![Humble Build Status](https://img.shields.io/github/actions/workflow/status/nathanshankar/gz_realsense/ros_ci_humble.yml?branch=main&label=Build)](https://github.com/nathanshankar/gz_realsense/actions/workflows/ros_ci_humble.yml) |
| **ROS 2 Jazzy** | [![Jazzy Build Status](https://img.shields.io/github/actions/workflow/status/nathanshankar/gz_realsense/ros_ci_jazzy.yml?branch=main&label=Build)](https://github.com/nathanshankar/gz_realsense/actions/workflows/ros_ci_jazzy.yml) |

---

## Packages

### 1. Gazebo Dynamic Projector Plugin (`gz_dynamic_projector`)
A Gazebo Sim system plugin that simulates the infrared (IR) emitter pattern of Intel RealSense™ depth cameras. It provides high-contrast texture to featureless surfaces, enabling accurate stereo matching in simulation.

#### Mathematical Model
*   **Pattern Generation**: Generates a semi-random distribution of points using grid-based jittering.

```math
pattern\_width = HFOV + |disparity\_offset|
```
*   **Geometric Projection**: Projects dots onto the scene using ray-scene intersection and surface normal calculation.
*   **Intensity Modulation**: Brightness is modulated by distance falloff, incidence angle (Lambert's Cosine Law), and surface albedo.
*   **Camera Projection**: 3D points are projected into 2D infrared image coordinates using intrinsic matrices $K$.

```math
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K \cdot P_{camera}
```
*   **Optical Simulation**: Includes radial vignetting and ambient light blending based on an "ambience score."

#### Topics
*   **Subscribed**: `{sensor_topic}/image`, `{sensor_topic}/camera_info`, `{depth_topic}/points`.
*   **Published**: `{sensor_topic}/ir/image` (with emitter pattern), `/{sensor_topic}/enhanced` .

---

### 2. RealSense Camera Simulation (`realsense_cam`)
Provides URDF models and launch configurations for the Intel RealSense D455 camera, integrated with the dynamic projector plugin.

#### Installation
Install dependencies:
```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-realsense2-*
```

#### Usage
To launch the simulation with the camera and Rviz:
```bash
ros2 launch realsense_cam depth.launch.py
```

#### URDF Integration
To include the D455 camera in your robot description:
```xml
<xacro:include filename="$(find realsense_cam)/urdf/d455.urdf.xacro"/>
```

Attach the camera to your robot's link:
```xml
<joint name="realsense_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="your_robot_link"/>
    <child link="base_screw"/>
</joint>
```

---

## License

This project is licensed under the Apache License 2.0.
