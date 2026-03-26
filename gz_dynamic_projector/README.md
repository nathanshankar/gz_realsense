# Gazebo Dynamic Projector Plugin

## Build Status

| Distribution | Status |
| :--- | :--- |
| **ROS 2 Humble** | [![Humble Build Status](https://img.shields.io/github/actions/workflow/status/nathanshankar/gz_realsense/ros_ci_humble.yml?branch=main&label=Build)](https://github.com/nathanshankar/gz_realsense/actions/workflows/ros_ci_humble.yml) |
| **ROS 2 Jazzy** | [![Jazzy Build Status](https://img.shields.io/github/actions/workflow/status/nathanshankar/gz_realsense/ros_ci_jazzy.yml?branch=main&label=Build)](https://github.com/nathanshankar/gz_realsense/actions/workflows/ros_ci_jazzy.yml) |

## Overview
The `gz_dynamic_projector` is a Gazebo Sim system plugin designed to simulate the infrared (IR) emitter pattern typically observed in Intel RealSense™ depth cameras. In real-world sensors, these patterns provide high-contrast texture to otherwise featureless surfaces, enabling the stereo matching algorithms to calculate depth accurately.

This plugin replicates that behavior by dynamically overlaying a synthetically generated dot pattern onto the infrared camera streams based on the 3D geometry of the scene.

## Mathematical Model

### Pattern Generation
The plugin generates a semi-random distribution of points across the camera's field of view (FOV). For a stereo configuration, the pattern width is adjusted to account for the baseline disparity:
```math
pattern\_width = HFOV + |disparity\_offset|
```

Dots are distributed using a grid-based jittering approach to ensure uniform coverage while maintaining a pseudo-random appearance.

### Geometric Projection
The core of the simulation lies in projecting these dots onto the scene. For each dot defined in angular space $(\theta_h, \theta_v)$, the plugin:

1.  **Ray-Scene Intersection**: Utilizes the available depth data (from a depth sensor or the IR cameras themselves) to determine the 3D point $\mathbf{P}_{world}$ where the projected dot hits a surface.
2.  **Surface Normal Calculation**: Computes the local surface normal $\mathbf{n}$ to determine the incidence angle.
3.  **Intensity Modulation**: The brightness of the rendered dot is modulated by several physical factors:
    *   **Distance Falloff**: Intensity decreases as a function of distance from the source.
    *   **Incidence Angle**: Uses Lambert's Cosine Law where $I \propto \mathbf{n} \cdot \mathbf{l}$ (where $\mathbf{l}$ is the reverse ray direction).
    *   **Surface Albedo**: The dot brightness is blended with the underlying surface intensity to simulate realistic light interaction.
4.  **Camera Projection**: The 3D point $\mathbf{P}_{world}$ is projected back into the 2D image coordinates $(u, v)$ of each infrared camera using their respective intrinsic matrices $K$:
```math
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K \cdot P_{camera}
```

### Optical Simulation
*   **Vignetting**: A radial falloff is applied to the IR images to simulate the lens characteristics of physical infrared sensors.
*   **Ambient Light Blending**: The laser power and ambient light levels are used to calculate an "ambience score," which inversely affects the visibility of the projected dots.

## Topics

### Subscribed Topics
*   **`{sensor_topic}/image`**: The raw infrared image stream from the Gazebo camera sensors.
*   **`{sensor_topic}/camera_info`**: Camera calibration data used for the projection math.
*   **`{depth_topic}/points`**: (Optional) Point cloud data used for high-accuracy ray-scene intersection.

### Published Topics
*   **`{sensor_topic}/ir/image`**: The final processed infrared image containing the simulated emitter pattern and vignetting effects.
*   **`/color/enhanced`**: (Optional) If enabled, a processed version of the RGB stream optimized for low-light visualization.

## Configuration
The plugin is configured via SDF/URDF parameters:
*   `mode`: "stereo" or "mono".
*   `laser_power`: Controls the density and intensity of the dots (0-100).
*   `pattern_type`: Choice of "dot", "oval", or "rectangle".
*   `disparity_offset`: Physical baseline between the cameras in meters.
