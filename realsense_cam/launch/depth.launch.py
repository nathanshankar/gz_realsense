import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'realsense_cam'
    file_subpath = 'urdf/test_d455_camera.urdf.xacro'

    gz_start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args': '-r ' + '/home/nathan/clearpath_dev/src/gz_dynamic_projector/worlds/dynamic_projector_world.sdf'
            }.items(),
    )

    # Add features
    node_spawn_entity = Node(package='ros_gz_sim', executable='create',
                             arguments=['-topic', '/robot_description',
                                        '-z', '0.5'],
                             output='screen')

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # joint state publisher (GUI) node
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/infra1/ir/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/infra1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/infra2/ir/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/infra2/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/color/image_raw', '/camera/color/image_raw'),
            ('/color/camera_info', '/camera/color/camera_info'),
            ('/depth/depth_image', '/camera/depth/image_raw'),
            ('/depth/camera_info', '/camera/depth/camera_info'),
            ('/infra1/ir/image', '/camera/infra1/image_raw'),
            ('/infra1/camera_info', '/camera/infra1/camera_info'),
            ('/infra2/ir/image', '/camera/infra2/image_raw'),
            ('/infra2/camera_info', '/camera/infra2/camera_info'),
            ('/depth/points', '/camera/depth/points')
        ]
    )

    # Rviz node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory(pkg_name), 'rviz', 'camera_view.rviz')]
    )

    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=False))
    ld.add_action(gz_start_world)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(ros_gz_bridge)
    # ld.add_action(node_joint_state_publisher)
    ld.add_action(node_rviz)
    return ld