# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def get_camera_configuration(context):

    configuration_file_path = LaunchConfiguration("camera_configuration_file_path").perform(
        context
    )

    with open(configuration_file_path, "r") as f:
        return yaml.safe_load(f)


def get_driver_configuration(context):

    configuration_file_path = LaunchConfiguration("driver_configuration_file_path").perform(
        context
    )

    with open(configuration_file_path, "r") as f:
        return yaml.safe_load(f)


def get_module_profile(camera_configuration, module_name):
    module_configuration = camera_configuration[module_name]
    image_width = module_configuration["image_width"]
    image_height = module_configuration["image_height"]
    frame_rate = module_configuration["frame_rate"]
    return f'{image_width}x{image_height}x{frame_rate}'


def get_module_image_format(camera_configuration, module_name):
    return camera_configuration[module_name]["image_format"]


def launch_setup(context, *args, **kwargs):

    camera_configuration = get_camera_configuration(context)
    driver_configuration = get_driver_configuration(context)
    executable = LaunchConfiguration("executable").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)

    driver = LaunchDescription()

    parameters = {
        'camera_name':  'camera',  # camera unique name
        'camera_namespace': '',  # 'namespace for camera
        'serial_no': '',  # choose device by serial number'
        'usb_port_id': '',  # choose device by usb port id'
        'device_type': '',  # choose device by type'
        'config_file': '',  # yaml config file,
        'json_file_path': '',  # allows advanced configuration
        'initial_reset': False,
        'accelerate_gpu_with_glsl':  False,  # enable GPU acceleration with GLSL
        'rosbag_filename': '',  # A realsense bagfile to run from as a device'
        'log_level': 'info',  # debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'
        'output': 'screen',  # pipe node output [screen|log]'
        'enable_color': True,  # description': 'enable color stream
        # 'rgb_camera.color_profile': '0,0,0',  # color stream profile
        # 'rgb_camera.color_format': 'RGB8',  # color stream format
        'rgb_camera.color_profile': get_module_profile(camera_configuration, "rgb_camera"),  # color stream profile
        'rgb_camera.color_format': get_module_image_format(camera_configuration, "rgb_camera"),  # color stream format
        'rgb_camera.enable_auto_exposure': True,  # enable/disable auto exposure for color image
        'enable_depth': True,  # enable depth stream,
        'enable_infra': False,  # enable infra0 stream,
        'enable_infra1': False,  # enable infra1 stream,
        'enable_infra2': False,  # enable infra2 stream',
        # 'depth_module.depth_profile': '0,0,0',  # depth stream profile,
        # 'depth_module.depth_format': 'Z16',  # depth stream format,
        'depth_module.depth_profile': get_module_profile(camera_configuration, "depth_camera"),  # depth stream profile,
        'depth_module.depth_format': get_module_image_format(camera_configuration, "depth_camera"),  # depth stream format,
        # 'depth_module.infra_profile': '0,0,0',  # infra streams (0/1/2) profile
        # 'depth_module.infra_format': 'RGB8',  # infra0 stream format
        'depth_module.infra_profile': get_module_profile(camera_configuration, "infrared_camera"),  # infra streams (0/1/2) profile
        'depth_module.infra_format': 'RGB8',  # infra0 stream format
        'depth_module.infra1_format': get_module_image_format(camera_configuration, "infrared_camera"),  # infra1 stream format
        'depth_module.infra2_format': get_module_image_format(camera_configuration, "infrared_camera"),  # infra2 stream format
        'depth_module.enable_auto_exposure': True,  # enable/disable auto exposure for depth image
        'depth_module.exposure': 8500,  # depth module manual exposure value,
        'depth_module.gain': 16,  # Depth module manual gain value'
        'depth_module.hdr_enabled': False,  # Depth module hdr enablement flag. Used for hdr_merge filter
        # 'depth_module.exposure.1' : '7500',  # Depth module first exposure value. Used for hdr_merge filter
        # 'depth_module.gain.1': '16', # Depth module first gain value. Used for hdr_merge filter
        # 'depth_module.exposure.2': '1', # Depth module second exposure value. Used for hdr_merge filter
        # 'depth_module.gain.2': '16', #Depth module second gain value. Used for hdr_merge filter',
        'enable_sync': False,  # enable sync mode
        'enable_rgbd': False,  # enable rgbd topic,
        'enable_gyro': False,  # enable gyro stream
        'enable_accel': False,  # enable accel stream
        'gyro_fps': 0,
        'accel_fps': 0,
        'unite_imu_method': 0,  # [0-None, 1-copy, 2-linear_interpolation]
        'clip_distance': -2.,
        'angular_velocity_cov': 0.01,
        'linear_accel_cov': 0.01,
        'diagnostics_period': 0.0,  # Rate of publishing diagnostics. 0=Disabled
        'publish_tf': False,  # enable/disable publishing static & dynamic TF
        'tf_publish_rate': 0.0,  # rate in Hz for publishing dynamic TF
        'pointcloud.enable': False,
        'pointcloud.stream_filter':  2,  # texture stream for pointcloud
        'pointcloud.stream_index_filter': 0,  # texture stream index for pointcloud'
        'pointcloud.ordered_pc': False,
        'pointcloud.allow_no_texture_points': False,
        'align_depth.enable': False,  # enable align depth filter
        'colorizer.enable': False,  # enable colorizer filter
        'decimation_filter.enable': False,  # enable_decimation_filter'
        'spatial_filter.enable': False,  # enable_spatial_filter'
        'temporal_filter.enable': False,  # enable_temporal_filter'
        'disparity_filter.enable': False,  # enable_disparity_filter,
        'hole_filling_filter.enable': False,  # enable_hole_filling_filter,
        'hdr_merge.enable': False,  # hdr_merge filter enablement flag'},
        'wait_for_device_timeout': -1.,  # Timeout for waiting for device to connect (Seconds)'
        'reconnect_timeout': 6.,  # Timeout(seconds) between consequtive reconnection attempts'},
    }

    parameters.update(driver_configuration)

    # TODO factoriser les remapping
    driver_node = Node(
        package="realsense2_camera",
        executable=executable,
        output="screen",
        name="driver",
        parameters=[parameters],
        remappings=[
            ("driver/aligned_depth_to_color/camera_info", "aligned_depth_to_color/camera_info"),
            ("driver/aligned_depth_to_color/image_raw", "aligned_depth_to_color/image_raw"),
            ("driver/aligned_depth_to_color/image_raw/compressed", "aligned_depth_to_color/image_raw/compressed"),
            ("driver/aligned_depth_to_color/image_raw/compressedDepth", "aligned_depth_to_color/image_raw/compressedDepth"),
            ("driver/aligned_depth_to_color/image_raw/theora", "aligned_depth_to_color/image_raw/theora"),
            ("driver/aligned_depth_to_color/metadata", "depth/metadata"),
            ("driver/color/camera_info", "rgb/camera_info"),
            ("driver/color/image_raw", "rgb/image_raw"),
            ("driver/color/image_raw/compressed", "rgb/image_raw/compressed"),
            ("driver/color/image_raw/compressedDepth", "rgb/image_raw/compressedDepth"),
            ("driver/color/image_raw/theora", "rgb/image_raw/theora"),
            ("driver/color/metadata", "depth/metadata"),
            ("driver/depth/camera_info", "depth/camera_info"),
            ("driver/depth/image_rect_raw", "depth/image_raw"),
            ("driver/depth/image_rect_raw/compressed", "depth/image_raw/compressed"),
            ("driver/depth/image_rect_raw/compressedDepth", "depth/image_raw/compressedDepth"),
            ("driver/depth/image_rect_raw/theora", "depth/image_raw/theora"),
            ("driver/depth/metadata", "depth/metadata"),
            ("driver/infra1/camera_info", "infrared_right/camera_info"),
            ("driver/infra1/image_rect_raw", "infrared_right/image_raw"),
            ("driver/infra1/image_rect_raw/compressed", "infrared_right/image_raw/compressed"),
            ("driver/infra1/image_rect_raw/compressedDepth", "infrared_right/image_raw/compressedDepth"),
            ("driver/infra1/image_rect_raw/theora", "infrared_right/image_raw/theora"),
            ("driver/infra1/metadata", "infrared_right/metadata"),
            ("driver/infra2/camera_info", "infrared_left/camera_info"),
            ("driver/infra2/image_rect_raw", "infrared_left/image_raw"),
            ("driver/infra2/image_rect_raw/compressed", "infrared_left/image_raw/compressed"),
            ("driver/infra2/image_rect_raw/compressedDepth", "infrared_left/image_raw/compressedDepth"),
            ("driver/infra2/image_rect_raw/theora", "infrared_left/image_raw/theora"),
            ("driver/infra2/metadata", "infrared_left/metadata"),
            ("driver/depth/color/points", "points"),
            ("driver/extrinsics/depth_to_color", "extrinsics/depth_to_color"),
            ("driver/extrinsics/depth_to_depth", "extrinsics/depth_to_depth"),
            ("driver/imu", "imu"),

        ],
    )

    driver.add_action(driver_node)

    return [driver]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("executable"))
    declared_arguments.append(DeclareLaunchArgument("frame_id"))
    declared_arguments.append(DeclareLaunchArgument("driver_configuration_file_path"))
    declared_arguments.append(DeclareLaunchArgument("camera_configuration_file_path"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
