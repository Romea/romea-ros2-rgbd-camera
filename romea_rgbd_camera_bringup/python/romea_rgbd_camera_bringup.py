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


from romea_camera_bringup import CameraMetaDescription, robot_urdf_prefix, device_namespace
from romea_rgbd_camera_description import get_rgbd_camera_specifications, get_rgbd_camera_geometry, urdf


class RGBDCameraMetaDescription(CameraMetaDescription):
    def __init__(self, meta_description_file_path):
        CameraMetaDescription.__init__(self, meta_description_file_path, "rgbd_camera")


def load_meta_description(meta_description_file_path):
    return RGBDCameraMetaDescription(meta_description_file_path)


def get_sensor_specifications(meta_description):
    return get_rgbd_camera_specifications(meta_description.get_type(), meta_description.get_model())


def get_sensor_geometry(meta_description):
    return get_rgbd_camera_geometry(meta_description.get_type(), meta_description.get_model())


def urdf_description(robot_namespace, mode, meta_description_file_path):

    meta_description = RGBDCameraMetaDescription(meta_description_file_path)

    ros_namespace = device_namespace(robot_namespace, meta_description.get_namespace(), meta_description.get_name())

    configuration = {}
    configuration["frame_rate"] = meta_description.get_frame_rate()
    configuration["resolution"] = meta_description.get_resolution()
    configuration["horizontal_fov"] = meta_description.get_horizontal_fov()
    configuration["video_format"] = meta_description.get_video_format()

    geometry = {}
    geometry["parent_link"] = meta_description.get_parent_link()
    geometry["xyz"] = meta_description.get_xyz()
    geometry["rpy"] = meta_description.get_rpy_rad()

    return urdf(
        robot_urdf_prefix(robot_namespace),
        mode,
        meta_description.get_name(),
        meta_description.get_type(),
        meta_description.get_model(),
        configuration,
        geometry,
        ros_namespace,
    )
