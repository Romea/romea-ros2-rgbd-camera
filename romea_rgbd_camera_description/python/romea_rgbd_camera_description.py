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

import math
import xacro

from ament_index_python.packages import get_package_share_directory

from romea_common_description import (
    evaluate_parameter,
    evaluate_parameter_from_list,
    evaluate_parameter_from_range,
    get_device_geometry_file_path,
    get_device_geometry,
    get_device_specifications,
    save_device_specifications_file,
)

from romea_stereo_camera_description import get_zed_complete_configuration


def image_width(resolution):
    return int(resolution.split("x")[0])


def image_height(resolution):
    return int(resolution.split("x")[1])


def evaluate_rgbd_camera_parameter(
    camera_type,
    camera_model,
    camera_component,
    specifications,
    user_configuration,
    parameter_name,
    key_value=None,
):
    return evaluate_parameter(
        camera_component,
        camera_type,
        camera_model,
        specifications,
        user_configuration,
        parameter_name,
        key_value,
    )


def evaluate_rgbd_camera_parameter_from_list(
    camera_type,
    camera_model,
    camera_component,
    specifications,
    user_configuration,
    parameter_name,
    key_value=None,
):
    return evaluate_parameter_from_list(
        camera_component,
        camera_type,
        camera_model,
        specifications,
        user_configuration,
        parameter_name,
        key_value,
    )


def evaluate_rgbd_camera_parameter_from_range(
    camera_type,
    camera_model,
    camera_component,
    specifications,
    user_configuration,
    parameter_name,
    key_value=None,
):
    return evaluate_parameter_from_range(
        camera_component,
        camera_type,
        camera_model,
        specifications,
        user_configuration,
        parameter_name,
        key_value,
    )


def get_rgbd_camera_geometry_file_path(camera_type, camera_model):
    if camera_type == "zed":
        return get_device_geometry_file_path("stereo_camera", camera_type, camera_model)
    else:
        return get_device_geometry_file_path("rgbd_camera", camera_type, camera_model)


def get_rgbd_camera_geometry(camera_type, camera_model):
    if camera_type == "zed":
        return get_device_geometry("stereo_camera", camera_type, camera_model)
    else:
        return get_device_geometry("rgbd_camera", camera_type, camera_model)


def get_rgbd_camera_specifications(camera_type, camera_model):
    if camera_type == "zed":
        return get_device_specifications("stereo_camera", camera_type, camera_model)
    else:
        return get_device_specifications("rgbd_camera", camera_type, camera_model)


def get_realesense_component_configuration(
    camera_model, component, specifications, user_configuration
):

    configuration = {}
    component_specifications = specifications[component]
    user_component_configuration = user_configuration[component]
    resolution = evaluate_rgbd_camera_parameter_from_list(
        "realsense",
        camera_model,
        component,
        component_specifications,
        user_component_configuration,
        "resolution",
    )

    configuration["image_width"] = image_width(resolution)
    configuration["image_height"] = image_height(resolution)

    configuration["horizontal_fov"] = (
        evaluate_rgbd_camera_parameter(
            "realsense",
            camera_model,
            component,
            component_specifications,
            user_component_configuration,
            "horizontal_fov",
        )
        * math.pi
        / 180.0
    )

    configuration["frame_rate"] = evaluate_rgbd_camera_parameter_from_list(
        "zed",
        camera_model,
        component,
        component_specifications,
        user_component_configuration,
        "frame_rate",
        resolution,
    )

    return configuration


def get_realsense_complete_configuration(camera_model, user_configuration):

    configuration = {}

    specifications = get_rgbd_camera_specifications("realsense", camera_model)
    configuration["rgb_camera"] = get_realesense_component_configuration(
        camera_model, "rgb_camera", specifications, user_configuration
    )

    configuration["infrared_camera"] = get_realesense_component_configuration(
        camera_model, "infrared_camera", specifications, user_configuration
    )

    configuration["depth_camera"] = get_realesense_component_configuration(
        camera_model, "depth_camera", specifications, user_configuration
    )

    return configuration


def get_rgbd_camera_complete_configuration(camera_type, camera_model, user_configuration):
    if camera_type == "zed":
        return get_zed_complete_configuration(camera_model, user_configuration)
    elif camera_type == "realsense":
        return get_realsense_complete_configuration(camera_model, user_configuration)
    else:
        raise LookupError(camera_type + "camera is not supported, cannot get camera configuration")


def get_rgbd_camera_type(camera_type):
    if camera_type == "zed":
        return "stereo_camera"
    elif camera_type == "realsense":
        return "infrared_stereo_camera"
    else:
        raise LookupError("Cannot retrieve camera type")


def urdf(prefix, mode, name, type, model, user_configuration, user_geometry, ros_namespace):

    complete_configuration = get_rgbd_camera_complete_configuration(
        type, model, user_configuration
    )
    complete_configuration_yaml_file = save_device_specifications_file(
        prefix, name, complete_configuration
    )
    geometry_yaml_file = get_rgbd_camera_geometry_file_path(type, model)

    xacro_file = (
        get_package_share_directory("romea_rgbd_camera_description")
        + "/urdf/rgbd_camera.xacro.urdf"
    )

    if mode == "simulation":
        mode += "_gazebo_classic"

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "name": name,
            # "type": type,
            # "model": model,
            "sensor_type": get_rgbd_camera_type(type),
            "sensor_config_yaml_file": complete_configuration_yaml_file,
            "geometry_config_yaml_file": geometry_yaml_file,
            "parent_link": user_geometry["parent_link"],
            "xyz": " ".join(map(str, user_geometry["xyz"])),
            "rpy": " ".join(map(str, user_geometry["rpy"])),
            "mesh_visual": str(True),
            "ros_namespace": ros_namespace,
        },
    )
    return urdf_xml.toprettyxml()
