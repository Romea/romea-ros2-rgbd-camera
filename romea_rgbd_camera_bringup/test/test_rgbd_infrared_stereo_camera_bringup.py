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


import os
import pytest
from numpy import deg2rad, radians

from romea_rgbd_camera_bringup import RGBDCameraMetaDescription, get_sensor_configuration, get_sensor_location


@pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(os.getcwd(), "test_rgbd_infrared_stereo_camera_bringup.yaml")
    return RGBDCameraMetaDescription(meta_description_file_path)


def test_get_name(meta_description):
    assert meta_description.get_name() == "rgbd_camera"


def test_get_namespace(meta_description):
    assert meta_description.get_namespace() == "ns"


# def test_has_driver_configuration(meta_description):
#     assert meta_description.has_driver_configuration() is True


# def test_get_driver_pkg(meta_description):
#     assert meta_description.get_driver_pkg() == "sick_scan"



def test_get_type(meta_description):
    assert meta_description.get_type() == "realsense"


def test_get_model(meta_description):
    assert meta_description.get_model() == "d435"


def test_get_rgb_camera_frame_rate(meta_description):
    assert meta_description.get_frame_rate("rgb_camera") == 30

def test_get_rgb_camera_resolution_(meta_description):
    assert meta_description.get_resolution("rgb_camera") == "1280x720"

def test_get_rgb_camera_horizontal_fov(meta_description):
    assert meta_description.get_horizontal_fov("rgb_camera") == None

def test_get_rgb_camera_video_format(meta_description):
    assert meta_description.get_video_format("rgb_camera") == None

def test_get_infrared_camera_frame_rate(meta_description):
    assert meta_description.get_frame_rate("infrared_camera") == 60

def test_get_infrared_camera_resolution_(meta_description):
    assert meta_description.get_resolution("infrared_camera") == "424x240"

def test_get_infrared_camera_horizontal_fov(meta_description):
    assert meta_description.get_horizontal_fov("infrared_camera") == None

def test_get_infrared_camera_video_format(meta_description):
    assert meta_description.get_video_format("infrared_camera") == None

def test_get_infrared_camera_frame_rate(meta_description):
    assert meta_description.get_frame_rate("depth_camera") == 90

def test_get_infrared_camera_resolution_(meta_description):
    assert meta_description.get_resolution("depth_camera") == "848x480"

def test_get_infrared_camera_horizontal_fov(meta_description):
    assert meta_description.get_horizontal_fov("depth_camera") == None

def test_get_infrared_camera_video_format(meta_description):
    assert meta_description.get_video_format("depth_camera") == None

def test_get_parent_link(meta_description):
    assert meta_description.get_parent_link() == "base_link"

def test_get_sensor_configuration(meta_description):
    configuration = get_sensor_configuration(meta_description)  
    assert configuration["rgb_camera"]["resolution"] == "1280x720" 
    assert configuration["rgb_camera"]["frame_rate"] == 30 
    assert configuration["rgb_camera"]["horizontal_fov"] == None 
    assert configuration["rgb_camera"]["video_format"] == None 
    assert configuration["infrared_camera"]["resolution"] == "424x240" 
    assert configuration["infrared_camera"]["frame_rate"] == 60 
    assert configuration["infrared_camera"]["horizontal_fov"] == None 
    assert configuration["infrared_camera"]["video_format"] == None 
    assert configuration["depth_camera"]["resolution"] == "848x480" 
    assert configuration["depth_camera"]["frame_rate"] == 90
    assert configuration["depth_camera"]["horizontal_fov"] == None 
    assert configuration["depth_camera"]["video_format"] == None 


def test_get_xyz(meta_description):
    assert meta_description.get_xyz() == [1.0, 2.0, 3.0]


def test_get_rpy_deg(meta_description):
    assert meta_description.get_rpy_deg() == [4.0, 5.0, 6.0]


def test_get_rpy_rad(meta_description):
    assert meta_description.get_rpy_rad() == radians([4.0, 5.0, 6.0]).tolist()

def test_get_sensor_location(meta_description):
    location = get_sensor_location(meta_description)  
    assert location["parent_link"] == "base_link"
    assert location["xyz"] == [1.0, 2.0, 3.0]
    assert location["rpy"] == radians([4.0, 5.0, 6.0]).tolist()


# def test_get_records(meta_description):
#     records = meta_description.get_records()
#     assert records["left/image_raw"] is True
#     assert records["left/camera_info"] is False
#     assert records["right/image_raw"] is True
#     assert records["right/camera_info"] is False