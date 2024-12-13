# romea_ros2_rgbd_camera

### 1) Overview ###

The romea_stereo_camera_bringup package provides  : 

 - **Launch files** able to launch ros2 camera drivers according a meta-description file provided by user (see next section for camera meta-description file overview), supported drivers are :

   - [???](TODO(Jean))
   - [???](TODO(Jean))

   It is possible to launch a driver via command line : 

    ```bash
    ros2 launch romea_rgbd_camera_bringup camera_driver.launch.py robot_namespace:=robot meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

 - A **python module** able to load and parse rgbd camera meta-description file as well as to create URDF description of the rgbd camera according a given meta-description.

 - a **ROS2 python executable** able to create rgbd camera URDF description via command line according a given meta-description file  :

  ```console
  ros2 run romea_rgbd_camera_bringup urdf_description.py robot_namespace:robot meta_description_file_path:/path_to_file/meta_description_file.yaml > camera.urdf`
  ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

   This URDF  can be directly concatened with mobile base and other sensor URDFs to create a complete URDF description of the robot.  

   

### 2) RGBD camera meta-description ###

The RGBD camera meta-description file is a YAML file with the following five main items:

- **name**: Specifies the name of the camera.
- **driver**: Specifies the configuration for the ROS2 driver used to control the RGBD camera.
- **configuration**: Defines basic specifications of the camera, such as type, model and resolution, field of view, frame rate, and video format for each sub devices
- **geometry**: Defines the position and orientation of the robot, with its position and orientation, used for URDF creation.
- **records**: Specifies the topics that will be recorded in ROS bags during experiments or simulations.

Example :
```yaml
 name: "rgbd_camera"  # name of the camera given by user
 driver: # rgbd camera driver configuration
    pkg: TODO(Jean)  
 configuration: # camera basic specifications
    type: realsense  #  type of camera
    model: d435  # model of camera
    rgb_camera:
       resolution: 1280x720 # resolution of image provided by RGB camera 
    infrared_camera:
       resolution: 1280x720 # resolution of image provided by Infared camera 
    depth_camera:
       resolution: 1280x720 # resolution of image provided by depth camera     
geometry: # geometry configuration 
  parent_link: "base_link"  # name of parent link where is located the camera
  xyz: [1.42, 0.0, 1.14] # its position in meters
  rpy: [0.0, 20.0, 0.0] # its orienation in degrees
records: # topic to be recorded
  rgb/camera_info: false # rgb camera info will not be recorded into bag
  rgb/image_raw: true # image raw image will be recorded into bag 
  depth/camera_info: false  # depth camera info will not be recorded into bag
  depth/image_raw: true # depth raw image will be recorded into bag 
  point_cloud/points: true # point cloud will be recorded into bag 
```

Supported stereo camera are listed in the following table :

|   type    | model |
| :-------: | :---: |
| realsense | d435  |
|           |       |

You can find specifications of each camera in config directory of romea_rgbd_description package.

### 5) Supported stereo camera ROS2 drivers

TODO(Jean)