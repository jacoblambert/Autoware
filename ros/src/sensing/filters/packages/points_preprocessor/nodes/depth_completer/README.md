## Image Processing for Basic Depth Completion (IP-Basic)
This depth completion node is a c++ and CUDA port of IP Basic, a depth completion tool which converts a sparse depth map into a dense depth map. Works best on Velodyne 64.

Original ip_basic repository can be found here: https://github.com/kujason/ip_basic

### Requirements

1. Camera intrinsics and Camera-LiDAR extrinsics file
1. Pointcloud projected to 2D depth map

### Usage

* Load your calibration file
* Run the cloud projector (pointcloud to depth map)
```bash
ROS_NAMESPACE=/camera_ns rosrun cloud_projector cloud_projector
```
* Run depth completion launch file:
```bash
roslaunch points_preprocessor depth_completer.launch
```

### Parameters
The following params can be set in the launch file or by calling rosrun direction:

|Parameter|Type|Default|Description|
|----------|----|-------|-----------|
|`input_image_topic`|*String*|"/depth_image_vis"|Depth image topic name (output of cloud projector)|
|`fill_type`|*String*|"multiscale"|"fast" or "multiscale", fast or more accurate|
|`extrapolate`|*Bool*|false|extrapolate depth to edges of the image|
|`blur_type`|*String*|"bilateral"|"gaussian" or bilateral|
|`use_gpu`|*Bool*|"true"|enable GPU acceleration|
