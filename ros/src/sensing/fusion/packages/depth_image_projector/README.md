# Cloud Projector
This node will project a depth_image from image space to camera space, then pointcloud space as defined in the TF tree using the Camera Intrinsics. The 3D extrapolation is based on the depth information provided in the `DepthImage.msg` message.

### Requirements

1. Camera intrinsics
1. Camera-LiDAR extrinsics
1. Depth image

### How to launch

* From a sourced terminal:

`ROS_NAMESPACE=/camera_ns rosrun depth_image_projector depth_image_projector`

### Parameters

Launch file available parameters:

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`depth_image_src`|*String* |Name of the Depth Image topic to subscribe.|Default `depth_image`|
|`points_src`|*String* |Name of the Pointcloud topic to publish.|Default `depth_pointcloud`|
|`camera_info_src`|*String*|Name of the CameraInfo topic that contains the intrinsic matrix for the Image.|`camera_info`|

### Subscriptions/Publications

```
Publications: 
 * /points_raw [sensor_msgs/PointCloud type]
 
Subscriptions: 
 * /depth_image [autoware_msgs/DepthImage]
 * /tf [tf2_msgs/TFMessage]
```
