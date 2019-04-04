# autoware_sensor_msgs/DepthImage
Header          header

float32         min_depth     # min. distance from camera in meters
float32         max_depth     # max. distance from camera in meters
float32         min_img_norm  # min value of depth_image, 0 is default
float32         max_img_norm  # max value of depth_image, 65535 is default (mono16)

sensor_msgs/Image   depth_image