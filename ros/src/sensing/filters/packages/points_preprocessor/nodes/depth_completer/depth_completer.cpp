#include <string>
#include <memory>
#include <stdexcept>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "autoware_msgs/DepthImage.h"

#include "depth_completer_core.hpp"
#ifdef CUDA_FOUND
#include "depth_completer_core_gpu.hpp"
#endif
//#include <chrono>

namespace {
  // Definition of the node name and interfaces
  const std::string kAppName = "depth_completer";
  const std::string kAppArgInputImage = "input_image_topic"; // argument name for image input. Default: "/image_cloud"
  const std::string kAppArgInputFillType = "fill_type";      // argument name for `fill_type` flag. Default: "multiscale"
  const std::string kAppArgInputExtrapolate = "extrapolate"; // argument name for `extrapolate` flag. Default: false
  const std::string kAppArgInputBlurType = "blur_type";      // argument name for 'blur_type' flag. Default: "bilateral"
  const std::string kAppArgInputUseGPU = "use_gpu";          // argument name for 'use_gpu' flag. Default: true

  // Utility class to wrap ROS stuff
  class RosWrapper {
  public:
    //
    // Constructor
    //
    RosWrapper() :
      fill_type_("multiscale"),
      extrapolate_(false),
      blur_type_("bilateral"),
      use_gpu_(false) {
      // Get execution parameters
      GetExecutionParams();

      // Construct core class
      if (use_gpu_) {
#ifdef CUDA_FOUND
        depth_completer_gpu_.reset(new DepthCompleterGPU());
#else
        use_gpu_= false;
        depth_completer_.reset(new DepthCompleter());
        ROS_INFO_STREAM("[" << kAppName << "] use_gpu is set but no CUDA was found. Running on CPU.");
#endif
      } else {
        depth_completer_.reset(new DepthCompleter());
      }

      // Register Subscriber
      ROS_INFO_STREAM("[" << kAppName << "] Subscribing to... " << image_topic_name_.c_str());
      image_sub_ = node_handle_.subscribe(image_topic_name_,
                                          10,
                                          &RosWrapper::DepthImageCallback,
                                          this
                                          );

      // Register Publisher
      depth_pub_vis_ = node_handle_.advertise<sensor_msgs::Image>("/points_depth_vis",
                                                              10);
      depth_pub_ = node_handle_.advertise<autoware_msgs::DepthImage>("/points_depth",
                                                              10);
    }  // RosWrapper()


    //
    // Destructor
    //
    ~RosWrapper() {}

  private:
    // ROS stuff
    ros::NodeHandle node_handle_;
    ros::Subscriber image_sub_;
    ros::Publisher depth_pub_;
    ros::Publisher depth_pub_vis_;
    std::string image_topic_name_;
    std::string image_frame_id_;
    float min_depth_, max_depth_;

    // Flags to control algorithm behavior
    std::string fill_type_;
    bool extrapolate_;
    std::string blur_type_;
    bool use_gpu_;

    // The core class instance of Depth completion
    std::unique_ptr<DepthCompleter> depth_completer_;
#ifdef CUDA_FOUND
    std::unique_ptr<DepthCompleterGPU> depth_completer_gpu_;
#endif
    //
    // Function to get execution paramters
    //
    void GetExecutionParams() {
      ros::NodeHandle private_handle("~");

      private_handle.param<std::string>(kAppArgInputImage,
                                        image_topic_name_,
                                        "/depth_image");

      private_handle.param<std::string>(kAppArgInputFillType,
                                        fill_type_,
                                        "multiscale");

      private_handle.param<bool>(kAppArgInputExtrapolate,
                                 extrapolate_,
                                 false);

      private_handle.param<std::string>(kAppArgInputBlurType,
                                        blur_type_,
                                        "bilateral");

      private_handle.param<bool>(kAppArgInputUseGPU,
                                 use_gpu_,
                                 true);

    }  // void GetExecutionParams()


    //
    // Callback function
    //
    void DepthImageCallback(const autoware_msgs::DepthImage::ConstPtr &in_depth_msg) {
      cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_depth_msg->depth_image, "mono16");
      cv::Mat input_depth_image = cv_image->image;
      image_frame_id_ = in_depth_msg->header.frame_id;
      min_depth_ = in_depth_msg->min_depth;
      max_depth_ = in_depth_msg->max_depth;

      //auto time_measurement_start = std::chrono::system_clock::now();

      // Load depth projections from uint16 image
      cv::Mat projected_depths;
//      input_depth_image.convertTo(projected_depths, CV_32F, 1.0);
      cv::normalize(input_depth_image, projected_depths, 255, 0, cv::NORM_MINMAX, CV_32F);
      double min_depth1, max_depth1;
      cv::minMaxLoc(input_depth_image, &min_depth1, &max_depth1);
      double min_depth2, max_depth2;
      cv::minMaxLoc(projected_depths, &min_depth2, &max_depth2);
      std::cout << "In: (" << min_depth1 << ", " << max_depth1 << ")" << ", (" << min_depth2 << ", " << max_depth2 << ")\n";


      // Execute completion core
      cv::Mat final_depths;
      if (fill_type_ == "fast") {
        if (use_gpu_) {
#ifdef CUDA_FOUND
          final_depths = depth_completer_gpu_->FillInFast(projected_depths,
                                                          extrapolate_,
                                                          blur_type_);
#endif
        } else {
        final_depths = depth_completer_->FillInFast(projected_depths,
                                                    extrapolate_,
                                                    blur_type_);
        }
      } else if (fill_type_ == "multiscale") {
        if (use_gpu_) {
#ifdef CUDA_FOUND
          final_depths = depth_completer_gpu_->FillInMultiScale(projected_depths,
                                                                extrapolate_,
                                                                blur_type_);
#endif
        } else {
        final_depths = depth_completer_->FillInMultiScale(projected_depths,
                                                          extrapolate_,
                                                          blur_type_);
        }
      } else {
        std::invalid_argument("Invalid fill_type: " + fill_type_);
      }

      if (final_depths.empty()) {
        ROS_ERROR_STREAM("Empty result is detected. Completion failed.");
        return;
      }
      cv::Mat final_image;
//      cv::Mat converted_depths;
//      final_depths.convertTo(converted_depths, CV_16UC1), 255/1.0);
      cv::normalize(final_depths, final_image, 65535, 0, cv::NORM_MINMAX, CV_16UC1);
//      final_depths.convertTo(final_image, CV_16UC1, 255/1.0);

      cv::minMaxLoc(final_depths, &min_depth1, &max_depth1);
      cv::minMaxLoc(final_image, &min_depth2, &max_depth2);
      std::cout << "Out: (" << min_depth1 << ", " << max_depth1 << ")" << ", (" << min_depth2 << ", " << max_depth2 << ")\n";


      //auto time_measurement_end = std::chrono::system_clock::now();
      //double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(time_measurement_end -  time_measurement_start).count();
      //std::cerr << "fill_type: " << fill_type_ << ", elapsed: " << elapsed << "[ms]" << std::endl;

      // Publish completion result as sensor_msgs/Image format (for visualization)
      cv_bridge::CvImage image_msg;
      image_msg.header = in_depth_msg->header;
      image_msg.header.frame_id = image_frame_id_;
      image_msg.encoding = sensor_msgs::image_encodings::MONO16;
      image_msg.image = final_image;

      depth_pub_vis_.publish(image_msg.toImageMsg());

      // Publish completion result as autoware_msgs/DepthImage
      autoware_msgs::DepthImage depth_image;
      depth_image.header = in_depth_msg->header;
      depth_image.min_depth = min_depth_;
      depth_image.max_depth = max_depth_;
      depth_image.min_img_norm = 0;
      depth_image.max_img_norm = 65535;
      depth_image.depth_image = *image_msg.toImageMsg();

      depth_pub_.publish(depth_image);
      }  // void DepthImageCallback()

  };  // class RosWrapper
}  // namespace


int main(int argc, char* argv[]) {
  ros::init(argc, argv, kAppName);

  RosWrapper depth_completer;

  ros::spin();

  return 0;
}
