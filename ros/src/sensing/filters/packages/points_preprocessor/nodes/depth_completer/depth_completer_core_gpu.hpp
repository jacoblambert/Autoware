#ifndef DEPTH_COMPLETER_CORE_GPU_H
#define DEPTH_COMPLETER_CORE_GPU_H

#include <map>
#include <utility>
#include <string>

#include "depth_completer_core.hpp"


class DepthCompleterGPU: public DepthCompleter {
public:
  //
  // Constructor
  //
  DepthCompleterGPU();


  //
  // Destructor
  //
  ~DepthCompleterGPU();


  //
  // GPU implementation version of FillInFast
  //
  virtual cv::Mat FillInFast(const cv::Mat& depth_map,
                             const bool& extrapolate=false,
                             const std::string& blur_type="bilateral",
                             const double& max_depth=100.0,
                             const std::string& custom_kernel_str="DIAMOND_KERNEL_5");

  //
  // GPU implementation version of FillInMultiScale
  //
  virtual cv::Mat FillInMultiScale(const cv::Mat& depth_map,
                                   const bool& extrapolate=false,
                                   const std::string& blur_type="bilateral",
                                   const double& max_depth=100.0,
                                   const std::string& dilation_kernel_far_str="CROSS_KERNEL_3",
                                   const std::string& dilation_kernel_med_str="CROSS_KERNEL_5",
                                   const std::string& dilation_kernel_near_str="CROSS_KERNEL_7");


private:
  // The pointers allocated on GPU memory
  float* dev_buffer1_;
  float* dev_buffer2_;  // As reading and writing is iterated, use mirror buffering
  float* dev_valid_pixels_near_;
  float* dev_valid_pixels_med_;
  float* dev_valid_pixels_far_;
  float* dev_dilated_near_;
  float* dev_dilated_med_;
  float* dev_dilated_far_;
  float* dev_top_mask_;
  float* dev_work_buffer_;

  // The flag that GPU memories are allocated
  bool is_dev_memory_ready_;

  // The memory pitch to access correct location on GPU memory
  size_t buffer_pitch_;

  // The number of threads per each GPU kernel block
  const int kNumThreadsPerBlock_ = 16;
  // const int kNumThreadsPerBlock_ = 24;

  // The heap size expansion rate on GPU
  const int kHeapSizeExpansionRate_ = 3;

  // The function to allocate and set data for each GPU memory
  void InitializeGPUMemory(const cv::Mat input);

  // The dictionary to link filter name and its data
  // Dictionary structure is:
  // - kernel_name (string)
  // - pointer and radius pair (pair)
  // -- pointer to the filter on GPU memory (unsigned char*)
  // -- filter radius (int)
  std::map<std::string, std::pair<unsigned char*, int> > filters_dict_;

  // Utility to handle mirror buffer
  float* dev_src_;
  float* dev_dst_;
  void SwitchSrcAndDist(void) {
    // This function assume that:
    // dev_src_ is initialized as dev_buffer1_
    // dev_dst_ is initialized as dev_buffer2_
    static bool switcher = true;
    if (switcher) {
      dev_src_ = dev_buffer2_;
      dev_dst_ = dev_buffer1_;
    } else {
      dev_src_ = dev_buffer1_;
      dev_dst_ = dev_buffer2_;
    }
    switcher = !switcher;
  }  // void SwitchSrcAndDist()
};

#endif // DEPTH_COMPLETER_CORE_GPU_H
