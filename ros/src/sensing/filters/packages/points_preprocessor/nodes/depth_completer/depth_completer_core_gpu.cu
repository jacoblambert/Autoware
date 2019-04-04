#include "depth_completer_core_gpu.hpp"
#include <stdexcept>
#include <cuda_runtime.h>
#include <thrust/transform.h>
#include <thrust/functional.h>
#include <thrust/extrema.h>
#include <thrust/fill.h>
#include <thrust/device_vector.h>
#include <thrust/reduce.h>
#include <thrust/replace.h>
#include <thrust/execution_policy.h>
#include <npp.h>
#include <nppi.h>

#define CUDA_ERROR_CHECK(err) {CudaErrorCheck((err), __FILE__, __LINE__);}

namespace {
  // The utility structure to handle memory information easily
  typedef struct {
    int img_height;
    int img_width;
    size_t pitch;

    template<typename T>
    __host__ __device__
    T* ptr(T* base_add, int y) {
      return (T*)((char*)base_add + y * pitch);
    }

    template<typename T>
    __host__ __device__
    T val(T* base_add, int y, int x) {
      if (x < 0 || x > img_width || y < 0 || y > img_height)
        return 0;
      else
        return ((T*)((char*)base_add + y * pitch))[x];
    }

    __host__ __device__
    size_t allocated_element_num() {
      return img_height * pitch / sizeof(float);
    }
  } MemoryInfo;


  // The utility enum to switch process behavior
  enum class MorphologyType : int {
    kDilation,
    kErosion,
    kMedianBlur,
  };

  enum class FilterDirection : int {
    kRow,
    kCol,
  };


  // Constant memory definition on GPU memory
  __constant__ unsigned char FULL_KERNEL_3[3 * 3];
  __constant__ unsigned char FULL_KERNEL_5[5 * 5];
  __constant__ unsigned char FULL_KERNEL_7[7 * 7];
  __constant__ unsigned char FULL_KERNEL_9[9 * 9];
  __constant__ unsigned char FULL_KERNEL_31[31 * 31];
  __constant__ unsigned char CROSS_KERNEL_3[3 * 3];
  __constant__ unsigned char CROSS_KERNEL_5[5 * 5];
  __constant__ unsigned char DIAMOND_KERNEL_5[5 * 5];
  __constant__ unsigned char CROSS_KERNEL_7[7 * 7];
  __constant__ unsigned char DIAMOND_KERNEL_7[7 * 7];

  //
  // Wrapper function to check CUDA API call status
  //
  inline void CudaErrorCheck(cudaError_t err,
                             const char* file,
                             const int line,
                             bool abort=true
                             ) {
    if (err != cudaSuccess) {
      std::cerr << "Error occured while CUDA API call: " << std::endl
                << cudaGetErrorString(err) << std::endl
                << "@" << file << "(" << line << ")" << std::endl;
      if (abort) exit(EXIT_FAILURE);
    }
  }  // inline void CudaErrorCheck()


  //
  // GPU memory allocate and initialize process
  //
  template <typename T>
  void AllocAndInitMemory(T& ptr, size_t& pitch, const int width_in_byte, const int height) {
    CUDA_ERROR_CHECK(cudaMallocPitch(&ptr,
                                     &pitch,
                                     width_in_byte,
                                     height
                                     ));
    CUDA_ERROR_CHECK(cudaMemset2D(ptr,
                                  pitch,
                                  0, // value to set for each byte to specified memory
                                  width_in_byte,
                                  height
                                  ));
    return;
  }  // void AllocAndInitMemory()


  //
  // Utility function to calculate proper value for GPU kernel grid size
  //
  inline int DivRoundUp(int value, int radix) {
    return (value + radix - 1) / radix;
  }  // inline int DivRoundUp()


  //
  // GPU kernel to calculate binary mask
  //
  __global__
  void CalculateBinaryMaskKernel(float* __restrict__ mask_near,
                                 float* __restrict__ mask_med,
                                 float* __restrict__ mask_far,
                                 float* __restrict__ src,
                                 MemoryInfo dev_mem) {
    // Calculate global index for this thread
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    if (y >= dev_mem.img_height || x >= dev_mem.img_width) {
      return;
    }

    // Set value according to each distance range
    float pixel_value = dev_mem.val(src, y, x);
    dev_mem.ptr(mask_near, y)[x] = (pixel_value > 0.1 && pixel_value <= 15.0);
    dev_mem.ptr(mask_med, y)[x] = (pixel_value > 15.0 && pixel_value <= 30.0);
    dev_mem.ptr(mask_far, y)[x] = (pixel_value > 30.0);

    return;
  }  // void CalculateBinaryMaskKernel()


  //
  // GPU kernel to invert value for all pixels
  //
  __global__
  void InvertKernel(float* __restrict__ src,
                    float* __restrict__ dst,
                    MemoryInfo dev_mem,
                    const float max_depth) {
    // Calculate global index for this thread
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    if (y >= dev_mem.img_height || x >= dev_mem.img_width) {
      return;
    }

    // Do invertion
    float current_val = dev_mem.ptr(src, y)[x];
    bool update_condition = (current_val > 0.1);
    dev_mem.ptr(dst, y)[x] = update_condition * (max_depth - current_val)
      + !update_condition * current_val;

    return;
  }  // void InvertKernel()


  //
  // The function performing basic morphology processes using
  // NPP (Nvidia Performance Primitive) library
  //
  Npp32u gScratchBufferSize;
  Npp8u* gDevScratchBuffer;
  void PerformBasicMorphology(float* src,
                              float* dst,
                              MemoryInfo dev_mem,
                              unsigned char* filter,
                              const int filter_radius,
                              const MorphologyType morphology_type) {
    // Set target region to whole image
    NppiSize src_size = {dev_mem.img_width, dev_mem.img_height};
    NppiSize roi_size = {dev_mem.img_width, dev_mem.img_height};
    NppiPoint src_offset = {0, 0};
    // X and Y offsets of the mask origin frame of reference w.r.t the source pixel
    // NppiPoint anchor = {0, 0};
    NppiPoint anchor = {filter_radius/2, filter_radius/2};

    NppiSize mask_size = {filter_radius, filter_radius};

    switch(morphology_type) {
      case MorphologyType::kDilation: {
        nppiDilateBorder_32f_C1R(src,
                                 dev_mem.pitch,
                                 src_size,
                                 src_offset,
                                 dst,
                                 dev_mem.pitch,
                                 roi_size,
                                 filter,
                                 mask_size,
                                 anchor,
                                 NPP_BORDER_REPLICATE  // Currently only the NPP_BORKER_REPLICATE border type operation is supported as of CUDA ver 10
                                 );
        break;
      }
      case MorphologyType::kErosion: {
        nppiErodeBorder_32f_C1R(src,
                                 dev_mem.pitch,
                                 src_size,
                                 src_offset,
                                 dst,
                                 dev_mem.pitch,
                                 roi_size,
                                 filter,
                                 mask_size,
                                 anchor,
                                 NPP_BORDER_REPLICATE  // Currently only the NPP_BORKER_REPLICATE border type operation is supported as of CUDA ver 10
                                 );
        break;
      }
      case MorphologyType::kMedianBlur: {
        // As there is no border considered version for Median filter,
        // apply the filter to image except for image outer bounds

        // Get scratch buffer size used by NPP library
        Npp32u buffer_size = 0;
        nppiFilterMedianGetBufferSize_32f_C1R(roi_size, mask_size, &buffer_size);
        if (gScratchBufferSize < buffer_size) {
          // Allocate Scratch buffer region if and only if re-allocation is needed
          // to avoid every-time allocation that is time consuming
          CUDA_ERROR_CHECK(cudaMalloc((void**)(&gDevScratchBuffer), sizeof(Npp8u) * buffer_size));
          gScratchBufferSize = buffer_size;
        }
        nppiFilterMedian_32f_C1R(src,
                                 dev_mem.pitch,
                                 dst,
                                 dev_mem.pitch,
                                 roi_size,
                                 mask_size,
                                 anchor,
                                 gDevScratchBuffer
                                 );
        break;
      }
    }
  }  // void PerformBasicMorphology()


  //
  // GPU kernel to combine multiple input
  //
  __global__
  void CombineKernel(float* true_pixel_src,
                     float* false_pixel_src,
                     float* condition_src,
                     float* dst,
                     MemoryInfo dev_mem) {
    // Calculate global index for this thread
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    if (y >= dev_mem.img_height || x >= dev_mem.img_width) {
      return;
    }

    // Conbine src and dilated result according to the condition
    bool combine_condition = (dev_mem.val(condition_src, y, x) > 0.1);
    dev_mem.ptr(dst, y)[x] = combine_condition * dev_mem.val(true_pixel_src, y, x)
      + !combine_condition * dev_mem.val(false_pixel_src, y, x);
  }  // void CombineKernel()


  //
  // GPU kernel to fill by combining dilated image
  //
  __global__
  void HoleFillKernel(float* src,
                      float* dilated,
                      float* dst,
                      MemoryInfo dev_mem) {
    // Calculate global index for this thread
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    if (y >= dev_mem.img_height || x >= dev_mem.img_width) {
      return;
    }

    // Calculate a top mask
    bool is_top = (dev_mem.val(src, y, x) > 0.1);
    for (int row = 0; row < y; row++) {
      // Check whether pixel that its value is smaller than 0.1 exists above
      is_top  = is_top  || (dev_mem.val(src, row, x) > 0.1);
    }

    // Get empty mask
    bool is_valid = (dev_mem.val(src, y, x) > 0.1);
    bool is_empty = !is_valid && is_top;

    // Hole fill
    dev_mem.ptr(dst, y)[x] = is_empty * dev_mem.val(dilated, y, x)
      + !is_empty * dev_mem.val(src, y, x);
  }  // void HoleFillKernel()


  //
  // GPU kernel to extrapolate top region whose pixel value is less than 0.1
  //
  __global__
  void ExtrapolateKernel(float* __restrict__ src,
                         float* __restrict__ dst,
                         MemoryInfo dev_mem) {
    // Calculate global index for this thread
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    if (y >= dev_mem.img_height || x >= dev_mem.img_width) {
      return;
    }

    // Search top value
    bool is_top_value_found = false;
    float top_value = 0.;
    for (int row = 0; row < dev_mem.img_height; row++) {
      float value = dev_mem.val(src, row, x);
      top_value = (value > 0.1 && !is_top_value_found) * value
        + !(value > 0.1 && !is_top_value_found) * top_value;
      is_top_value_found = (top_value > 0.1);
    }

    // Extrapolate if this pixel is target
    bool is_extrapolate_target = true;
    for (int row = 0; row < y; row++) {
      is_extrapolate_target = is_extrapolate_target && (dev_mem.val(src, row, x) <= 0.1);
    }
    dev_mem.ptr(dst, y)[x] = is_extrapolate_target * top_value
      + !is_extrapolate_target * dev_mem.val(src, y, x);

  }  // void ExtrapolateKernel()


  //
  // GPU kernel to calculate top mask
  //
  __global__
  void CalculateTopMaskKernel(float* __restrict__ src,
                              float* __restrict__ top_mask,
                              MemoryInfo dev_mem) {
    // Calculate global index for this thread
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    if (y >= dev_mem.img_height || x >= dev_mem.img_width) {
      return;
    }

    // Fill false if all pixels' value of the above rows are less than 0.1
    bool is_top = (dev_mem.val(src, y, x) > 0.1);
    for (int row = 0; row < y; row++) {
      is_top = is_top || (dev_mem.val(src, row, x) > 0.1);
    }
    dev_mem.ptr(top_mask, y)[x] = is_top;
  }  // void CalculateTopMaskKernel()


  //
  // GPU kernel to fill large holes with masked dilations
  //
  __global__
  void FillLargeHoleKernel(float* __restrict__ src,
                           float* __restrict__ top_mask,
                           float* __restrict__ dilated,
                           float* __restrict__ dst,
                           MemoryInfo dev_mem) {
    // Calculate global index for this thread
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    if (y >= dev_mem.img_height || x >= dev_mem.img_width) {
      return;
    }

    // Fill large hole according to the condition
    float current_val = dev_mem.val(src, y, x);
    bool is_empty = (current_val < 0.1) && static_cast<bool>(dev_mem.val(top_mask, y, x));
    dev_mem.ptr(dst, y)[x] = is_empty * dev_mem.val(dilated, y, x)
      + !is_empty * current_val;
  }  // void FillLargeHoleKernel()


  //
  // GPU kernel to merge blurred source and create valid pixels
  //
  __global__
  void MergeBlurredAndCreateValidPixelKernel(float* __restrict__ src,
                                             float* __restrict__ blurred,
                                             float* __restrict__ top_mask,
                                             float* __restrict__ dst,
                                             float* __restrict__ valid_pixels,
                                             MemoryInfo dev_mem) {
    // Calculate global index for this thread
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    if (y >= dev_mem.img_height || x >= dev_mem.img_width) {
      return;
    }

    float current_val = dev_mem.val(src, y, x);
    bool is_valid = (current_val > 0.1) && static_cast<bool>(dev_mem.val(top_mask, y, x));
    dev_mem.ptr(valid_pixels, y)[x] = is_valid;
    dev_mem.ptr(dst, y)[x] = is_valid * dev_mem.val(blurred, y, x)
      + !is_valid * current_val;
  }  // void MergeBlurredAndCreateValidPixelKernel()


  //
  // Perform gaussian blur using NPP library
  //
  void GaussianBlur(float* src,
                    float* dst,
                    MemoryInfo dev_mem,
                    NppiMaskSize mask_size
                    ) {
    NppiSize src_size = {dev_mem.img_width, dev_mem.img_height};
    NppiPoint src_offset = {0, 0};
    nppiFilterGaussBorder_32f_C1R (src,
                                   dev_mem.pitch,
                                   src_size,
                                   src_offset,
                                   dst,
                                   dev_mem.pitch,
                                   src_size, // set same size of ROI as image size
                                   mask_size,
                                   NPP_BORDER_REPLICATE);
  }  // GaussianBlur()


  //
  // Perform Bilateral filter using NPP libary
  //
  void BilateralFiltering(float* src,
                          float* dst,
                          MemoryInfo dev_mem,
                          const int filter_diameter,
                          const float sigma_color,
                          const float sigma_space) {
    NppiPoint src_offset = {0, 0};
    // Set same size of ROI as the input image
    NppiSize src_size = {dev_mem.img_width, dev_mem.img_height};
    nppiFilterBilateralGaussBorder_32f_C1R (src,
                                            dev_mem.pitch,
                                            src_size,
                                            src_offset,
                                            dst,
                                            dev_mem.pitch,
                                            src_size,  // Set same size of ROI as the input image
                                            static_cast<int>(filter_diameter/2),  // The radius of the round filter kernel to be used
                                            1, //const int nStepBetweenSrcPixels,
                                            std::pow(sigma_color, 2), // const Npp32f nValSquareSigma,
                                            std::pow(sigma_space, 2), //const Npp32f nPosSquareSigma,
                                            NPP_BORDER_REPLICATE);
  }  // void BilateralFiltering()


  //
  // GPU kernel to merge src image and bilateral blurred image
  //
  __global__
  void MergeBilateralKernel(float* __restrict__ src,
                            float* __restrict__ bilateral_blurred,
                            float* __restrict__ condition_src,
                            float* __restrict__ dst,
                            MemoryInfo dev_mem) {
    // Calculate global index for this thread
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    if (y >= dev_mem.img_height || x >= dev_mem.img_width) {
      return;
    }

    // Merge images according to condition
    bool is_valid = static_cast<bool>(dev_mem.val(condition_src, y, x));
    dev_mem.ptr(dst, y)[x] = is_valid * dev_mem.val(bilateral_blurred, y, x)
      + !is_valid * dev_mem.val(src, y, x);
  } // void MergeBilateralKernel()

}  // namespace


DepthCompleterGPU::DepthCompleterGPU():
  dev_buffer1_(nullptr),
  dev_buffer2_(nullptr),
  dev_valid_pixels_near_(nullptr),
  dev_valid_pixels_med_(nullptr),
  dev_valid_pixels_far_(nullptr),
  is_dev_memory_ready_(false) {
  // Expand heap size on GPU so that all working threads allocate working buffer
  size_t current_heap_size;
  CUDA_ERROR_CHECK(cudaDeviceGetLimit(&current_heap_size, cudaLimitMallocHeapSize));
  CUDA_ERROR_CHECK(cudaDeviceSetLimit(cudaLimitMallocHeapSize, current_heap_size * kHeapSizeExpansionRate_));

  // Copy all dilation kernel to GPU constant memory
  // and set dictionary entry
  size_t copy_size_in_byte = kernels_["FULL_KERNEL_3"].size().height * kernels_["FULL_KERNEL_3"].step;
  CUDA_ERROR_CHECK(cudaMemcpyToSymbolAsync(FULL_KERNEL_3,
                                           kernels_["FULL_KERNEL_3"].data,
                                           copy_size_in_byte, // size in bytes to copy
                                           0,//copy_size_in_byte, // offset from start of symbol in bytes
                                           cudaMemcpyHostToDevice
                                           ));
  void* pointer_to_const = nullptr;
  CUDA_ERROR_CHECK(cudaGetSymbolAddress(&pointer_to_const, FULL_KERNEL_3));
  filters_dict_["FULL_KERNEL_3"] = std::make_pair(static_cast<unsigned char*>(pointer_to_const), 3);

  copy_size_in_byte = kernels_["FULL_KERNEL_5"].size().height * kernels_["FULL_KERNEL_5"].step;
  CUDA_ERROR_CHECK(cudaMemcpyToSymbolAsync(FULL_KERNEL_5,
                                           kernels_["FULL_KERNEL_5"].data,
                                           copy_size_in_byte,
                                           0,//copy_size_in_byte,
                                           cudaMemcpyHostToDevice
                                           ));
  CUDA_ERROR_CHECK(cudaGetSymbolAddress(&pointer_to_const, FULL_KERNEL_5));
  filters_dict_["FULL_KERNEL_5"] = std::make_pair(static_cast<unsigned char*>(pointer_to_const), 5);

  copy_size_in_byte = kernels_["FULL_KERNEL_7"].size().height * kernels_["FULL_KERNEL_7"].step;
  CUDA_ERROR_CHECK(cudaMemcpyToSymbolAsync(FULL_KERNEL_7,
                                           kernels_["FULL_KERNEL_7"].data,
                                           copy_size_in_byte,
                                           0,//copy_size_in_byte,
                                           cudaMemcpyHostToDevice
                                           ));
  CUDA_ERROR_CHECK(cudaGetSymbolAddress(&pointer_to_const, FULL_KERNEL_7));
  filters_dict_["FULL_KERNEL_7"] = std::make_pair(static_cast<unsigned char*>(pointer_to_const), 7);

  copy_size_in_byte = kernels_["FULL_KERNEL_9"].size().height * kernels_["FULL_KERNEL_9"].step;
  CUDA_ERROR_CHECK(cudaMemcpyToSymbolAsync(FULL_KERNEL_9,
                                           kernels_["FULL_KERNEL_9"].data,
                                           copy_size_in_byte,
                                           0,//copy_size_in_byte,
                                           cudaMemcpyHostToDevice
                                           ));
  CUDA_ERROR_CHECK(cudaGetSymbolAddress(&pointer_to_const, FULL_KERNEL_9));
  filters_dict_["FULL_KERNEL_9"] = std::make_pair(static_cast<unsigned char*>(pointer_to_const), 9);

  copy_size_in_byte = kernels_["FULL_KERNEL_31"].size().height * kernels_["FULL_KERNEL_31"].step;
  CUDA_ERROR_CHECK(cudaMemcpyToSymbolAsync(FULL_KERNEL_31,
                                           kernels_["FULL_KERNEL_31"].data,
                                           copy_size_in_byte,
                                           0,//copy_size_in_byte,
                                           cudaMemcpyHostToDevice
                                           ));
  CUDA_ERROR_CHECK(cudaGetSymbolAddress(&pointer_to_const, FULL_KERNEL_31));
  filters_dict_["FULL_KERNEL_31"] = std::make_pair(static_cast<unsigned char*>(pointer_to_const), 31);

  copy_size_in_byte = kernels_["CROSS_KERNEL_3"].size().height * kernels_["CROSS_KERNEL_3"].step;
  CUDA_ERROR_CHECK(cudaMemcpyToSymbolAsync(CROSS_KERNEL_3,
                                           kernels_["CROSS_KERNEL_3"].data,
                                           copy_size_in_byte,
                                           0,//copy_size_in_byte,
                                           cudaMemcpyHostToDevice
                                           ));
  CUDA_ERROR_CHECK(cudaGetSymbolAddress(&pointer_to_const, CROSS_KERNEL_3));
  filters_dict_["CROSS_KERNEL_3"] = std::make_pair(static_cast<unsigned char*>(pointer_to_const), 3);

  copy_size_in_byte = kernels_["CROSS_KERNEL_5"].size().height * kernels_["CROSS_KERNEL_5"].step;
  CUDA_ERROR_CHECK(cudaMemcpyToSymbolAsync(CROSS_KERNEL_5,
                                           kernels_["CROSS_KERNEL_5"].data,
                                           copy_size_in_byte,
                                           0,//copy_size_in_byte,
                                           cudaMemcpyHostToDevice
                                           ));
  CUDA_ERROR_CHECK(cudaGetSymbolAddress(&pointer_to_const, CROSS_KERNEL_5));
  filters_dict_["CROSS_KERNEL_5"] = std::make_pair(static_cast<unsigned char*>(pointer_to_const), 5);

  copy_size_in_byte = kernels_["DIAMOND_KERNEL_5"].size().height * kernels_["DIAMOND_KERNEL_5"].step;
  CUDA_ERROR_CHECK(cudaMemcpyToSymbolAsync(DIAMOND_KERNEL_5,
                                           kernels_["DIAMOND_KERNEL_5"].data,
                                           copy_size_in_byte,
                                           0,//copy_size_in_byte,
                                           cudaMemcpyHostToDevice
                                           ));
  CUDA_ERROR_CHECK(cudaGetSymbolAddress(&pointer_to_const, DIAMOND_KERNEL_5));
  filters_dict_["DIAMOND_KERNEL_5"] = std::make_pair(static_cast<unsigned char*>(pointer_to_const), 5);

  copy_size_in_byte = kernels_["CROSS_KERNEL_7"].size().height * kernels_["CROSS_KERNEL_7"].step;
  CUDA_ERROR_CHECK(cudaMemcpyToSymbolAsync(CROSS_KERNEL_7,
                                           kernels_["CROSS_KERNEL_7"].data,
                                           copy_size_in_byte,
                                           0,//copy_size_in_byte,
                                           cudaMemcpyHostToDevice
                                           ));
  CUDA_ERROR_CHECK(cudaGetSymbolAddress(&pointer_to_const, CROSS_KERNEL_7));
  filters_dict_["CROSS_KERNEL_7"] = std::make_pair(static_cast<unsigned char*>(pointer_to_const), 7);

  copy_size_in_byte = kernels_["DIAMOND_KERNEL_7"].size().height * kernels_["DIAMOND_KERNEL_7"].step;
  CUDA_ERROR_CHECK(cudaMemcpyToSymbolAsync(DIAMOND_KERNEL_7,
                                           kernels_["DIAMOND_KERNEL_7"].data,
                                           copy_size_in_byte,
                                           0,//copy_size_in_byte,
                                           cudaMemcpyHostToDevice
                                           ));
  CUDA_ERROR_CHECK(cudaGetSymbolAddress(&pointer_to_const, DIAMOND_KERNEL_7));
  filters_dict_["DIAMOND_KERNEL_7"] = std::make_pair(static_cast<unsigned char*>(pointer_to_const), 7);
}  // DepthCompleterGPU::DepthCompleterGPU()


DepthCompleterGPU::~DepthCompleterGPU() {
  if (is_dev_memory_ready_) {
    CUDA_ERROR_CHECK(cudaFree(dev_buffer1_));
    CUDA_ERROR_CHECK(cudaFree(dev_buffer2_));
    CUDA_ERROR_CHECK(cudaFree(dev_valid_pixels_near_));
    CUDA_ERROR_CHECK(cudaFree(dev_valid_pixels_med_));
    CUDA_ERROR_CHECK(cudaFree(dev_valid_pixels_far_));
    CUDA_ERROR_CHECK(cudaFree(dev_dilated_near_));
    CUDA_ERROR_CHECK(cudaFree(dev_dilated_med_));
    CUDA_ERROR_CHECK(cudaFree(dev_dilated_far_));
    CUDA_ERROR_CHECK(cudaFree(dev_top_mask_));
    CUDA_ERROR_CHECK(cudaFree(dev_work_buffer_));
  }
  CUDA_ERROR_CHECK(cudaFree(gDevScratchBuffer));
}  // DepthCompleterGPU::~DepthCompleter()


//
// Fast depth completion.
//
cv::Mat
DepthCompleterGPU::FillInFast(const cv::Mat &depth_map,
                              const bool& extrapolate,
                              const std::string& blur_type,
                              const double& max_depth,
                              const std::string& custom_kernel_str) {
  cv::Mat depths_in = depth_map.clone();
  const int img_height = depths_in.size().height;
  const int img_width = depths_in.size().width;
  const int img_width_in_byte = depths_in.step;

  // Allocate GPU memory if need and copy data
  if (!is_dev_memory_ready_) {
    InitializeGPUMemory(depths_in);
    is_dev_memory_ready_ = true;
  }

  // Copy input data to GPU memory
  CUDA_ERROR_CHECK(cudaMemcpy2DAsync(dev_src_,
                                     buffer_pitch_,
                                     depths_in.data,
                                     img_width_in_byte, // pitch of source memory
                                     img_width_in_byte,
                                     img_height,
                                     cudaMemcpyHostToDevice
                                     ));

  // Calculate binary masks before inversion
  dim3 block_dim(kNumThreadsPerBlock_, kNumThreadsPerBlock_, 1);
  dim3 grid_dim(DivRoundUp(img_width, block_dim.x),
                DivRoundUp(img_height, block_dim.y),
                1);

  MemoryInfo mem_info = {img_height, img_width, buffer_pitch_};

  // Invert
  InvertKernel<<<grid_dim, block_dim>>>(dev_src_,  // src
                                        dev_dst_,  // dst
                                        mem_info,
                                        max_depth
                                        );

  SwitchSrcAndDist();

  // Dilate
  PerformBasicMorphology(dev_src_,
                         dev_dst_,
                         mem_info,
                         filters_dict_[custom_kernel_str].first, // pointer to the filter
                         filters_dict_[custom_kernel_str].second, // filter radius
                         MorphologyType::kDilation);

  SwitchSrcAndDist();

  // Hole closing
  //   Closing process is defined as :
  //   dst = close(src, element) = erode(dilate(src, element))
  PerformBasicMorphology(dev_src_,
                         dev_dst_,
                         mem_info,
                         filters_dict_["FULL_KERNEL_5"].first,  // pointer to the filetr
                         filters_dict_["FULL_KERNEL_5"].second,  // filter radius
                         MorphologyType::kDilation
                         );

  SwitchSrcAndDist();

  PerformBasicMorphology(dev_src_,
                         dev_dst_,
                         mem_info,
                         filters_dict_["FULL_KERNEL_5"].first,  // pointer to the filetr
                         filters_dict_["FULL_KERNEL_5"].second,  // filter radius
                         MorphologyType::kErosion
                         );
  SwitchSrcAndDist();

  // Fill empty spaces with dilated values
  PerformBasicMorphology(dev_src_,
                         dev_work_buffer_,
                         mem_info,
                         filters_dict_["FULL_KERNEL_9"].first,
                         filters_dict_["FULL_KERNEL_9"].second,
                         MorphologyType::kDilation
                         );

  CombineKernel<<<grid_dim, block_dim>>>(dev_src_, // true_pixel_src
                                         dev_work_buffer_, // false_pixel_src
                                         dev_src_,         // condition_src
                                         dev_dst_,         // dst
                                         mem_info);
  SwitchSrcAndDist();

  // Extend highest pixel to top of image
  if (extrapolate) {
    ExtrapolateKernel<<<grid_dim, block_dim>>>(dev_src_,
                                               dev_dst_,
                                               mem_info);

    SwitchSrcAndDist();

    // Large Fill
    PerformBasicMorphology(dev_src_,
                           dev_work_buffer_,
                           mem_info,
                           filters_dict_["FULL_KERNEL_31"].first, // pointer to the fileter
                           filters_dict_["FULL_KERNEL_31"].second, // filter radius
                           MorphologyType::kDilation
                           );

    CombineKernel<<<grid_dim, block_dim>>>(dev_src_, // true_pixel_src
                                           dev_work_buffer_, // false_pixel_src
                                           dev_src_,         // condition_src
                                           dev_dst_,         // dst
                                           mem_info);
    SwitchSrcAndDist();
  }

  // Median blur
  PerformBasicMorphology(dev_src_,
                         dev_dst_,
                         mem_info,
                         filters_dict_["FULL_KERNEL_5"].first, // pointer to the filter
                         filters_dict_["FULL_KERNEL_5"].second, // filter radius
                         MorphologyType::kMedianBlur);

  SwitchSrcAndDist();

  // Bilateral or gaussian blur
  if (blur_type == "bilateral") {
    // Bilateral blur
    BilateralFiltering(dev_src_,
                       dev_dst_,
                       mem_info,
                       5,       // filster radius
                       1.5,     // sigma_color
                       2.0      // sigma_space
                       );

    SwitchSrcAndDist();

  } else if (blur_type == "gaussian") {
    // Gaussian blur
    GaussianBlur(dev_src_,
                 dev_work_buffer_,
                 mem_info,
                 NPP_MASK_SIZE_5_X_5);

    CombineKernel<<<grid_dim, block_dim>>>(dev_work_buffer_, // true_pixel_src
                                           dev_src_, // false_pixel_src
                                           dev_src_,         // condition_src
                                           dev_dst_,         // dst
                                           mem_info);

    SwitchSrcAndDist();

  } else {
    std::invalid_argument("Invalid blur_type: " + blur_type);
  }

  // Invert
  InvertKernel<<<grid_dim, block_dim>>>(dev_src_,  // src
                                        dev_dst_,  // dst
                                        mem_info,
                                        max_depth
                                        );

  // Wait until all GPU operations are complete
  //  and then copy result data
  CUDA_ERROR_CHECK(cudaDeviceSynchronize());

  cv::Mat depths_out = cv::Mat::zeros(depths_in.size(), CV_32F);
  CUDA_ERROR_CHECK(cudaMemcpy2D(depths_out.data,
                                img_width_in_byte,
                                dev_dst_,
                                buffer_pitch_,
                                img_width_in_byte,
                                img_height,
                                cudaMemcpyDeviceToHost
                                ));
  return depths_out;

}  // DepthCompleterGPU::FillInFast()


//
// Multi-scale dilation version with additional noise removeal
// that provides better qualitative results.
//
cv::Mat
DepthCompleterGPU::FillInMultiScale(const cv::Mat &depth_map,
                                    const bool& extrapolate,
                                    const std::string& blur_type,
                                    const double& max_depth,
                                    const std::string& dilation_kernel_far_str,
                                    const std::string& dilation_kernel_med_str,
                                    const std::string& dilation_kernel_near_str) {
  cv::Mat depths_in = depth_map.clone();
  const int img_height = depths_in.size().height;
  const int img_width = depths_in.size().width;
  const int img_width_in_byte = depths_in.step;

  // Allocate GPU memory if need and copy data
  if (!is_dev_memory_ready_) {
    InitializeGPUMemory(depths_in);
    is_dev_memory_ready_ = true;
  }


  // Copy input data to GPU memory
  CUDA_ERROR_CHECK(cudaMemcpy2DAsync(dev_src_,
                                     buffer_pitch_,
                                     depths_in.data,
                                     img_width_in_byte, // pitch of source memory
                                     img_width_in_byte,
                                     img_height,
                                     cudaMemcpyHostToDevice
                                     ));


  // Calculate binary masks before inversion
  dim3 block_dim(kNumThreadsPerBlock_, kNumThreadsPerBlock_, 1);
  dim3 grid_dim(DivRoundUp(img_width, block_dim.x),
                DivRoundUp(img_height, block_dim.y),
                1);

  MemoryInfo mem_info = {img_height, img_width, buffer_pitch_};

  CalculateBinaryMaskKernel<<<grid_dim, block_dim>>>(dev_valid_pixels_near_,
                                                     dev_valid_pixels_med_,
                                                     dev_valid_pixels_far_,
                                                     dev_src_, // src
                                                     mem_info
                                                     );

  // Invert (and offset)
  InvertKernel<<<grid_dim, block_dim>>>(dev_src_,  // src
                                        dev_dst_,  // dst
                                        mem_info,
                                        max_depth
                                        );

  SwitchSrcAndDist();

  // Multi-scale dilation and combine dilated versions starting farthest to nearest
  {  // farthest range dilation
    thrust::device_ptr<float> src = thrust::device_pointer_cast(dev_src_);
    thrust::device_ptr<float> far = thrust::device_pointer_cast(dev_valid_pixels_far_);
    thrust::device_ptr<float> work = thrust::device_pointer_cast(dev_work_buffer_);
    thrust::transform(src, src + mem_info.allocated_element_num(), far, work, thrust::multiplies<float>());
    PerformBasicMorphology(dev_work_buffer_,
                           dev_dilated_far_,
                           mem_info,
                           filters_dict_[dilation_kernel_far_str].first, // pointer to the filetr
                           filters_dict_[dilation_kernel_far_str].second, // filter radius
                           MorphologyType::kDilation
                           );
    CombineKernel<<<grid_dim, block_dim>>>(dev_dilated_far_, // true_pixel_src
                                           dev_src_,         // false_pixel_src
                                           dev_dilated_far_, // condition_src
                                           dev_dst_,         // dst
                                           mem_info);
  }
  {  // med range dilation
    thrust::device_ptr<float> src = thrust::device_pointer_cast(dev_src_);
    thrust::device_ptr<float> med = thrust::device_pointer_cast(dev_valid_pixels_med_);
    thrust::device_ptr<float> work = thrust::device_pointer_cast(dev_work_buffer_);
    thrust::transform(src, src + mem_info.allocated_element_num(), med, work, thrust::multiplies<float>());
    PerformBasicMorphology(dev_work_buffer_,
                           dev_dilated_med_,
                           mem_info,
                           filters_dict_[dilation_kernel_med_str].first,  // pointer to the filetr
                           filters_dict_[dilation_kernel_med_str].second,  // filter radius
                           MorphologyType::kDilation
                           );
    // Update dev_dst_ contents
    CombineKernel<<<grid_dim, block_dim>>>(dev_dilated_med_, // true_pixel_src
                                           dev_dst_,         // false_pixel_src
                                           dev_dilated_med_, // condition_src
                                           dev_dst_,         // dst
                                           mem_info);
  }
  {  // nearest range dilation
    thrust::device_ptr<float> src = thrust::device_pointer_cast(dev_src_);
    thrust::device_ptr<float> near = thrust::device_pointer_cast(dev_valid_pixels_near_);
    thrust::device_ptr<float> work = thrust::device_pointer_cast(dev_work_buffer_);
    thrust::transform(src, src + mem_info.allocated_element_num(), near, work, thrust::multiplies<float>());
    PerformBasicMorphology(dev_work_buffer_,
                           dev_dilated_near_,
                           mem_info,
                           filters_dict_[dilation_kernel_near_str].first,  // pointer to the filetr
                           filters_dict_[dilation_kernel_near_str].second,  // filter radius
                           MorphologyType::kDilation
                           );
    // Update dev_dst_ contents
    CombineKernel<<<grid_dim, block_dim>>>(dev_dilated_near_, // true_pixel_src
                                           dev_dst_, // false_pixel_src
                                           dev_dilated_near_, // condition_src
                                           dev_dst_,          // dst
                                           mem_info);
  }

  SwitchSrcAndDist();

  // Small hole closure
  //   Closing process is defined as :
  //   dst = close(src, element) = erode(dilate(src, element))
  PerformBasicMorphology(dev_src_,
                         dev_dst_,
                         mem_info,
                         filters_dict_["FULL_KERNEL_5"].first,  // pointer to the filetr
                         filters_dict_["FULL_KERNEL_5"].second,  // filter radius
                         MorphologyType::kDilation
                         );

  SwitchSrcAndDist();

  PerformBasicMorphology(dev_src_,
                         dev_dst_,
                         mem_info,
                         filters_dict_["FULL_KERNEL_5"].first,  // pointer to the filetr
                         filters_dict_["FULL_KERNEL_5"].second,  // filter radius
                         MorphologyType::kErosion
                         );
  SwitchSrcAndDist();


  // Median blur to remove outliers
  PerformBasicMorphology(dev_src_,
                         dev_work_buffer_,
                         mem_info,
                         filters_dict_["FULL_KERNEL_5"].first,  // pointer to the filetr
                         filters_dict_["FULL_KERNEL_5"].second,  // filter radius
                         MorphologyType::kMedianBlur
                         );

  CombineKernel<<<grid_dim, block_dim>>>(dev_work_buffer_,  // true_pixel_src
                                         dev_src_,          // false_pixel_src
                                         dev_src_,          // condition_src
                                         dev_dst_,          // dst
                                         mem_info
                                         );
  SwitchSrcAndDist();

  // Hole fill
  PerformBasicMorphology(dev_src_,
                         dev_work_buffer_,
                         mem_info,
                         filters_dict_["FULL_KERNEL_9"].first,
                         filters_dict_["FULL_KERNEL_9"].second,
                         MorphologyType::kDilation
                         );

  HoleFillKernel<<<grid_dim, block_dim>>>(dev_src_,
                                          dev_work_buffer_,
                                          dev_dst_,
                                          mem_info);
  SwitchSrcAndDist();

  // Extend highest pixel to top o fimage or create top mask
  thrust::device_ptr<float> top_mask = thrust::device_pointer_cast(dev_top_mask_);
  thrust::fill(top_mask, top_mask + mem_info.allocated_element_num(), true);
  if (extrapolate) {
    ExtrapolateKernel<<<grid_dim, block_dim>>>(dev_src_,
                                               dev_dst_,
                                               mem_info);
    SwitchSrcAndDist();
  } else {
    // Calculate a top mask
    CalculateTopMaskKernel<<<grid_dim, block_dim>>>(dev_src_,
                                                    dev_top_mask_,
                                                    mem_info);
  }

  // Fill large holes with masked dilation
  for (int i = 0; i < 6; i++) {
    PerformBasicMorphology(dev_src_,
                           dev_work_buffer_,
                           mem_info,
                           filters_dict_["FULL_KERNEL_5"].first, // pointer to the fileter
                           filters_dict_["FULL_KERNEL_5"].second, // filter radius
                           MorphologyType::kDilation
                           );

    FillLargeHoleKernel<<<grid_dim, block_dim>>>(dev_src_,
                                                 dev_top_mask_,
                                                 dev_work_buffer_,
                                                 dev_dst_,
                                                 mem_info);

    SwitchSrcAndDist();
  }

  // Median blur
  PerformBasicMorphology(dev_src_,
                         dev_work_buffer_,
                         mem_info,
                         filters_dict_["FULL_KERNEL_5"].first, // pointer to the filter
                         filters_dict_["FULL_KERNEL_5"].second, // filter radius
                         MorphologyType::kMedianBlur
                         );

  MergeBlurredAndCreateValidPixelKernel<<<grid_dim, block_dim>>>(dev_src_,
                                                                 dev_work_buffer_,
                                                                 dev_top_mask_,
                                                                 dev_dst_,
                                                                 dev_valid_pixels_far_,  // Use this buffer anyway
                                                                 mem_info);
  SwitchSrcAndDist();

  if (blur_type == "gaussian") {
    // Gaussian blur
    GaussianBlur(dev_src_,
                 dev_work_buffer_,
                 mem_info,
                 NPP_MASK_SIZE_5_X_5);
    MergeBlurredAndCreateValidPixelKernel<<<grid_dim, block_dim>>>(dev_src_, // src
                                                                   dev_work_buffer_, // blurred
                                                                   dev_top_mask_,
                                                                   dev_dst_,
                                                                   dev_valid_pixels_far_,  // Use this buffer anyway
                                                                   mem_info);
    SwitchSrcAndDist();
  } else if (blur_type == "bilateral") {
    // Bilateral blur
    BilateralFiltering(dev_src_,
                       dev_work_buffer_,
                       mem_info,
                       5, // filter_size
                       0.5, // sigma_color
                       2.0  // sigma_space
                       );

    MergeBilateralKernel<<<grid_dim, block_dim>>>(dev_src_, // src
                                                  dev_work_buffer_, // bilateral_blurred
                                                  dev_valid_pixels_far_, // condition_src
                                                  dev_dst_, // dst
                                                  mem_info
                                                  );
    SwitchSrcAndDist();
  } else {
    std::invalid_argument("Invalid blur_type: " + blur_type);
  }

  // Invert (and offset)
  InvertKernel<<<grid_dim, block_dim>>>(dev_src_,  // src
                                        dev_dst_,  // dst
                                        mem_info,
                                        max_depth
                                        );

  // Wait until all GPU operations are complete
  //  and then copy result data
  CUDA_ERROR_CHECK(cudaDeviceSynchronize());

  cv::Mat depths_out = cv::Mat::zeros(depths_in.size(), CV_32F);
  CUDA_ERROR_CHECK(cudaMemcpy2D(depths_out.data,
                                img_width_in_byte,
                                dev_dst_,
                                buffer_pitch_,
                                img_width_in_byte,
                                img_height,
                                cudaMemcpyDeviceToHost
                                ));
  return depths_out;

}  // DepthCompleterGPU::FillInMultiScale()


//
// Allocate and set data for each GPU memory
//
void DepthCompleterGPU::InitializeGPUMemory(const cv::Mat input) {
  const int img_height = input.size().height;
  const int img_width_in_byte = input.step;

  AllocAndInitMemory(dev_buffer1_          , buffer_pitch_, img_width_in_byte, img_height);
  AllocAndInitMemory(dev_buffer2_          , buffer_pitch_, img_width_in_byte, img_height);
  AllocAndInitMemory(dev_valid_pixels_near_, buffer_pitch_, img_width_in_byte, img_height);
  AllocAndInitMemory(dev_valid_pixels_med_ , buffer_pitch_, img_width_in_byte, img_height);
  AllocAndInitMemory(dev_valid_pixels_far_ , buffer_pitch_, img_width_in_byte, img_height);
  AllocAndInitMemory(dev_dilated_near_     , buffer_pitch_, img_width_in_byte, img_height);
  AllocAndInitMemory(dev_dilated_med_      , buffer_pitch_, img_width_in_byte, img_height);
  AllocAndInitMemory(dev_dilated_far_      , buffer_pitch_, img_width_in_byte, img_height);
  AllocAndInitMemory(dev_top_mask_         , buffer_pitch_, img_width_in_byte, img_height);
  AllocAndInitMemory(dev_work_buffer_      , buffer_pitch_, img_width_in_byte, img_height);

  dev_src_ = dev_buffer1_;
  dev_dst_ = dev_buffer2_;

}  // void DepthCompleterGPU::InitializeGPUMemory()
