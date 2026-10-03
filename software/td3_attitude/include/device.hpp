#pragma once
#include <torch/torch.h>

// Flip to false to time the CPU path against the GPU path.
constexpr bool USE_CUDA = true;

inline torch::Device get_device() {
  static const torch::Device dev =
      (USE_CUDA && torch::cuda::is_available()) ? torch::Device(torch::kCUDA, 0)
                                                : torch::Device(torch::kCPU);
  return dev;
}