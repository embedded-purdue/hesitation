/**
 * @file device.hpp
 * @brief PyTorch/LibTorch compute device detection and selection utility.
 *
 * Configures CUDA GPU acceleration if available, with automatic fallback
 * to CPU execution for training and evaluation.
 *
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
 */

#pragma once
#include <torch/torch.h>

/// Master switch for CUDA acceleration. Set to false to force CPU execution.
constexpr bool USE_CUDA = true;

/**
 * @brief Returns the compute device for LibTorch tensors and modules.
 *
 * Evaluates CUDA availability once and returns a static torch::Device reference
 * (CUDA device 0 if supported and enabled, CPU otherwise).
 *
 * @return torch::Device Target compute device.
 */
inline torch::Device get_device() {
  static const torch::Device dev =
      (USE_CUDA && torch::cuda::is_available()) ? torch::Device(torch::kCUDA, 0)
                                                : torch::Device(torch::kCPU);
  return dev;
}