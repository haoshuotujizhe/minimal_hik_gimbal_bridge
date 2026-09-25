#pragma once

#include "bridge/crop_region.hpp"
#include "bridge/frame.hpp"
#include "bridge/options.hpp"

#include <array>
#include <deque>
#include <opencv2/core/mat.hpp>

namespace bridge
{

class FramePreprocessor
{
public:
  explicit FramePreprocessor(const Options & options);

  int output_size() const;
  void sync_runtime_options(const Options & options);
  cv::Mat process_rgb24(const FrameInfo & frame);

private:
  void reset_history();
  cv::Mat center_circle_mask(const cv::Size & size);

  std::array<double, 4> rotation_matrix_ = {1.0, 0.0, 0.0, 1.0};
  int crop_size_ = 0;
  double crop_center_x_ = 0.5;
  double crop_center_y_ = 0.5;
  int output_size_ = 300;
  int target_bitrate_kbps_ = 88;
  bool static_simplify_ = true;
  int motion_threshold_ = 14;
  int motion_erode_px_ = 1;
  int motion_dilate_px_ = 3;
  int motion_trail_frames_ = 5;
  double trail_disable_motion_ratio_ = 0.30;
  double bg_update_alpha_ = 0.01;
  double bg_blur_sigma_ = 1.2;
  int center_clear_size_ = 100;
  int center_clear_radius_ = 96;
  bool force_monochrome_ = false;
  cv::Mat background_gray_f32_;
  cv::Mat motion_erode_kernel_;
  cv::Mat motion_dilate_kernel_;
  cv::Mat center_circle_mask_cache_;
  int center_circle_mask_radius_ = -1;
  std::deque<cv::Mat> motion_mask_history_;
  std::deque<cv::Mat> trail_frame_history_;
};

}  // namespace bridge
