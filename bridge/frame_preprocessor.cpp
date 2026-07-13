#include "bridge/frame_preprocessor.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace bridge
{

namespace
{

cv::Mat apply_rotation_matrix_bgr(const cv::Mat & input_bgr, const std::array<double, 4> & matrix)
{
  if (input_bgr.empty()) {
    return {};
  }

  switch (classify_right_angle_rotation(matrix)) {
    case RightAngleRotation::kIdentity:
      return input_bgr;
    case RightAngleRotation::kRotate90CounterClockwise: {
      cv::Mat rotated;
      cv::rotate(input_bgr, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
      return rotated;
    }
    case RightAngleRotation::kRotate90Clockwise: {
      cv::Mat rotated;
      cv::rotate(input_bgr, rotated, cv::ROTATE_90_CLOCKWISE);
      return rotated;
    }
    case RightAngleRotation::kRotate180: {
      cv::Mat rotated;
      cv::rotate(input_bgr, rotated, cv::ROTATE_180);
      return rotated;
    }
    case RightAngleRotation::kUnsupported:
    default:
      break;
  }

  const double cx = (static_cast<double>(input_bgr.cols) - 1.0) * 0.5;
  const double cy = (static_cast<double>(input_bgr.rows) - 1.0) * 0.5;
  cv::Mat affine = (cv::Mat_<double>(2, 3) <<
    matrix[0], matrix[1], (1.0 - matrix[0]) * cx - matrix[1] * cy,
    matrix[2], matrix[3], -matrix[2] * cx + (1.0 - matrix[3]) * cy);

  cv::Mat rotated;
  cv::warpAffine(
    input_bgr,
    rotated,
    affine,
    input_bgr.size(),
    cv::INTER_LINEAR,
    cv::BORDER_CONSTANT,
    cv::Scalar::all(0));
  return rotated;
}

}  // namespace

FramePreprocessor::FramePreprocessor(const Options & options)
: rotation_matrix_(options.rotation_matrix),
  crop_size_(std::max(0, options.crop_size)),
  crop_center_x_(std::clamp(options.crop_center_x, 0.0, 1.0)),
  crop_center_y_(std::clamp(options.crop_center_y, 0.0, 1.0)),
  output_size_(std::clamp(options.video_size, 120, 480)),
  target_bitrate_kbps_(options.video_bitrate_kbps),
  static_simplify_(options.static_simplify),
  motion_threshold_(std::max(0, options.motion_threshold)),
  motion_erode_px_(std::clamp(options.motion_erode_px, 0, 20)),
  motion_dilate_px_(std::clamp(options.motion_dilate_px, 0, 20)),
  motion_trail_frames_(std::clamp(options.motion_trail_frames, 0, 180)),
  trail_disable_motion_ratio_(std::clamp(options.trail_disable_motion_ratio, 0.0, 1.0)),
  bg_update_alpha_(std::clamp(options.bg_update_alpha, 0.001, 0.2)),
  bg_blur_sigma_(std::max(0.0, options.bg_blur_sigma)),
  center_clear_size_(std::max(0, options.center_clear_size)),
  center_clear_radius_(std::max(0, options.center_clear_radius)),
  force_monochrome_(options.force_monochrome)
{
}

int FramePreprocessor::output_size() const
{
  return output_size_;
}

void FramePreprocessor::sync_runtime_options(const Options & options)
{
  const auto next_rotation = options.rotation_matrix;
  const int next_crop_size = std::max(0, options.crop_size);
  const double next_crop_center_x = std::clamp(options.crop_center_x, 0.0, 1.0);
  const double next_crop_center_y = std::clamp(options.crop_center_y, 0.0, 1.0);

  const bool crop_changed =
    crop_size_ != next_crop_size ||
    std::fabs(crop_center_x_ - next_crop_center_x) >= 1e-6 ||
    std::fabs(crop_center_y_ - next_crop_center_y) >= 1e-6 ||
    rotation_matrix_ != next_rotation;

  rotation_matrix_ = next_rotation;
  crop_size_ = next_crop_size;
  crop_center_x_ = next_crop_center_x;
  crop_center_y_ = next_crop_center_y;

  if (crop_changed) {
    reset_history();
  }
}

cv::Mat FramePreprocessor::process_rgb24(const FrameInfo & frame)
{
  if (frame.rgb24.empty() || frame.width == 0 || frame.height == 0) {
    return {};
  }

  cv::Mat input_rgb(
    static_cast<int>(frame.height),
    static_cast<int>(frame.width),
    CV_8UC3,
    const_cast<uint8_t *>(frame.rgb24.data()));
  cv::Mat input_bgr;
  cv::cvtColor(input_rgb, input_bgr, cv::COLOR_RGB2BGR);
  cv::Mat rotated_bgr = apply_rotation_matrix_bgr(input_bgr, rotation_matrix_);

  const auto crop_region = compute_square_crop_region(
    rotated_bgr.size(), crop_size_, crop_center_x_, crop_center_y_);
  if (crop_region.rect.width <= 0 || crop_region.rect.height <= 0) {
    return {};
  }
  cv::Mat cropped = rotated_bgr(crop_region.rect);

  cv::Mat resized;
  cv::resize(cropped, resized, cv::Size(output_size_, output_size_), 0, 0, cv::INTER_LINEAR);
  cv::Mat working = resized;

  if (force_monochrome_) {
    cv::Mat gray;
    cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray, working, cv::COLOR_GRAY2BGR);
  }

  if (center_clear_radius_ > 0) {
    cv::Mat detail_blur;
    cv::Mat detailed_focus;
    cv::GaussianBlur(working, detail_blur, cv::Size(), 0.85, 0.85);
    cv::addWeighted(working, 1.45, detail_blur, -0.45, 0.0, detailed_focus);

    cv::Mat focused = cv::Mat::zeros(working.size(), working.type());
    detailed_focus.copyTo(focused, center_circle_mask(working.size()));
    reset_history();
    return focused;
  }

  if (!static_simplify_) {
    reset_history();
    return working;
  }

  cv::Mat gray;
  cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
  if (background_gray_f32_.empty() || background_gray_f32_.size() != gray.size()) {
    reset_history();
    gray.convertTo(background_gray_f32_, CV_32F);
    return working;
  }

  cv::Mat bg_u8;
  cv::convertScaleAbs(background_gray_f32_, bg_u8);

  cv::Mat diff;
  cv::absdiff(gray, bg_u8, diff);

  cv::Mat motion_mask;
  cv::threshold(diff, motion_mask, motion_threshold_, 255, cv::THRESH_BINARY);
  if (motion_erode_px_ > 0) {
    if (motion_erode_kernel_.empty()) {
      const int kernel_size = 2 * motion_erode_px_ + 1;
      motion_erode_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    }
    cv::erode(motion_mask, motion_mask, motion_erode_kernel_, cv::Point(-1, -1), 1);
  }
  if (motion_dilate_px_ > 0) {
    if (motion_dilate_kernel_.empty()) {
      const int kernel_size = 2 * motion_dilate_px_ + 1;
      motion_dilate_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    }
    cv::dilate(motion_mask, motion_mask, motion_dilate_kernel_, cv::Point(-1, -1), 1);
  }

  const double motion_ratio = motion_mask.total() > 0
    ? static_cast<double>(cv::countNonZero(motion_mask)) / static_cast<double>(motion_mask.total())
    : 0.0;
  const bool suppress_trail = motion_ratio >= trail_disable_motion_ratio_;

  cv::Mat focus_mask = motion_mask.clone();
  if (center_clear_size_ > 0) {
    const int clear_size = std::min({center_clear_size_, working.cols, working.rows});
    const int clear_x = std::max(0, working.cols / 2 - clear_size / 2);
    const int clear_y = std::max(0, working.rows / 2 - clear_size / 2);
    const int clear_w = std::min(clear_size, working.cols - clear_x);
    const int clear_h = std::min(clear_size, working.rows - clear_y);
    cv::rectangle(focus_mask, cv::Rect(clear_x, clear_y, clear_w, clear_h), cv::Scalar(255), cv::FILLED);
  }

  cv::Mat static_base = working.clone();

  // Pacific-style: desaturate static areas for low bitrate to save bandwidth
  // (matching Pacific_doorlock_sniper's approach for ≤80kbps encoding)
  if (!force_monochrome_ && target_bitrate_kbps_ <= 80) {
    cv::Mat gray_static;
    cv::cvtColor(static_base, gray_static, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray_static, static_base, cv::COLOR_GRAY2BGR);
  }

  cv::Mat blurred_static;
  cv::GaussianBlur(static_base, blurred_static, cv::Size(), bg_blur_sigma_, bg_blur_sigma_);

  cv::Mat focused = blurred_static.clone();
  // Pacific-style: copy original working frame to motion areas (not sharpened)
  working.copyTo(focused, focus_mask);

  if (motion_trail_frames_ > 0) {
    motion_mask_history_.push_back(motion_mask.clone());
    // Pacific-style: store original frame (not sharpened) for trail history
    trail_frame_history_.push_back(working.clone());
    const auto max_history = static_cast<std::size_t>(motion_trail_frames_ + 1);
    while (motion_mask_history_.size() > max_history) {
      motion_mask_history_.pop_front();
    }
    while (trail_frame_history_.size() > max_history) {
      trail_frame_history_.pop_front();
    }

    const auto history_size = motion_mask_history_.size();
    if (!suppress_trail && history_size > 1 && history_size == trail_frame_history_.size()) {
      cv::Mat trail_mask = motion_mask.clone();
      cv::Mat trail_img = working.clone();
      for (std::size_t i = 0; i + 1 < history_size; ++i) {
        cv::bitwise_or(trail_mask, motion_mask_history_[i], trail_mask);
        cv::max(trail_img, trail_frame_history_[i], trail_img);
      }
      trail_img.copyTo(focused, trail_mask);
    }
  } else {
    motion_mask_history_.clear();
    trail_frame_history_.clear();
  }

  cv::accumulateWeighted(gray, background_gray_f32_, bg_update_alpha_);
  return focused;
}

void FramePreprocessor::reset_history()
{
  motion_mask_history_.clear();
  trail_frame_history_.clear();
  background_gray_f32_.release();
  motion_erode_kernel_.release();
  motion_dilate_kernel_.release();
}

cv::Mat FramePreprocessor::center_circle_mask(const cv::Size & size)
{
  const int radius = std::min({center_clear_radius_, size.width / 2, size.height / 2});
  if (radius <= 0) {
    return {};
  }

  if (!center_circle_mask_cache_.empty() &&
      center_circle_mask_cache_.size() == size &&
      center_circle_mask_radius_ == radius) {
    return center_circle_mask_cache_;
  }

  center_circle_mask_cache_ = cv::Mat::zeros(size, CV_8UC1);
  cv::circle(
    center_circle_mask_cache_,
    cv::Point(size.width / 2, size.height / 2),
    radius,
    cv::Scalar(255),
    cv::FILLED,
    cv::LINE_AA);
  center_circle_mask_radius_ = radius;
  return center_circle_mask_cache_;
}

}  // namespace bridge
