#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace bridge
{

struct Options
{
  std::string config_path = "";
  std::string camera_serial_number = "";
  double exposure_ms = 10.0;
  double gain = 12.0;
  std::array<double, 4> rotation_matrix = {1.0, 0.0, 0.0, 1.0};
  double crop_center_x = 0.5;
  double crop_center_y = 0.5;
  int send_interval_ms = 20;
  std::string ffmpeg_path = "ffmpeg";
  int video_size = 300;
  int video_fps = 60;
  int video_bitrate_kbps = 116;
  int video_gop = 0;  // 0 = auto: fps for normal, max(2*fps,30) for low-bitrate
  int crop_size = 0;
  bool static_simplify = true;
  int motion_threshold = 14;
  int motion_erode_px = 1;
  int motion_dilate_px = 2;
  int motion_trail_frames = 3;
  double trail_disable_motion_ratio = 0.30;
  double bg_update_alpha = 0.01;
  double bg_blur_sigma = 1.2;
  int center_clear_size = 100;
  bool force_monochrome = false;
  bool test_pattern = false;
  bool preview = false;
  bool list_cameras = false;
  std::string viewer_ip = "";
  int viewer_port = 3335;
  std::string video_serial = "/dev/ttyUSB0";
  uint32_t video_serial_baud = 921600;
};

Options parse_args(int argc, char ** argv);
bool save_config(Options & options, std::string * error = nullptr);
void print_help();

}  // namespace bridge
