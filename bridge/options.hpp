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
  double exposure_ms = 10.0;   //10.0
  double gain = 12.0;    //12.0
  std::array<double, 4> rotation_matrix = {1.0, 0.0, 0.0, 1.0};
  double crop_center_x = 0.40;     //从相机原始画面中裁剪中心正方形区域
  double crop_center_y = 0.40;
  int send_interval_ms = 20;    //发送间隔 50Hz
  std::string ffmpeg_path = "ffmpeg";
  int video_size = 480;   //输出图像边长，分辨率，细节  硬上限480
  int video_fps = 30;   //编码帧率
  int video_bitrate_kbps = 116;   //H264 编码目标码率 116上限
  double video_latency_s = 12.0;   //目标延迟秒数 → 联动推导 bufsize/GOP/rc-lookahead (延迟与画质权衡的唯一杠杆)
  int video_gop = 0;   //关键帧间隔(帧): 0=自动=video_latency_s×video_fps; 设非0手动覆盖
  int crop_size = 0;
  bool static_simplify = true;    //静态简化开关
  int motion_threshold = 14;    //运动检测灵冥度 14
  int motion_erode_px = 1;    //运动掩吗腐蚀半径
  int motion_dilate_px = 7;     //运动掩吗膨胀半径 8
  int motion_trail_frames = 5;    //历史拖影帧数 5
  double trail_disable_motion_ratio = 0.08;   //全局运动禁用拖影阈值 0.15 0.20
  double bg_update_alpha = 0.01;    //背景模型更新速度
  double bg_blur_sigma = 1.00;   //静态区域模糊强度
  int center_clear_size = 100;    //roi
  int center_clear_radius = 0;    //roi  116 不走拖影逻辑
  bool force_monochrome = false;   //强制灰度
  bool test_pattern = false;
  bool preview = true;   //预览窗口，调曝光增益
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
