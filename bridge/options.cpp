#include "bridge/options.hpp"

#include "bridge/common.hpp"

#include <opencv2/core/persistence.hpp>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

namespace bridge
{
namespace
{

uint32_t parse_u32(const std::string & value)
{
  return static_cast<uint32_t>(std::stoul(value, nullptr, 0));
}

bool file_exists(const std::string & path)
{
  return !path.empty() && ::access(path.c_str(), F_OK) == 0;
}

struct ConfigSelection
{
  std::string path;
  bool explicit_path = false;
};

void read_yaml_rotation_matrix(const cv::FileNode & node, const char * name, std::array<double, 4> & target)
{
  if (node.empty()) {
    return;
  }

  cv::Mat matrix;
  node >> matrix;
  if (matrix.empty()) {
    throw std::runtime_error(std::string("配置项 ") + name + " 不能为空矩阵");
  }
  if (matrix.rows != 2 || matrix.cols != 2) {
    throw std::runtime_error(std::string("配置项 ") + name + " 必须是 2x2 矩阵");
  }

  cv::Mat matrix64;
  matrix.convertTo(matrix64, CV_64F);
  target[0] = matrix64.at<double>(0, 0);
  target[1] = matrix64.at<double>(0, 1);
  target[2] = matrix64.at<double>(1, 0);
  target[3] = matrix64.at<double>(1, 1);
}

void read_yaml_double(const cv::FileNode & node, const char * name, double & target)
{
  if (node.empty()) {
    return;
  }
  if (!node.isInt() && !node.isReal()) {
    throw std::runtime_error(std::string("配置项 ") + name + " 必须是数字");
  }
  node >> target;
}

void read_yaml_int(const cv::FileNode & node, const char * name, int & target)
{
  if (node.empty()) {
    return;
  }
  if (!node.isInt()) {
    throw std::runtime_error(std::string("配置项 ") + name + " 必须是整数");
  }
  node >> target;
}

void read_yaml_string(const cv::FileNode & node, const char * name, std::string & target)
{
  if (node.empty()) {
    return;
  }
  if (!node.isString()) {
    throw std::runtime_error(std::string("配置项 ") + name + " 必须是字符串");
  }
  node >> target;
}

void load_yaml_config(const std::string & path, Options & options)
{
  cv::FileStorage storage(path, cv::FileStorage::READ);
  if (!storage.isOpened()) {
    throw std::runtime_error("打开 YAML 配置失败: " + path);
  }

  read_yaml_double(storage["exposure_ms"], "exposure_ms", options.exposure_ms);
  read_yaml_double(storage["gain"], "gain", options.gain);

  const auto camera_node = storage["camera"];
  if (!camera_node.empty()) {
    read_yaml_string(camera_node["serial_number"], "camera.serial_number", options.camera_serial_number);
    read_yaml_double(camera_node["exposure_ms"], "camera.exposure_ms", options.exposure_ms);
    read_yaml_double(camera_node["gain"], "camera.gain", options.gain);
  }

  const auto image_node = storage["image"];
  if (!image_node.empty()) {
    read_yaml_rotation_matrix(image_node["rotation_matrix"], "image.rotation_matrix", options.rotation_matrix);
    read_yaml_double(image_node["crop_center_x"], "image.crop_center_x", options.crop_center_x);
    read_yaml_double(image_node["crop_center_y"], "image.crop_center_y", options.crop_center_y);
    read_yaml_int(image_node["center_clear_radius"], "image.center_clear_radius", options.center_clear_radius);
    read_yaml_double(image_node["video_latency_s"], "image.video_latency_s", options.video_latency_s);
    options.crop_center_x = std::clamp(options.crop_center_x, 0.0, 1.0);
    options.crop_center_y = std::clamp(options.crop_center_y, 0.0, 1.0);
    options.center_clear_radius = std::max(0, options.center_clear_radius);
    options.video_latency_s = std::clamp(options.video_latency_s, 0.5, 10.0);
  }
}

ConfigSelection select_config_path(int argc, char ** argv)
{
  ConfigSelection selection;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg != "--config") {
      continue;
    }
    if (i + 1 >= argc) {
      throw std::runtime_error("--config 缺少参数");
    }
    selection.path = argv[++i];
    selection.explicit_path = true;
    return selection;
  }

  if (file_exists(kDefaultConfigPath)) {
    selection.path = kDefaultConfigPath;
  }
  return selection;
}

}  // namespace

void print_help()
{
  std::cout
    << "minimal_hik_gimbal_bridge\n"
    << "  --config <path>          YAML 配置文件，默认自动尝试 config/bridge.yaml\n"
    << "  --camera-serial <serial>绑定指定海康相机序列号，覆盖 YAML camera.serial_number\n"
    << "  --list-cameras          枚举当前 USB 海康相机后退出\n"
    << "  --exposure-ms <value>    海康曝光时间，默认 10.0 ms\n"
    << "  --gain <value>           海康增益，默认 12.0\n"
    << "  --send-interval-ms <n>   串口发送周期，默认 20 ms，对应 0x0310 50Hz\n"
    << "  --ffmpeg <path>          H264 编码器路径，默认 ffmpeg\n"
    << "  --video-size <n>         0310 视频输出边长，默认 300\n"
    << "  --video-fps <n>          0310 视频编码帧率，默认 30\n"
    << "  --video-bitrate-kbps <n> 0310 视频目标码率，默认 116 kbit/s\n"
    << "  --video-latency-s <f>    目标延迟秒数(0.5~10)，联动推导 bufsize/GOP/rc-lookahead，默认 3.0\n"
    << "  --video-gop <n>          H264 GOP(帧)，0=自动=video_latency_s×fps，设非0手动覆盖\n"
    << "  --crop-size <n>          预处理中心裁剪边长，0 表示自动取最小边\n"
    << "  --static-simplify        开启静态区域简化预处理，默认开启\n"
    << "  --no-static-simplify     显式关闭静态区域简化预处理\n"
    << "  --motion-threshold <n>   运动检测阈值，默认 14\n"
    << "  --motion-erode-px <n>    运动掩码腐蚀像素，默认 2\n"
    << "  --motion-dilate-px <n>   运动掩码膨胀像素，默认 6\n"
    << "  --motion-trail-frames <n>拖影历史帧数，默认 0\n"
    << "  --trail-disable-motion-ratio <f> 全局运动占比超阈值时禁用拖影，默认 0.55\n"
    << "  --bg-update-alpha <f>    背景更新 alpha，默认 0.01\n"
    << "  --bg-blur-sigma <f>      静态区域模糊 sigma，默认 1.5\n"
    << "  --center-clear-size <n>  中心保护区边长，默认 180\n"
    << "  --center-clear-radius <n>中心圆形保真半径，默认 112；大于 0 时圆外压黑\n"
    << "  --force-monochrome       预处理后强制灰度\n"
    << "  --test-pattern           无相机时使用内置测试图案源\n"
    << "  --preview                显示相机原画面，并在窗口内调曝光/增益，按 S 保存 YAML\n"
    << "  --viewer-ip <ip>         可选 0x0310 UDP 调试目标 IP，默认关闭\n"
    << "  --viewer-port <n>        0x0310 UDP 目标端口，默认 3335\n"
    << "  --video-serial <path>    图传TX串口路径，默认 /dev/ttyUSB0\n"
    << "  --video-serial-baud <n>  图传TX串口波特率，默认 921600\n"
    << "  --help                   显示帮助\n";
}

bool save_config(Options & options, std::string * error)
{
  const std::string path = options.config_path.empty() ? kDefaultConfigPath : options.config_path;
  cv::FileStorage storage(path, cv::FileStorage::WRITE);
  if (!storage.isOpened()) {
    if (error != nullptr) {
      *error = "打开 YAML 配置失败: " + path;
    }
    return false;
  }

  storage << "camera" << "{";
  if (!options.camera_serial_number.empty()) {
    storage << "serial_number" << options.camera_serial_number;
  }
  storage << "exposure_ms" << options.exposure_ms;
  storage << "gain" << options.gain;
  storage << "}";

  cv::Mat rotation_matrix = (cv::Mat_<double>(2, 2) <<
    options.rotation_matrix[0], options.rotation_matrix[1],
    options.rotation_matrix[2], options.rotation_matrix[3]);
  storage << "image" << "{";
  storage << "rotation_matrix" << rotation_matrix;
  storage << "crop_center_x" << std::clamp(options.crop_center_x, 0.0, 1.0);
  storage << "crop_center_y" << std::clamp(options.crop_center_y, 0.0, 1.0);
  storage << "center_clear_radius" << std::max(0, options.center_clear_radius);
  storage << "video_latency_s" << std::clamp(options.video_latency_s, 0.5, 10.0);
  storage << "}";
  storage.release();

  options.config_path = path;
  return true;
}

Options parse_args(int argc, char ** argv)
{
  Options options;
  const auto config_selection = select_config_path(argc, argv);

  if (!config_selection.path.empty()) {
    load_yaml_config(config_selection.path, options);
    options.config_path = config_selection.path;
  }

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto require_value = [&](const char * name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string(name) + " 缺少参数");
      }
      return argv[++i];
    };

    if (arg == "--config") {
      require_value("--config");
    } else if (arg == "--camera-serial") {
      options.camera_serial_number = require_value("--camera-serial");
    } else if (arg == "--list-cameras") {
      options.list_cameras = true;
    } else if (arg == "--exposure-ms") {
      options.exposure_ms = std::stod(require_value("--exposure-ms"));
    } else if (arg == "--gain") {
      options.gain = std::stod(require_value("--gain"));
    } else if (arg == "--send-interval-ms") {
      options.send_interval_ms = static_cast<int>(parse_u32(require_value("--send-interval-ms")));
    } else if (arg == "--ffmpeg") {
      options.ffmpeg_path = require_value("--ffmpeg");
    } else if (arg == "--video-size") {
      options.video_size = static_cast<int>(parse_u32(require_value("--video-size")));
    } else if (arg == "--video-fps") {
      options.video_fps = static_cast<int>(parse_u32(require_value("--video-fps")));
    } else if (arg == "--video-bitrate-kbps") {
      options.video_bitrate_kbps = static_cast<int>(parse_u32(require_value("--video-bitrate-kbps")));
    } else if (arg == "--video-latency-s") {
      options.video_latency_s = std::stod(require_value("--video-latency-s"));
    } else if (arg == "--video-gop") {
      options.video_gop = static_cast<int>(parse_u32(require_value("--video-gop")));
    } else if (arg == "--crop-size") {
      options.crop_size = static_cast<int>(parse_u32(require_value("--crop-size")));
    } else if (arg == "--static-simplify") {
      options.static_simplify = true;
    } else if (arg == "--no-static-simplify") {
      options.static_simplify = false;
    } else if (arg == "--motion-threshold") {
      options.motion_threshold = static_cast<int>(parse_u32(require_value("--motion-threshold")));
    } else if (arg == "--motion-erode-px") {
      options.motion_erode_px = static_cast<int>(parse_u32(require_value("--motion-erode-px")));
    } else if (arg == "--motion-dilate-px") {
      options.motion_dilate_px = static_cast<int>(parse_u32(require_value("--motion-dilate-px")));
    } else if (arg == "--motion-trail-frames") {
      options.motion_trail_frames = static_cast<int>(parse_u32(require_value("--motion-trail-frames")));
    } else if (arg == "--trail-disable-motion-ratio") {
      options.trail_disable_motion_ratio = std::stod(require_value("--trail-disable-motion-ratio"));
    } else if (arg == "--bg-update-alpha") {
      options.bg_update_alpha = std::stod(require_value("--bg-update-alpha"));
    } else if (arg == "--bg-blur-sigma") {
      options.bg_blur_sigma = std::stod(require_value("--bg-blur-sigma"));
    } else if (arg == "--center-clear-size") {
      options.center_clear_size = static_cast<int>(parse_u32(require_value("--center-clear-size")));
    } else if (arg == "--center-clear-radius") {
      options.center_clear_radius = static_cast<int>(parse_u32(require_value("--center-clear-radius")));
    } else if (arg == "--force-monochrome") {
      options.force_monochrome = true;
    } else if (arg == "--test-pattern") {
      options.test_pattern = true;
    } else if (arg == "--preview") {
      options.preview = true;
    } else if (arg == "--viewer-ip") {
      options.viewer_ip = require_value("--viewer-ip");
    } else if (arg == "--viewer-port") {
      options.viewer_port = static_cast<int>(parse_u32(require_value("--viewer-port")));
    } else if (arg == "--video-serial") {
      options.video_serial = require_value("--video-serial");
    } else if (arg == "--video-serial-baud") {
      options.video_serial_baud = parse_u32(require_value("--video-serial-baud"));
    } else if (arg == "--help" || arg == "-h") {
      print_help();
      std::exit(0);
    } else {
      throw std::runtime_error("未知参数: " + arg);
    }
  }

  return options;
}

}  // namespace bridge
