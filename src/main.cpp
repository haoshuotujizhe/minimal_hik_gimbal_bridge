#include "bridge/camera_preview.hpp"
#include "bridge/common.hpp"
#include "bridge/frame.hpp"
#include "bridge/frame_preprocessor.hpp"
#include "bridge/h264_encoder.hpp"
#include "bridge/hik_camera.hpp"
#include "bridge/options.hpp"
#include "bridge/protocol.hpp"
#include "bridge/serial_port.hpp"
#include "bridge/udp_sender.hpp"

#include <opencv2/core/mat.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

namespace
{

std::atomic_bool g_running = true;

struct AnnexBStartCode
{
  std::size_t offset = 0;
  std::size_t bytes = 0;
};

bool find_annexb_start_code(
  const uint8_t * buffer,
  std::size_t size,
  std::size_t from,
  AnnexBStartCode & start_code)
{
  if (buffer == nullptr || size < 3 || from >= size) {
    return false;
  }

  for (std::size_t i = from; i + 2 < size; ++i) {
    if (buffer[i] != 0 || buffer[i + 1] != 0) {
      continue;
    }
    if (buffer[i + 2] == 1) {
      start_code.offset = i;
      start_code.bytes = 3;
      return true;
    }
    if (i + 3 < size && buffer[i + 2] == 0 && buffer[i + 3] == 1) {
      start_code.offset = i;
      start_code.bytes = 4;
      return true;
    }
  }

  return false;
}

bool chunk_contains_sps(const uint8_t * buffer, std::size_t size)
{
  AnnexBStartCode start_code{};
  std::size_t search_from = 0;
  while (find_annexb_start_code(buffer, size, search_from, start_code)) {
    const auto nal_header_index = start_code.offset + start_code.bytes;
    if (nal_header_index < size) {
      const auto nal_type = static_cast<uint8_t>(buffer[nal_header_index] & 0x1FU);
      // 只检查 SPS (type 7)，IDR slice (type 5) 不标记为重置
      if (nal_type == 7U) {
        return true;
      }
    }
    search_from = start_code.offset + start_code.bytes;
  }
  return false;
}

void on_signal(int)
{
  g_running = false;
}

}  // namespace

int main(int argc, char ** argv)
{
  g_running = true;

  try {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    bridge::Options options = bridge::parse_args(argc, argv);
    if (options.list_cameras) {
      const auto devices = bridge::HikCamera::list_devices();
      if (devices.empty()) {
        std::cout << "[bridge] no Hik USB cameras found." << std::endl;
      }
      for (const auto & device : devices) {
        std::cout << "[bridge] camera index=" << device.index
                  << " serial_number=" << (device.serial_number.empty() ? "<empty>" : device.serial_number)
                  << " model=" << (device.model_name.empty() ? "<empty>" : device.model_name)
                  << " user_defined_name=" << (device.user_defined_name.empty() ? "<empty>" : device.user_defined_name)
                  << std::endl;
      }
      return 0;
    }

    bridge::CameraPreviewWindow preview(options);

    if (!options.config_path.empty()) {
      std::cout << "[bridge] loaded config: " << options.config_path
                << " exposure_ms=" << options.exposure_ms
                << " camera_serial_number=" << (options.camera_serial_number.empty() ? "<first>" : options.camera_serial_number)
                << " crop_center=(" << options.crop_center_x << ',' << options.crop_center_y << ')'
                << " gain=" << options.gain
                << " rotation_matrix=[[" << options.rotation_matrix[0] << ',' << options.rotation_matrix[1]
                << "],[" << options.rotation_matrix[2] << ',' << options.rotation_matrix[3] << "]]"
                << std::endl;
    }

    bridge::HikCamera camera;
    if (options.test_pattern) {
      std::cout << "[bridge] using built-in test pattern source." << std::endl;
    } else {
      std::cout << "[bridge] Hik camera reconnect loop enabled." << std::endl;
      if (!options.camera_serial_number.empty()) {
        std::cout << "[bridge] binding Hik camera serial_number=" << options.camera_serial_number << std::endl;
      }
    }
    if (preview.enabled()) {
      std::cout << "[bridge] raw preview enabled; adjust exposure/gain in the preview window, press S to save YAML." << std::endl;
    }

    bridge::UdpSender viewer_udp;
    if (!options.viewer_ip.empty()) {
      if (viewer_udp.open(options.viewer_ip, options.viewer_port)) {
        std::cout << "[bridge] 0x0310 UDP target: " << options.viewer_ip << ":" << options.viewer_port << std::endl;
      } else {
        std::cerr << "[bridge] WARNING: 0x0310 UDP not available, video will only go via serial" << std::endl;
      }
    }

    bridge::SerialPort video_serial;
    if (!options.video_serial.empty()) {
      std::cout << "[bridge] 0x0310 video serial reconnect loop: " << options.video_serial
                << " baud=" << options.video_serial_baud << std::endl;
    }

    bridge::FrameInfo latest_frame{};
    bridge::FramePreprocessor preprocessor(options);
    bridge::H264EncoderProcess video_encoder;
    std::atomic_uint32_t video_sequence = 0;
    std::atomic_bool video_reset_pending = true;
    auto last_video_submit = bridge::Clock::time_point{};
    std::atomic_uint32_t sent_packets = 0;
    uint32_t frames_in_window = 0;
    double fps = 0.0;
    auto last_report = bridge::Clock::now();
    auto fps_window = bridge::Clock::now();
    auto last_test_frame = bridge::Clock::time_point{};
    uint32_t test_pattern_sequence = 0;
    auto next_camera_retry = bridge::Clock::time_point{};
    bridge::Clock::time_point last_camera_open_error{};
    bridge::Clock::time_point last_camera_runtime_error{};
    std::atomic_uint32_t video_serial_seq = 0;

    const auto maybe_open_camera = [&](bridge::Clock::time_point now) {
      if (options.test_pattern || camera.is_open() || now < next_camera_retry) {
        return;
      }

      try {
        camera.open_first(options.exposure_ms, options.gain, options.camera_serial_number);
        std::cout << "[bridge] Hik camera ready. exposure_ms=" << options.exposure_ms
                  << " gain=" << options.gain << std::endl;
      } catch (const std::exception & error) {
        next_camera_retry = now + bridge::kDeviceReconnectInterval;
        if (bridge::should_log_at_interval(now, last_camera_open_error, bridge::kReconnectLogInterval)) {
          std::cerr << "[bridge] Hik camera reconnect failed: " << error.what() << std::endl;
        }
      }
    };

    std::cout << "[bridge] preprocess crop=" << options.crop_size
              << " output=" << preprocessor.output_size() << 'x' << preprocessor.output_size()
              << " center_circle_radius=" << options.center_clear_radius
              << " static_simplify=" << (options.static_simplify ? "on" : "off")
              << " trail=" << options.motion_trail_frames
              << " erode=" << options.motion_erode_px
              << " dilate=" << options.motion_dilate_px
              << " mono=" << (options.force_monochrome ? "on" : "off")
              << std::endl;

    const auto send_interval = std::chrono::milliseconds(options.send_interval_ms);
    std::thread sender_thread([&]() {
      auto next_send = bridge::Clock::now() + send_interval;
      auto next_video_serial_retry = bridge::Clock::time_point{};
      bridge::Clock::time_point last_video_serial_open_error{};
      bridge::Clock::time_point last_video_serial_write_error{};

      while (g_running.load()) {
        const auto now = bridge::Clock::now();
        if (now < next_send) {
          std::this_thread::sleep_until(next_send);
          continue;
        }
        while (next_send <= now) {
          next_send += send_interval;
        }

        if (!options.video_serial.empty() && !video_serial.is_open() && now >= next_video_serial_retry) {
          try {
            video_serial.open_or_throw(options.video_serial, options.video_serial_baud);
            std::cout << "[bridge] 0x0310 video serial ready: " << options.video_serial
                      << " baud=" << options.video_serial_baud << std::endl;
          } catch (const std::exception & error) {
            next_video_serial_retry = now + bridge::kDeviceReconnectInterval;
            if (bridge::should_log_at_interval(now, last_video_serial_open_error, bridge::kReconnectLogInterval)) {
              std::cerr << "[bridge] 0x0310 video serial reconnect failed: " << error.what() << std::endl;
            }
          }
        }

        std::array<uint8_t, bridge::protocol::kCustomClientVideo0310PayloadBytes> video_chunk{};
        std::size_t video_chunk_size = 0;
        const bool packet_ready = video_encoder.pop_chunk(video_chunk, video_chunk_size);
        if (!packet_ready) {
          continue;
        }

        const bool has_sps = chunk_contains_sps(video_chunk.data(), video_chunk_size);
        const bool mark_reset = video_reset_pending.load() && has_sps;

        bridge::protocol::CustomClientVideo0310Chunk packet{};
        bridge::protocol::fill_custom_client_0310_video_chunk(
          packet,
          mark_reset ? bridge::protocol::kCustomClientVideo0310FlagReset : 0U,
          static_cast<uint8_t>(video_sequence.fetch_add(1)),
          video_chunk.data(),
          video_chunk_size);

        if (video_serial.is_open()) {
          try {
            uint8_t ref_frame[320];
            std::size_t ref_len = 0;
            bridge::protocol::build_referee_frame(
              ref_frame, ref_len,
              bridge::protocol::kCustomClient0310CmdId,
              static_cast<uint8_t>(video_serial_seq.fetch_add(1)),
              reinterpret_cast<const uint8_t *>(&packet),
              sizeof(packet));
            video_serial.write_all(ref_frame, ref_len);
          } catch (const std::exception & error) {
            video_serial.close();
            next_video_serial_retry = now + bridge::kDeviceReconnectInterval;
            if (bridge::should_log_at_interval(now, last_video_serial_write_error, bridge::kReconnectLogInterval)) {
              std::cerr << "[video-serial] write error, retrying: " << error.what() << std::endl;
            }
          }
        }
        viewer_udp.send(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        sent_packets.fetch_add(1);
        if (mark_reset && has_sps) {
          video_reset_pending = false;
        }
      }
    });

    while (g_running.load()) {
      const auto now = bridge::Clock::now();
      maybe_open_camera(now);
      preprocessor.sync_runtime_options(options);

      bridge::FrameInfo frame{};
      bool have_frame = false;
      const bool need_rgb24 = true;
      if (options.test_pattern) {
        const auto test_frame_interval = std::chrono::milliseconds(
          std::max(1, 1000 / std::clamp(options.video_fps, 1, 120)));
        if (last_test_frame == bridge::Clock::time_point{} || now - last_test_frame >= test_frame_interval) {
          frame = bridge::make_test_pattern_frame(test_pattern_sequence++, now);
          last_test_frame = now;
          have_frame = true;
        }
      } else if (camera.is_open()) {
        const auto grab_result = camera.grab(frame, 50, need_rgb24);
        if (grab_result == bridge::CameraGrabResult::Success) {
          have_frame = true;
        } else if (grab_result == bridge::CameraGrabResult::DeviceError) {
          next_camera_retry = now + bridge::kDeviceReconnectInterval;
          if (bridge::should_log_at_interval(now, last_camera_runtime_error, bridge::kReconnectLogInterval)) {
            std::cerr << "[bridge] Hik camera lost, retrying: " << camera.last_error() << std::endl;
          }
        }
      }

      if (have_frame) {
        latest_frame = frame;
        ++frames_in_window;

        if (!latest_frame.rgb24.empty()) {
          if (!video_encoder.started() ||
              video_encoder.input_width() != static_cast<uint16_t>(preprocessor.output_size()) ||
              video_encoder.input_height() != static_cast<uint16_t>(preprocessor.output_size())) {
            video_encoder.start(
              static_cast<uint16_t>(preprocessor.output_size()),
              static_cast<uint16_t>(preprocessor.output_size()),
              options);
            video_sequence = 0;
            video_reset_pending = true;
            last_video_submit = bridge::Clock::time_point{};
          }

          const auto frame_interval = std::chrono::milliseconds(
            std::max(1, 1000 / std::clamp(options.video_fps, 1, 120)));
          if (last_video_submit == bridge::Clock::time_point{} || now - last_video_submit >= frame_interval) {
            cv::Mat processed = preprocessor.process_rgb24(latest_frame);
            std::vector<uint8_t> processed_bytes;
            if (!processed.empty()) {
              if (!processed.isContinuous()) {
                processed = processed.clone();
              }
              processed_bytes.assign(processed.datastart, processed.dataend);
            }

            if (processed_bytes.empty() || !video_encoder.submit_rgb_frame(processed_bytes)) {
              std::cerr << "[video-0310] ffmpeg stdin closed, restarting encoder" << std::endl;
              video_encoder.start(
                static_cast<uint16_t>(preprocessor.output_size()),
                static_cast<uint16_t>(preprocessor.output_size()),
                options);
              video_sequence = 0;
              video_reset_pending = true;
            }
            last_video_submit = now;
          }
        }
      } else if (options.test_pattern) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      preview.pump(options, camera, have_frame ? &latest_frame : nullptr);

      const auto fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_window).count();
      if (fps_elapsed >= 1000) {
        fps = frames_in_window * 1000.0 / static_cast<double>(fps_elapsed);
        frames_in_window = 0;
        fps_window = now;
      }

      if (now - last_report >= std::chrono::seconds(1)) {
        std::cout << "[bridge] frame_seq=" << latest_frame.sequence
                  << " resolution=" << latest_frame.width << 'x' << latest_frame.height
                  << " camera=" << (options.test_pattern ? "test" : (camera.is_open() ? "online" : "reconnecting"))
                  << " fps=" << std::fixed << std::setprecision(1) << fps
                  << " sent=" << sent_packets.load()
                  << " video_tx=" << (video_serial.is_open() ? "online" : "reconnecting")
                  << " video_backlog=" << video_encoder.queued_bytes()
                  << " video_seq=" << video_sequence.load();
        std::cout << std::endl;
        last_report = now;
      }
    }

    g_running = false;
    if (sender_thread.joinable()) {
      sender_thread.join();
    }
    return 0;
  } catch (const std::exception & error) {
    g_running = false;
    std::cerr << "[bridge] fatal: " << error.what() << std::endl;
    return 1;
  }
}
