#pragma once

#include "bridge/options.hpp"
#include "bridge/protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <sys/types.h>
#include <thread>
#include <vector>

namespace bridge
{

class H264EncoderProcess
{
public:
  H264EncoderProcess();
  ~H264EncoderProcess();

  bool started() const;
  uint16_t input_width() const;
  uint16_t input_height() const;

  void start(uint16_t width, uint16_t height, const Options & options);
  void stop();
  bool submit_rgb_frame(const std::vector<uint8_t> & frame);
  bool pop_chunk(
    std::array<uint8_t, protocol::kCustomClientVideo0310PayloadBytes> & chunk,
    std::size_t & chunk_size);
  std::size_t queued_bytes() const;
  /// 丢弃编码缓冲区中第一个 resync NAL (SPS/PPS/IDR) 之前的所有数据。
  /// 用于限速积压超限时裁剪，保证接收端能通过 reset flag 快速重新同步。
  void drop_to_resync_nal();
private:
  static void close_fd(int & fd);
  static void close_pair(int (&fds)[2]);

  void stdout_loop();
  void stderr_loop();

  pid_t pid_ = -1;
  int stdin_fd_ = -1;
  int stdout_fd_ = -1;
  int stderr_fd_ = -1;
  uint16_t input_width_ = 0;
  uint16_t input_height_ = 0;
  std::thread stdout_thread_;
  std::thread stderr_thread_;
  mutable std::mutex buffer_mutex_;
  std::vector<uint8_t> encoded_buffer_;
};

}  // namespace bridge
