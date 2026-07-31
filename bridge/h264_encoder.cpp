#include "bridge/h264_encoder.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

namespace bridge
{

namespace
{

constexpr std::size_t kTargetSliceMaxBytes = 280U;

struct AnnexBStartCode
{
  std::size_t offset = 0;
  std::size_t bytes = 0;
};

bool find_annexb_start_code(
  const std::vector<uint8_t> & buffer,
  std::size_t from,
  AnnexBStartCode & start_code)
{
  if (buffer.size() < 3 || from >= buffer.size()) {
    return false;
  }

  for (std::size_t i = from; i + 2 < buffer.size(); ++i) {
    if (buffer[i] != 0 || buffer[i + 1] != 0) {
      continue;
    }
    if (buffer[i + 2] == 1) {
      start_code.offset = i;
      start_code.bytes = 3;
      return true;
    }
    if (i + 3 < buffer.size() && buffer[i + 2] == 0 && buffer[i + 3] == 1) {
      start_code.offset = i;
      start_code.bytes = 4;
      return true;
    }
  }

  return false;
}

}  // namespace

H264EncoderProcess::H264EncoderProcess() = default;

H264EncoderProcess::~H264EncoderProcess()
{
  stop();
}

bool H264EncoderProcess::started() const
{
  return pid_ > 0;
}

uint16_t H264EncoderProcess::input_width() const
{
  return input_width_;
}

uint16_t H264EncoderProcess::input_height() const
{
  return input_height_;
}

void H264EncoderProcess::start(uint16_t width, uint16_t height, const Options & options)
{
  stop();

  input_width_ = width;
  input_height_ = height;

  int stdin_pipe[2] = {-1, -1};
  int stdout_pipe[2] = {-1, -1};
  int stderr_pipe[2] = {-1, -1};

  if (::pipe(stdin_pipe) != 0 || ::pipe(stdout_pipe) != 0 || ::pipe(stderr_pipe) != 0) {
    close_pair(stdin_pipe);
    close_pair(stdout_pipe);
    close_pair(stderr_pipe);
    throw std::runtime_error("创建 ffmpeg 管道失败: " + std::string(std::strerror(errno)));
  }

  const auto clamped_size = std::clamp(options.video_size, 120, 480);
  const auto clamped_fps = std::clamp(options.video_fps, 2, 60);  //10
  const auto clamped_bitrate = std::clamp(options.video_bitrate_kbps, 40, 116);
  const auto clamped_gop = std::clamp(options.video_gop, 1, clamped_fps * 600); //
  const std::string input_size = std::to_string(width) + "x" + std::to_string(height);
  const std::string video_filter =
    "hqdn3d=4:3:6:4,"
    "eq=contrast=1.12:saturation=0.75:gamma=1.05,"
    "format=yuv420p";
  // 折中方案(延迟~3s): 去掉 zerolatency, 开启 mbtree/B帧/多参考帧提升画质,
  // 但把 rc-lookahead 限制在 30 帧(≈1s)控制编码前向延迟
  const std::string x264_params =
    "repeat-headers=1:nal-hrd=cbr:force-cfr=1:ref=5:bframes=2:"
    "b-adapt=2:rc-lookahead=30:sync-lookahead=20:"
    "aq-mode=2:aq-strength=1.2:mbtree=1:qcomp=0.75:"
    "subme=9:trellis=2:deblock=1,1:"
    "slice-max-size=" +
    std::to_string(kTargetSliceMaxBytes);

  std::vector<std::string> args = {
    options.ffmpeg_path,
    "-hide_banner",
    "-loglevel", "warning",
    "-f", "rawvideo",
    "-pix_fmt", "bgr24",
    "-s", input_size,
    "-r", std::to_string(clamped_fps),
    "-i", "pipe:0",
    "-an",
    "-vf", video_filter,
    "-c:v", "libx264",
    "-preset", "veryslow",
    "-b:v", std::to_string(clamped_bitrate) + "k",
    "-maxrate", std::to_string(clamped_bitrate) + "k",
    // bufsize = bitrate × 5 ≈ 580k ≈ 72.5 kB, 允许 I 帧 ~35 kB → 传输约 2-3 秒
    "-bufsize", std::to_string(clamped_bitrate * 5) + "k",
    "-g", std::to_string(clamped_gop),
    "-keyint_min", std::to_string(clamped_gop / 2),
    "-sc_threshold", "40",
    "-x264-params", x264_params,
    "-pix_fmt", "yuv420p",
    "-f", "h264",
    "pipe:1"};

  const auto child_pid = ::fork();
  if (child_pid < 0) {
    close_pair(stdin_pipe);
    close_pair(stdout_pipe);
    close_pair(stderr_pipe);
    throw std::runtime_error("启动 ffmpeg 失败: " + std::string(std::strerror(errno)));
  }

  if (child_pid == 0) {
    ::dup2(stdin_pipe[0], STDIN_FILENO);
    ::dup2(stdout_pipe[1], STDOUT_FILENO);
    ::dup2(stderr_pipe[1], STDERR_FILENO);
    close_pair(stdin_pipe);
    close_pair(stdout_pipe);
    close_pair(stderr_pipe);

    std::vector<char *> argv;
    argv.reserve(args.size() + 1U);
    for (auto & arg : args) {
      argv.push_back(arg.data());
    }
    argv.push_back(nullptr);
    ::execvp(argv[0], argv.data());
    ::_exit(127);
  }

  pid_ = child_pid;
  close_fd(stdin_pipe[0]);
  close_fd(stdout_pipe[1]);
  close_fd(stderr_pipe[1]);
  stdin_fd_ = stdin_pipe[1];
  stdout_fd_ = stdout_pipe[0];
  stderr_fd_ = stderr_pipe[0];

  stdout_thread_ = std::thread([this] { stdout_loop(); });
  stderr_thread_ = std::thread([this] { stderr_loop(); });

  std::cout << "[video-0310] ffmpeg started input=" << input_size
            << " output=" << clamped_size << "x" << clamped_size
            << " fps=" << clamped_fps
            << " bitrate=" << clamped_bitrate << "kbit/s"
            << " gop=" << clamped_gop
            << " slice_max=" << kTargetSliceMaxBytes << "B" << std::endl;
}

void H264EncoderProcess::stop()
{
  if (stdin_fd_ >= 0) {
    close_fd(stdin_fd_);
  }

  if (pid_ > 0) {
    ::kill(pid_, SIGTERM);
  }

  if (stdout_thread_.joinable()) {
    stdout_thread_.join();
  }
  if (stderr_thread_.joinable()) {
    stderr_thread_.join();
  }

  if (pid_ > 0) {
    int status = 0;
    ::waitpid(pid_, &status, 0);
    pid_ = -1;
  }

  close_fd(stdout_fd_);
  close_fd(stderr_fd_);

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  encoded_buffer_.clear();
}

bool H264EncoderProcess::submit_rgb_frame(const std::vector<uint8_t> & frame)
{
  if (stdin_fd_ < 0 || frame.empty()) {
    return false;
  }

  std::size_t offset = 0;
  while (offset < frame.size()) {
    const auto written = ::write(stdin_fd_, frame.data() + offset, frame.size() - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    if (written < 0 && errno == EPIPE) {
      return false;
    }
    throw std::runtime_error("写入 ffmpeg stdin 失败: " + std::string(std::strerror(errno)));
  }
  return true;
}

bool H264EncoderProcess::pop_chunk(
  std::array<uint8_t, protocol::kCustomClientVideo0310PayloadBytes> & chunk,
  std::size_t & chunk_size)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  chunk.fill(0);
  if (encoded_buffer_.empty()) {
    chunk_size = 0;
    return false;
  }

  AnnexBStartCode start_code{};
  if (find_annexb_start_code(encoded_buffer_, 0, start_code) && start_code.offset > 0) {
    encoded_buffer_.erase(
      encoded_buffer_.begin(),
      encoded_buffer_.begin() + static_cast<long>(start_code.offset));
  }

  std::size_t packet_bytes = 0;
  while (!encoded_buffer_.empty()) {
    AnnexBStartCode current_start{};
    if (!find_annexb_start_code(encoded_buffer_, 0, current_start) || current_start.offset != 0) {
      if (packet_bytes > 0) {
        break;
      }

      chunk_size = std::min(chunk.size(), encoded_buffer_.size());
      std::copy_n(encoded_buffer_.begin(), chunk_size, chunk.begin());
      encoded_buffer_.erase(
        encoded_buffer_.begin(),
        encoded_buffer_.begin() + static_cast<long>(chunk_size));
      return true;
    }

    AnnexBStartCode next_start{};
    if (!find_annexb_start_code(encoded_buffer_, current_start.bytes, next_start)) {
      break;
    }

    const std::size_t nal_bytes = next_start.offset;
    if (nal_bytes > chunk.size()) {
      if (packet_bytes == 0) {
        chunk_size = chunk.size();
        std::copy_n(encoded_buffer_.begin(), chunk_size, chunk.begin());
        encoded_buffer_.erase(
          encoded_buffer_.begin(),
          encoded_buffer_.begin() + static_cast<long>(chunk_size));
        return true;
      }
      break;
    }

    if (packet_bytes + nal_bytes > chunk.size()) {
      break;
    }

    std::copy_n(encoded_buffer_.begin(), nal_bytes, chunk.begin() + static_cast<long>(packet_bytes));
    packet_bytes += nal_bytes;
    encoded_buffer_.erase(
      encoded_buffer_.begin(),
      encoded_buffer_.begin() + static_cast<long>(nal_bytes));
  }

  if (packet_bytes > 0) {
    chunk_size = packet_bytes;
    return true;
  }

  chunk_size = 0;
  return false;
}

std::size_t H264EncoderProcess::queued_bytes() const
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  return encoded_buffer_.size();
}

void H264EncoderProcess::close_fd(int & fd)
{
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

void H264EncoderProcess::close_pair(int (&fds)[2])
{
  close_fd(fds[0]);
  close_fd(fds[1]);
}

void H264EncoderProcess::stdout_loop()
{
  std::array<uint8_t, 4096> buffer{};
  while (true) {
    const auto bytes_read = ::read(stdout_fd_, buffer.data(), buffer.size());
    if (bytes_read > 0) {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      encoded_buffer_.insert(encoded_buffer_.end(), buffer.begin(), buffer.begin() + bytes_read);
      continue;
    }
    if (bytes_read < 0 && errno == EINTR) {
      continue;
    }
    break;
  }
}

void H264EncoderProcess::stderr_loop()
{
  std::array<char, 512> buffer{};
  std::string line;
  while (true) {
    const auto bytes_read = ::read(stderr_fd_, buffer.data(), buffer.size());
    if (bytes_read > 0) {
      line.append(buffer.data(), buffer.data() + bytes_read);
      std::size_t newline = 0;
      while ((newline = line.find('\n')) != std::string::npos) {
        const auto message = line.substr(0, newline);
        line.erase(0, newline + 1U);
        if (!message.empty()) {
          std::cerr << "[ffmpeg] " << message << std::endl;
        }
      }
      continue;
    }
    if (bytes_read < 0 && errno == EINTR) {
      continue;
    }
    break;
  }
}

}  // namespace bridge
