# minimal_hik_gimbal_bridge

`minimal_hik_gimbal_bridge` 是一个独立于 ROS 的机载端最小工程，用来把海康相机画面直接接到 RoboMaster 2026 官方 `0x0310` 自定义客户端链路上。

它的职责很明确：

- 按配置绑定指定海康相机；未配置时打开第一台海康相机
- 打开图传发送端 USB-TTL 串口，默认 `/dev/ttyUSB0 @ 921600`
- 默认生成官方裁判系统 `0x0310` 帧并直接写入图传发送端

## 功能概览

- 海康相机自动枚举与取流
- 海康相机掉线后持续重连
- 图传发送端串口掉线后持续重连
- 串口 `921600 8N1` 通讯
- 官方 `0x0310` 视频模式
- 紧凑 `0x0310` H264 分片
- `--preview` 原画预览和曝光/增益实时调节，支持按键保存配置
- OpenCV 静态简化 / 运动掩码 / 拖影预处理

## 目录结构

```text
minimal_hik_gimbal_bridge/
├── CMakeLists.txt
├── bridge/
│   ├── camera_preview.cpp / camera_preview.hpp
│   ├── frame_preprocessor.cpp / frame_preprocessor.hpp
│   ├── h264_encoder.cpp / h264_encoder.hpp
│   ├── hik_camera.cpp / hik_camera.hpp
│   ├── options.cpp / options.hpp
│   ├── protocol.hpp
│   ├── serial_port.cpp / serial_port.hpp
│   └── udp_sender.cpp / udp_sender.hpp
├── config/
│   └── bridge.yaml
├── deploy/
├── scripts/
├── src/
│   └── main.cpp
├── third_party/
│   ├── hikrobot/
│   └── serial/
├── tools/
│   └── player_end_emulator.py
└── README.md
```

`tools/player_end_emulator.py` 只用于实验室：它把串口里的官方 `0x0310` 帧转成 MQTT `CustomByteBlock`，方便没有真实选手端时复现官方链路。

## 环境要求

- Ubuntu Linux
- CMake 3.16+
- C++17 编译器
- OpenCV（`core` + `imgproc` + `highgui`）
- 系统 `ffmpeg`

推荐安装：

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev ffmpeg
```

## 海康 SDK

仓库已经自带一份 HikRobot SDK：

- `third_party/hikrobot/include`
- `third_party/hikrobot/lib/<amd64|arm64>`

默认构建直接使用这份 vendored SDK，不再依赖外部 `sp_vision` 工程。

USB 相机运行时还需要 MVS 的 transport layer（例如 `libMvUsb3vTL.so`）。`scripts/run-bridge.sh` 会在存在 `/opt/MVS/lib/64` 时自动把它加进 `LD_LIBRARY_PATH`，这样真实 USB 相机可以正常枚举；如果你的 MVS 安装在其他位置，可以运行前设置 `SYSTEM_MVS_RUNTIME_PATH=/path/to/MVS/lib/64`。

如果你的 SDK 放在别处，编译时指定：

```bash
cmake -S . -B build -DHIKROBOT_SDK_DIR=/your/hikrobot
```

## YAML 配置

曝光、增益和图像旋转矩阵现在默认从 `config/bridge.yaml` 读取，直接改这个文件：

```yaml
camera:
  serial_number: "DA0000000"
  exposure_ms: 10.0
  gain: 12.0
image:
  rotation_matrix: !!opencv-matrix
    rows: 2
    cols: 2
    dt: d
    data: [ 1., 0., 0., 1. ]
  crop_center_x: 0.5
  crop_center_y: 0.5
```

如果你想用别的文件，运行时加：

```bash
./build/minimal_hik_gimbal_bridge --config /path/to/bridge.yaml
```

如果现场会同时插多台海康 USB 相机，先只插目标相机，运行：

```bash
./scripts/run-bridge.sh --list-cameras
```

把输出里的 `serial_number` 写入 `camera.serial_number`，之后程序会固定打开这台相机。也可以临时用命令行覆盖：

```bash
./scripts/run-bridge.sh --camera-serial DA0000000
```

旋转矩阵只从 YAML 读取；命令行参数仍然只覆盖 `--exposure-ms` 和 `--gain`。

如果你开了 `--preview`，可以在预览窗口里直接拖动 ROI 方框，选择实际发送出去的画面区域；按 `S` 会把当前曝光、增益、旋转矩阵和 ROI 中心一起保存回 YAML。

## 编译

```bash
cmake -S . -B build
cmake --build build -j
```

编译产物：

```text
build/minimal_hik_gimbal_bridge
```

## 这条链路做的是什么

当前主路径是直连图传发送端串口：

海康相机 -> H264 -> 紧凑视频分片 -> 官方 `0x0310` 裁判帧 -> `/dev/ttyUSB0` -> 图传发送端。

官方约束已经固化到默认值里：

- 串口：`921600 8N1`
- 命令号：`0x0310`
- 自定义数据段：固定 300 字节
- 发送频率：50Hz，也就是每 20ms 一包

## `0x0310` 视频模式

默认工作模式是 `0x0310` 视频：

海康帧 -> RGB24 -> OpenCV 预处理 -> ffmpeg/libx264 -> 紧凑视频分片 -> `0x0310`

### 预处理内容

当前默认预处理包括：

- 中心正方形裁剪
- 按 YAML 旋转矩阵做旋转校正
- 固定边长缩放到 `video_size`
- 中心圆形 ROI 保真，默认半径 `112` 像素
- 圆外直接压黑，尽量把码率留给中心区域
- 如关闭中心圆模式，才回退到外围静态区域灰度化、降纹理和模糊
- 后级轻量降噪与颜色压缩优化

### 紧凑视频分片格式

`0x0310` 数据段固定 300 字节：

- 3 字节头：`flags_and_payload_hi` / `sequence` / `payload_bytes_lo`
- 297 字节 H264 净荷

约束：

- 默认 H264
- 默认 50Hz（每 20ms 一包）
- 默认串口波特率 `921600`
- `flags & 1` 表示码流重启
- 编码侧按接近单包净荷的 slice 大小收敛，尽量减少一个丢包破坏多个 NAL

## 编码和串口建议

- `0x0310` 50Hz 建议使用 `921600 8N1`
- 不要用 `115200` 负担 333 字节的高频 relay 包
- 默认码率已经顶到官方小包链路净荷上限附近，约 `116 kbit/s`
- 默认 GOP 已缩到 `10`，无重传链路下更快从丢包中恢复

## 运行

标准视频模式直接运行脚本即可：

```bash
./scripts/run-bridge.sh
```

这个脚本只做两件事：设置海康 MVS 运行时路径，然后启动 `build/minimal_hik_gimbal_bridge`。
默认会把完整 MVS 运行时 `/opt/MVS/lib/64` 和仓库内 `third_party/hikrobot/lib/<arch>` 加进 `LD_LIBRARY_PATH`。

等价命令：

```bash
LD_LIBRARY_PATH=/opt/MVS/lib/64:./third_party/hikrobot/lib/amd64 ./build/minimal_hik_gimbal_bridge
```

一键安装本机自启动：

```bash
./scripts/install-autostart.sh
```

这个脚本会自动构建 bridge，然后在当前仓库下生成 `bin/` 并安装桌面自启动项 `RoboMaster Hik Bridge`。
开机自启日志默认写到：`logs/bridge-autostart.log`。

如果你已经手动编译过，可以跳过构建：

```bash
./scripts/install-autostart.sh --skip-build
```

查看车载端开机自启日志：

```bash
tail -f logs/bridge-autostart.log
```

默认行为：

- 自动打开第一台海康相机
- 海康相机掉线后每 `200ms` 持续重连
- 自动打开 `/dev/ttyUSB0` 图传发送端串口
- 图传发送端串口掉线后每 `200ms` 持续重连
- 每 20ms 生成并发送一包官方 `0x0310` 数据
- 默认在 `0x0310` 自定义数据段中发送紧凑 H264 分片
- 默认直接封成官方 `0x0310` 裁判帧写入图传发送端
- 默认关闭本地 `0x0310` UDP 调试输出
- 每秒打印抓帧、FPS、发送计数和 backlog

## 常用参数

- `--video-serial <path>`：直连图传发送端的 USB-TTL 串口路径，默认 `/dev/ttyUSB0`
- `--video-serial-baud <rate>`：图传发送端串口波特率，默认 `921600`
- `--config <path>`：YAML 配置文件，默认自动尝试 `config/bridge.yaml`
- `--camera-serial <serial>`：绑定指定海康相机序列号，覆盖 YAML `camera.serial_number`
- `--list-cameras`：列出当前 USB 海康相机后退出
- `--preview`：显示海康原画，并在窗口里直接调曝光和增益
- `--ffmpeg <path>`：ffmpeg 路径
- `--video-size <n>`：输出边长，默认 `300`
- `--video-fps <n>`：编码帧率，默认 `30`
- `--video-bitrate-kbps <n>`：码率，默认 `116`
- `--video-gop <n>`：GOP，默认 `10`
- `--crop-size <n>`：中心裁剪边长
- `--no-static-simplify`：关闭静态简化和拖影
- `--motion-threshold <n>`：运动阈值
- `--motion-erode-px <n>`：掩码腐蚀
- `--motion-dilate-px <n>`：掩码膨胀
- `--motion-trail-frames <n>`：拖影历史帧数，默认 `0`
- `--trail-disable-motion-ratio <f>`：全局运动比例过高时禁用拖影
- `--bg-update-alpha <f>`：背景更新速度
- `--bg-blur-sigma <f>`：静态区域模糊强度，默认 `1.9`
- `--center-clear-size <n>`：中心保真区边长，默认 `170`
- `--center-clear-radius <n>`：中心圆形保真半径，默认 `112`；大于 `0` 时圆外压黑
- `--force-monochrome`：强制灰度
- `--viewer-ip <ip>`：打开本地 `0x0310` UDP 调试输出，正常官方链路不需要

## 裸编码排障

如果你怀疑问题出在预处理阶段，可以临时关闭它：

```bash
./build/minimal_hik_gimbal_bridge \
  --no-static-simplify
```

## 解码端对接约定

图传发送端收到的是完整官方裁判系统 `0x0310` 帧。解码端需要做的只有两步：

1. 从 `0x0310` 自定义数据段取出 300 字节数据
2. 按 3 字节紧凑头解析 H264 分片并送入解码器

不需要做的事情：

- 不需要重新编码视频
- 不需要处理历史兼容头

## 最小验证流程

1. 先只接海康相机，确认 `frame_seq` 在增长。
2. 再接串口，确认 `sent` 在增长。
3. 最后在解码端订阅 `CustomByteBlock`，确认紧凑分片和 H264 解码正常。

## 常见问题

### 1. 为什么现在可以直接往图传发送端串口发？

这台电脑已经通过 USB-TTL 接到图传发送端串口。按官方协议，图传发送端串口就是 `921600 8N1` 的裁判系统帧入口；这里写入的是完整 `0x0310` 官方帧，不是自定义私有包。

### 2. 为什么必须 `921600`？

`0x0310` 视频模式是 50Hz 高频小包链路，`115200` 的物理带宽通常不够稳。

### 3. backlog 偶尔升高是否正常？

少量抖动是正常的，只要 backlog 能持续回落、远端能正常解码即可。

cmake -S . -B build
cmake --build build -j
./build/minimal_hik_gimbal_bridge --viewer-ip 127.0.0.1 --viewer-port 3335