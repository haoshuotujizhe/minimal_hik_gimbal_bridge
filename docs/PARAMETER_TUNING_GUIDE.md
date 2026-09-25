# minimal_hik_gimbal_bridge 参数调优指南

> 基于 Pacific_doorlock_sniper 编码逻辑，适用于 RoboMaster 2026 0x0310 视频链路。

---

## 目录

1. [数据流与带宽模型](#1-数据流与带宽模型)
2. [硬限速机制](#2-硬限速机制)
3. [参数速查表](#3-参数速查表)
4. [编码与带宽参数](#4-编码与带宽参数)
5. [预处理与视觉效果参数](#5-预处理与视觉效果参数)
6. [运动检测与拖影参数](#6-运动检测与拖影参数)
7. [相机参数](#7-相机参数)
8. [网络与串口参数](#8-网络与串口参数)
9. [调试参数](#9-调试参数)
10. [常见问题排查](#10-常见问题排查)
11. [调参速查表](#11-调参速查表)

---

## 1. 数据流与带宽模型

```
海康相机 → 中心裁剪 → 缩放到 output_size
    → 背景减除 + 运动检测
        ├── 静态区域: 去饱和 + 高斯模糊 → 低码率
        └── 运动区域: 保留原始像素 + N帧拖影叠加
    → H264编码(libx264) → Annex-B字节流
    → 紧凑分片(297字节/包) → 0x0310帧(300字节/包)
    → UDP 或 图传串口 → 接收端(rm-native-viewer)
```

### 带宽模型

| 约束项 | 数值 | 说明 |
|:---|:---|:---|
| 0x0310 数据段 | 300 字节/包 | 3字节头 + 297字节H264净荷 |
| 每包净荷 | 2.4 Kbit | = 300 bytes × 8 |
| 发送频率 | 50 Hz | = 每 20ms 一包 |
| 物理层带宽上限 | ≈ 15 kB/s (120 kbps) | = 300 bytes × 50 Hz |
| 实际可用视频码率 | ≈ 116 kbps | 减去协议开销后的 H264 流 |

```
┌─────────────────────────────────────────────┐
│          0x0310 数据段 (300 bytes)            │
├──────┬──────┬──────┬─────────────────────────┤
│flags │ seq  │len_lsb│  H264 payload (297B)    │
│ 1B   │ 1B   │ 1B   │                         │
├──────┴──────┴──────┴─────────────────────────┤
│          300 bytes = 2.4 Kbit                │
└─────────────────────────────────────────────┘
```

**关键公式**：
- 实际发送带宽(kbps) = `delta_packets × 300 × 8 / 时间间隔(秒) / 1000`
- 理论最大带宽 = `300 × 8 × 50 / 1000 = 120 kbps`
- H264 编码码率应 ≤ 116 kbps，留 4 kbps 给协议头和抖动

---

## 2. 硬限速机制

### 设计原理

在发送线程中实现了与 Pacific_doorlock_sniper 完全一致的**滑动窗口硬限速**，从传输层面严格保证任何时间窗口内的发送字节数不超过上限，**绝对不会**出现超带宽的情况。

```
发送线程每 20ms 唤醒
  │
  ├─ 1. 从编码器取一包 H264 数据 (pop_chunk)
  │
  ├─ 2. 构造 0x0310 视频分片 (300 bytes)
  │
  ├─ 3. 【硬限速检查】滑动窗口限速
  │     ├─ 驱逐窗口外的旧记录 (>200ms)
  │     ├─ 当前窗口字节数 + 300 > 3000 ?
  │     │   YES → 丢弃本包, rate_limited_drops++, continue
  │     │   NO  → 记录本包, 继续发送
  │     └─
  ├─ 4. 串口发送 (裁判帧封装)
  └─ 5. UDP 发送 (调试用)
```

### 限速参数（硬编码在 `src/main.cpp` 中）

| 参数 | 值 | 说明 |
|:---|:---|:---|
| `kBandwidthLimitBps` | **15000.0** (15 kB/s) | 硬带宽上限 |
| `kRateWindowSec` | **0.2** (200ms) | 滑动窗口长度 |
| `kRateWindowByteLimit` | **3000** bytes | 200ms 窗口内的字节上限 |
| `kPacketWireBytes` | **300** bytes | 每包计入的字节数 |

### 为什么用 200ms 短窗口

| 窗口长度 | 字节上限 | 每窗口最多包数 | 效果 |
|:---|:---|:---|:---|
| 1.0s | 15000 | 50包 | 允许前50包瞬间发出，然后等1秒 |
| **0.2s** | **3000** | **10包** | 每200ms均匀10包，流量平滑 |
| 0.1s | 1500 | 5包 | 过于严格，可能误杀正常抖动 |

200ms 窗口在平滑度和容错性之间取得平衡：每 200ms 最多发 10 包（等效 50Hz 的平均速率），但不会出现 1 秒窗口那种"前 50 包瞬间发出"的突发。

### 带宽保证

```
任何 200ms 窗口内:  sent_bytes ≤ 3000 bytes
任何 1.0s 窗口内:   sent_bytes ≤ 15000 bytes (因为 5×3000)
等效速率:           ≤ 15 kB/s (绝对保证)
```

### 日志解读

```
# 正常情况：看不到限速日志
[bridge] ... tx_bw=14.65kB/s limit=15.0kB/s rate_drops=0 ...

# 编码器短时超过 15kB/s 被拦截
[bridge] RATE LIMIT: window=2800/3000B (14.0kB/s) dropping packet, total_drops=1

# 如果 rate_drops 持续快速增长，说明编码码率设置过高
[bridge] ... tx_bw=15.00kB/s limit=15.0kB/s rate_drops=152 ...
```

- `rate_drops` 偶尔个位数 → 正常，编码器偶尔微超
- `rate_drops` 持续快速增长 → 需要降低 `video_bitrate_kbps` 或 `video_size`

---

## 3. 参数速查表

| 参数 | 文件 | 默认值 | 范围 | 作用 |
|:---|:---|:---|:---|:---|
| `video_bitrate_kbps` | `options.hpp` | 116 | 40~116 | H264 编码目标码率 |
| `video_size` | `options.hpp` | 300 | 120~480 | 输出图像边长(正方形) |
| `video_fps` | `options.hpp` | 30 | 10~60 | 编码帧率 |
| `video_gop` | `options.hpp` | 10 | 1~fps×12 | 关键帧间隔 |
| `send_interval_ms` | `options.hpp` | 20 | — | 发送间隔(50Hz=20ms) |
| `motion_trail_frames` | `options.hpp` | **5** | 0~180 | 拖影历史帧数 |
| `motion_dilate_px` | `options.hpp` | **3** | 0~20 | 运动掩码膨胀半径 |
| `motion_erode_px` | `options.hpp` | **1** | 0~20 | 运动掩码腐蚀半径 |
| `motion_threshold` | `options.hpp` | 14 | 0~255 | 运动检测灵敏度 |
| `bg_blur_sigma` | `options.hpp` | **1.2** | 0~10 | 静态区域模糊强度 |
| `bg_update_alpha` | `options.hpp` | 0.01 | 0.001~0.2 | 背景模型更新速度 |
| `trail_disable_motion_ratio` | `options.hpp` | **0.30** | 0.0~1.0 | 全局运动时禁用拖影阈值 |
| `center_clear_size` | `options.hpp` | **100** | 0~480 | 中心保真区边长(始终当运动) |
| `center_clear_radius` | `options.hpp` | 112 | 0~480 | 中心圆ROI半径(>0时圆外全黑) |
| `force_monochrome` | `options.hpp` | false | — | 强制全图灰度 |
| `static_simplify` | `options.hpp` | true | — | 启用静态简化 |
| `crop_size` | `options.hpp` | 0 | 0~ | 中心裁剪边长(0=不裁剪) |
| `exposure_ms` | `options.hpp` | 10.0 | — | 相机曝光时间(毫秒) |
| `gain` | `options.hpp` | 12.0 | — | 相机模拟增益 |
| `viewer_ip` | `options.hpp` | "" | — | UDP 目标IP(空=禁用UDP) |
| `viewer_port` | `options.hpp` | 3335 | — | UDP 目标端口 |
| `video_serial` | `options.hpp` | `/dev/ttyUSB0` | — | 图传串口路径 |
| `video_serial_baud` | `options.hpp` | 921600 | — | 串口波特率 |

---

## 4. 编码与带宽参数

### 3.1 `video_bitrate_kbps` — H264 编码目标码率

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `int video_bitrate_kbps = 116;` |

**作用**：直接控制 ffmpeg/libx264 的 `-b:v` 和 `-maxrate` 参数。码率越高，画面质量越好，但带宽占用越大。

**效果**：
- **提高** (如 116→150，需同时提高带宽上限)：画面更清晰，静态区域模糊痕迹更少
- **降低** (如 116→60)：画面更模糊，块效应更明显，但丢包率降低
- **注意**：此值不能超过物理带宽上限 (≈116kbps 净荷)，否则必然丢包

**调参建议**：
- 画面模糊、块效应严重 → 提高 `video_bitrate_kbps`（但保持 ≤116）
- backlog 持续增长、丢包 → 降低 `video_bitrate_kbps`
- 码率 > 80kbps 时，静态区域不会去饱和(保留颜色)

---

### 3.2 `video_size` — 输出图像边长

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `int video_size = 300;` |

**作用**：预处理后输出给编码器的正方形图像边长。图像总像素 = `video_size² × 3 bytes`。

**效果**：
- **提高** (如 300→400)：画面分辨率更高、细节更多，但编码数据量大幅增加
- **降低** (如 300→200)：画面更模糊，但大幅节省带宽

**带宽影响**：像素数增长与所需码率呈近似线形关系。从 300→400，所需码率约增长 (400/300)² ≈ 1.78×。

**调参建议**：
- 需要更多细节 → 提高 `video_size`，同时提高 `video_bitrate_kbps`
- 带宽紧张、丢包严重 → 降低 `video_size`

---

### 3.3 `video_fps` — 编码帧率

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `int video_fps = 30;` |

**作用**：每秒编码和发送的帧数。受 `send_interval_ms` (50Hz 包率) 约束。

**效果**：
- **提高** (如 30→50)：画面更流畅，但每秒数据量增加
- **降低** (如 30→15)：画面卡顿，但每帧可获得的码率更多

**调参建议**：
- 画面卡顿但带宽有余量 → 提高 `video_fps`
- 画面模糊但流畅度可接受 → 降低 `video_fps`，让每帧获得更多码率
- 建议 `video_fps` ≤ 50（受 50Hz 包率限制）

---

### 3.4 `video_gop` — 关键帧间隔

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `int video_gop = 10;` |

**作用**：每隔多少帧插入一个 IDR 关键帧（完整帧）。GOP 越小，丢包后恢复越快，但 I 帧开销越大。

**效果**：
- **减小** (如 10→5)：丢包后更快恢复，但 I 帧占用更多码率
- **增大** (如 10→30)：码率利用更高效，但丢包后恢复更慢

**调参建议**：
- 丢包频繁、花屏时间长 → 减小 `video_gop`
- 带宽紧张、希望更高效利用码率 → 增大 `video_gop`

---

### 3.5 `send_interval_ms` — 发送间隔

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `int send_interval_ms = 20;` |

**作用**：两包之间的最小间隔。与 50Hz 频率对应。

**注意**：
- 20ms = 50Hz（标准值，一般不需要改）
- 改为 40ms = 25Hz 会降低一半带宽，但画面更新率也减半
- 建议保持 20ms 不变，通过 `video_bitrate_kbps` 调节带宽

---

## 5. 预处理与视觉效果参数

### 4.1 `static_simplify` — 静态简化开关

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `bool static_simplify = true;` |

**作用**：启用 Pacific 风格的运动感知画面简化。关闭后，整帧不做任何静态模糊/去饱和处理。

**效果**：
- **开启(true)**：静态区域被模糊+低码率时去饱和，运动区域保持清晰 → 大幅节省带宽
- **关闭(false)**：整帧保留全细节 → 画面统一，但带宽需求大幅增加

**调参建议**：
- 排障时临时关闭，确认问题是否在预处理阶段
- 画面模糊到无法接受 → 考虑关闭或提高 `video_bitrate_kbps`

---

### 4.2 `center_clear_radius` — 中心圆形保真ROI

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `int center_clear_radius = 112;` |

**作用**：当 > 0 时，**独占模式**：只保留中心圆形区域（锐化处理），圆外全部压黑。这会直接跳过静态简化逻辑。

**效果**：
- **> 0**：圆外全黑，所有码率集中到中心目标 → 中心区域画质极高
- **= 0**：使用 `static_simplify` + `center_clear_size` 的常规管道

**注意**：此参数与 `static_simplify` 互斥。`center_clear_radius > 0` 时，`static_simplify` 和 `motion_trail_frames` 都不会生效。

**调参建议**：
- 只关心中心目标(如装甲板区域) → 设置合适的半径
- 需要看到周围环境 → 设为 0，使用 `static_simplify` 模式

---

### 4.3 `center_clear_size` — 中心保真区边长

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `int center_clear_size = 100;` |

**作用**：在 `static_simplify` 模式下，画面中心的方形区域**强制标记为运动区域**，不会被模糊处理。

**效果**：
- **增大** (如 100→200)：更大中心区域保持清晰
- **减小** (如 100→0)：中心区域也按运动检测结果处理

**调参建议**：
- 瞄准点附近画面经常变模糊 → 增大 `center_clear_size`
- 带宽紧张 → 减小或设为 0

---

### 4.4 `bg_blur_sigma` — 静态区域模糊强度

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `double bg_blur_sigma = 1.2;` |

**作用**：静态区域高斯模糊的 sigma 值。值越大，静态区域越模糊（越省码率）。

**效果**：
- **增大** (如 1.2→2.5)：静态区域更模糊，更省码率
- **减小** (如 1.2→0.5)：静态区域更清晰，但码率增加

**调参建议**：
- 静态背景纹理太多、浪费码率 → 增大 sigma
- 静态区域模糊到看不清背景 → 减小 sigma

---

### 4.5 `bg_update_alpha` — 背景模型更新速度

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `double bg_update_alpha = 0.01;` |

**作用**：背景模型学习率（`cv::accumulateWeighted` 的 alpha）。控制背景模型多快适应场景变化。

**效果**：
- **增大** (如 0.01→0.05)：背景更快适应光照变化，但静止物体更快被"吸收"为背景
- **减小** (如 0.01→0.005)：背景更稳定，但对光照变化敏感

**调参建议**：
- 光照频繁变化 → 增大 alpha
- 静止物体被误判为背景 → 减小 alpha

---

### 4.6 `force_monochrome` — 强制灰度

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `bool force_monochrome = false;` |

**作用**：开启后，整帧转为灰度（BGR→GRAY→BGR），颜色信息丢失但数据量减少约 2/3。

**效果**：
- **true**：大幅节省码率，但丢失颜色信息
- **false**：保留颜色，码率更高

**调参建议**：
- 不需要颜色信息（如仅检测形状）→ 开启
- 需要颜色识别 → 保持关闭

---

### 4.7 `crop_size` — 中心裁剪边长

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `int crop_size = 0;` |

**作用**：从相机原始画面中裁剪中心正方形区域。0 表示不裁剪，使用整帧。

**效果**：
- **> 0**：只取中心区域，减少无关内容
- **= 0**：使用整个相机画面

**调参建议**：
- 相机画面边缘有无用信息 → 设置合适的裁剪值
- 需要完整的广角视野 → 设为 0

---

## 6. 运动检测与拖影参数

### 5.1 `motion_threshold` — 运动检测灵敏度

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `int motion_threshold = 14;` |

**作用**：像素与背景模型的差异超过此阈值时，判定为"运动"。

**效果**：
- **减小** (如 14→8)：更敏感 → 更多区域被视为运动 → 更多区域保持清晰 → 码率升高
- **增大** (如 14→25)：更迟钝 → 更多区域被视为静态 → 更多区域被模糊 → 码率降低

**调参建议**：
- 小幅运动检测不到、运动物体被模糊 → 减小 threshold
- 噪声被误判为运动、码率过高 → 增大 threshold

---

### 5.2 `motion_erode_px` — 运动掩码腐蚀半径

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `int motion_erode_px = 1;` |

**作用**：对运动掩码进行形态学腐蚀（erode）。消除孤立的噪声点。

**效果**：
- **增大** (如 1→4)：过滤更多噪声，但小物体可能丢失运动标记
- **减小** (如 1→0)：不腐蚀，所有检测到的运动都保留

**调参建议**：
- 画面有大量小噪声点被误判为运动 → 增大 erode
- 小物体(远处的机器人)运动检测不到 → 减小 erode

---

### 5.3 `motion_dilate_px` — 运动掩码膨胀半径（**影响拖影**）

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `int motion_dilate_px = 3;` |

**作用**：对运动掩码进行形态学膨胀（dilate）。扩大运动区域，让运动物体的边缘也被保留。

**效果**：
- **增大** (如 3→6)：运动区域扩大 → **拖影更粗更长**，但码率增加
- **减小** (如 3→0)：运动区域缩小，拖影仅限物体核心

**调参建议**：
- 拖影太细、不够明显 → 增大 dilate (配合 `motion_trail_frames`)
- 码率过高、拖影太宽 → 减小 dilate

---

### 5.4 `motion_trail_frames` — 拖影历史帧数（**核心拖影参数**）

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `int motion_trail_frames = 5;` |

**作用**：控制拖影的"尾巴长度"。每帧保留前 N 帧的运动掩码和像素值，在联合运动区域上应用时域最大值叠加（`cv::max`）。

```
工作原理:
  当前帧运动掩码 ∪ 历史帧1运动掩码 ∪ ... ∪ 历史帧N运动掩码 = 联合掩码
  当前帧像素  max  历史帧1像素  max  ... max  历史帧N像素 = 拖影帧
  拖影帧.copyTo(输出, 联合掩码)  ← 只在有历史运动的区域叠加拖影
```

**效果**：
- **增大** (如 5→10)：拖影尾巴更长、更明显，快速移动物体留下长残影
- **减小** (如 5→1)：拖影几乎不可见
- **= 0**：完全禁用拖影效果

**带宽影响**：拖影帧数增加 → 更多像素被标记为"运动" → 码率轻微增加

**调参建议**：
- 拖影不够长 → 增大 `motion_trail_frames` (同时适度增大 `motion_dilate_px`)
- 拖影太长导致画面混乱 → 减小 frames
- 带宽紧张 → 适度减小

---

### 5.5 `trail_disable_motion_ratio` — 全局运动禁用拖影阈值

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `double trail_disable_motion_ratio = 0.30;` |

**作用**：当画面中运动像素比例超过此阈值时，认为"整个画面都在动"（如相机云台大幅转动），临时禁用拖影，避免全屏拖影残像。

**效果**：
- **增大** (如 0.30→0.60)：更少禁用拖影 → 拖影在更多场景下可见
- **减小** (如 0.30→0.10)：更容易触发禁用 → 大范围运动时拖影自动消失

**调参建议**：
- 云台转动时全屏拖影太乱 → 降低此值
- 希望任何时候都有拖影 → 设为 1.0 (永不自动禁用)

---

## 7. 相机参数

### 6.1 `exposure_ms` — 曝光时间

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `double exposure_ms = 10.0;` |

**作用**：海康相机的曝光时间(毫秒)。影响画面亮度。

**效果**：
- **增大** → 画面更亮，但运动模糊增加
- **减小** → 画面更暗，但运动更清晰

**调参建议**：
- 室内光线不足 → 增大曝光
- 运动物体模糊 → 减小曝光 + 增大 gain 补偿

---

### 6.2 `gain` — 模拟增益

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `double gain = 12.0;` |

**作用**：海康相机的模拟增益。提高画面亮度但增加噪声。

**效果**：
- **增大** → 画面更亮，但噪声增多
- **减小** → 画面更暗，但更干净

**调参建议**：
- 曝光已最大但画面仍暗 → 增大 gain
- 画面噪声多 → 减小 gain + 增大曝光补偿

---

## 8. 网络与串口参数

### 7.1 `viewer_ip` / `viewer_port` — UDP 调试输出

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `std::string viewer_ip = "";` |
| `bridge/options.hpp` | `int viewer_port = 3335;` |

**作用**：UDP 直连调试输出（与 rm-native-viewer 的 `--enable-0310-udp` 配合）。正式比赛走串口，不需要。

---

### 7.2 `video_serial` / `video_serial_baud` — 图传串口

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `std::string video_serial = "/dev/ttyUSB0";` |
| `bridge/options.hpp` | `uint32_t video_serial_baud = 921600;` |

**作用**：直连图传发送端的 USB-TTL 串口。波特率必须 ≥ 921600 才能承载 50Hz 300B 的数据流。

**注意**：不要使用 115200 — 115200 bps ≈ 11.5 kB/s < 15 kB/s (所需)。

---

## 9. 调试参数

### 8.1 `test_pattern` — 测试图像

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `bool test_pattern = false;` |

**作用**：生成内置测试图案，不依赖海康相机。用于验证编码链路。

### 8.2 `preview` — 预览窗口

| 文件 | 行 |
|:---|:---|
| `bridge/options.hpp` | `bool preview = false;` |

**作用**：显示海康原画预览窗口，可调节曝光/增益并保存配置。

---

## 10. 常见问题排查

### 问题 1：画面模糊/块效应严重

**原因**：码率不足以编码当前画面复杂度。

**排查方法**：
1. 观察日志中的 `tx_bw` 是否接近 `limit=15.0kB/s`
2. 观察 `video_backlog` 是否持续增长

**解决方案**（按优先级）：
- 降低 `video_size`（最有效，像素数平方级减少）
- 降低 `video_fps`（减少每秒数据量）
- 提高 `bg_blur_sigma`（静态区域更省码率）
- 减小 `motion_dilate_px`、`motion_erode_px`（缩小运动区域）
- 开启 `force_monochrome`（大幅减少数据量）
- 增大 `center_clear_radius`（只保留中心区域）

---

### 问题 2：rate_drops 持续增长（被硬限速丢包）

**原因**：编码器输出码率持续超过 15 kB/s 硬上限，滑动窗口限速器强制丢包。

**排查方法**：
- 观察日志中 `rate_drops` 是否以每秒 >5 的速度增长
- 日志中是否出现 `RATE LIMIT: ... dropping packet` 警告
- `tx_bw` 持续显示 15.00kB/s（顶满上限）

**解决方案**（按优先级）：
1. 降低 `video_bitrate_kbps` (如 116→90，最直接)
2. 降低 `video_size` (如 300→260，平方级减少数据量)
3. 降低 `video_fps` (如 30→25)
4. 增大 `bg_blur_sigma` (如 1.2→2.0)
5. 开启 `force_monochrome`

---

### 问题 3：backlog 持续增长、丢包

**原因**：编码输出速度超过发送速度(15kB/s)。

**排查方法**：
- 观察 `video_backlog` 是否持续增长而非波动
- 日志中是否出现 backlog 警告

**解决方案**（按优先级）：
1. 降低 `video_bitrate_kbps` (如 116→80)
2. 降低 `video_size` (如 300→240)
3. 降低 `video_fps` (如 30→20)
4. 增大 `bg_blur_sigma` (如 1.2→2.0)
5. 减小 `motion_trail_frames` (如 5→2)

---

### 问题 4：拖影太淡/不可见

**原因**：拖影参数不匹配当前场景。

**解决方案**：
1. 确认 `motion_trail_frames > 0`（默认现在为 5）
2. 确认 `center_clear_radius = 0`（非零时不会走拖影逻辑）
3. 确认 `static_simplify = true`
4. 增大 `motion_trail_frames` (如 5→10)
5. 增大 `motion_dilate_px` (如 3→6，让拖影更粗)
6. 减小 `motion_threshold` (如 14→10，更容易检测到运动)

---

### 问题 5：拖影太多/画面太乱

**原因**：全局运动时拖影未自动禁用，或拖影参数太激进。

**解决方案**：
1. 降低 `trail_disable_motion_ratio` (如 0.30→0.15)
2. 减小 `motion_trail_frames` (如 5→2)
3. 减小 `motion_dilate_px` (如 3→1)
4. 增大 `motion_threshold` (如 14→20，减少误检测)

---

### 问题 6：静态物体逐渐"消失"

**原因**：`bg_update_alpha` 太大，静止物体被快速吸收进背景模型。

**解决方案**：
- 降低 `bg_update_alpha` (如 0.01→0.005)

---

### 问题 7：花屏/解码失败

**原因**：丢包导致 H264 码流失同步。

**排查方法**：
- 观察 `video_backlog` 是否很高
- 观察是否有 `sequence gap` 日志（接收端）

**解决方案**：
1. 降低 `video_bitrate_kbps` 减少 backlog
2. 减小 `video_gop` (如 10→5) 加快恢复
3. 确认串口波特率 ≥ 921600
4. 检查 `send_interval_ms = 20`（不要随意增大）

---

### 问题 8：tx_bw 远低于 15kB/s 上限

**原因**：编码器没有产生足够数据。通常发生在画面非常简单(大量黑色/静态区域)或 `video_bitrate_kbps` 过低时。

**说明**：这不一定是问题。在 `center_clear_radius > 0` 模式下，圆外全黑，码率会大幅降低。只要画面正常显示即可。

---

## 11. 调参速查表

### 我想要...

| 目标 | 调哪些参数 | 方向 |
|:---|:---|:---|
| **画面更清晰** | `video_bitrate_kbps`, `video_size` | ↑ |
| **rate_drops 持续增长** | `video_bitrate_kbps`, `video_size`, `video_fps` | ↓ |
| **降低带宽/减少丢包** | `video_bitrate_kbps`, `video_size`, `video_fps` | ↓ |
| **拖影更明显/更长** | `motion_trail_frames`, `motion_dilate_px` | ↑ |
| **拖影更淡/更短** | `motion_trail_frames`, `motion_dilate_px` | ↓ |
| **静态区域更干净** | `bg_blur_sigma` | ↑ |
| **运动检测更敏感** | `motion_threshold` | ↓ |
| **运动检测更稳定** | `motion_threshold`, `motion_erode_px` | ↑ |
| **节省码率给中心区域** | `center_clear_radius` | >0 |
| **全画面都保留** | `center_clear_radius` | 0 |
| **中心区域一直清晰** | `center_clear_size` | ↑ |
| **丢包后更快恢复** | `video_gop` | ↓ |
| **更流畅** | `video_fps` | ↑ |
| **光线暗时补光** | `exposure_ms`, `gain` | ↑ |

---

### 推荐配置组合

#### 默认配置（平衡画质与带宽）

```yaml
video_size: 300
video_fps: 30
video_bitrate_kbps: 116
video_gop: 10
motion_trail_frames: 5
motion_dilate_px: 3
motion_erode_px: 1
bg_blur_sigma: 1.2
center_clear_radius: 0       # 使用 static_simplify 模式
center_clear_size: 100
```

#### 拖影优先（最明显的运动拖影）

```yaml
motion_trail_frames: 10      # 更长的拖影尾巴
motion_dilate_px: 6          # 更粗的拖影
motion_erode_px: 0           # 不腐蚀，保留所有运动像素
motion_threshold: 10         # 更敏感的运动检测
trail_disable_motion_ratio: 0.60  # 更少自动禁用拖影
```

#### 画质优先（带宽允许时）

```yaml
video_size: 400
video_bitrate_kbps: 116      # 顶满带宽
video_fps: 30
bg_blur_sigma: 0.8           # 静态区域更清晰
motion_trail_frames: 3       # 适度拖影
```

#### 低带宽（丢包严重时）

```yaml
video_size: 240
video_bitrate_kbps: 60
video_fps: 20
bg_blur_sigma: 2.5           # 大幅模糊静态区域
motion_trail_frames: 2
motion_dilate_px: 1
force_monochrome: true       # 最后手段
```

#### 中心ROI模式（只看中间目标）

```yaml
center_clear_radius: 120     # 圆圈半径（像素）
video_size: 400              # 可以调高（圆外全黑很省码率）
video_bitrate_kbps: 116
# 注意: 此模式下 static_simplify/motion_trail_frames 不生效
```
