# 延迟与画质联动调参指南

> 版本: v1.0 (2026-07-31)
> 适用: `minimal_hik_gimbal_bridge` 0x0310 视频链路

---

## 1. 核心思路：一个旋钮控制延迟与画质的权衡

在 **15 kB/s (2.4Kbit × 50Hz)** 的带宽硬约束下，清晰度和延迟本质上是一对矛盾：

```
带宽固定 = 15 kB/s
画质 = 单帧可用的比特数
延迟 = 攒齐一帧高清图需要的时间

                 ┌─────────────────────┐
  延迟 (s)  ────→│   单个高清 I 帧     │────→ 画质
                 │   (bufsize 预算)    │
                 └─────────────────────┘
```

**根本矛盾**：带宽就这么点，要么每秒传很多小帧（低延迟、模糊），要么攒几秒才出一张高清大帧（高延迟、清晰）。

本项目用一个参数 **`video_latency_s`（目标延迟秒数）** 统一控制这个权衡，其余编码参数（bufsize / GOP / rc-lookahead）自动联动，无需分别调节。

---

## 2. 联动模型

### 2.1 硬约束（不可修改）

| 约束 | 值 | 来源 |
|:---|:---|:---|
| 每包数据 | 300 字节 = 2.4 Kbit | 0x0310 协议 `kCustomClient0310PayloadBytes` |
| 发送频率 | 50 Hz (每 20ms 一包) | `send_interval_ms = 20` |
| 带宽上限 | 15,000 B/s = 120 kbit/s | 发送端硬限速器 (200ms 滑动窗口) |

### 2.2 联动推导（`h264_encoder.cpp` 自动计算）

```
输入:  video_latency_s      ← 唯一的调参旋钮 (0.5 ~ 10 秒)
       video_fps            ← 独立参数 (2 ~ 60)
       video_bitrate_kbps   ← 独立参数 (40 ~ 116, 常驻 116)

推导1:  bufsize (kbit)  = max(latency_s × 120,  bitrate_kbps)
        └─ 编码器 VBV 缓冲区大小，决定单帧(主要是 I 帧)能借多大预算
        └─ 为什么 ≥ bitrate: bufsize 小于码率时 CBR 码率控制失效

推导2:  GOP (帧) = latency_s × video_fps
        └─ I 帧间隔 = 延迟秒数，即画面完整刷新周期
        └─ 若手动设置 video_gop > 0，则覆盖此推导

推导3:  rc-lookahead = clamp(latency_s × fps × 0.25, 10, 60) 帧
        └─ 编码前向延迟 ≈ 延迟的 1/4 (每 1 帧 ≈ 1/fps 秒)
        └─ sync-lookahead = rc-lookahead × 2/3
```

### 2.3 端到端延迟构成

```
总延迟 ≈ 编码前向延迟 (rc-lookahead/fps)
       + I帧传输时间 (bufsize预算 / 15kB/s)
       + 接收端缓冲等待
       + B帧重排 (≈2帧/fps)

默认 (latency_s=3, fps=30):
  rc-lookahead = 3×30×0.25 = 22帧 ≈ 0.75s
  bufsize      = 3×120 = 360kbit ≈ 45kB → I帧传输 ~3s (仅首帧/场景切换)
  持续运行      = 仅 P 帧实时解码 ≈ 1~1.5s
```

---

## 3. 参数对照表（实测推导值）

以下为 `video_fps=30`、`video_bitrate_kbps=116` 时不同 `video_latency_s` 的推导结果：

| `video_latency_s` | bufsize (kbit) | bufsize (kB) | GOP (帧) | GOP (秒) | rc-lookahead (帧) | 编码前向 (s) | I帧传输 (s) | 体验 |
|:---|:---|:---|:---|:---|:---|:---|:---|:---|
| 0.5 | 116 (被码率兜底) | 14.5 | 15 | 0.5 | 10 | 0.33 | ~1.0 | 低延迟、画质接近原版 |
| 1.0 | 120 | 15 | 30 | 1.0 | 10 | 0.33 | ~1.0 | 画质小幅提升 |
| 2.0 | 240 | 30 | 60 | 2.0 | 15 | 0.50 | ~2.0 | 明显清晰 |
| **3.0 (默认)** | **360** | **45** | **90** | **3.0** | **22** | **0.75** | **~3.0** | **画质/延迟平衡** |
| 5.0 | 600 | 75 | 150 | 5.0 | 37 | 1.25 | ~5.0 | 很清晰、延迟明显 |
| 10.0 | 1200 | 150 | 300 | 10.0 | 60 | 2.00 | ~10.0 | 极致画质、重延迟 |

> 注: I帧传输时间按"单帧预算 / 15kB/s"估算，实际因 P 帧穿插传输，持续运行期延迟更低。

---

## 4. 如何配置

### 4.1 命令行

```bash
./build/minimal_hik_gimbal_bridge --video-latency-s 3.0 --video-fps 30
```

### 4.2 YAML 配置 (`config/bridge.yaml`)

```yaml
image:
   video_latency_s: 3.0
```

### 4.3 代码默认值 (`bridge/options.hpp`)

```cpp
double video_latency_s = 3.0;   // 目标延迟秒数
int    video_gop       = 0;     // 0 = 自动 (latency_s × fps)，非 0 手动覆盖
```

---

## 5. 参数职责解耦说明

| 参数 | 职责 | 联动关系 | 建议调法 |
|:---|:---|:---|:---|
| `video_latency_s` | **延迟/画质权衡** | 推导 bufsize、GOP、lookahead | 唯一旋钮，先调它 |
| `video_fps` | 流畅度/帧率 | 影响 GOP 帧数、lookahead 帧数 | 独立调节 |
| `video_bitrate_kbps` | 带宽利用率 | bufsize 下限兜底 | 固定 116 即可 |
| `video_gop` | I 帧间隔覆盖 | 非 0 时覆盖推导2 | 手动微调时用 |
| `video_size` | 分辨率 | 无联动 | 独立调节画质上限 |
| `send_interval_ms` | 发送频率 | 无联动 | 固定 20ms |

---

## 6. 调参建议

### 6.1 延迟偏高 → 调小 `video_latency_s`

| 目标 | 设置 | 预期延迟 | 预期画质 |
|:---|:---|:---|:---|
| 画面流畅优先 | `--video-latency-s 1.0` | ~1s | 小幅优于原版 |
| 平衡点 | `--video-latency-s 3.0` | ~3s | 明显清晰 |
| 极致画质 | `--video-latency-s 10.0` | ~10s | 最高清 |

### 6.2 画质不满意 → 在固定延迟下提升

1. 提高 `video_size`（分辨率上限，但码率不够会糊）
2. 确认 `video_bitrate_kbps=116`（已顶满）
3. 确认 `video_latency_s` 已到能接受的延迟上限
4. 若运动模糊：增大 `video_fps`，但会摊薄单帧预算 → 需同步增大 `video_latency_s`

### 6.3 特定场景快速配置

| 场景 | 推荐配置 |
|:---|:---|
| 装甲板识别（要快） | `--video-latency-s 1.0 --video-fps 30` |
| 远程观察（要清） | `--video-latency-s 5.0 --video-fps 20` |
| 固定静态目标（要最清） | `--video-latency-s 10.0 --video-fps 15` |

---

## 7. 调参后的验证方法

### 7.1 启动后观察日志

```bash
[bridge] ... fps=30.0 sent=... tx_bw=...kB/s video_backlog=...
```

- `video_backlog` 持续增长 → 编码器产出超过 15kB/s → 硬限速丢包 → **降低画质或延迟**
- `tx_bw` 长期顶满 15.0kB/s → 编码器满负荷 → 观察是否丢包

### 7.2 接收端 (rm-native-viewer) 观察

- 启动后首帧出现时间 ≈ `video_latency_s`
- 画面更新时是否明显停顿 → 延迟感知
- 场景切换时是否花屏等待 → GOP 相关

---

## 8. 常见问题

### Q1: 改了 `video_latency_s` 但没生效？

- 检查是否被 YAML 覆盖：`config/bridge.yaml` 中 `image.video_latency_s` 优先于代码默认值
- 命令行参数优先于 YAML

### Q2: 手动设置 `video_gop` 后延迟没变？

- GOP 只影响 I 帧间隔，不影响 bufsize 推导
- 延迟主要由 `bufsize`（= latency_s × 120）决定
- 若想完全手动，需同时设 `video_latency_s` 和 `video_gop`

### Q3: 延迟能低于 0.5s 吗？

- 可以设 `--video-latency-s 0.3`，但 bufsize 会被码率兜底到 116kbit
- 此时画质接近原始低延迟方案（因为 bufsize 已最小化）
- 想真正低延迟，还需重新考虑 `-tune zerolatency`（当前已移除）

---

## 9. 相关代码位置

| 功能 | 文件 |
|:---|:---|
| 联动推导逻辑 | `bridge/h264_encoder.cpp` start() 开头 |
| `video_latency_s` 参数定义 | `bridge/options.hpp` |
| 命令行解析 | `bridge/options.cpp` parse_args() |
| YAML 读写 | `bridge/options.cpp` load_yaml_config() / save_config() |
| 发送端硬限速 | `src/main.cpp` 发送线程 |
