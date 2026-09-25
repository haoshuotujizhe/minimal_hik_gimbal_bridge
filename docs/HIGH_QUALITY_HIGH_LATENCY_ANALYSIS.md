# "高画质高延迟"方案分析

> 同带宽约束(15 kB/s, 2.4Kbit×50Hz)下，为什么别人画质更清晰，延迟却更高？

---

## 1. 一句话答案

**别人的编码器不是 `tune=zerolatency`。他们让 H264 编码器拥有巨大的 VBV 缓冲区和很长的 GOP，用几秒甚至十几秒的时间"攒"出一个高质量 I 帧。**

---

## 2. 当前管道的五把"低延迟锁"

```
h264_encoder.cpp 第 122-132 行:

-c:v libx264
-preset veryslow           ✅ 这个是对的，最慢预设=最高压缩效率
-tune zerolatency          🔴 最大元凶 —— 禁用几乎所有质量优化
-b:v 116k                  ✅ 码率顶满
-maxrate 116k              ✅ 
-bufsize 116k              🔴 第二大元凶 —— I帧被锁死在 ~14.5kB 以下
-g 10                      🟡 每0.3秒一个I帧，浪费码率
-keyint_min 10             🟡 不允许编码器智能选择I帧位置
-sc_threshold 0            🟡 禁用场景切换检测
-bf 0                      🟡 无B帧，帧间压缩效率低
-x264-params ref=1         🟡 只有1个参考帧
```

### 🔴 锁1：`-tune zerolatency`

x264 官方文档原文：zerolatency 关闭以下所有优化：

| 被关闭的特性 | 作用 | 质量损失 |
|:---|:---|:---|
| `rc-lookahead` | 码率控制提前看40+帧 | 无法合理分配码率 |
| `sync-lookahead` | 线程同步提前看 | 多线程效率下降 |
| `mbtree` | 宏块树——x264 最重要的质量优化之一 | **质量损失 20-40%** |
| `b-frames` | B帧——双向预测，压缩效率极高 | 帧间压缩效率大幅下降 |
| `aq-mode` 强制回退 | 自适应量化模式受限 | 暗部/平坦区域质量差 |

**同样的 116kbps，关掉 zerolatency 能提升 30-50% 的感知质量。**

### 🔴 锁2：`-bufsize 116k`（= 14.5 kB）

这是 VBV（Video Buffering Verifier）缓冲区大小。H264 CBR 编码器靠它来控制码率：

```
VBV 模型：
  ┌─────────────────────────────────────┐
  │         解码器缓冲区 (bufsize)        │
  │  ┌─────────────────────────────┐    │
  │  │  已缓存的未解码数据            │    │
  │  │  ← 以 15 kB/s 速度填入        │    │
  │  │  → 以解码速度排出              │    │
  │  └─────────────────────────────┘    │
  │  缓冲区容量 = bufsize = 14.5 kB     │
  │  填满时间 = 14.5kB ÷ 15kB/s ≈ 1s   │
  └─────────────────────────────────────┘

当前: bufsize=116k → 缓冲区1秒就满 → I帧最多 ~14.5 kB
别人: bufsize=2000k → 缓冲区可攒17秒 → I帧可达 ~100 kB
```

**bufsize 直接决定了一个 I 帧能有多大。bufsize 每增大 10 倍，I 帧上限增大 10 倍。**

### 🟡 锁3-5：GOP=10, ref=1, bf=0

| 参数 | 当前 | 问题 | 高画质方案 |
|:---|:---|:---|:---|
| `-g` (GOP) | 10 | 每 0.3s 一个 I 帧，I 帧频繁→单个 I 帧预算少 | 300-600 |
| `-bf` (B帧) | 0 | B 帧比 P 帧小 30-50%，省下的码率给 I 帧 | 3 |
| `-ref` (参考帧) | 1 | 更多参考帧 = 更好的运动估计 = 更小的 P 帧 | 5 |
| `sc_threshold` | 0 | 无法在场景切换时智能插入 I 帧 | 40 |

---

## 3. "高画质高延迟"方案的运作机理

### 时间线

```
时间轴 (秒):  0     1     2     3     4     5     6     7     8     9    10
             ├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
             │                                                    │
发送端:       │←── I帧传输 ─────────────────────────────────────→│
             │   30kB I帧, 分布在 100 个包, 耗时 2 秒              │
             │   + P帧穿插传输(运动向量, 很小)                     │
             │                                                    │
             │←── P帧 × 299 个 ────────────────────────────────→│
             │   每个 50-150 字节, 仅传输运动差异                  │
             │                                                    │
接收端:       │←── 缓冲攒包 ──→│← 解码+显示 ───────────────────→│
             │   收到足够数据    │   I帧解码, 画面突然变清晰        │
             │   解码器等待      │   后续P帧持续微调                │
             │   显示上一帧      │                                 │
             │                  │                                 │
画质:        [  模糊的历史帧  ] [         🔥 突然清晰          ]
延迟:        ←──────────── 2-10 秒端到端延迟 ─────────────→
```

### 带宽分配对比

```
当前方案 (低延迟, GOP=10):
  I帧(I)  P  P  P  P  P  P  P  P  P  I  P  P  ...
  ├3kB┤  ├200B each──┤              ├3kB┤
  每0.3秒一个小I帧, 每个只有3kB → 模糊
  总带宽: 3kB×3 + 200B×27 ≈ 14.4 kB/s ✓

高画质方案 (高延迟, GOP=300):
  I帧(I)  P  P  P  ...  (300个P帧)  ...  P  P  I  ...
  ├30kB┤ ├100B each──────────────────────┤    ├30kB┤
  每10秒一个大I帧, 每个有30kB → 极其清晰
  总带宽: 30kB×0.1 + 100B×29.9 ≈ 3kB + 3kB ≈ 6 kB/s ← 还有余量!
```

高画质方案下带宽使用**更低**（因为 P 帧只有运动向量，非常小），大部分时间带宽根本没有用满。剩余的带宽可以让 I 帧更大、P 帧更精细，或者提高 resolution。

---

## 4. 具体要改什么

### 改动位置：`bridge/h264_encoder.cpp`

每个参数的改动和原因：

```cpp
// ===== 第 20 行：slice 大小 =====
// 当前: constexpr std::size_t kTargetSliceMaxBytes = 280U;
// 改为: slice 大小不变。280字节保证每个NAL单元能放进一个300字节的0x0310包
//       (300 - 3字节头 - 一些余量 = 280)
//       一帧有多个slice没问题, Annex-B格式天然支持

// ===== 第 99 行：fps 下界 =====
// 当前: const auto clamped_fps = std::clamp(options.video_fps, 10, 60);
// 改为: const auto clamped_fps = std::clamp(options.video_fps, 2, 60);
// 原因: 允许更低帧率, 让每帧拿更多比特

// ===== 第 100 行：码率范围 =====
// 当前: const auto clamped_bitrate = std::clamp(options.video_bitrate_kbps, 40, 116);
// 不需要改, 116kbps 15kB/s 是协议限制

// ===== 第 101 行：GOP 上限 =====
// 当前: const auto clamped_gop = std::clamp(options.video_gop, 1, clamped_fps * 12);
// 改为: const auto clamped_gop = std::clamp(options.video_gop, 1, clamped_fps * 600);
// 原因: 允许极长GOP. 2fps×600 = 1200帧 = 10分钟一个I帧

// ===== 第 107-109 行：x264 参数 (核心改动) =====
// 当前:
//   const std::string x264_params =
//     "repeat-headers=1:nal-hrd=cbr:force-cfr=1:ref=1:slice-max-size=" +
//     std::to_string(kTargetSliceMaxBytes);
//
// 改为:
//   const std::string x264_params =
//     "repeat-headers=1:nal-hrd=cbr:force-cfr=1:ref=5:bframes=3:"
//     "b-adapt=2:rc-lookahead=60:sync-lookahead=30:"
//     "aq-mode=2:aq-strength=1.2:mbtree=1:qcomp=0.75:"
//     "subme=9:trellis=2:deblock=1,1:"
//     "slice-max-size=" + std::to_string(kTargetSliceMaxBytes);

// ===== 第 124 行：删除 tune=zerolatency =====
// 当前: "-tune", "zerolatency",
// 改为: 删掉这行, 或改为 "-tune", "film",

// ===== 第 127 行：bufsize (最关键改动) =====
// 当前: "-bufsize", std::to_string(clamped_bitrate) + "k",
// 改为: "-bufsize", std::to_string(clamped_bitrate * 15) + "k",
// 原因: bufsize = 116k × 15 = 1740k ≈ 218 kB
//       允许 I 帧最大 ~27 kB, 约 1.8 秒的带宽预算
//       (更大的 bufsize 可能导致某些 x264 版本不稳定, 15x 是安全值)

// ===== 第 128-129 行：GOP 和 keyint_min =====
// 当前: "-g", std::to_string(clamped_gop),
//       "-keyint_min", std::to_string(clamped_gop),
// 改为: "-g", std::to_string(clamped_gop),
//       "-keyint_min", std::to_string(clamped_gop / 2),
// 原因: keyint_min = GOP/2 允许编码器在场景切换时灵活放置I帧

// ===== 第 130 行：场景检测 =====
// 当前: "-sc_threshold", "0",
// 改为: "-sc_threshold", "40",
// 原因: 重新启用场景切换检测, 画面突变时自动插I帧

// ===== 第 131 行：B帧 =====
// 当前: "-bf", "0",
// 改为: 删除这行（B帧数量由 x264-params 中的 bframes=3 控制）
// 原因: B帧需要帧重排序, 增加延迟但大幅提升压缩效率
```

### 改动位置：`bridge/options.hpp`

```cpp
// 当前: int video_gop = 10;
// 改为: int video_gop = 300;   // 10秒一个I帧(video_fps=30时)
```

---

## 5. 延迟量化

### 各环节延迟分解

```
环节                    当前方案              高画质方案
─────────────────────────────────────────────────────────
相机曝光+读出            17ms                   17ms
预处理(OpenCV)           ~5ms                   ~5ms
H264编码                 ~20ms(zerolatency)     ~100ms(veryslow+lookahead)
B帧重排序                0ms(无B帧)             ~100ms(3 B帧)
I帧传输                  0.2s(3kB)             2-7s(30-100kB)
解码缓冲                 0.1s                   1-3s
─────────────────────────────────────────────────────────
端到端延迟                ~0.3s                  3-10s
```

### 延迟 vs 画质等高线

```
画质 ▲
    │                ★ 高画质方案
    │                    (GOP=300, bufsize=2000k)
    │              /
    │            /
    │          /
    │        /    ● 折中方案
    │      /       (GOP=60, bufsize=500k)
    │    /
    │  /  ■ 当前方案
    │/    (GOP=10, bufsize=116k, zerolatency)
    └──────────────────────────────→ 延迟
    0.3s    1s        5s        10s
```

---

## 6. 为什么需要改接收端（可能不需要）

**好消息**：rm-native-viewer 大概率不需要改。

```rust
// rm-native-viewer 的 mqtt_receiver_loop():
// 持续收 0x0310 chunk, 提取 payload, 写入 encoded_tx 通道
// → decoder_loop: 收 encoded 数据, 写入 ffmpeg stdin
// → ffmpeg 解码器自己会缓冲, 攒够一帧才输出
```

ffmpeg 解码器的内部缓冲行为：
- 收到 10kB 的 I 帧分布在 34 个包里 → ffmpeg 持续收 Annex-B 数据 → 内部 buffer 增长 → NAL 单元完整后解码 → 输出帧
- P 帧很小 → 收完立刻解码输出
- 大 I 帧需要几秒收完 → 解码器等待 → 显示冻结 → I 帧收完 → 画面突然更新为高清

**这就是"高画质高延迟"现象的完整机制。** 接收端天然支持，不需要改代码。

唯一可能的问题是：
1. `probesize=65536` (65kB) — 如果 I 帧超过 65kB，ffmpeg 的 probe 阶段可能截断。但 probe 只影响初始分析，完整帧通过 stdin 送入不受此限制。
2. `analyzeduration=1000000` (1M microseconds = 1s) — 初始分析时长。如果 I 帧传输超过 1 秒，可能需要调大。

---

## 7. 推荐配置组合

### 激进高画质（最大延迟 ~10s, 最高画质）

```yaml
# options.hpp / bridge.yaml
video_fps: 10               # 低帧率, 每帧 1.5kB 预算
video_gop: 600              # 10分钟一个I帧(实际上场景检测会提前插)
video_bitrate_kbps: 116     # 顶满带宽
```

```
h264_encoder.cpp 改动:
-tune film                  # 替换 zerolatency
-bufsize 1740k              # 116k × 15, 允许I帧 ~27kB
-g 600 -keyint_min 300      # 长GOP + 灵活的I帧位置
-bf 3 -ref 5                # B帧 + 多参考帧
-sc_threshold 40            # 场景检测
x264-params: rc-lookahead=60:mbtree=1:subme=9:...
```

### 折中方案（延迟 ~1-2s, 画质明显提升）

```yaml
video_fps: 20
video_gop: 120              # 4秒一个I帧(video_fps=30时)
video_bitrate_kbps: 116
```

```
h264_encoder.cpp 改动:
(删除 -tune zerolatency)    # 不设tune, 用默认
-bufsize 580k               # 116k × 5, 允许I帧 ~9kB
-g 120 -keyint_min 60
-bf 2 -ref 3
-sc_threshold 40
x264-params: rc-lookahead=40:sync-lookahead=20:mbtree=1:subme=8:...
```

### 当前方案（最低延迟 ~0.3s）

保持现状不动，作为"低延迟模式"保留。

---

## 8. 常见问题

### Q: 大 I 帧传输期间丢包怎么办？

**A**: 这是最大的风险。一个 30kB 的 I 帧分布在 100 个包中，任何一个包丢失都可能导致：
- 当前 I 帧无法解码（花屏或跳过）
- 后续 P 帧全部无法解码（因为参考帧丢失）
- 直到下一个 I 帧才能恢复

**缓解措施**：
1. 适当控制 I 帧大小（不要超过 50kB）
2. 保留合理的 GOP（不要超过 600）
3. 确保 `repeat-headers=1`（每个 I 帧前重复 SPS/PPS）
4. 接收端已有 SPS 阻塞+超时透传机制

### Q: 为什么不用 MJPEG 直接传高质量 JPEG？

**A**: 可以，但需要完全重写编码管道。对于 300×300 的图像：
- JPEG quality 90: ~15-25 kB → 50-83 个包 → 1-1.7 秒延迟
- JPEG quality 95: ~25-40 kB → 83-133 个包 → 1.7-2.7 秒延迟

H264 长 GOP 方案的优势：P 帧间歇更新运动信息，画面不完全冻结。MJPEG 方案画面会完全冻结在上一帧直到下一帧收完。

### Q: B 帧会不会让延迟失控？

**A**: 3 个 B 帧意味着编码顺序和显示顺序最多差 3 帧。在 10fps 下这是 300ms 的额外延迟。在 2fps 下是 1.5s。所以低 fps 时建议 bf=1 或 bf=0。高 fps(20+)时 bf=3 是安全的。
