# VID2 音视频打包工具

脚本：`D:/deepwork/rollback_bright/lab_ex5_i2s/doc/convert/make_av_video.py`。
仅使用 Python 标准库；从普通媒体转换时调用已经安装的 FFmpeg，不安装或下载任何组件。

当前实际 SPI 时钟为 **25MHz**，旧工具注释中的 50MHz 不适用于本次交付。
实测 `spi_master DIV0` 连续字节间隔为 **36clks**，有效吞吐约 **2.78MB/s**。
结合 320x240 RGB565、300 视频扇区及 2 倍输出的实际通路，15fps 也不能稳态。
音视频包最终默认并推荐 **12fps**（83.33ms/帧）：每帧每声道4000采样点、32音频扇区，
含扇区补零的平均包读取量为 `169984 * 12 = 2039808 B/s`。
15/20/24/25fps 继续支持，均为需要更高实际吞吐的实验档位。

## 使用

以下命令在 `D:/deepwork/rollback_bright/lab_ex5_i2s/doc/convert` 运行。
输出目录必须已存在，扩展名严格使用大写 `.VID`。

```powershell
# 零外部依赖的 3 秒 FPGA 验收片，默认 12fps，共 36 帧。
python -B make_av_video.py --demo 3 --out D:/deepwork/_av_package_20261031/vid2_tool_codex_20261031_9b72/DEMO12.VID

# 独立标准库检查：所有头字段、保留字节、精确文件长度、每帧音频扇区尾补零。
python -B make_av_video.py --inspect D:/deepwork/_av_package_20261031/vid2_tool_codex_20261031_9b72/DEMO12.VID

# 从同一个有音轨的视频转换；--ffmpeg 省略时查 PATH。
python -B make_av_video.py --input 'D:/media/source.mp4' --out 'D:/media/MOVIE.VID' --ffmpeg 'C:/ffmpeg/bin/ffmpeg.exe'

# 本机本轮实际用于测试的已安装程序：
# C:/Users/huang/AppData/Roaming/Python/Python314/site-packages/imageio_ffmpeg/binaries/ffmpeg-win-x86_64-v7.1.exe

# 可选 --fps 12/15/20/24/25；默认推荐12，15/20/24/25需更高吞吐，仅实验。
# 可选 --ffprobe 指向已安装程序进行输入预检查。
# 不要求安装 ffprobe。普通视频输入必须同时有视频和音频流。

# 已有 VID1 无原片时，另配规范 WAV；逐字节复制现有视频，不再翻转。
python -B make_av_video.py --vid1 VIDEO.VID --wav 'D:/media/track.wav' --out 'D:/media/REPACK.VID'
```

VID1+WAV 路径明确假设：两个独立素材的 **VID1 第 0 帧与 WAV 第 0 个采样点同时开始**。
工具无法从这两份文件推断历史同步关系。WAV 必须是无压缩 48kHz、双声道、16bit PCM；
单声道或其他规格明确拒绝。继承 VID1 头的帧率；显式指定不同 `--fps` 时拒绝，不伪造重采样。
接受现有工具实际使用的 512 字节 VID1 头，不接受描述中旧式 16 字节头。

普通视频输入使用 FFmpeg 自动转换采样率、位深和声道，单声道可转换为立体声。
FFmpeg 一次打开同一源，`-copyts -start_at_zero` 将两条轨道按同一个输入原点平移，
没有分别使用 `PTS-STARTPTS`。视频通过 `fps=...:start_time=0` 在共享零点补齐延迟首帧；
音频通过 `aresample=48000:async=1:first_pts=0` 在共享零点补齐初始静音。
音频正好按最终视频帧数截断，不足整帧及缺失的后续音频均补零。
视频帧数由 FFmpeg 的 fps 滤镜输出决定；演示帧数为 `ceil(seconds * fps)`。

原视频滤镜顺序包含 `fps,scale=320:240,vflip`，RGB565LE 输出适配现有
`frame_fifo_write WRITE_V_FLIP=1`，确保上板正立。VID1 的方向保持原字节。
演示同样按倒序行存储：显示后应为顶部 8 行白条、其余区域颜色闪烁。
每个整数秒开始时出现一次约 50ms 的红/绿/蓝/白轮换闪烁和 1kHz 双声道短 beep，
两者起始点严格同步；闪光宽度量化到视频帧边界。beep 峰值 12000（16bit）。
默认12fps每次一帧闪光（约83.33ms），同帧开头为50ms beep，余下约33.33ms静音，
适合观察和听取起点是否同步。每帧音频有效16000字节，尾部再补384字节零。

## 格式合同

所有多字节字段均为 little endian。文件首扇区 512B，未列出的字节全部为零。

| 偏移 | 类型 | 值 |
| --- | --- | --- |
| 0 | 4 bytes | `VID2` |
| 4 | u16 | 320 |
| 6 | u16 | 240 |
| 8 | u16 | 12 / 15 / 20 / 24 / 25；普通视频与演示默认 12 |
| 10 | u32 | frames，必须大于零 |
| 14 | u16 | audio_sectors = ceil(samples_per_frame * 4 / 512) |
| 16 | u32 | 48000 |
| 20 | u16 | samples_per_frame = 48000 / fps |
| 22 | u8 | 2 |
| 23 | u8 | 16 |
| 24 | u16 | **300** |
| 26 | u16 | **1** |
| 28–511 | 484 bytes | 全零 |

每帧 `[300 sectors RGB565LE][audio_sectors sectors PCM16LE L,R 交织]`。
只有音频块前 `samples_per_frame * 4` 字节是有效采样，尾部扇区 padding 不参与播放。

| fps | 每声道每帧采样点 | 音频扇区 | 音频有效字节 | 尾部补零字节 | 帧总字节 |
| --- | --- | --- | --- | --- | --- |
| 12 | 4000 | 32 | 16000 | 384 | 169984 |
| 15 | 3200 | 25 | 12800 | 0 | 166400 |
| 20 | 2400 | 19 | 9600 | 128 | 163328 |
| 24 | 2000 | 16 | 8000 | 192 | 161792 |
| 25 | 1920 | 15 | 7680 | 0 | 161280 |

第 n 帧视频起点为 `512 + n * frame_bytes`，音频起点再加 `153600`。
精确文件大小为 `512 + frames * frame_bytes`。格式本身不按 FIFO 容量裁剪音轨；
本次硬件使用 4096 样本 FIFO，默认预装完整一帧音轨后起播，支持的五种帧率均可容纳。

## 资源、原文件保护

逐帧打包，内存不随片长增长。普通媒体先让 FFmpeg 流式解码到唯一临时目录的两个磁盘文件，
随后逐帧读取并交织；磁盘峰值约为解码后的 RGB565+PCM 加输出 VID2（约输出大小的两倍，
超长音轨还占额外临时空间）。不会把整片装进内存。VID1+WAV 直接逐帧读取。

输出默认拒绝覆盖，`--force` 仅在完整生成且校验通过之后替换；即使 `--force` 也拒绝覆盖输入，
包括同一路径、符号链接解析后的同一路径及硬链接别名。
无 force 时使用同卷硬链接原子发布，避免预检查后另一进程创建输出导致覆盖；输出所在
文件系统须支持硬链接（本轮测试的 NTFS 支持）。请先输出到工作盘，再自行安排后续介质操作。
子进程使用参数列表及 `shell=False` 默认值，不拼接 shell 命令。
失败仅清理本次创建的唯一临时文件/目录，保留已有输出与其他文件。未对 TF 卡执行任何操作。

后续上板时，由用户将本地验收通过的文件**普通复制**到目标介质，并使用现有
`doc/tools/check_frag.py` 检查簇链连续性；不要调用会清卡的旧 `sync_to_sd.py`。
本轮不执行复制、卡扫描或原始设备读写。`doc/tools/check_sd_card.py` 的 `check_vid`
新增 VID2 分支复用本工具的标准库检查器，返回原约定 `(msg, (w,h,fps,frames))`；
VID1 原有诊断、BMP/WAV 检查与扫描流程均保留。

## 测试

```powershell
$env:AV_TEST_FFMPEG = 'C:/Users/huang/AppData/Roaming/Python/Python314/site-packages/imageio_ffmpeg/binaries/ffmpeg-win-x86_64-v7.1.exe'
$env:AV_TEST_TMP = 'D:/deepwork/_av_package_20261031/vid2_tool_codex_20261031_9b72'
python -B -m unittest -v test_make_av_video
```

未配置 FFmpeg 且 PATH 无可用程序时，仅跳过真实媒体集成测试，其余标准库测试仍可运行。
测试从本地生成的原始 RGB、PCM、WAV 构造无损 MKV，不下载媒体、不读取或更改现有 TF 视频。
覆盖字段/偏移、五种帧率（含12fps）、精确像素与 PCM 字节、RGB565 字节序、vflip、VID1 不翻转、
5 秒输入原点+200ms 延迟音轨、晚到视频、短音轨/空音轨/长音轨、单声道拒绝与转换、
44.1kHz 重采样、无音轨拒绝、完整性检查、覆盖竞争、失败清理及流式内存上限。
另用本地 fixture 检验卡诊断兼容分支、坏头/长度/补零拒绝、VID1 本地比较及不触发卡扫描。
内存测试模拟 2000 帧、约 326MB 输出，要求打包循环 Python 峰值小于 2MB。
`DEMO12.VID` 和 `demo12_inspect.json` 是最终12fps验收产物：3秒、36帧、6119936字节。
旧 `DEMO15.VID`、`DEMO20.VID` 原样保留，其对应检查JSON也保留。
`test_default12_final.log` 记录最终默认12fps的完整29项测试结果，包含真实FFmpeg测试与全部五种帧率。
`test_final.log` 和 `test_fps12_15_targeted.log` 为前两轮测试历史记录。
