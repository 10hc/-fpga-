# MJPEG 实验模块

本目录是隔离的实验模块，尚未加入现有 fixed 工程文件。当前可烧录、已由用户验证的基线仍是 build6。

## 已实现

- 纯硬件 baseline JPEG 解码：BEP-JPEG 紧凑配置 FAST=0、RASTER_OUT=1、8192字节行缓冲。
- 320×240输入、RGB565精度行缓存、两倍缩放到640×480，输出32位RGB/零填充帧字。
- MJP1扇区容器读取：长度和帧序检查、物理地址溢出检查、整扇区验证、末字节标记、循环及取消排空。
- 截断/非法JPEG错误上报、无进展超时、输出背压下数据保持。错误帧像素属于临时写入，必须等成功done及SDRAM写完成后才切换显示。
- `pack_mjpeg.py` 离线素材打包与检查，默认转为baseline 4:2:0；需要Python及Pillow。

## 接口与接入边界

顶层 `rtl/mjpeg_stream_top.sv` 的所有接口在50MHz同一时钟域。`next_frame` 必须在解码完成且帧写完成/中止后再给出。现有板级SD/视频写接口为100MHz，尚未完成跨时钟桥接及帧提交控制，不能直接把此模块接到原接口，也不要只因文件存在就加入当前TD工程烧录。

目前只解析项目自定义的MJP1容器，不解析AVI/MOV/MP4，不兼容原VID1文件头；未实现音频复用、播放节拍或音画同步。fps与48kHz样本单位的PTS已作为元数据输出，实际调度由后续板级控制器完成。存储仍要求连续扇区文件。

## 验证与资源

实验构建：`D:\deepwork\_mjpeg_eval_20261031\td_stream2`。50MHz完整布局布线为5195 LUT、2351寄存器、16 BRAM9K、18 DSP；Setup WNS +0.169ns、Hold WNS +0.136ns。该数字属于独立实验顶层，不是完整播放器。

真实SD扇区控制器加模拟命令/字节层，在两种字节间隔下共12帧、3686400输出字逐字校验通过。JPEG原始输出11项、缩放通路12项及中途复位、容器错误15项、打包工具11项通过。报告和日志：`D:\deepwork\_mjpeg_eval_20261031\MJPEG_EVALUATION_REPORT.md`。

## 文件顺序与许可

`rtl/files.f` 列出紧凑核及实验模块的编译顺序，package必须最先编译。TD项目要使用SystemVerilog文件类型，其XML组名为System_Verilog。

JPEG上游： https://github.com/merajhasan88/BEP-JPEG-Decoder ，固定提交 `bb08f3305e97727b3b89049071c7a18bb7e2e75e`。上游RTL未修改，Apache-2.0与NOTICE.md随代码保留；NOTICE含Independent JPEG Group算法来源说明。jpeg_video_pipe、mjpeg_sector_source、mjpeg_stream_top及素材工具为本轮接入代码。

## 素材工具例子

```powershell
python pack_mjpeg.py --output demo_mjp1.vid --fps 20 image01.jpg image02.jpg
python pack_mjpeg.py --inspect demo_mjp1.vid
```

输入图片须320×240，工具拒绝覆盖现有输出。输出仅供后续MJPEG实验工程使用；当前成功上板的fixed/build6不能播放它，暂勿替换TF卡上的VIDEO.VID。
