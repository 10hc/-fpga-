#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 .VID 文件里每一帧上下翻转（行序颠倒）。

为什么需要：
    FPGA 写入通道开着 WRITE_V_FLIP（frame_read_write 的 VFLIP），
    这是为了纠正 BMP 图片自下而上的行序（PC 上存的 BMP 是倒着的）。
    但 make_video.py 生成的 .VID 是自上而下的正常行序，
    写进 SDRAM 后被镜像一次，屏幕上就成了“倒立”的画面。

所以：.VID 预先翻转一次，FPGA 再镜像一次 -> 屏幕正立。
     本脚本是自逆的（再跑一次就变回原样）。

用法:
    python flip_vid.py VIDEO.VID                 # 原地翻转（先写 .tmp 再替换）
    python flip_vid.py VIDEO.VID --preview p.png # 顺便导出翻转前后对比图
"""
import argparse
import os
import struct
import sys

import numpy as np

MAGIC = b"VID1"


def rgb565_to_image(buf, w, h):
    """RGB565 小端字节 -> PIL 图片（仅用于预览）"""
    from PIL import Image
    a = np.frombuffer(buf, dtype="<u2", count=w * h).reshape(h, w)
    r = ((a >> 11) & 0x1F) << 3
    g = ((a >> 5) & 0x3F) << 2
    b = (a & 0x1F) << 3
    return Image.fromarray(np.dstack([r, g, b]).astype(np.uint8), "RGB")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="VIDEO.VID")
    ap.add_argument("--preview", default=None,
                    help="导出 PNG：左边翻转前、右边翻转后（仅看第 1 帧和第 60 帧）")
    a = ap.parse_args()

    with open(a.path, "rb") as f:
        head = f.read(512)
        if len(head) < 512 or head[0:4] != MAGIC:
            sys.exit("不是 .VID 文件（magic = %r）" % head[0:4])
        w, h, fps, frames = struct.unpack_from("<HHHI", head, 4)
        data = f.read()

    row = w * 2
    per_frame = w * h * 2
    full = min(frames, len(data) // per_frame)
    print("分辨率 %dx%d, %d fps, 头部声明 %d 帧, 实际完整帧 %d 帧, 每帧 %d 字节"
          % (w, h, fps, frames, full, per_frame))
    if full != frames:
        print("提示: 文件比头部声明短 %d 字节（末帧不完整，按原样保留）"
              % (frames * per_frame - len(data)))

    arr = np.frombuffer(data, dtype=np.uint8, count=full * per_frame)
    arr = arr.reshape(full, h, row)[:, ::-1, :]          # 逐帧上下翻转
    out = bytearray(arr.tobytes())
    out += data[full * per_frame:]                        # 尾巴原样保留

    if a.preview:
        from PIL import Image
        idx = [0, min(60, full - 1)]
        tiles = []
        for n in idx:
            before = data[n * per_frame:(n + 1) * per_frame]
            after = bytes(out[n * per_frame:(n + 1) * per_frame])
            tiles.append(rgb565_to_image(before, w, h))
            tiles.append(rgb565_to_image(after, w, h))
        canvas = Image.new("RGB", (w * 2 + 12, h * len(idx) + 6 * (len(idx) - 1)), (40, 40, 40))
        for i, t in enumerate(tiles):
            canvas.paste(t, ((i % 2) * (w + 12), (i // 2) * (h + 6)))
        canvas.save(a.preview)
        print("对比图已保存: %s（左=翻转前 右=翻转后，上=第1帧 下=第%d帧）"
              % (a.preview, idx[1] + 1))

    tmp = a.path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(head)
        f.write(out)
    os.replace(tmp, a.path)
    print("已翻转: %s（%d 字节）" % (a.path, 512 + len(out)))


if __name__ == "__main__":
    main()