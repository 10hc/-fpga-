#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_video.py —— 生成 FPGA 播放用的视频文件（.VID 专用格式）

格式（全部小端）：
    偏移 0  : "VID1"        4 字节魔数
    偏移 4  : width   u16   源宽（本工程固定 320）
    偏移 6  : height  u16   源高（本工程固定 240）
    偏移 8  : fps     u16   目标帧率
    偏移 10 : frames  u32   帧数
    偏移 14 : reserved 2 字节
    偏移 16 : 逐帧数据，每帧 width*height*2 字节（RGB565，小端）

为什么是 320x240 RGB565：
    每帧 320*240*2 = 153600 字节 = 恰好 300 个扇区（512B），
    帧与扇区边界对齐，FPGA 侧无需处理半扇区；SD 卡高速 SPI 50MHz
    实测约 5MB/s，可支撑约 30fps，FPGA 侧再 2 倍放大到 640x480 全屏。

用法：
    python make_video.py                     # 生成默认测试视频
    python make_video.py --images 图1.bmp 图2.bmp --seconds 8
"""

import argparse
import math
import os
import struct
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 320, 240
MAGIC = b"VID1"
HEADER = 16


# ---------------------------------------------------------------- 工具
def load_font(size):
    """尽量找一个能显示数字/英文的字体"""
    for name in ("arialbd.ttf", "arial.ttf", "consolab.ttf", "msyhbd.ttc", "msyh.ttc"):
        p = os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts", name)
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()


def to_rgb565(img):
    """PIL RGB 图 -> RGB565 小端字节（numpy 加速）"""
    a = np.asarray(img.convert("RGB"), dtype=np.uint16)
    v = ((a[:, :, 0] >> 3) << 11) | ((a[:, :, 1] >> 2) << 5) | (a[:, :, 2] >> 3)
    return v.astype("<u2").tobytes()


def fit_cover(img, w, h):
    """等比缩放并居中裁剪到 w x h"""
    iw, ih = img.size
    s = max(w / iw, h / ih)
    nw, nh = int(iw * s + 0.5), int(ih * s + 0.5)
    img = img.resize((nw, nh), Image.LANCZOS)
    return img.crop(((nw - w) // 2, (nh - h) // 2, (nw - w) // 2 + w, (nh - h) // 2 + h))


# ---------------------------------------------------------------- 画面合成
class Scene:
    """滚动全景 + 弹跳球 + 帧号 + 进度条（明显看得出是"视频"）"""

    def __init__(self, images, font_big, font_small):
        # 把每张图缩到 320x240，横向拼成全景（拼两份以便无缝滚动）
        tiles = [fit_cover(Image.open(p), W, H) for p in images] if images else []
        if not tiles:
            tiles = [self._synth_tile(i) for i in range(3)]
        pano = Image.new("RGB", (W * len(tiles), H))
        for i, t in enumerate(tiles):
            pano.paste(t, (i * W, 0))
        self.pano_w = pano.width
        self.pano = Image.new("RGB", (pano.width * 2, H))
        self.pano.paste(pano, (0, 0))
        self.pano.paste(pano, (pano.width, 0))
        self.font_big = font_big
        self.font_small = font_small

    @staticmethod
    def _synth_tile(seed):
        """没有图片时生成彩色渐变块"""
        t = Image.new("RGB", (W, H))
        d = ImageDraw.Draw(t)
        for y in range(0, H, 4):
            for x in range(0, W, 4):
                d.rectangle([x, y, x + 3, y + 3],
                            fill=((x * 255 // W + seed * 80) % 256,
                                  (y * 255 // H + seed * 40) % 256,
                                  ((x + y) * 255 // (W + H) + seed * 120) % 256))
        return t

    def frame(self, n, total, fps):
        # 全景滚动（每帧 1.2 像素，8 秒走完一圈）
        ox = int((n * 1.2) % self.pano_w)
        img = self.pano.crop((ox, 0, ox + W, H)).copy()
        d = ImageDraw.Draw(img)

        # 弹跳球
        bx = int((W - 40) * (0.5 - 0.5 * math.cos(n * 0.07)))
        by = int(30 + 20 * abs(math.sin(n * 0.11)))
        d.ellipse([bx, by, bx + 34, by + 34], fill=(255, 80, 60), outline=(255, 255, 255))

        # 旋转方块
        cx, cy, r = 268, 196, 22
        pts = []
        for k in range(4):
            a = n * 0.06 + k * math.pi / 2
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
        d.polygon(pts, fill=(60, 220, 255), outline=(255, 255, 255))

        # 顶部标题条
        d.rectangle([0, 0, W, 26], fill=(0, 0, 0))
        d.text((6, 4), "HDMI MULTIMEDIA PLAYER  320x240", font=self.font_small, fill=(255, 255, 255))

        # 帧号（大号）
        d.text((8, H - 52), "F%04d" % n, font=self.font_big, fill=(255, 255, 0))

        # 底部进度条
        d.rectangle([0, H - 8, W, H], fill=(0, 0, 0))
        d.rectangle([0, H - 8, int(W * n / max(1, total - 1)), H], fill=(0, 255, 120))

        return img


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", nargs="*", default=None, help="用于制作滚动全景的图片")
    ap.add_argument("--out", default="VIDEO.VID", help="输出文件名")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--preview", default=None, help="可选：导出一张预览 PNG")
    args = ap.parse_args()

    images = args.images or []
    if not images:
        # 默认：用工程自带的三张 BMP
        root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "TF卡图片")
        for n in ("草莓_640x480_24bit_显示正常_工程适配版.bmp",
                  "苹果_640x480_24bit_显示正常_工程适配版.bmp",
                  "西瓜_640x480_24bit_显示正常_工程适配版.bmp"):
            p = os.path.normpath(os.path.join(root, n))
            if os.path.exists(p):
                images.append(p)
        print("使用工程自带图片:", [os.path.basename(p) for p in images])

    total = int(args.seconds * args.fps)
    scene = Scene(images, load_font(30), load_font(15))

    with open(args.out, "wb") as f:
        # 头部独占一个扇区（512 字节），帧数据从扇区 1 开始，
        # 这样每帧恰好 300 个扇区，FPGA 侧无需处理偏移
        head = bytearray(512)
        head[0:4] = MAGIC
        struct.pack_into("<HHHI", head, 4, W, H, args.fps, total)
        f.write(head)
        for n in range(total):
            img = scene.frame(n, total, args.fps)
            f.write(to_rgb565(img))
            if args.preview and n == total // 3:
                img.save(args.preview)
            if (n + 1) % 25 == 0:
                print(f"  已生成 {n+1}/{total} 帧")

    size = os.path.getsize(args.out)
    per_frame = W * H * 2
    print(f"\n完成: {args.out}")
    print(f"  {W}x{H} RGB565, {args.fps}fps, {total} 帧, 时长 {total/args.fps:.1f}s")
    print(f"  文件大小 {size} 字节 ({size/1024/1024:.1f} MB)")
    print(f"  每帧 {per_frame} 字节 = {per_frame//512} 个扇区（{per_frame%512} 余数）")
    assert per_frame % 512 == 0, "每帧必须整除扇区大小"


if __name__ == "__main__":
    main()