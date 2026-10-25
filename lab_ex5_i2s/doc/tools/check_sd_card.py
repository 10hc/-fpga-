#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 TF 卡（或 U 盘）里的文件是否符合 FPGA 工程要求。

检查项：
  1. 图片：必须是 640x480、24 位、无压缩的 BMP（文件大小 = 54 + 640*480*3 = 921654）
  2. 音频：必须是 48kHz / 16bit / 2 声道的 PCM WAV
  3. 视频：.VID 头部 'VID1' + 宽 320 高 240，且每帧 320*240*2 字节
  4. 和本地已翻转/补齐的 VIDEO.VID 对比（判断卡上是不是新版）
  5. 尝试读卷引导扇区，算出数据区起始扇区（FPGA 扫描范围要覆盖到）

用法: python check_sd_card.py E:\  [--local-vid <本地VIDEO.VID路径>]
"""
import argparse
import hashlib
import os
import struct
import sys


def check_bmp(path):
    with open(path, "rb") as f:
        head = f.read(54)
    if len(head) < 54 or head[0:2] != b"BM":
        return "不是 BMP（缺少 BM 标记）"
    size, = struct.unpack_from("<I", head, 2)
    off, = struct.unpack_from("<I", head, 10)
    w, h = struct.unpack_from("<ii", head, 18)
    planes, bpp = struct.unpack_from("<HH", head, 26)
    comp, = struct.unpack_from("<I", head, 30)
    real = os.path.getsize(path)
    problems = []
    if (w, h) != (640, 480):
        problems.append(f"尺寸 {w}x{h} 不是 640x480")
    if bpp != 24:
        problems.append(f"{bpp} 位不是 24 位")
    if comp != 0:
        problems.append(f"压缩方式 {comp} 不是 0（无压缩）")
    if real != 54 + 640 * 480 * 3:
        problems.append(f"文件大小 {real} 不等于 921654")
    if size != real:
        problems.append(f"头部声明大小 {size} 与实际 {real} 不符")
    return "OK" if not problems else "；".join(problems)


def check_wav(path):
    with open(path, "rb") as f:
        d = f.read(64)
    if d[0:4] != b"RIFF" or d[8:12] != b"WAVE":
        return "不是 WAV（RIFF/WAVE 标记缺失）"
    fmt, = struct.unpack_from("<H", d, 20)
    ch, rate = struct.unpack_from("<HI", d, 22)
    bits, = struct.unpack_from("<H", d, 34)
    p = []
    if fmt != 1:
        p.append(f"fmt={fmt} 不是 PCM")
    if ch != 2:
        p.append(f"{ch} 声道不是 2")
    if rate != 48000:
        p.append(f"{rate}Hz 不是 48000Hz")
    if bits != 16:
        p.append(f"{bits} 位不是 16 位")
    return "OK" if not p else "；".join(p)


def check_vid(path, local=None):
    with open(path, "rb") as f:
        head = f.read(512)
    if head[0:4] != b"VID1":
        return "不是 .VID（缺少 VID1 标记）", None
    w, h, fps, frames = struct.unpack_from("<HHHI", head, 4)
    real = os.path.getsize(path)
    need = 512 + frames * w * h * 2
    p = []
    if (w, h) != (320, 240):
        p.append(f"分辨率 {w}x{h} 不是 320x240")
    if real < need:
        p.append(f"文件 {real} 字节，比头部声明的 {need} 少 {need - real}")
    msg = "OK" if not p else "；".join(p)
    if local and os.path.exists(local):
        def sha256_file(pth):
            m = hashlib.sha256()
            with open(pth, "rb") as f:
                for chunk in iter(lambda: f.read(1 << 20), b""):
                    m.update(chunk)
            return m.hexdigest()
        a, b = sha256_file(path), sha256_file(local)
        same = (a == b)
        msg += f"；与本地 {os.path.basename(local)} {'完全相同(新翻转版)' if same else '不同(旧版或未翻转)'}"
        # 头部相同但内容不同时，再比一下“是否是本地文件翻转后的样子”
        if not same:
            with open(path, "rb") as f:
                sa = f.read(512 + 240 * 320 * 2)
            with open(local, "rb") as f:
                sb = f.read(512 + 240 * 320 * 2)
            fa = sa[512:].reshape if False else None
            try:
                import numpy as np
                A = np.frombuffer(sa[512:], dtype=np.uint8).reshape(240, 640)[::-1].reshape(-1)
                B = np.frombuffer(sb[512:], dtype=np.uint8).reshape(240, 640)
                if A.tobytes() == B.tobytes():
                    msg += "（卡上是本地文件的上下镜像版本）"
            except Exception:
                pass
    return msg, (w, h, fps, frames)


def boot_sector_info(drive_root):
    """尝试读卷引导扇区（需要管理员权限，失败就跳过）"""
    letter = drive_root.rstrip("\\/")[:2]
    path = "\\\\.\\" + letter
    try:
        with open(path, "rb") as f:
            bs = f.read(512)
    except Exception as e:
        return f"读不到引导扇区（{e.__class__.__name__}: {e}），跳过"
    if len(bs) < 90:
        return "引导扇区太短"
    bps, = struct.unpack_from("<H", bs, 11)
    spc = bs[13]
    rsvd, = struct.unpack_from("<H", bs, 14)
    nfats = bs[16]
    fatsz, = struct.unpack_from("<I", bs, 36)
    rootclus, = struct.unpack_from("<I", bs, 44)
    data_start = rsvd + nfats * fatsz
    return (f"扇区 {bps}B x{spc}簇, 保留 {rsvd}, FAT×{nfats} 每个 {fatsz} 扇区, "
            f"根目录簇 {rootclus}, 数据区起始扇区 {data_start} ({data_start * bps / 1048576:.1f}MB)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root", help="盘符，例如 E:\\")
    ap.add_argument("--local-vid", default=None)
    a = ap.parse_args()

    root = a.root
    print(f"== 检查 {root} ==")
    print("[卷信息]", boot_sector_info(root))

    names = os.listdir(root)
    bmps = [n for n in names if n.lower().endswith(".bmp")]
    wavs = [n for n in names if n.lower().endswith(".wav")]
    vids = [n for n in names if n.lower().endswith(".vid")]

    print(f"\n[1] 图片 {len(bmps)} 个")
    for n in sorted(bmps):
        print(f"  {n}  ({os.path.getsize(os.path.join(root, n))} 字节)  -> {check_bmp(os.path.join(root, n))}")

    print(f"\n[2] 音频 {len(wavs)} 个")
    for n in sorted(wavs):
        print(f"  {n}  ({os.path.getsize(os.path.join(root, n))} 字节)  -> {check_wav(os.path.join(root, n))}")

    print(f"\n[3] 视频 {len(vids)} 个")
    for n in sorted(vids):
        msg, info = check_vid(os.path.join(root, n), a.local_vid)
        print(f"  {n}  ({os.path.getsize(os.path.join(root, n))} 字节)")
        if info:
            print(f"      头部: {info[0]}x{info[1]} {info[2]}fps {info[3]} 帧")
        print(f"      -> {msg}")

    others = [n for n in names if not n.lower().endswith((".bmp", ".wav", ".vid"))]
    if others:
        print(f"\n[其他] {others}")


if __name__ == "__main__":
    main()