#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
dump_sd_sectors.py  <盘符>

把 TF 卡的真实扇区 dump 出来，给 ModelSim 里的 fat32_scan 做离线验证：
  LBA0(MBR) / 分区起始扇区(VBR) / FAT表 / 根目录扇区  都存成 .bin
同时打印一张"卡里到底有什么"的表（含长文件名、起始簇、起始扇区、是否连续）。

用法：  python dump_sd_sectors.py E
输出目录： D:\deepwork\_carddump\
"""
import os
import struct
import sys

OUT = r"D:\deepwork\_carddump"
SEC = 512


def rd(f, sector, count=1):
    """读扇区（必须扇区对齐）"""
    f.seek(sector * SEC)
    data = f.read(count * SEC)
    if len(data) < count * SEC:
        raise IOError("short read at sector %d" % sector)
    return data


def le16(b, o):
    return b[o] | (b[o + 1] << 8)


def le32(b, o):
    return b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)


def lfn_name(entries):
    """从 LFN 目录项拼出长文件名（entries 是 0x0F 项列表，已按序号排好）"""
    s = []
    for e in entries:
        for a, n in ((1, 10), (14, 12), (28, 4)):
            chunk = e[a:a + n]
            for i in range(0, len(chunk), 2):
                lo, hi = chunk[i], chunk[i + 1]
                if lo == 0 and hi == 0:
                    return "".join(s)
                if lo == 0xFF and hi == 0xFF:
                    return "".join(s)
                s.append(chr(lo) if hi == 0 else "?")
    return "".join(s)


def short_name(e):
    base = e[0:8].decode("latin1").rstrip()
    ext = e[8:11].decode("latin1").rstrip()
    return base + ("." + ext if ext else "")


def main():
    if len(sys.argv) < 2:
        print("用法: python dump_sd_sectors.py <盘符，如 E>")
        return 2
    letter = sys.argv[1].strip().rstrip(":")
    path = r"\\.\%s:" % letter
    if not os.path.isdir(OUT):
        os.makedirs(OUT)
    print("打开裸设备 %s ..." % path)
    f = open(path, "rb")

    # ---------- LBA0 ----------
    lba0 = rd(f, 0)
    open(os.path.join(OUT, "lba0.bin"), "wb").write(lba0)
    print("\n== LBA0 (0x55AA=%02X%02X) ==" % (lba0[511], lba0[510]))
    parts = []
    for k in range(4):
        o = 446 + 16 * k
        ptype = lba0[o + 4]
        start = le32(lba0, o + 8)
        size = le32(lba0, o + 12)
        if ptype or size:
            parts.append((start, size, ptype))
            print("  分区%d: type=0x%02X 起始LBA=%d 扇区数=%d (%.2f MB)"
                  % (k + 1, ptype, start, size, size * SEC / 1048576.0))
    if not parts:
        print("  (没有分区表 -> 可能是 superfloppy，VBR 直接在 LBA0)")

    # 判断 LBA0 到底是 MBR 还是 VBR(superfloppy)
    VALID_TYPES = (0x01, 0x04, 0x05, 0x06, 0x07, 0x0B, 0x0C, 0x0E, 0x0F, 0x83, 0xEE)
    is_vbr = (le16(lba0, 11) == SEC) and (lba0[13] in (1, 2, 4, 8, 16, 32, 64, 128))
    good_parts = [p for p in parts if p[2] in VALID_TYPES and 0 < p[1] < 0x40000000
                  and p[0] < 0x40000000]
    part_start = 0
    if good_parts and not is_vbr:
        for start, size, ptype in good_parts:
            if ptype in (0x0B, 0x0C):
                part_start = start
                break
        else:
            part_start = good_parts[0][0]
        print("  -> 识别为 MBR，按分区起始 LBA = %d 解析 BPB" % part_start)
    else:
        print("  -> LBA0 本身就是 VBR（superfloppy：无有效分区表，"
              "每扇区=%d 每簇=%d 扇区）" % (le16(lba0, 11), lba0[13]))
        part_start = 0

    # ---------- VBR / BPB ----------
    vbr = rd(f, part_start)
    open(os.path.join(OUT, "vbr.bin"), "wb").write(vbr)
    bps = le16(vbr, 11)
    spc = vbr[13]
    reserved = le16(vbr, 14)
    nfat = vbr[16]
    fat_size = le16(vbr, 22) or le32(vbr, 36)
    root_cluster = le32(vbr, 44)
    tot_sec = le16(vbr, 19) or le32(vbr, 32)
    print("\n== VBR (扇区 %d, 签名 0x%02X%02X) ==" % (part_start, vbr[511], vbr[510]))
    print("  每扇区字节=%d  每簇扇区=%d  保留扇区=%d  FAT数=%d  FAT大小=%d 扇区"
          % (bps, spc, reserved, nfat, fat_size))
    print("  根目录簇=%d  总扇区=%d (%.2f MB)" % (root_cluster, tot_sec,
                                                tot_sec * SEC / 1048576.0))
    if bps != SEC:
        print("  !! 每扇区字节不是 512，FPGA 侧按 512 处理会出错")
    data_start = part_start + reserved + nfat * fat_size
    print("  数据区起始扇区 = %d + %d + %d*%d = %d (%.2f MB)"
          % (part_start, reserved, nfat, fat_size, data_start,
             data_start * SEC / 1048576.0))

    # ---------- FAT ----------
    fat_sec = part_start + reserved
    fat0 = rd(f, fat_sec, 2)
    open(os.path.join(OUT, "fat0.bin"), "wb").write(fat0)
    print("\n== FAT 表起始扇区 %d（已存 fat0.bin，2 扇区） ==" % fat_sec)

    def fat_entry(c):
        off = c * 4
        sec = fat_sec + off // SEC
        if sec != fat_sec and sec != fat_sec + 1:
            return None
        b = fat0
        return le32(b, off % SEC)

    # ---------- 根目录 ----------
    clu = root_cluster
    root_sectors = []
    for _ in range(8):                      # 最多跟 8 个簇
        s0 = data_start + (clu - 2) * spc
        for i in range(spc):
            root_sectors.append(s0 + i)
        nxt = fat_entry(clu)
        if nxt is None or nxt >= 0x0FFFFFF8 or nxt < 2:
            break
        clu = nxt
    print("\n== 根目录簇链 = 起始 %d，共 %d 个扇区 ==" % (root_cluster, len(root_sectors)))
    dirs = b""
    for i, s in enumerate(root_sectors):
        blk = rd(f, s)
        dirs += blk
        open(os.path.join(OUT, "root%d.bin" % i), "wb").write(blk)

    # ---------- 解析目录项 ----------
    print("\n== 根目录内容 ==")
    print("%-14s %-46s %-6s %-10s %-10s %s"
          % ("8.3名", "长文件名", "属性", "起始簇", "大小", "起始扇区/连续"))
    lfn_buf = []
    count = 0
    for o in range(0, len(dirs), 32):
        e = dirs[o:o + 32]
        if len(e) < 32 or e[0] == 0x00:
            break
        if e[0] == 0xE5:
            lfn_buf = []
            continue
        attr = e[11]
        if attr == 0x0F:
            lfn_buf.insert(0, e)
            continue
        if attr & 0x08:
            lfn_buf = []
            continue
        cl = ((e[20] | (e[21] << 8)) << 16) | e[26] | (e[27] << 8)   # FAT32: HI@20, LO@26
        size = le32(e, 28)
        name = lfn_name(lfn_buf) or short_name(e)
        lfn_buf = []
        if cl < 2:
            print("  %-14s %-46s 0x%02X   簇=%d (无效)" % (short_name(e), name, attr, cl))
            continue
        s0 = data_start + (cl - 2) * spc
        nsec = (size + SEC - 1) // SEC
        cont = "?"
        if size:
            nxt = fat_entry(cl)
            cont = "是" if (nxt == cl + 1 or nxt is None or
                            nxt >= 0x0FFFFFF8 or nxt == 0) else "链 %d" % nxt
        print("  %-14s %-46s 0x%02X   簇=%-8d %-10d 扇区 %d 起, %d 个, 连续=%s"
              % (short_name(e), name[:46], attr, cl, size, s0, nsec, cont))
        count += 1
    print("\n  共 %d 个文件/目录项" % count)
    print("\n原始扇区已保存到 %s（lba0.bin / vbr.bin / fat0.bin / root*.bin）" % OUT)
    f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())