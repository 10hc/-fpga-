#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""列出 FAT32 卡上每个文件占用的物理扇区范围（直接读卷，不需要管理员）。

FPGA 的扫描窗口是固定的（当前 SCAN_MAX_SECTOR = 131071，即前 64MB），
所以必须确认：每个文件（含簇链）都落在窗口内。

用法: python map_sd_files.py E:
"""
import struct
import sys


def open_volume(letter):
    letter = letter.rstrip("\\/:")
    return open("\\\\.\\" + letter + ":", "rb")


def main():
    letter = sys.argv[1] if len(sys.argv) > 1 else "E:"
    f = open_volume(letter)

    bs = f.read(512)
    if bs[510:512] != b"\x55\xaa":
        sys.exit("不是有效的引导扇区")
    bps, = struct.unpack_from("<H", bs, 11)
    spc = bs[13]
    rsvd, = struct.unpack_from("<H", bs, 14)
    nfats = bs[16]
    fatsz, = struct.unpack_from("<I", bs, 36)
    rootclus, = struct.unpack_from("<I", bs, 44)
    totsec, = struct.unpack_from("<I", bs, 32)
    data_start = rsvd + nfats * fatsz
    print(f"卷 {letter}: {bps}B/扇区 x{spc}扇区/簇, 保留 {rsvd}, FAT×{nfats} 各 {fatsz} 扇区, "
          f"总扇区 {totsec} ({totsec*bps/2**30:.1f}GB)")
    print(f"数据区起始扇区 {data_start} ({data_start*bps/2**20:.1f}MB), 根目录簇 {rootclus}")
    print(f"FPGA 扫描窗口: 0 ~ 131071 扇区 (0 ~ 64.0MB)\n")

    def cluster_lba(c):
        return data_start + (c - 2) * spc

    def read_cluster(c):
        f.seek(cluster_lba(c) * bps)
        return f.read(spc * bps)

    def fat_next(c):
        # 裸卷读取必须按扇区对齐，所以整扇区读进来再取那 4 个字节
        byte = rsvd * bps + c * 4
        sector = (byte // bps) * bps
        f.seek(sector)
        d = f.read(bps)
        v, = struct.unpack_from("<I", d, byte - sector)
        return v & 0x0FFFFFFF

    def chain(c, limit=2000000):
        out = []
        seen = set()
        while 2 <= c < 0x0FFFFFF8 and c not in seen and len(out) < limit:
            seen.add(c)
            out.append(c)
            c = fat_next(c)
        return out

    # 遍历根目录（根目录可能跨多簇）
    entries = []
    for rc in chain(rootclus):
        data = read_cluster(rc)
        for off in range(0, len(data), 32):
            e = data[off:off + 32]
            if len(e) < 32 or e[0] == 0x00:
                break
            if e[0] == 0xE5 or e[11] == 0x0F:      # 删除项 / 长文件名项
                continue
            if e[11] & 0x08 and e[0] != 0x05:       # 卷标
                continue
            name = e[0:8].decode("gbk", "replace").strip()
            ext = e[8:11].decode("gbk", "replace").strip()
            attr = e[11]
            if attr & 0x10:
                continue                            # 目录
            hi, = struct.unpack_from("<H", e, 20)
            lo, = struct.unpack_from("<H", e, 26)
            first = (hi << 16) | lo
            size, = struct.unpack_from("<I", e, 28)
            entries.append((f"{name}.{ext}" if ext else name, first, size))

    print(f"根目录文件 {len(entries)} 个：")
    worst = 0
    for name, first, size in sorted(entries, key=lambda x: x[1]):
        ch = chain(first)
        if not ch:
            print(f"  {name:24s} 起始簇 {first} 空")
            continue
        lbas = [cluster_lba(c) for c in ch]
        lo, hi = min(lbas), max(lbas) + spc - 1
        total_bytes = sum(spc * bps for _ in ch)
        contiguous = all(b - a == spc for a, b in zip(lbas, lbas[1:]))
        inwin = hi <= 131071
        worst = max(worst, hi)
        print(f"  {name:24s} {size:>9} 字节 -> 扇区 {lo} ~ {hi} "
              f"({lo*bps/2**20:.1f} ~ {(hi+1)*bps/2**20:.1f}MB) "
              f"簇数 {len(ch)}{' 连续' if contiguous else ' 分段!'}"
              f"{'  OK' if inwin else '  ★超出扫描窗口!'}")
    print(f"\n最远的扇区: {worst} ({(worst+1)*512/2**20:.1f}MB) "
          f"{'-> 在 64MB 窗口内 OK' if worst <= 131071 else '-> 超出窗口，FPGA 扫不到!'}")


if __name__ == "__main__":
    main()