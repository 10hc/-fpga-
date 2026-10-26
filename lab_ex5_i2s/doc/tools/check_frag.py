#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
check_frag.py <盘符>

检查 TF 卡里每个文件的簇链是否**连续**。
本工程（fat32_scan + bmp_read + video_player）都按"连续扇区"读文件，
一旦文件在卡上被拆开（碎片），读出来的就是乱码 —— 这正是"切图后乱码"的经典原因。

用法:  python check_frag.py E
"""
import os
import sys

SEC = 512


def rd(f, sector, count=1):
    f.seek(sector * SEC)
    return f.read(count * SEC)


def u16(b, o):
    return b[o] | (b[o + 1] << 8)


def u32(b, o):
    return b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)


def lfn_name(entries):
    s = []
    for e in entries:
        for a, n in ((1, 10), (14, 12), (28, 4)):
            ch = e[a:a + n]
            for i in range(0, len(ch), 2):
                lo, hi = ch[i], ch[i + 1]
                if (lo == 0 and hi == 0) or (lo == 0xFF and hi == 0xFF):
                    return "".join(s)
                s.append(chr(lo) if hi == 0 else "?")
    return "".join(s)


def main():
    letter = sys.argv[1].strip().rstrip(":") if len(sys.argv) > 1 else "E"
    f = open(r"\\.\%s:" % letter, "rb")

    vbr = rd(f, 0)
    bps = u16(vbr, 11)
    spc = vbr[13]
    reserved = u16(vbr, 14)
    nfat = vbr[16]
    fat_size = u16(vbr, 22) or u32(vbr, 36)
    root_cluster = u32(vbr, 44)
    data_start = reserved + nfat * fat_size
    print("每扇区=%d 每簇=%d 扇区 保留=%d FAT=%d×%d 数据区=%d 根簇=%d"
          % (bps, spc, reserved, nfat, fat_size, data_start, root_cluster))

    fat = rd(f, reserved, fat_size)

    def fat_of(c):
        o = c * 4
        return u32(fat, o) & 0x0FFFFFFF

    # 根目录簇链
    clu = root_cluster
    dirs = b""
    for _ in range(64):
        dirs += rd(f, data_start + (clu - 2) * spc, spc)
        nxt = fat_of(clu)
        if nxt >= 0x0FFFFFF8 or nxt < 2:
            break
        clu = nxt

    print("\n%-16s %-40s %-9s %-8s %-9s %s"
          % ("8.3名", "长文件名", "大小", "簇数", "首扇区", "簇链是否连续"))
    lfn = []
    bad = 0
    for o in range(0, len(dirs), 32):
        e = dirs[o:o + 32]
        if len(e) < 32 or e[0] == 0x00:
            break
        if e[0] == 0xE5:
            lfn = []
            continue
        attr = e[11]
        if attr == 0x0F:
            lfn.insert(0, e)
            continue
        if attr & 0x08:
            lfn = []
            continue
        cl = ((e[20] | (e[21] << 8)) << 16) | e[26] | (e[27] << 8)
        size = u32(e, 28)
        name = lfn_name(lfn) or (e[0:8].decode("latin1").strip() + "." +
                                 e[8:11].decode("latin1").strip())
        lfn = []
        if cl < 2:
            continue
        nclu = (size + bps * spc - 1) // (bps * spc) if size else 0
        chain = []
        c = cl
        for _ in range(nclu + 2):
            chain.append(c)
            if len(chain) >= nclu:
                break
            nxt = fat_of(c)
            if nxt >= 0x0FFFFFF8 or nxt == 0:
                break
            c = nxt
        cont = (chain == list(range(cl, cl + len(chain))))
        if size and not cont:
            bad += 1
        extra = ""
        if size and not cont:
            extra = "   <-- 碎片! 链: %s%s" % (chain[:6], "..." if len(chain) > 6 else "")
        print("%-16s %-40s %-9d %-8d %-9d %s%s"
              % (e[0:8].decode("latin1").strip(), name[:40], size, nclu,
                 data_start + (cl - 2) * spc, "是" if cont else "否", extra))

    print("\n== 结论：%s ==" % ("有 %d 个文件被拆开（碎片）→ 必须重排或改读簇链" % bad
                              if bad else "所有文件都连续，设计可以按连续扇区读"))
    f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())