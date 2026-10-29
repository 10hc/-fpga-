# -*- coding: utf-8 -*-
"""从 TF 卡原始卷读出 A1.WAV，算出 FPGA 侧“前 4 个流式扇区”的 16 位字节校验和。

FPGA 的约定：file_sector = 文件所在首扇区（fat32_scan 给出）
             data_start  = file_sector + 1
             sum16       = data_start 起连续 4 个扇区（2048 字节）逐字节求和，取低 16 位

用法: python wav_sum16.py E
"""
import sys, struct, os

def read_sectors(f, lba, n):
    f.seek(lba * 512)
    return f.read(n * 512)

def main():
    letter = sys.argv[1].strip().rstrip(':').upper()
    path = r"\\.\%s:" % letter
    f = open(path, 'rb')
    vbr = read_sectors(f, 0, 1)
    assert vbr[510:512] == b'\x55\xaa', "no VBR signature at LBA0"
    bps      = struct.unpack('<H', vbr[11:13])[0]
    spc      = vbr[13]
    reserved = struct.unpack('<H', vbr[14:16])[0]
    nfats    = vbr[16]
    fatsz    = struct.unpack('<I', vbr[36:40])[0]
    rootclus = struct.unpack('<I', vbr[44:48])[0]
    data_lba = reserved + nfats * fatsz
    print("bps=%d spc=%d reserved=%d nfats=%d fatsz=%d rootclus=%d data_lba=%d"
          % (bps, spc, reserved, nfats, fatsz, rootclus, data_lba))

    def clus_lba(c):
        return data_lba + (c - 2) * spc

    fat_lba = reserved
    def next_clus(c):
        raw = read_sectors(f, fat_lba + (c * 4) // 512, 1)
        return struct.unpack('<I', raw[(c * 4) % 512:(c * 4) % 512 + 4])[0] & 0x0FFFFFFF

    # 遍历根目录，找 A1.WAV
    target = None
    clus = rootclus
    while clus < 0x0FFFFFF8:
        d = read_sectors(f, clus_lba(clus), spc)
        for i in range(0, len(d), 32):
            e = d[i:i + 32]
            if e[0] == 0:
                break
            if e[0] == 0xE5:
                continue
            if e[11] == 0x0F:          # LFN
                continue
            name = e[0:8].decode('ascii', 'replace').strip()
            ext = e[8:11].decode('ascii', 'replace').strip()
            full = (name + '.' + ext) if ext else name
            if full.upper() == "A1.WAV":
                hi = struct.unpack('<H', e[20:22])[0]
                lo = struct.unpack('<H', e[26:28])[0]
                size = struct.unpack('<I', e[28:32])[0]
                target = ((hi << 16) | lo, size)
                break
        if target:
            break
        clus = next_clus(clus)
    assert target, "A1.WAV not found in root dir"
    first, size = target
    file_lba = clus_lba(first)
    print("A1.WAV: first_cluster=%d  first_sector(LBA)=%d  size=%d" % (first, file_lba, size))

    # 校验文件头（和 FPGA 的判定一致）
    head = read_sectors(f, file_lba, 1)
    ok = (head[0:4] == b'RIFF' and head[8:12] == b'WAVE' and head[36:40] == b'data'
          and struct.unpack('<H', head[20:22])[0] == 1
          and struct.unpack('<H', head[22:24])[0] == 2
          and struct.unpack('<I', head[24:28])[0] == 48000
          and struct.unpack('<H', head[34:36])[0] == 16)
    print("header check (RIFF/WAVE/data/PCM/2ch/48k/16bit):", "PASS" if ok else "FAIL")
    print("chunk size @40:", struct.unpack('<I', head[40:44])[0])

    data_start = file_lba + 1
    seg = read_sectors(f, data_start, 4)
    s = sum(seg) & 0xFFFF
    print("data_start=%d  bytes=%d  SUM16=0x%04X (%d)" % (data_start, len(seg), s, s))

    vals = struct.unpack('<%dh' % (len(seg)//2), seg[:len(seg)//2*2])
    print("首 8 个样本:", vals[:8], " 峰值:", max(vals), min(vals))
    f.close()

if __name__ == '__main__':
    main()