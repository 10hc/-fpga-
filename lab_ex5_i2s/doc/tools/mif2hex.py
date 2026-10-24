# -*- coding: utf-8 -*-
"""
mif2hex.py —— 把 Quartus/安路 的 .mif 文件转成 ModelSim $readmemh 能直接读的纯十六进制文件。

原因：.mif 带 WIDTH/DEPTH/ADDRESS_RADIX/DATA_RADIX 头和 "addr : data;" 语法，
      $readmemh 解析不了，会把 ROM 当空的（仿真里字库全 0，但综合/上板正常，
      因为 TD 的 BRAM 生成器认识 .mif）。仿真想看到真字模就跑一下本脚本。

用法：python mif2hex.py <in.mif> [out.hex]
"""
import re
import sys


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else re.sub(r"\.mif$", ".hex", src)
    depth = None
    values = {}
    for line in open(src, "r", encoding="utf-8", errors="replace"):
        s = line.split("--")[0].split("%")[0].strip()
        if not s:
            continue
        m = re.match(r"DEPTH\s*=\s*(\d+)", s, re.I)
        if m:
            depth = int(m.group(1))
            continue
        m = re.match(r"([0-9A-Fa-f]+)\s*:\s*([0-9A-Fa-fxXzZ]+)\s*;?", s)
        if m:
            values[int(m.group(1), 16)] = m.group(2)
            continue
        m = re.match(r"\[\s*([0-9A-Fa-f]+)\s*\.\.\s*([0-9A-Fa-f]+)\s*\]\s*:\s*([0-9A-Fa-f]+)\s*;?", s)
        if m:
            a, b, v = int(m.group(1), 16), int(m.group(2), 16), m.group(3)
            for i in range(a, b + 1):
                values[i] = v
    if depth is None:
        depth = (max(values) + 1) if values else 0
    out = ["@%04X %s" % (0, values.get(0, "00"))]
    for i in range(1, depth):
        out.append(values.get(i, "00"))
    open(dst, "w", encoding="utf-8").write("\n".join(out) + "\n")
    print("wrote %s (%d words, 非零 %d 个)" % (dst, depth, sum(1 for i in range(depth) if values.get(i, "00").strip("0") != "")))
    return 0


if __name__ == "__main__":
    sys.exit(main())