# -*- coding: utf-8 -*-
"""
pdfgrep.py —— 免依赖 PDF 文本搜索（只用标准库 zlib + re）

原理：PDF 的页面内容流通常是 Flate 压缩的，解压后文本以
      (字符串) Tj  或  [(a) -2 (b)] TJ  的形式出现。
      本脚本把所有能解压的流解出来，抽掉 PDF 转义，然后在文本里搜关键字。

用法：
    python pdfgrep.py <file.pdf> <关键字1> [关键字2 ...]
输出：命中的行（含关键字的那一整段文本），以及上下文窗口。
"""
import re
import sys
import zlib


def unescape_pdf(s: bytes) -> str:
    out = []
    i = 0
    while i < len(s):
        c = s[i]
        if c == 0x5C and i + 1 < len(s):          # backslash
            n = s[i + 1]
            m = {0x6E: 0x0A, 0x72: 0x0D, 0x74: 0x09,
                 0x62: 0x08, 0x66: 0x0C,
                 0x28: 0x28, 0x29: 0x29, 0x5C: 0x5C}
            if n in m:
                out.append(m[n]); i += 2; continue
            if 0x30 <= n <= 0x37:                  # octal \ddd
                j = i + 1
                oct_digits = b""
                while j < len(s) and len(oct_digits) < 3 and 0x30 <= s[j] <= 0x37:
                    oct_digits += bytes([s[j]]); j += 1
                out.append(int(oct_digits, 8) & 0xFF); i = j; continue
            i += 2; continue
        out.append(c)
        i += 1
    return bytes(out).decode("latin-1")


def extract_text_streams(data: bytes):
    """返回所有可解压流解出来的 latin-1 文本拼接结果。"""
    chunks = []
    for m in re.finditer(rb"stream\r?\n", data):
        start = m.end()
        end = data.find(b"endstream", start)
        if end < 0:
            continue
        raw = data[start:end]
        for candidate in (raw, raw.rstrip(b"\r\n")):
            try:
                chunks.append(zlib.decompress(candidate).decode("latin-1"))
                break
            except Exception:
                continue
    return chunks


def strings_in(stream_text: str):
    """抽出 (..) 和 <..> 里的字符串，按出现顺序拼接（近似阅读顺序）。"""
    parts = []
    for m in re.finditer(r"\((?:\\.|[^\\()])*\)|<[0-9A-Fa-f\s]+>", stream_text):
        tok = m.group(0)
        if tok.startswith("("):
            parts.append(unescape_pdf(tok[1:-1].encode("latin-1")))
        else:
            hexs = re.sub(r"\s", "", tok[1:-1])
            if len(hexs) % 2:
                continue
            try:
                b = bytes.fromhex(hexs)
            except ValueError:
                continue
            # 常见两种编码：单字节；或 UTF-16BE
            if len(b) >= 2 and b[0] == 0xFE and b[1] == 0xFF:
                parts.append(b[2:].decode("utf-16-be", "replace"))
            elif b.count(0) > len(b) // 3:
                parts.append(b.decode("utf-16-be", "replace"))
            else:
                parts.append(b.decode("latin-1"))
    return parts


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    path, keys = sys.argv[1], sys.argv[2:]
    data = open(path, "rb").read()
    streams = extract_text_streams(data)
    print("[*] 解出 %d 个内容流" % len(streams))
    n = 0
    for si, st in enumerate(streams):
        parts = strings_in(st)
        text = "\t".join(parts)
        for k in keys:
            for m in re.finditer(re.escape(k), text, re.IGNORECASE):
                a = max(0, m.start() - 120)
                b = min(len(text), m.end() + 120)
                seg = text[a:b].replace("\t", " | ")
                print("[流%d] ...%s..." % (si, seg))
                n += 1
    print("[*] 命中 %d 处" % n)
    return 0


if __name__ == "__main__":
    sys.exit(main())