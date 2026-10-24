# -*- coding: utf-8 -*-
"""pdfdump.py —— 把 PDF 解出的文本按流写到 txt，便于用 grep/read 精查。"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pdfgrep import extract_text_streams, strings_in  # noqa: E402


def main():
    path = sys.argv[1]
    outdir = sys.argv[2] if len(sys.argv) > 2 else "."
    data = open(path, "rb").read()
    for i, st in enumerate(extract_text_streams(data)):
        parts = strings_in(st)
        txt = "\n".join(parts)
        fn = os.path.join(outdir, os.path.basename(path) + ".s%d.txt" % i)
        open(fn, "w", encoding="utf-8").write(txt)
        print("wrote", fn, len(txt), "chars")


if __name__ == "__main__":
    main()