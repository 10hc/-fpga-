#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scan_multi_driver.py —— 粗查 Verilog「同一信号被多处驱动」

为什么需要它：综合器只会一次报一个
    ERROR: net 'xxx[n]' is constantly driven from multiple places in ...
而 ModelSim 完全不报这个错（仿真照跑）。本脚本一次扫全部文件，提前找出所有同类问题。
本项目已经被这个坑坑了两次（wav_audio 的 rdp、video_player 的 pace_cnt）。

判定规则（按 module 分段，避免跨模块误报）：
  1) 同一信号在 2 个及以上 always 块里被赋值
  2) 同一信号既有 always 赋值又有 assign 连续赋值
用法：
    python scan_multi_driver.py <目录或文件> [...]
返回码：0 = 干净；1 = 发现问题
"""
import re
import sys
from pathlib import Path

# 行内任意位置的赋值（不要求行首），排除 == / >= / <= 比较
ASSIGN_ANY_RE = re.compile(r'([A-Za-z_]\w*)\s*(?:\[[^\[\]]*\])?\s*(<=|=)(?!=|>)')
ASSIGN_CONT_RE = re.compile(r'\bassign\s+(?:\[[^\[\]]*\]\s*)?([A-Za-z_]\w*)')


def strip_comment(line):
    i = line.find('//')
    return line[:i] if i >= 0 else line


def in_parens(text):
    """判断位置 text 是否处在未闭合的括号内（用于排除 if(...)/for(...) 里的比较）"""
    return text.count('(') > text.count(')')


def split_modules(text):
    """按 module/endmodule 分段，避免不同模块同名信号误报"""
    lines = text.split('\n')
    mods, cur = [], None
    for i, ln in enumerate(lines):
        m = re.match(r'\s*module\s+(\w+)', ln)
        if m and cur is None:
            cur = [m.group(1), i, []]
        if cur is not None:
            cur[2].append(ln)
            if re.match(r'\s*endmodule\b', ln):
                mods.append(tuple(cur))
                cur = None
    if cur is not None:
        mods.append(tuple(cur))
    return mods


def find_always_blocks(lines):
    """用 begin/end 配对粗切 always 块（单语句 always 以分号结束）"""
    blocks, i, n = [], 0, len(lines)
    while i < n:
        if re.search(r'\balways\b', strip_comment(lines[i])):
            depth, seen_begin, j = 0, False, i
            while j < n:
                ln = strip_comment(lines[j])
                if re.search(r'\bbegin\b', ln):
                    seen_begin = True
                depth += len(re.findall(r'\bbegin\b', ln))
                depth -= len(re.findall(r'\bend\b', ln))
                if seen_begin and depth <= 0:
                    break
                if not seen_begin and ';' in ln:
                    break
                j += 1
            blocks.append((i, min(j, n - 1)))
            i = j + 1
        else:
            i += 1
    return blocks


def scan_module(name, lines):
    problems = []
    owners = {}                                   # signal -> set(驱动源)
    for bi, (s, e) in enumerate(find_always_blocks(lines)):
        for k in range(s, e + 1):
            ln = strip_comment(lines[k])
            if re.match(r'\s*assign\b', ln):
                continue                          # assign 单独算一类驱动源
            for m in ASSIGN_ANY_RE.finditer(ln):
                if in_parens(ln[:m.start()]):
                    continue                      # if(...)/for(...) 里的比较，跳过
                owners.setdefault(m.group(1), set()).add('always#%d' % bi)
    for ln in lines:
        ln = strip_comment(ln)
        for m in ASSIGN_CONT_RE.finditer(ln):
            owners.setdefault(m.group(1), set()).add('assign')
    for sig, srcs in sorted(owners.items()):
        al = sorted(x for x in srcs if x.startswith('always'))
        if len(al) >= 2 or (al and 'assign' in srcs):
            problems.append((name, sig, sorted(srcs)))
    return problems


def scan_file(path):
    try:
        text = Path(path).read_text(encoding='utf-8', errors='replace')
    except Exception as e:
        print('  ! 读取失败 %s: %s' % (path, e))
        return [], 0
    mods, out = split_modules(text), []
    for name, _base, lines in mods:
        out.extend(scan_module(name, lines))
    return out, len(mods)


def walk(targets, excludes=()):
    files = []
    for t in targets:
        p = Path(t)
        if p.is_dir():
            files.extend(sorted(p.rglob('*.v')))
        elif p.suffix.lower() == '.v':
            files.append(p)
    if excludes:
        files = [f for f in files
                 if not any(x and x in str(f) for x in excludes)]
    return files


def main():
    args = sys.argv[1:]
    excludes, targets = [], []
    i = 0
    while i < len(args):
        if args[i] == '--exclude' and i + 1 < len(args):
            excludes.append(args[i + 1]); i += 2; continue
        targets.append(args[i]); i += 1
    if not targets:
        print('用法: python scan_multi_driver.py [--exclude 子串] <目录或文件> [...]')
        return 1
    files = walk(targets, excludes)
    nprob = nmods = 0
    for f in files:
        probs, nm = scan_file(f)
        nmods += nm
        if probs:
            print('[X] %s' % f)
            for name, sig, srcs in probs:
                nprob += 1
                print('      module %s: 信号 %s 被多处驱动 -> %s' % (name, sig, ', '.join(srcs)))
    print('-' * 72)
    print('扫描 %d 个文件 / %d 个 module，发现 %d 处多驱动' % (len(files), nmods, nprob))
    return 1 if nprob else 0


if __name__ == '__main__':
    sys.exit(main())