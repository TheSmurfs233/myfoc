#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Documentation consistency check for this repository.

Run from anywhere:

    python test/doc_check.py

Hard failures (exit code 1) catch things that are objectively wrong:

  1. mojibake          U+FFFD replacement characters in any document
  2. non-ASCII source  User/**/*.c|h must stay ASCII (Keil's encoding is GBK)
  3. broken tables     a markdown table row split across two lines
  4. help drift        app_cmd.c help lines != test/accept/s1_accept.py
  5. wave drift        WAVE_CH in board_cfg.h != accept script
  6. version drift     FW_BUILD_ID != the "固件 #N" shown in docs/01 and README

Warnings (exit code stays 0) point at text that is probably stale:

  7. stale markers     TBD / 待测 / 待提供 / 未开始 / 下一步：S<n> ...
                       in the current-state documents. Historical entries in
                       00_开发日志.md and the TBD list in 18_待确认清单.md are
                       skipped on purpose.

Why this exists: the S8 review found docs/01 sections 3-6 still describing the
S3 state, plus resolved TBDs in docs 10/12/16/17. A script is cheaper than
remembering to grep.
"""

import glob
import os
import re
import sys

# Windows consoles default to GBK; documents contain Chinese and emoji.
try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DOCS = sorted(glob.glob(os.path.join(ROOT, 'docs', '*.md')))
SRC = sorted(glob.glob(os.path.join(ROOT, 'User', '**', '*.c'), recursive=True) +
             glob.glob(os.path.join(ROOT, 'User', '**', '*.h'), recursive=True))
ACCEPT = os.path.join(ROOT, 'test', 'accept', 's1_accept.py')
BOARD_CFG = os.path.join(ROOT, 'User', 'Cfg', 'board_cfg.h')
APP_CMD = os.path.join(ROOT, 'User', 'App', 'app_cmd.c')
PROGRESS = os.path.join(ROOT, 'docs', '01_项目进度.md')
README = os.path.join(ROOT, 'docs', 'README.md')

# documents that describe the *current* state; 00 is history, 18 is a TBD list
STALE_SCAN_SKIP = ('00_开发日志.md', '18_待确认清单.md')

STALE_PATTERNS = [
    (r'\bTBD\b', 'TBD'),
    (r'待测', '待测'),
    (r'待提供', '待提供'),
    (r'待定', '待定'),
    (r'未开始', '未开始'),
    (r'尚未实施', '尚未实施'),
    (r'下一步[：:]\s*S\d', '下一步：S<n>'),
    (r'⬜', 'todo box'),
]

errors = []
warnings = []


def err(msg):
    errors.append(msg)


def warn(msg):
    warnings.append(msg)


def read(path):
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        return f.read()


def rel(path):
    return os.path.relpath(path, ROOT).replace('\\', '/')


# ---------------------------------------------------------------- 1. mojibake
for p in DOCS + [os.path.join(ROOT, 'test', 'README.md')]:
    if not os.path.exists(p):
        continue
    for n, line in enumerate(read(p).splitlines(), 1):
        if '\ufffd' in line:
            err('%s:%d mojibake (U+FFFD)' % (rel(p), n))

# ------------------------------------------------------------ 2. ascii source
for p in SRC:
    data = open(p, 'rb').read()
    if any(b > 127 for b in data):
        err('%s contains non-ASCII bytes' % rel(p))

# ---------------------------------------------------------- 3. broken tables
for p in DOCS:
    in_fence = False
    for n, line in enumerate(read(p).splitlines(), 1):
        t = line.strip()
        if t.startswith('```'):
            in_fence = not in_fence
            continue
        if in_fence or not t.startswith('|'):
            continue
        if len(t) > 2 and not t.endswith('|'):
            err('%s:%d table row not closed with |' % (rel(p), n))

# ------------------------------------------- 3b. table column consistency
CELL_SPLIT = re.compile(r'(?<!\\)\|')


def table_cells(line):
    parts = CELL_SPLIT.split(line)
    # a well formed row starts and ends with '|' -> first/last parts empty
    return len(parts) - 2


for p in DOCS:
    in_fence = False
    header = None
    for n, line in enumerate(read(p).splitlines(), 1):
        t = line.strip()
        if t.startswith('```'):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        if not t.startswith('|'):
            header = None
            continue
        cols = table_cells(t)
        if header is None:
            header = cols
        elif cols != header:
            err('%s:%d table has %d cells, header has %d (unescaped | ?)'
                % (rel(p), n, cols, header))

# ------------------------------------------------------- 4/5. accept script
help_count = None
wave_ch = None
if os.path.exists(APP_CMD):
    src = read(APP_CMD)
    i = src.find('static void print_help')
    j = src.find('\n}', i)
    if i > 0 and j > i:
        help_count = sum(1 for l in src[i:j].splitlines() if 'log_printf' in l)
if os.path.exists(BOARD_CFG):
    m = re.search(r'#define\s+WAVE_CH\s+(\d+)u', read(BOARD_CFG))
    if m:
        wave_ch = int(m.group(1))
if os.path.exists(ACCEPT):
    acc = read(ACCEPT)
    m = re.search(r'help:\s*(\d+)\s*lines', acc)
    if m and help_count is not None and int(m.group(1)) != help_count:
        err('help lines: app_cmd.c has %d, accept expects %s' % (help_count, m.group(1)))
    m = re.search(r'!=\s*(\d+)\]', acc)
    if m and wave_ch is not None and int(m.group(1)) != wave_ch:
        err('wave channels: board_cfg.h has %d, accept expects %s' % (wave_ch, m.group(1)))

# ------------------------------------------------------------- 6. build id
build_id = None
if os.path.exists(BOARD_CFG):
    m = re.search(r'#define\s+FW_BUILD_ID\s+(\d+)u', read(BOARD_CFG))
    if m:
        build_id = int(m.group(1))
if build_id is not None:
    # Historical rows (S1..S7) record the build used at that stage, so only the
    # newest stage row and the README baseline are checked for drift.
    if os.path.exists(PROGRESS):
        # Compare against the HIGHEST stage row (the project's current stage),
        # not a hard-coded one, so adding S9/S10/... needs no code change.
        best = None
        for n, line in enumerate(read(PROGRESS).splitlines(), 1):
            m = re.search(r'\| \*\*S(\d+)(?:\.\d+)?\*\* \|', line)
            if not m:
                continue
            stage = int(m.group(1))
            if best is None or stage > best[0]:
                best = (stage, n, line)
        if best is not None:
            n, line = best[1], best[2]
            m = re.search(r'固件\s*#(\d+)', line)
            if m and int(m.group(1)) != build_id:
                err('%s:%d S%d row says 固件 #%s, board_cfg.h has #%d'
                    % (rel(PROGRESS), n, best[0], m.group(1), build_id))
    if os.path.exists(README):
        for n, line in enumerate(read(README).splitlines(), 1):
            if '基线' not in line:
                continue
            m = re.search(r'#(\d+)', line)
            if m and int(m.group(1)) != build_id:
                err('%s:%d baseline says #%s, board_cfg.h has #%d'
                    % (rel(README), n, m.group(1), build_id))

# --------------------------------------------------------- 7. stale markers
for p in DOCS:
    if os.path.basename(p) in STALE_SCAN_SKIP:
        continue
    in_fence = False
    for n, line in enumerate(read(p).splitlines(), 1):
        t = line.strip()
        if t.startswith('```'):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        # A heading such as "## 7. 已知限制与 TBD" is a section title, and the
        # progress figure "图例：完成/进行中/未开始" is a legend: neither is a
        # stale statement.
        if t.startswith('#'):
            continue
        if '图例' in line:
            continue
        for pat, label in STALE_PATTERNS:
            if re.search(pat, line):
                warn('%s:%d [%s] %s' % (rel(p), n, label, t[:70]))
                break

# ----------------------------------------------------------------- report
print('doc_check: %d documents, %d source files' % (len(DOCS), len(SRC)))
if help_count is not None:
    print('doc_check: help lines %d, wave channels %s, build id %s'
          % (help_count, wave_ch, build_id))

if warnings:
    print()
    print('--- warnings (stale text, review manually) ---')
    for w in warnings:
        print('  ' + w)

if errors:
    print()
    print('--- errors ---')
    for e in errors:
        print('  ' + e)
    print()
    print('RESULT: FAIL (%d errors, %d warnings)' % (len(errors), len(warnings)))
    sys.exit(1)

print()
print('RESULT: PASS (%d warnings)' % len(warnings))
sys.exit(0)
