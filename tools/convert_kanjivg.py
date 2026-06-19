#!/usr/bin/env python3
"""
KanjiVG → LumiGlyph グリフデータ変換ツール

【データソース】
  https://github.com/KanjiVG/kanjivg/releases から
  kanjivg-YYYYMMDD.zip をダウンロードし展開する。
  kanji/ フォルダ内に 04e00.svg のような個別 SVG ファイルが入っている。

【使い方】
  python tools/convert_kanjivg.py <kanji_dir> <characters> [出力ファイル]

【例】
  python tools/convert_kanjivg.py kanjivg/kanji "あいうえおアイウエオ0123456789" src/glyphs_data.h

【出力】
  src/glyphs_data.h  ← PlatformIO ビルドに自動的に取り込まれる

【座標系】
  KanjiVG: 109×109 px、y 軸下向き
  グリフ単位: int8_t、中心 (18, 75) mm、1 unit = 0.5 mm、y 軸上向き
  文字サイズ: 約 ±27 unit = ±13.5 mm（26×26 mm の描画エリア）
"""

import re
import xml.etree.ElementTree as ET
import sys
import math
from pathlib import Path

# ─── 座標変換パラメータ ───────────────────────────────
KVG_SIZE   = 109.0
KVG_CENTER = KVG_SIZE / 2.0          # 54.5
GLYPH_SCALE_MM = 0.5                 # 1 unit = 0.5 mm（firmware と一致させる）
DRAW_WIDTH_MM  = 28.0                # 文字幅 (mm)
UNIT_PER_KVG   = DRAW_WIDTH_MM / KVG_SIZE / GLYPH_SCALE_MM   # ≈ 0.514
GLYPH_PENUP    = -128                # ペンアップマーカー（int8_t 最小値）

# ─── SVG パスパーサー ─────────────────────────────────

def _tokenize(d: str):
    """SVG path d 属性をトークン列に分解する。"""
    pat = re.compile(
        r'([MmLlHhVvCcSsQqTtZz])'
        r'|([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)'
    )
    return [(m.group(1), None if m.group(1) else float(m.group(2)))
            for m in pat.finditer(d)]


def _parse_path_absolute(d: str):
    """SVG path を絶対座標コマンド列 [(cmd, [args]), ...] に変換する。"""
    tokens = _tokenize(d)

    # コマンドと数値引数に分離
    raw = []
    cmd = None
    nums = []
    for letter, num in tokens:
        if letter is not None:
            if cmd is not None:
                raw.append((cmd, nums))
            cmd, nums = letter, []
        else:
            nums.append(num)
    if cmd is not None:
        raw.append((cmd, nums))

    # 絶対座標に変換 + 暗黙の繰り返しを展開
    result = []
    cx, cy = 0.0, 0.0   # カレント位置
    sx, sy = 0.0, 0.0   # パス開始位置 (Z 用)
    prev_ctrl = None     # 直前の制御点 (S, T 用)

    n_args = {'M':2,'L':2,'H':1,'V':1,'C':6,'S':4,'Q':4,'T':2,'A':7,'Z':0}

    for cmd, args in raw:
        rel = cmd.islower()
        uc  = cmd.upper()

        if uc == 'Z':
            result.append(('Z', []))
            cx, cy = sx, sy
            prev_ctrl = None
            continue

        n = n_args.get(uc, 0)
        if n == 0:
            continue

        first_m = True
        i = 0
        while i + n <= len(args):
            chunk = list(args[i:i+n])
            i += n

            # M の 2 点目以降は L として扱う
            effective = uc if (uc != 'M' or first_m) else 'L'
            first_m = False

            # 相対 → 絶対
            if rel:
                if effective in ('M','L','T'):
                    chunk = [chunk[0]+cx, chunk[1]+cy]
                elif effective == 'H':
                    chunk = [chunk[0]+cx]
                elif effective == 'V':
                    chunk = [chunk[0]+cy]
                elif effective == 'C':
                    chunk = [chunk[0]+cx,chunk[1]+cy, chunk[2]+cx,chunk[3]+cy,
                             chunk[4]+cx,chunk[5]+cy]
                elif effective == 'S':
                    chunk = [chunk[0]+cx,chunk[1]+cy, chunk[2]+cx,chunk[3]+cy]
                elif effective == 'Q':
                    chunk = [chunk[0]+cx,chunk[1]+cy, chunk[2]+cx,chunk[3]+cy]

            # H / V → L
            if effective == 'H':
                effective, chunk = 'L', [chunk[0], cy]
            elif effective == 'V':
                effective, chunk = 'L', [cx, chunk[0]]

            # S → C (前の制御点を反射)
            if effective == 'S':
                if prev_ctrl and prev_ctrl[0] == 'C':
                    rx, ry = 2*cx - prev_ctrl[1], 2*cy - prev_ctrl[2]
                else:
                    rx, ry = cx, cy
                effective, chunk = 'C', [rx, ry] + chunk

            # T → Q
            if effective == 'T':
                if prev_ctrl and prev_ctrl[0] == 'Q':
                    rx, ry = 2*cx - prev_ctrl[1], 2*cy - prev_ctrl[2]
                else:
                    rx, ry = cx, cy
                effective, chunk = 'Q', [rx, ry] + chunk

            result.append((effective, chunk))

            # カレント位置を更新
            if effective == 'M':
                cx, cy = chunk[0], chunk[1]; sx, sy = cx, cy; prev_ctrl = None
            elif effective == 'L':
                cx, cy = chunk[0], chunk[1]; prev_ctrl = None
            elif effective == 'C':
                prev_ctrl = ('C', chunk[2], chunk[3])
                cx, cy = chunk[4], chunk[5]
            elif effective == 'Q':
                prev_ctrl = ('Q', chunk[0], chunk[1])
                cx, cy = chunk[2], chunk[3]

    return result


# ─── ベジェ曲線サンプリング ────────────────────────────

def _cubic(p0, p1, p2, p3, t):
    mt = 1 - t
    return (mt**3*p0[0] + 3*mt**2*t*p1[0] + 3*mt*t**2*p2[0] + t**3*p3[0],
            mt**3*p0[1] + 3*mt**2*t*p1[1] + 3*mt*t**2*p2[1] + t**3*p3[1])

def _quadratic(p0, p1, p2, t):
    mt = 1 - t
    return (mt**2*p0[0] + 2*mt*t*p1[0] + t**2*p2[0],
            mt**2*p0[1] + 2*mt*t*p1[1] + t**2*p2[1])

def _sample_cubic(p0, p1, p2, p3, n=14):
    return [_cubic(p0,p1,p2,p3, i/n) for i in range(1, n+1)]

def _sample_quad(p0, p1, p2, n=10):
    return [_quadratic(p0,p1,p2, i/n) for i in range(1, n+1)]


# ─── Ramer-Douglas-Peucker 簡略化 ─────────────────────

def _perp_dist(pt, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    L = math.hypot(dx, dy)
    if L < 1e-10:
        return math.hypot(pt[0]-a[0], pt[1]-a[1])
    return abs(dx*(a[1]-pt[1]) - dy*(a[0]-pt[0])) / L

def rdp(pts, eps):
    if len(pts) < 3:
        return pts
    dmax, idx = 0, 0
    for i in range(1, len(pts)-1):
        d = _perp_dist(pts[i], pts[0], pts[-1])
        if d > dmax:
            dmax, idx = d, i
    if dmax > eps:
        return rdp(pts[:idx+1], eps)[:-1] + rdp(pts[idx:], eps)
    return [pts[0], pts[-1]]


# ─── パス → ストローク変換 ────────────────────────────

def path_to_strokes(d: str, bezier_n=14, rdp_eps=1.2):
    """SVG path d → ストロークリスト（KanjiVG 座標）"""
    cmds = _parse_path_absolute(d)
    strokes, cur = [], []
    cx, cy = 0.0, 0.0

    for cmd, args in cmds:
        if cmd == 'M':
            if cur:
                strokes.append(rdp(cur, rdp_eps) if rdp_eps > 0 else cur)
            cx, cy = args[0], args[1]
            cur = [(cx, cy)]
        elif cmd == 'L':
            cur.append((args[0], args[1]))
            cx, cy = args[0], args[1]
        elif cmd == 'C':
            p0=(cx,cy); p1=(args[0],args[1]); p2=(args[2],args[3]); p3=(args[4],args[5])
            cur.extend(_sample_cubic(p0,p1,p2,p3, bezier_n))
            cx, cy = args[4], args[5]
        elif cmd == 'Q':
            p0=(cx,cy); p1=(args[0],args[1]); p2=(args[2],args[3])
            cur.extend(_sample_quad(p0,p1,p2, bezier_n))
            cx, cy = args[2], args[3]
        elif cmd == 'Z':
            if cur:
                cur.append(cur[0])   # パスを閉じる

    if cur:
        strokes.append(rdp(cur, rdp_eps) if rdp_eps > 0 else cur)
    return strokes


# ─── 座標変換 ──────────────────────────────────────────

def kvg_to_unit(kx, ky):
    """KanjiVG 座標 → グリフ単位 (int8_t)"""
    ux =  (kx - KVG_CENTER) * UNIT_PER_KVG
    uy = -(ky - KVG_CENTER) * UNIT_PER_KVG   # y 反転
    return max(-127, min(127, round(ux))), max(-127, min(127, round(uy)))


# ─── グリフデータ生成 ─────────────────────────────────

def strokes_to_data(strokes):
    """ストロークリスト → int8_t データ列"""
    data = []
    for stroke in strokes:
        if not stroke:
            continue
        # 最初の点はペンアップ移動
        ux, uy = kvg_to_unit(*stroke[0])
        data += [GLYPH_PENUP, ux, uy]
        # 残りの点はペンダウン
        for pt in stroke[1:]:
            ux, uy = kvg_to_unit(*pt)
            data += [ux, uy]
    return data


def svg_file_to_data(svg_path: Path):
    """SVG ファイル → グリフデータ"""
    tree = ET.parse(svg_path)
    root = tree.getroot()
    ns = root.tag.split('}')[0].lstrip('{') if '}' in root.tag else ''
    tag = lambda t: f'{{{ns}}}{t}' if ns else t

    all_strokes = []
    for path_elem in root.iter(tag('path')):
        d = path_elem.get('d', '')
        if d:
            all_strokes.extend(path_to_strokes(d))
    return strokes_to_data(all_strokes)


# ─── ヘッダ生成 ────────────────────────────────────────

def to_c_array(data, name):
    vals = ', '.join(str(v) for v in data)
    return f'static const int8_t PROGMEM {name}[] = {{{vals}}};\n'


def generate_header(char_data: dict, output_path: Path):
    """
    char_data: { codepoint(int): [int8_t, ...] }
    """
    lines = [
        '// Auto-generated by tools/convert_kanjivg.py — DO NOT EDIT',
        '// Re-generate: python tools/convert_kanjivg.py <kanji_dir> "<chars>" src/glyphs_data.h',
        '#pragma once',
        '#include "glyphs.h"',
        '',
    ]

    var_map = {}
    for cp, data in sorted(char_data.items()):
        vname = f'gd_{cp:04X}'
        var_map[cp] = vname
        lines.append(f'// U+{cp:04X}  {chr(cp)}')
        lines.append(to_c_array(data, vname))

    lines += [
        f'#define GLYPH_TABLE_SIZE {len(char_data)}',
        'static const GlyphDef PROGMEM glyph_table[] = {',
    ]
    for cp in sorted(char_data):
        char_str = chr(cp).encode('unicode_escape').decode('ascii')
        lines.append(
            f'    {{0x{cp:04X}u, {var_map[cp]}, {len(char_data[cp])}, '
            f'"{char_str}"}},  // {chr(cp)}'
        )
    lines += ['};', '']

    output_path.write_text('\n'.join(lines), encoding='utf-8')
    print(f'[OK] {len(char_data)} glyphs → {output_path}')


# ─── メイン ────────────────────────────────────────────

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    kanji_dir = Path(sys.argv[1])
    chars     = sys.argv[2]
    out_path  = Path(sys.argv[3]) if len(sys.argv) > 3 else Path('src/glyphs_data.h')

    if not kanji_dir.is_dir():
        print(f'エラー: {kanji_dir} はディレクトリではありません')
        sys.exit(1)

    char_data = {}
    missing   = []

    for ch in chars:
        cp = ord(ch)
        # KanjiVG のファイル名は 5 桁 hex lowercase (例: 03042.svg → あ)
        # ただし 5 桁未満は 0 埋め
        svg = kanji_dir / f'{cp:05x}.svg'
        if not svg.exists():
            missing.append(ch)
            continue
        try:
            data = svg_file_to_data(svg)
            if data:
                char_data[cp] = data
                print(f'  {ch} (U+{cp:04X}): {len(data)} bytes')
            else:
                print(f'  {ch} (U+{cp:04X}): データなし')
        except Exception as e:
            print(f'  {ch} (U+{cp:04X}): エラー — {e}')

    if missing:
        print(f'\n見つからなかった文字: {"".join(missing)}')
        print('ヒント: KanjiVG は漢字・ひらがな・カタカナを収録。'
              'ASCII 数字などは kanji/ フォルダに含まれない場合があります。')

    if char_data:
        generate_header(char_data, out_path)
    else:
        print('変換できた文字がありません。kanji_dir を確認してください。')
        sys.exit(1)


if __name__ == '__main__':
    main()
