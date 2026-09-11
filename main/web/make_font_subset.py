#!/usr/bin/env python3
"""Rebuild the embedded LingHei (造字工坊凌黑体) subset used by main/web/index.html.

为什么需要子集：凌黑体完整字体 1.4MB，固件塞不下。而界面里只有**标题类元素**
用到了它（h1/h2/页签/滑块选项），这些位置的汉字总共几十个，子集化后只有几 KB。

收集范围必须和 CSS 里的 --font-heading 用到的元素保持一致，否则新增的标题文字
会回落到系统中文字体（看起来就是"字体不对"）。所以改完 UI 文案后重跑本脚本即可：

    python main/web/make_font_subset.py

依赖 fonttools（pip install fonttools brotli）。字体文件默认取 Windows 本机的
%LOCALAPPDATA%\\Microsoft\\Windows\\Fonts\\MFLingHei_Noncommercial-Regular.otf。
"""
import base64
import os
import re
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
HTML = os.path.join(HERE, 'index.html')

DEFAULT_FONT = os.path.expandvars(
    r'%LOCALAPPDATA%\Microsoft\Windows\Fonts\MFLingHei_Noncommercial-Regular.otf')

# 与 --font-heading 的适用范围一致：卡片标题、页签、二选一滑块选项
PATTERNS = [
    r'<h2>(.*?)</h2>',                       # 卡片标题
    r"\[[\s']*tab-[a-z]+[\s']*,\s*'([^']+)'\]",  # 页签（TABS 数组）
    r'class="seg-btn">([^<]*)<',             # 滑块选项
]


def collect_chars(html: str) -> list:
    text = []
    for p in PATTERNS:
        text += re.findall(p, html, re.S)
    cjk = sorted({c for t in text for c in t if '一' <= c <= '鿿'})
    return cjk


def main() -> int:
    font = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_FONT
    if not os.path.isfile(font):
        print('找不到字体文件: %s' % font, file=sys.stderr)
        return 1

    from fontTools import subset

    with open(HTML, encoding='utf-8') as f:
        html = f.read()

    chars = collect_chars(html)
    if not chars:
        print('没收集到汉字，检查 PATTERNS 是否与页面结构脱节了', file=sys.stderr)
        return 1
    print('需要 %d 个汉字: %s' % (len(chars), ''.join(chars)))

    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'subset.woff2')
        subset.main([font, '--text=' + ''.join(chars), '--output-file=' + out,
                     '--flavor=woff2', '--layout-features=', '--no-hinting',
                     '--desubroutinize', '--drop-tables+=DSIG'])
        with open(out, 'rb') as f:
            data = f.read()

    b64 = base64.b64encode(data).decode('ascii')
    old = re.search(r'@font-face\s*\{[^}]*"CamLink LingHei"[^}]*\}', html, re.S)
    if not old:
        print('index.html 里找不到 "CamLink LingHei" 的 @font-face', file=sys.stderr)
        return 1

    new = ('@font-face {\n'
           '  font-family: "CamLink LingHei";\n'
           '  src: url(data:font/woff2;base64,%s) format("woff2");\n'
           '  font-weight: 400; font-style: normal; font-display: swap;\n'
           '}' % b64)

    html = html[:old.start()] + new + html[old.end():]
    with open(HTML, 'w', encoding='utf-8', newline='') as f:
        f.write(html)

    print('子集 %d 字节（原字体 %d），已写回 index.html'
          % (len(data), os.path.getsize(font)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
