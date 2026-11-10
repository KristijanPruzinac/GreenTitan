#!/usr/bin/env python3
"""Writes overview.svg: every electrical part as a photo card, no pins and no wiring.
The groups come from the `overview:` list in common.yml. Run through render.sh."""
import base64, html, os, textwrap, yaml

HERE = os.path.dirname(os.path.abspath(__file__))
STATUS = {"#bfe8bf": "owned", "#ffe08a": "to buy", "#bcd6ff": "own board"}
CW, CH, GAP, COLS, PAD = 190, 190, 22, 5, 30

def card(x, y, part, count, name):
    colour = part.get("bgcolor_title", "#dddddd")
    out = [f'<rect x="{x}" y="{y}" width="{CW}" height="{CH}" rx="10" fill="#fff" stroke="{colour}" stroke-width="3"/>']
    img = part.get("image", {}).get("src")
    if img:
        data = base64.b64encode(open(os.path.join(HERE, img), "rb").read()).decode()
        out.append(f'<image x="{x+35}" y="{y+10}" width="120" height="100" preserveAspectRatio="xMidYMid meet" href="data:image/jpeg;base64,{data}"/>')
    else:
        out.append(f'<rect x="{x+45}" y="{y+25}" width="100" height="70" rx="6" fill="#f2f2f2"/>'
                   f'<text x="{x+CW/2}" y="{y+65}" text-anchor="middle" font-size="11" fill="#999">no photo</text>')
    label = name or part["type"].split(",")[0]
    if count > 1:
        label = f"{count}x {label}"
    for i, line in enumerate(textwrap.wrap(label, 27)[:3]):
        out.append(f'<text x="{x+CW/2}" y="{y+128+i*15}" text-anchor="middle" font-size="12.5" fill="#222">{html.escape(line)}</text>')
    out.append(f'<rect x="{x+CW/2-32}" y="{y+CH-24}" width="64" height="16" rx="8" fill="{colour}"/>'
               f'<text x="{x+CW/2}" y="{y+CH-12}" text-anchor="middle" font-size="10.5" fill="#333">{STATUS.get(colour, "")}</text>')
    return out

doc = yaml.safe_load(open(os.path.join(HERE, "common.yml"), encoding="utf-8"))
body, y = [], PAD
for group in doc["overview"]:
    body.append(f'<text x="{PAD}" y="{y+18}" font-size="20" font-weight="bold" fill="#222">{html.escape(group["group"])}</text>')
    y += 34
    for i, item in enumerate(group["parts"]):
        part = item.get("part", item)
        if i and i % COLS == 0:
            y += CH + GAP
        body += card(PAD + (i % COLS) * (CW + GAP), y, part, item.get("count", 1), item.get("name"))
    y += CH + GAP + 14
width = 2 * PAD + COLS * CW + (COLS - 1) * GAP
svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {y}" width="{width}" font-family="Arial, sans-serif">'
       f'<rect width="100%" height="100%" fill="#fafafa"/>' + "".join(body) + "</svg>")
open(os.path.join(HERE, "overview.svg"), "w", encoding="utf-8").write(svg)
