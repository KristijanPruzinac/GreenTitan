#!/usr/bin/env python3
"""Writes wiring.svg from wiring.yml: the parts as photos, joined by drawn cables with a plug at each end.
Run through render.sh."""
import base64, html, os, textwrap, yaml

HERE = os.path.dirname(os.path.abspath(__file__))
BORDER = {"owned": "#4caf50", "buy": "#f0a800", "board": "#4a86e8"}
WIRE = ["#d32f2f", "#222222", "#1976d2", "#f9a825", "#388e3c", "#8e24aa", "#ef6c00", "#6d4c41"]
PITCH = 5

doc = yaml.safe_load(open(os.path.join(HERE, "wiring.yml"), encoding="utf-8"))
nodes = dict(doc["nodes"])
nodes["carrier"] = doc["carrier"]
out = []

def box(n):
    (x, y), (w, h) = n["at"], n["size"]
    return x - w / 2, y - h / 2, w, h

def esc(s):
    return html.escape(str(s))

# Which side of each part a cable leaves from, and its slot along that side.
ends = {k: {"l": [], "r": [], "t": [], "b": []} for k in nodes}
links = []
for a, b, wires, label, *rest in doc["links"]:
    (ax, ay), (bx, by) = nodes[a]["at"], nodes[b]["at"]
    aw, ah = nodes[a]["size"]; bw, bh = nodes[b]["size"]
    if abs(bx - ax) - (aw + bw) / 2 >= abs(by - ay) - (ah + bh) / 2:
        sa, sb = ("r", "l") if bx > ax else ("l", "r")
    else:
        sa, sb = ("b", "t") if by > ay else ("t", "b")
    link = {"a": a, "b": b, "wires": wires, "label": label, "sa": sa, "sb": sb, "ok": bool(rest and rest[0])}
    ends[a][sa].append((by if sa in "lr" else bx, link, "pa"))
    ends[b][sb].append((ay if sb in "lr" else ax, link, "pb"))
    links.append(link)
for k, sides in ends.items():
    x, y, w, h = box(nodes[k])
    for side, lst in sides.items():
        lst.sort(key=lambda e: e[0])
        for i, (_, link, key) in enumerate(lst):
            f = (i + 1) / (len(lst) + 1)
            link[key] = {"l": (x, y + h * f), "r": (x + w, y + h * f), "t": (x + w * f, y), "b": (x + w * f, y + h)}[side]

def cable(link):
    (x1, y1), (x2, y2) = link["pa"], link["pb"]
    wires = link["wires"]
    horiz = link["sa"] in "lr"
    if isinstance(wires, list):
        strands = [(w.split("=")[1] if "=" in w else WIRE[i % len(WIRE)], 2.6) for i, w in enumerate(wires)]
        named = not any("=" in w for w in wires)
    elif wires == "power":
        strands = [("#d32f2f", 5), ("#222222", 5)]
    elif wires == "ribbon":
        strands = [("#9e9e9e", 2.2)] * 9 + [("#d32f2f", 2.2)]
    elif wires == "coax":
        strands = [("#222222", 5)]
    else:
        strands = [("#555555", 7)]
    n = len(strands)
    pitch = PITCH + (3 if wires == "power" else 0) - (2 if wires == "ribbon" else 0)
    for i, (colour, width) in enumerate(strands):
        o = (i - (n - 1) / 2) * pitch
        if horiz:
            mx = (x1 + x2) / 2
            d = f"M{x1},{y1+o} C{mx},{y1+o} {mx},{y2+o} {x2},{y2+o}"
        else:
            my = (y1 + y2) / 2
            d = f"M{x1+o},{y1} C{x1+o},{my} {x2+o},{my} {x2+o},{y2}"
        out.append(f'<path d="{d}" fill="none" stroke="{colour}" stroke-width="{width}" stroke-linecap="round"/>')
    span = (n - 1) * pitch + 12
    for (px, py), side in ((link["pa"], link["sa"]), (link["pb"], link["sb"])):
        if side in "lr":
            px0 = px - 14 if side == "l" else px
            out.append(f'<rect x="{px0}" y="{py-span/2}" width="14" height="{span}" rx="3" fill="#f5f5f5" stroke="#444" stroke-width="1.5"/>')
        else:
            py0 = py - 14 if side == "t" else py
            out.append(f'<rect x="{px-span/2}" y="{py0}" width="{span}" height="14" rx="3" fill="#f5f5f5" stroke="#444" stroke-width="1.5"/>')
    text = link["label"]
    if isinstance(wires, list) and named:
        text += ": " + " ".join(wires)
    lx, ly = (x1 + x2) / 2, (y1 + y2) / 2
    w = 6.1 * len(text) + 14
    edge = "#1b7a2b" if link["ok"] else "#c62828"
    labels.append(f'<rect x="{lx-w/2}" y="{ly-10}" width="{w}" height="20" rx="10" fill="#fff" stroke="{edge}" stroke-width="1.5"/>'
                  f'<text x="{lx}" y="{ly+4}" text-anchor="middle" font-size="11.5" fill="#222">{esc(text)}</text>')

def part(k, n):
    x, y, w, h = box(n)
    colour = BORDER.get(n.get("status"), "#888")
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="#fff" stroke="{colour}" stroke-width="3"/>')
    if n.get("img"):
        data = base64.b64encode(open(os.path.join(HERE, n["img"]), "rb").read()).decode()
        out.append(f'<image x="{x+4}" y="{y+4}" width="{w-8}" height="{h-8}" preserveAspectRatio="xMidYMid meet" href="data:image/jpeg;base64,{data}"/>')
    else:
        out.append(f'<rect x="{x+6}" y="{y+6}" width="{w-12}" height="{h-12}" rx="5" fill="#eeeeee"/>')
    lines = textwrap.wrap(n["label"], max(8, int(w / 6.6) + (8 if n.get("img") else -1)))
    for i, line in enumerate(lines):
        if n.get("img"):
            ty, tw = y + h + 17 + i * 15, 7.2 * len(line) + 8
            out.append(f'<rect x="{x+w/2-tw/2}" y="{ty-12}" width="{tw}" height="16" fill="#fafafa" opacity="0.85"/>')
        else:
            ty = y + h / 2 + 4 + (i - (len(lines) - 1) / 2) * 15
        out.append(f'<text x="{x+w/2}" y="{ty}" text-anchor="middle" font-size="12.5" font-weight="bold" fill="#222">{esc(line)}</text>')
    if n.get("term"):
        text, ok = n["term"]
        colour = "#1b7a2b" if ok else "#c62828"
        tl = textwrap.wrap(("OK: " if ok else "? ") + text, 34)
        ty0 = (y + h + 17 + len(lines) * 15) if n.get("img") else (y + h + 15)
        tw = max(len(t) for t in tl) * 5.6 + 10
        out.append(f'<rect x="{x+w/2-tw/2}" y="{ty0-11}" width="{tw}" height="{len(tl)*13+4}" rx="4" fill="#fff" stroke="{colour}"/>')
        for i, t in enumerate(tl):
            out.append(f'<text x="{x+w/2}" y="{ty0+i*13}" text-anchor="middle" font-size="10.5" fill="{colour}">{esc(t)}</text>')

W, H = doc["canvas"]
cx, cy, cw, ch = box(doc["carrier"])
out.append(f'<rect width="{W}" height="{H}" fill="#fafafa"/>')
out.append(f'<rect x="20" y="6" width="{W-40}" height="40" rx="8" fill="#fdecea" stroke="#c62828" stroke-width="2"/>'
           f'<text x="{W/2}" y="32" text-anchor="middle" font-size="16" font-weight="bold" fill="#c62828">'
           f'DRAFT, NOT CONFIRMED. Green = checked against the bought listing or the maker document (part terminals, cable end to end). Red = not confirmed.</text>')
out.append(f'<rect x="{cx}" y="{cy}" width="{cw}" height="{ch}" rx="14" fill="#e3edfb" stroke="#4a86e8" stroke-width="3"/>'
           f'<text x="{cx+cw/2}" y="{cy+28}" text-anchor="middle" font-size="17" font-weight="bold" fill="#1c4587">{esc(doc["carrier"]["label"])}</text>')
labels = []
for link in links:
    cable(link)
for k, n in doc["nodes"].items():
    part(k, n)
out += labels
lx = 30
for name, key in (("owned", "owned"), ("to buy", "buy"), ("own board", "board")):
    out.append(f'<rect x="{lx}" y="{H-34}" width="18" height="18" rx="4" fill="#fff" stroke="{BORDER[key]}" stroke-width="3"/>'
               f'<text x="{lx+26}" y="{H-20}" font-size="13" fill="#222">{name}</text>')
    lx += 110
open(os.path.join(HERE, "wiring.svg"), "w", encoding="utf-8").write(
    f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" font-family="Arial, sans-serif">' + "".join(out) + "</svg>")
