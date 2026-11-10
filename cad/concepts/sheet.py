"""Contact sheets from the concept renders (plain Python + Pillow, not Blender).
python cad/concepts/sheet.py"""
import os
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, "renders")
CONCEPTS = [("concept_a", "A  ·  EXO", "The frame is the look: a sealed pod inside a 2020 cage"),
            ("concept_b", "B  ·  SHELL", "One body over a hidden frame, rails surface in the deck"),
            ("concept_c", "C  ·  BELT", "The frame is a waistline: one ring of 2020 between tub and shell")]
EXO = ("exo", "EXO", "Concept A developed: real parts, separate bays, the mower as a module (below: low view, lid off)")


def font(size):
    for f in ("bahnschrift.ttf", "segoeuib.ttf", "arialbd.ttf"):
        try:
            return ImageFont.truetype(os.path.join("C:/Windows/Fonts", f), size)
        except OSError:
            pass
    return ImageFont.load_default()


def label(img, title, sub):
    d = ImageDraw.Draw(img)
    d.text((36, 28), title, font=font(46), fill=(28, 30, 32))
    d.text((38, 84), sub, font=font(22), fill=(70, 74, 78))


def sheet(tag, title, sub, below=()):
    """Hero with rear/side/top beside it; `below` adds a row of larger views under it."""
    hero = Image.open(os.path.join(R, f"{tag}_hero.png")).convert("RGB")
    w, h = hero.size
    sw, sh = w // 3, h // 3
    bw = (w + sw) // len(below) if below else 0
    bh = bw * h // w
    out = Image.new("RGB", (w + sw, h + bh), (255, 255, 255))
    out.paste(hero, (0, 0))
    for i, v in enumerate(("rear", "side", "top")):
        im = Image.open(os.path.join(R, f"{tag}_{v}.png")).convert("RGB").resize((sw, sh), Image.LANCZOS)
        out.paste(im, (w, i * sh))
    for i, v in enumerate(below):
        im = Image.open(os.path.join(R, f"{tag}_{v}.png")).convert("RGB").resize((bw, bh), Image.LANCZOS)
        out.paste(im, (i * bw, h))
    label(out, title, sub)
    p = os.path.join(R, f"{tag}_sheet.png")
    out.save(p)
    return p


def overview():
    ims = []
    for tag, title, sub in CONCEPTS:
        im = Image.open(os.path.join(R, f"{tag}_hero.png")).convert("RGB")
        im = im.resize((im.width * 2 // 3, im.height * 2 // 3), Image.LANCZOS)
        label(im, title, "")
        ims.append(im)
    out = Image.new("RGB", (sum(i.width for i in ims), ims[0].height), (255, 255, 255))
    x = 0
    for im in ims:
        out.paste(im, (x, 0))
        x += im.width
    p = os.path.join(R, "overview.png")
    out.save(p)
    return p


if __name__ == "__main__":
    if os.path.exists(os.path.join(R, "exo_hero.png")):
        print(sheet(*EXO, below=("low", "open")))
    if all(os.path.exists(os.path.join(R, c[0] + "_hero.png")) for c in CONCEPTS):
        for c in CONCEPTS:
            print(sheet(*c))
        print(overview())
