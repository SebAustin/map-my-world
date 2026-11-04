#!/usr/bin/env python3
"""Generate the poster Landmarks: one PNG texture per poster plus the OGRE material script
Gazebo uses to paint them.

Each poster is a seeded random collage of high-contrast shapes under a large letter, so every
poster gives the camera many corners and no two posters look alike. Output goes into a Gazebo
model folder that worlds reference as model://landmarks.

    python3 scripts/make_landmarks.py models/landmarks
"""

import argparse
import os
import random

from PIL import Image, ImageDraw, ImageFont

POSTERS = "ABCDEFGHIJ"
SIZE = (512, 384)  # 4:3, matching the 0.8 x 0.6 m panels in the World
SHAPES_PER_POSTER = 40
MATERIAL_FILE = "landmarks.material"

PALETTE = [
    (230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200), (245, 130, 48),
    (145, 30, 180), (70, 240, 240), (240, 50, 230), (0, 0, 0), (255, 255, 255),
]


def material_name(letter):
    return "Landmark/Poster%s" % letter


def texture_name(letter):
    return "poster_%s.png" % letter.lower()


def draw_poster(letter, seed):
    """Return a deterministic, feature-rich poster image for one letter."""
    rng = random.Random(seed)
    background = PALETTE[rng.randrange(len(PALETTE))]
    img = Image.new("RGB", SIZE, background)
    draw = ImageDraw.Draw(img)
    w, h = SIZE
    for _ in range(SHAPES_PER_POSTER):
        colour = PALETTE[rng.randrange(len(PALETTE))]
        x0, y0 = rng.randrange(w), rng.randrange(h)
        x1, y1 = x0 + rng.randrange(20, 140), y0 + rng.randrange(20, 140)
        kind = rng.randrange(3)
        if kind == 0:
            draw.rectangle([x0, y0, x1, y1], fill=colour, outline=(0, 0, 0), width=3)
        elif kind == 1:
            draw.ellipse([x0, y0, x1, y1], fill=colour, outline=(0, 0, 0), width=3)
        else:
            draw.polygon([(x0, y1), ((x0 + x1) // 2, y0), (x1, y1)], fill=colour, outline=(0, 0, 0))
    font = _font(220)
    box = draw.textbbox((0, 0), letter, font=font)
    pos = ((w - (box[2] - box[0])) // 2 - box[0], (h - (box[3] - box[1])) // 2 - box[1])
    draw.text(pos, letter, font=font, fill=(255, 255, 255), stroke_width=10, stroke_fill=(0, 0, 0))
    return img


def _font(size):
    for path in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                 "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
                 "/Library/Fonts/Arial Bold.ttf"):
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default(size=size)


def material_script(letters):
    blocks = []
    for letter in letters:
        blocks.append(
            "material %s\n"
            "{\n"
            "  technique\n"
            "  {\n"
            "    pass\n"
            "    {\n"
            "      ambient 0.8 0.8 0.8 1.0\n"
            "      diffuse 1.0 1.0 1.0 1.0\n"
            "      texture_unit\n"
            "      {\n"
            "        texture %s\n"
            "        filtering anisotropic\n"
            "      }\n"
            "    }\n"
            "  }\n"
            "}\n" % (material_name(letter), texture_name(letter)))
    return "\n".join(blocks)


def model_config():
    return ("<?xml version=\"1.0\"?>\n"
            "<model>\n"
            "  <name>landmarks</name>\n"
            "  <version>1.0</version>\n"
            "  <sdf version=\"1.6\">model.sdf</sdf>\n"
            "  <description>Poster textures for the Map My World Landmarks (materials only).</description>\n"
            "</model>\n")


def model_sdf():
    # Gazebo needs an SDF file to accept the folder as a model; the posters themselves live in the World.
    return ("<?xml version=\"1.0\"?>\n"
            "<sdf version=\"1.6\">\n"
            "  <model name=\"landmarks\">\n"
            "    <static>true</static>\n"
            "    <link name=\"link\"/>\n"
            "  </model>\n"
            "</sdf>\n")


def write_model(out_dir, letters=POSTERS):
    textures = os.path.join(out_dir, "materials", "textures")
    scripts = os.path.join(out_dir, "materials", "scripts")
    os.makedirs(textures, exist_ok=True)
    os.makedirs(scripts, exist_ok=True)
    for i, letter in enumerate(letters):
        draw_poster(letter, seed=i).save(os.path.join(textures, texture_name(letter)), optimize=True)
    with open(os.path.join(scripts, MATERIAL_FILE), "w") as f:
        f.write(material_script(letters))
    with open(os.path.join(out_dir, "model.config"), "w") as f:
        f.write(model_config())
    with open(os.path.join(out_dir, "model.sdf"), "w") as f:
        f.write(model_sdf())


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out_dir", help="model folder to write, e.g. models/landmarks")
    write_model(parser.parse_args().out_dir)


if __name__ == "__main__":
    main()
