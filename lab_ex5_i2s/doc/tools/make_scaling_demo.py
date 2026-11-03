"""Generate native-resolution 24-bit BMPs for FPGA scaling board validation.

The SD inputs retain their original sizes. PNG previews show the expected
640x480 aspect-fit result, and must not be copied to the SD card as substitutes.
Requires Pillow; run with Python, no card writes are performed.
"""
import json
import struct
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def make_image(w, h, label):
    image = Image.new("RGB", (w, h), (28, 36, 55))
    draw = ImageDraw.Draw(image)
    step = max(8, min(w, h) // 12)
    for x in range(0, w, step):
        draw.line((x, 0, x, h), fill=(65, 76, 94))
    for y in range(0, h, step):
        draw.line((0, y, w, y), fill=(65, 76, 94))
    radius = min(w, h) // 3
    for r in (radius, radius * 2 // 3, radius // 3):
        draw.ellipse((w//2-r, h//2-r, w//2+r, h//2+r), outline=(230, 230, 230), width=max(1, step//8))
    size = max(14, min(w, h) // 14)
    try:
        font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", size)
    except OSError:
        font = ImageFont.load_default(size=size)
    band = max(size * 2, h // 10)
    draw.rectangle((0, 0, w-1, band), fill=(150, 25, 30))
    draw.rectangle((0, h-band, w-1, h-1), fill=(25, 50, 150))
    draw.text((size*2, size//3), f"TOP  {label}", font=font, fill="white")
    draw.text((size*2, h-band+size//3), "BOTTOM", font=font, fill="white")
    box = max(8, min(w, h) // 12)
    for x, y, color in ((0, 0, "red"), (w-box, 0, "lime"), (0, h-box, "blue"), (w-box, h-box, "yellow")):
        draw.rectangle((x, y, x+box-1, y+box-1), fill=color)
    draw.rectangle((0, 0, w-1, h-1), outline="white", width=max(1, min(w, h)//180))
    return image


def write_bmp(path, image, top_down):
    w, h = image.size
    stride = (w*3+3)//4*4
    rows = image.tobytes("raw", "BGR")
    with path.open("wb") as output:
        output.write(struct.pack("<2sIHHI", b"BM", 54+stride*h, 0, 0, 54))
        output.write(struct.pack("<IiiHHIIiiII", 40, w, -h if top_down else h, 1, 24, 0, stride*h, 2835, 2835, 0, 0))
        for y in (range(h) if top_down else range(h-1, -1, -1)):
            output.write(rows[y*w*3:(y+1)*w*3])
            output.write(b"\xa5"*(stride-w*3))
    # Independently verify file header, signed height and decoded image rows.
    blob = path.read_bytes()
    assert struct.unpack_from("<I", blob, 2)[0] == len(blob) == 54+stride*h
    assert struct.unpack_from("<i", blob, 22)[0] == (-h if top_down else h)
    with Image.open(path) as decoded:
        assert decoded.convert("RGB").tobytes() == image.tobytes()


def main():
    target = Path(__file__).resolve().parents[1] / "convert" / "scaling_test_bmp"
    target.mkdir(parents=True, exist_ok=True)
    metadata = []
    sheet = Image.new("RGB", (1280, 960))
    cases = [(320, 240, False), (1920, 1080, False), (241, 479, True), (641, 481, False)]
    for index, (w, h, td) in enumerate(cases):
        image = make_image(w, h, f"{w}x{h}")
        path = target / f"{index}.BMP"
        write_bmp(path, image, td)
        if w*480 >= h*640:
            dw, dh = 640, max(1, h*640//w)
        else:
            dw, dh = max(1, w*480//h), 480
        left, top = (640-dw)//2, (480-dh)//2
        preview = Image.new("RGB", (640, 480))
        # Explicit floor mapping matches the documented FPGA sampling rule.
        src, dst = image.load(), preview.load()
        for y in range(dh):
            for x in range(dw):
                dst[left+x, top+y] = src[x*w//dw, y*h//dh]
        preview.save(target / f"expected_{index}.png")
        sheet.paste(preview, ((index%2)*640, (index//2)*480))
        metadata.append(dict(file=path.name, width=w, height=h, top_down=td,
                             stride=(w*3+3)//4*4, fit_width=dw, fit_height=dh,
                             left=left, top=top, size_bytes=path.stat().st_size))
    sheet.save(target / "expected_all.png")
    (target / "manifest.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"Verified {len(cases)} BMP files in {target}")


if __name__ == "__main__":
    main()
