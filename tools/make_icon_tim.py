#!/usr/bin/env python3
"""make_icon_tim.py — emit a 16x16, 16-colour (4bpp) TIM for the MiniCAD save icon.

Output is a standard PSn00bSDK/PsyQ TIM:
  u32 0x00000010      magic
  u32 0x00000008      flags: pmode=0 (4bpp) | bit3 set (has CLUT)
  CLUT block:  u32 bnum, u16 x, u16 y, u16 w(=16), u16 h(=1), 16 x u16 BGR555
  pixel block: u32 bnum, u16 x, u16 y, u16 w(=4), u16 h(=16), 16*16 4bpp nibbles

The same 16x16/4bpp/16-CLUT layout is exactly what the PS1 save-header icon
frames want, so memcard.c parses this TIM and copies the CLUT + pixels straight
into frames 1..3 of the save header.

A tiny pixel-art cube-with-hole on a steel-blue field (matches the viewport).
"""
import struct, sys

W = H = 16

# 16-colour BGR555 palette. index 0 = transparent/background handled by STP bit.
def bgr555(r, g, b, stp=0):
    r >>= 3; g >>= 3; b >>= 3
    return (stp << 15) | (b << 10) | (g << 5) | r

PAL = [
    bgr555(40, 60, 90),    # 0 background steel-blue
    bgr555(0, 0, 0),       # 1 outline black
    bgr555(230, 170, 60),  # 2 cube face A (orange-ish, the boss colour)
    bgr555(200, 140, 40),  # 3 cube face B
    bgr555(160, 110, 30),  # 4 cube face C (darker top)
    bgr555(20, 30, 50),    # 5 hole shadow
    bgr555(255, 255, 255), # 6 highlight
    bgr555(120, 120, 130), # 7 grey
] + [bgr555(0, 0, 0) for _ in range(8)]

# 16x16 index map. 0=bg.  A little isometric cube with a bore on the top.
ART = [
    "0000000000000000",
    "0000011111100000",
    "0000164444461000",
    "0001644455444610",
    "0016444555544461",
    "0164445555554446",
    "1644445555544441",
    "1264444444444421",
    "1226666666666221",
    "1222222222222221",
    "1322222222222231",
    "1332222222222331",
    "0133222222223310",
    "0013322222233100",
    "0001333333331000",
    "0000011111100000",
]

def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "assets/icon.tim"
    buf = bytearray()
    buf += struct.pack("<I", 0x10)          # magic
    buf += struct.pack("<I", 0x08)          # flags: 4bpp + CLUT present

    # CLUT block
    clut = bytearray()
    for c in PAL:
        clut += struct.pack("<H", c)
    clut_block = struct.pack("<IHHHH", 12 + len(clut), 0, 0, 16, 1) + clut
    buf += clut_block

    # pixel block: 4bpp -> 2 px per byte, width in halfwords = 16/4 = 4
    px = bytearray()
    for row in ART:
        assert len(row) == 16
        for x in range(0, 16, 2):
            lo = int(row[x], 16)
            hi = int(row[x + 1], 16)
            px.append((hi << 4) | lo)
    pix_block = struct.pack("<IHHHH", 12 + len(px), 0, 0, 4, 16) + px
    buf += pix_block

    with open(out, "wb") as f:
        f.write(buf)
    print(f"wrote {out} ({len(buf)} bytes): 16x16 4bpp, {len(PAL)}-colour CLUT")

if __name__ == "__main__":
    main()
