#!/usr/bin/env python3
"""rip_mcad.py — extract MiniCAD-PSX saves from a raw PS1 memory-card image.

Reads a standard 131,072-byte .mcr/.mcd/.bin card dump (DuckStation, PCSX,
ePSXe, or a hardware dumper), finds MiniCAD saves by product code, follows the
directory block-chain, verifies the .mcad crc32, and writes each save out as a
host-side .mcad file.

Card layout (psxspx / psdevwiki):
  16 blocks x 8192 bytes. Block 0 = directory (frames 0..15 are dir entries,
  128 bytes each). Blocks 1..15 = files. A save's first block uses frame 0 for
  the title and frames 1..3 for the icon; frames 4..63 are payload. Linked
  blocks chain via the directory entry's "next block" field.

Usage:
    python3 rip_mcad.py card.mcr            # list MiniCAD saves
    python3 rip_mcad.py card.mcr --extract  # write .mcad files
"""
import sys, struct, argparse, zlib, os

BLOCK = 8192
FRAME = 128
CARD  = 131072
PRODUCT_PREFIX = b"BASLUS-MCAD"   # MiniCAD product code (matches the on-card filename)

MCAD_MAGIC = b"MCAD"

def load_card(path):
    with open(path, "rb") as f:
        data = f.read()
    # tolerate small headers (e.g. some dumps prepend bytes); align to 128KB tail
    if len(data) > CARD:
        data = data[-CARD:]
    if len(data) != CARD:
        raise SystemExit(f"not a 128KB card image ({len(data)} bytes)")
    return data

def dir_entries(card):
    """Yield (slot, state, filesize, next_block, filename) from block 0."""
    for slot in range(15):
        off = (slot + 1) * FRAME          # dir frame 1..15 describe blocks 1..15
        frame = card[off:off+FRAME]
        state = frame[0]                  # 0x51/0xA1 = first block of a file, etc.
        filesize, = struct.unpack_from("<I", frame, 4)
        next_blk, = struct.unpack_from("<H", frame, 8)   # 0xFFFF = last/none
        name = frame[0x0A:0x0A+20].split(b"\x00")[0]
        yield slot, state, filesize, next_blk, name

def gather_chain(card, start_slot, entries):
    """Follow the linked block list for a file starting at start_slot."""
    chain = [start_slot]
    by_slot = {e[0]: e for e in entries}
    cur = start_slot
    while True:
        _, _, _, nxt, _ = by_slot[cur]
        if nxt == 0xFFFF or nxt >= 15:
            break
        chain.append(nxt)
        cur = nxt
    return chain

def block_payload(card, slot, is_first):
    """Return the payload bytes of one block (skip title+icon on first block)."""
    base = (slot + 1) * BLOCK            # block 0 is directory; data blocks 1..15
    if is_first:
        return card[base + 4*FRAME : base + BLOCK]   # frames 4..63
    return card[base : base + BLOCK]                 # all frames

def extract_saves(card, do_write):
    entries = list(dir_entries(card))
    found = 0
    for slot, state, filesize, nxt, name in entries:
        if not name.startswith(PRODUCT_PREFIX):
            continue
        # only first-block entries (state high bit set per psxspx) start a file
        if state not in (0x51, 0xA1, 0x53):  # link-start variants
            continue
        chain = gather_chain(card, slot, entries)
        payload = b"".join(
            block_payload(card, s, is_first=(i == 0)) for i, s in enumerate(chain)
        )
        if payload[:4] != MCAD_MAGIC:
            print(f"slot {slot}: {name.decode(errors='replace')} — no MCAD magic, skipping")
            continue
        version = payload[4]
        feat_count = payload[5]
        plen = payload[6] | (payload[7] << 8)
        crc_stored, = struct.unpack_from("<I", payload, 8)
        body = payload[12:12+plen]
        crc_calc = zlib.crc32(body) & 0xFFFFFFFF
        ok = (crc_calc == crc_stored)
        found += 1
        print(f"slot {slot}: {name.decode(errors='replace')}  "
              f"v{version}  features={feat_count}  payload={plen}B  "
              f"crc={'OK' if ok else 'BAD'}  blocks={len(chain)}")
        if do_write and ok:
            out = name.decode(errors="replace").strip() + ".mcad"
            with open(out, "wb") as f:
                f.write(payload[:12+plen])
            print(f"        -> wrote {out} ({12+plen} bytes)")
    if not found:
        print("No MiniCAD saves found on this card.")

def main():
    ap = argparse.ArgumentParser(description="Extract MiniCAD-PSX saves from a PS1 card image.")
    ap.add_argument("card", help="path to .mcr/.mcd/.bin card image (128KB)")
    ap.add_argument("--extract", action="store_true", help="write .mcad files")
    args = ap.parse_args()
    card = load_card(args.card)
    extract_saves(card, args.extract)

if __name__ == "__main__":
    main()
