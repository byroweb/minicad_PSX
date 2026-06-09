#!/usr/bin/env bash
# build_psx.sh — build the MiniCAD-PSX PlayStation executable.
#
# The PSn00bSDK toolchain + SDK live outside the repo (they're ~150 MB of
# prebuilt binaries from the v0.24 GitHub release). This script pins the env so
# the build "just works" without remembering the magic variables.
#
#   Toolchain (bare-metal mipsel-none-elf GCC 12.3.0):  $PSX_TOOLCHAIN/bin
#   SDK (libpsn00b + elf2x/mkpsxiso/etc):               $PSX_SDK
#
# NOTE: the Debian `mipsel-linux-gnu-gcc` is the WRONG toolchain (Linux glibc
# ABI). PSn00bSDK needs the bare-metal `mipsel-none-elf-*` GCC; that's what the
# release zip ships and what this script points at.
#
# Usage:  tools/build_psx.sh [clean]
set -euo pipefail

PSX_ROOT="${PSX_ROOT:-$HOME/psx}"
PSX_TOOLCHAIN="${PSX_TOOLCHAIN:-$PSX_ROOT/toolchain}"
PSX_SDK="${PSX_SDK:-$PSX_ROOT/PSn00bSDK-0.24-Linux}"

export PSN00BSDK_LIBS="$PSX_SDK/lib/libpsn00b"
export PATH="$PSX_TOOLCHAIN/bin:$PSX_SDK/bin:$PATH"

if [ ! -x "$PSX_TOOLCHAIN/bin/mipsel-none-elf-gcc" ]; then
    echo "ERROR: mipsel-none-elf-gcc not found under $PSX_TOOLCHAIN" >&2
    echo "Get it from: https://github.com/Lameguy64/PSn00bSDK/releases (v0.24)" >&2
    echo "  gcc-mipsel-none-elf-12.3.0-linux.zip -> $PSX_TOOLCHAIN" >&2
    echo "  PSn00bSDK-0.24-Linux.zip            -> $PSX_SDK" >&2
    exit 1
fi

REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"

if [ "${1:-}" = "clean" ]; then rm -rf build-psx; fi

cmake -B build-psx -DCMAKE_TOOLCHAIN_FILE="$PSN00BSDK_LIBS/cmake/sdk.cmake" \
      -DCMAKE_BUILD_TYPE=Release
cmake --build build-psx

echo
echo "Built: $REPO/build-psx/minicad.exe"
echo "Sideload in DuckStation, or wrap into a CD image to boot from disc."
