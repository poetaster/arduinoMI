#!/usr/bin/env python3
import sys
from pathlib import Path

SIZE = 4096  # Fixed array size

def syx_to_c_array(syx_path, array_name):
    # Read full .syx file
    data = Path(syx_path).read_bytes()

    # Skip standard SysEx start byte (0xF0) and header (assume 6 bytes total)
    start_index = 1 + 5 if len(data) > 6 and data[0] == 0xF0 else 0
    # Skip end byte 0xF7 if present
    end_index = -1 if data[-1] == 0xF7 else len(data)

    raw_bytes = data[start_index:end_index]

    # Ensure array is exactly SIZE
    if len(raw_bytes) < SIZE:
        raw_bytes += bytes(SIZE - len(raw_bytes))
    elif len(raw_bytes) > SIZE:
        raw_bytes = raw_bytes[:SIZE]

    # Output as C array
    print(f"const uint8_t {array_name}[] = {{")
    for i, b in enumerate(raw_bytes):
        if i % 12 == 0:
            print("  ", end="")
        print(f"{b:3d}, ", end="")
        if i % 12 == 11:
            print()
    print("\n};")
    # print(f"const size_t {array_name}_len = {SIZE};")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python syx_to_c_array.py <file.syx> <array_name>")
        sys.exit(1)

    syx_to_c_array(sys.argv[1], sys.argv[2])
