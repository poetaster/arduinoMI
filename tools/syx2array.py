#!/usr/bin/env python3

'''
original from ledlaux @ https://github.com/poetaster/arduinoMI/pull/12#issuecomment-3667649383
'''

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



'''
============================================================
resources.cc
============================================================

1. Copy your new SYX bank array got from the script:
   (syx_bank_3)

2. Update the FM patches table:

   const uint8_t* const fm_patches_table[] = {
       syx_bank_0,
       syx_bank_1,
       syx_bank_2,
       syx_bank_3,  
   };

============================================================
resources.h
============================================================

1. Declare the new SYX bank:

   extern const uint8_t syx_bank_3[];

============================================================
user_data.h
============================================================

1. Add a case for the new bank:

   case 3:
       return syx_bank_3;

============================================================
voice.cc
============================================================

1. Register the new engine instance:

   engines_.RegisterInstance(&six_op_engine_3, true, 1.0f, 1.0f);

2. Define engine indices:

   const int six_op_0_index = 16;
   const int six_op_1_index = 17;
   const int six_op_2_index = 18;
   const int six_op_3_index = 19;

3. Map engine indices to SYX banks:

   else if (engine_index == 16) {
       data = plaits::syx_bank_0;
   } else if (engine_index == 17) {
       data = plaits::syx_bank_1;
   } else if (engine_index == 18) {
       data = plaits::syx_bank_2;
   } else if (engine_index == 19) {
       data = plaits::syx_bank_3;   
   }

============================================================
voice.h
============================================================

1. Add a new SixOpEngine instance:

   SixOpEngine six_op_engine_3;
'''
