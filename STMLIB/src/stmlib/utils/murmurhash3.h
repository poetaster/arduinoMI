// murmurhash3.h - Fixed with inline keywords for Arduino compatibility
#ifndef STMLIB_UTILS_MURMURHASH3_H_
#define STMLIB_UTILS_MURMURHASH3_H_

#include <stdint.h>

// MUST be inline to avoid multiple definition errors in Arduino
inline uint32_t rotl32(uint32_t x, int8_t r) {
  return (x << r) | (x >> (32 - r));
}

inline uint32_t fmix32(uint32_t h) {
  h ^= h >> 16;
  h *= 0x85ebca6b;
  h ^= h >> 13;
  h *= 0xc2b2ae35;
  h ^= h >> 16;
  return h;
}

inline void MurmurHash3_x86_32(const void* key, int len, uint32_t seed, void* out) {
  const uint8_t* data = (const uint8_t*)key;
  const int nblocks = len / 4;

  uint32_t h1 = seed;

  const uint32_t c1 = 0xcc9e2d51;
  const uint32_t c2 = 0x1b873593;

  // body
  const uint32_t* blocks = (const uint32_t*)(data + nblocks * 4);

  for (int i = -nblocks; i; i++) {
    uint32_t k1 = blocks[i];

    k1 *= c1;
    k1 = rotl32(k1, 15);
    k1 *= c2;

    h1 ^= k1;
    h1 = rotl32(h1, 13);
    h1 = h1 * 5 + 0xe6546b64;
  }

  // tail
  const uint8_t* tail = (const uint8_t*)(data + nblocks * 4);

  uint32_t k1 = 0;

  switch (len & 3) {
    case 3: k1 ^= tail[2] << 16; // fall through
    case 2: k1 ^= tail[1] << 8;  // fall through
    case 1: k1 ^= tail[0];
            k1 *= c1;
            k1 = rotl32(k1, 15);
            k1 *= c2;
            h1 ^= k1;
  }

  // finalization
  h1 ^= len;
  h1 = fmix32(h1);

  *(uint32_t*)out = h1;
}

#endif  // STMLIB_UTILS_MURMURHASH3_H_