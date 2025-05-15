#include <iostream>


/*
int main() {
    const char* const list[] = {"zip", "zam", "bam"};
    const size_t len = sizeof(list) / sizeof(list[0]);

    for (size_t i = 0; i < len; ++i)
        std::cout << list[i] << "\n";

    const std::vector<std::string> v(list, list + len);
    std::copy(v.begin(), v.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
}*/
  
  
const char* const oscnames[] = {
"CSAW",
"MORPH",
"SAW_SQUARE",
"SINE_TRIANGLE",
"BUZZ",
"SQUARE_SUB",
"SAW_SUB",
"SQUARE_SYNC",
"SAW_SYNC",
"TRIPLE_SAW",
"TRIPLE_SQUARE",
"TRIPLE_TRIANGLE",
"TRIPLE_SINE",
"TRIPLE_RING_MOD",
"SAW_SWARM",
"SAW_COMB",
"TOY",
"DIGITAL_FILTER_LP",
"DIGITAL_FILTER_PK",
"DIGITAL_FILTER_BP",
"DIGITAL_FILTER_HP",
"VOSIM",
"VOWEL",
"VOWEL_FOF",
"HARMONICS",
"FM",
"FEEDBACK_FM",
"CHAOTIC_FEEDBACK_FM",
"PLUCKED",
"BOWED",
"BLOWN",
"FLUTED",
"STRUCK_BELL",
"STRUCK_DRUM",
"KICK",
"CYMBAL",
"SNARE",
"WAVETABLES",
"WAVE_MAP",
"WAVE_LINE",
"WAVE_PARAPHONIC",
"FILTERED_NOISE",
"TWIN_PEAKS_NOISE",
"CLOCKED_NOISE",
"GRANULAR_CLOUD",
"PARTICLE_NOISE",
  };
 const size_t nameslength = sizeof(oscnames) / sizeof(oscnames[0]);
