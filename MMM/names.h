const char* const oscnames[] = {
   "Virtual Analog", "Waveshaping", "FM", "Grain", "Additive", "Wavetable", "Chord", "Speech",
  "Swarm", "Noise", "Particle", "String", "Modal", "Bass Drum", "Snare Drum", "Hi-Hat", 
  "Six OP 0", "Six OP 1", "Six OP 2","Virtual Analog VCF", "Phase Distortion", "Wave Terrain"
  };

const size_t nameslength = sizeof(oscnames) / sizeof(oscnames[0]);


const char* const modelnames[] = {
  "MODAL_RESO",
  "SYMPA_STRING",
  "MODULATED_STRING",
  "2-OP_FM_VOICE",
  "SYMPATHETIC_STRING_Q",
  "STRING_AND_REVERB"
};
 
const char* const FXnames[] = {
  "FX_FORMANT",
  "FX_CHORUS",
  "FX_REVERB",
  "FX_FORMANT",
  "FX_ENSEMBLE",
  "FX_REVERB"
};


const size_t modelslength = sizeof(modelnames) / sizeof(modelnames[0]);

const char* const cloudnames[] = {
  "GRANULAR",
  "STRETCH",
  "LOOPING",
  "SPECTRAL"
};
const size_t cloudnameslength = sizeof(cloudnames) / sizeof(cloudnames[0]);

const char* const braidsnames[] = {
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

const size_t braidslength = sizeof(braidsnames) / sizeof(braidsnames[0]);
