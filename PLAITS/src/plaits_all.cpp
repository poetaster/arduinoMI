// braids_all.cpp - Single compilation unit for BRAIDS
// Only includes the .inc files that exist in poetaster's arduinoMI port

#include <STMLIB.h>

#include "plaits/resources.inc"
#include "plaits/dsp/voice.inc"
#include "plaits/dsp/speech/lpc_speech_synth.inc"
#include "plaits/dsp/speech/lpc_speech_synth_controller.inc"
#include "plaits/dsp/speech/lpc_speech_synth_phonemes.inc"
#include "plaits/dsp/speech/lpc_speech_synth_words.inc"
#include "plaits/dsp/speech/naive_speech_synth.inc"
#include "plaits/dsp/speech/sam_speech_synth.inc"
#include "plaits/dsp/engine/additive_engine.inc"
#include "plaits/dsp/engine/bass_drum_engine.inc"
#include "plaits/dsp/engine/chord_engine.inc"
#include "plaits/dsp/engine/fm_engine.inc"
#include "plaits/dsp/engine/grain_engine.inc"
#include "plaits/dsp/engine/hi_hat_engine.inc"
#include "plaits/dsp/engine/modal_engine.inc"
#include "plaits/dsp/engine/noise_engine.inc"
#include "plaits/dsp/engine/particle_engine.inc"
#include "plaits/dsp/engine/snare_drum_engine.inc"
#include "plaits/dsp/engine/speech_engine.inc"
#include "plaits/dsp/engine/string_engine.inc"
#include "plaits/dsp/engine/swarm_engine.inc"
#include "plaits/dsp/engine/virtual_analog_engine.inc"
#include "plaits/dsp/engine/waveshaping_engine.inc"
#include "plaits/dsp/engine/wavetable_engine.inc"
#include "plaits/dsp/engine/wave_terrain_engine.inc"
#include "plaits/dsp/physical_modelling/modal_voice.inc"
#include "plaits/dsp/physical_modelling/resonator.inc"
#include "plaits/dsp/physical_modelling/string.inc"
#include "plaits/dsp/physical_modelling/string_voice.inc"

#include "plaits/dsp/engine/virtual_analog_vcf_engine.inc"
#include "plaits/dsp/engine/phase_distortion_engine.inc"

