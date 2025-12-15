#pragma once

// ==================== RESOURCES ====================
#include "plaits/resources.h"

// ==================== CORE DSP ====================
#include "plaits/dsp/dsp.h"
#include "plaits/dsp/voice.h"
#include "plaits/dsp/envelope.h"

// ==================== SPEECH ====================
#include "plaits/dsp/speech/lpc_speech_synth.h"
#include "plaits/dsp/speech/lpc_speech_synth_controller.h"
#include "plaits/dsp/speech/lpc_speech_synth_words.h"
#include "plaits/dsp/speech/naive_speech_synth.h"
#include "plaits/dsp/speech/sam_speech_synth.h"

// ==================== DRUMS ====================
#include "plaits/dsp/drums/analog_bass_drum.h"
#include "plaits/dsp/drums/analog_snare_drum.h"
#include "plaits/dsp/drums/hi_hat.h"
#include "plaits/dsp/drums/synthetic_bass_drum.h"
#include "plaits/dsp/drums/synthetic_snare_drum.h"

// ==================== FX ====================
#include "plaits/dsp/fx/fx_engine.h"
#include "plaits/dsp/fx/low_pass_gate.h"
#include "plaits/dsp/fx/overdrive.h"
#include "plaits/dsp/fx/sample_rate_reducer.h"
#include "plaits/dsp/fx/ensemble.h"
#include "plaits/dsp/fx/diffuser.h"

// ==================== NOISE ====================
#include "plaits/dsp/noise/smooth_random_generator.h"
#include "plaits/dsp/noise/fractal_random_generator.h"
#include "plaits/dsp/noise/clocked_noise.h"
#include "plaits/dsp/noise/particle.h"
#include "plaits/dsp/noise/dust.h"

// ==================== OSCILLATORS ====================
#include "plaits/dsp/oscillator/oscillator.h"
#include "plaits/dsp/oscillator/sine_oscillator.h"
#include "plaits/dsp/oscillator/string_synth_oscillator.h"
#include "plaits/dsp/oscillator/variable_saw_oscillator.h"
#include "plaits/dsp/oscillator/variable_shape_oscillator.h"
#include "plaits/dsp/oscillator/vosim_oscillator.h"
#include "plaits/dsp/oscillator/wavetable_oscillator.h"
#include "plaits/dsp/oscillator/z_oscillator.h"
#include "plaits/dsp/oscillator/formant_oscillator.h"
#include "plaits/dsp/oscillator/grainlet_oscillator.h"
#include "plaits/dsp/oscillator/harmonic_oscillator.h"
#include "plaits/dsp/oscillator/super_square_oscillator.h"
#include "plaits/dsp/oscillator/nes_triangle_oscillator.h"

// ==================== PHYSICAL MODELLING ====================
#include "plaits/dsp/physical_modelling/string.h"
#include "plaits/dsp/physical_modelling/string_voice.h"
#include "plaits/dsp/physical_modelling/resonator.h"
#include "plaits/dsp/physical_modelling/modal_voice.h"
#include "plaits/dsp/physical_modelling/delay_line.h"

// ==================== CORE ENGINES ====================
#include "plaits/dsp/engine/engine.h"
#include "plaits/dsp/engine/additive_engine.h"
#include "plaits/dsp/engine/bass_drum_engine.h"
#include "plaits/dsp/engine/chord_engine.h"
#include "plaits/dsp/engine/fm_engine.h"
#include "plaits/dsp/engine/grain_engine.h"
#include "plaits/dsp/engine/hi_hat_engine.h"
#include "plaits/dsp/engine/modal_engine.h"
#include "plaits/dsp/engine/noise_engine.h"
#include "plaits/dsp/engine/particle_engine.h"
#include "plaits/dsp/engine/snare_drum_engine.h"
#include "plaits/dsp/engine/speech_engine.h"
#include "plaits/dsp/engine/string_engine.h"
#include "plaits/dsp/engine/swarm_engine.h"
#include "plaits/dsp/engine/virtual_analog_engine.h"
#include "plaits/dsp/engine/waveshaping_engine.h"
#include "plaits/dsp/engine/wavetable_engine.h"



// ==================== ENGINE2 / FM7 / DX7 / PD / WT / SM ====================
#include "plaits/dsp/engine2/six_op_engine.h"             // DX7 / 6-op FM
//#include "plaits/dsp/engine2/phase_distortion_engine.h"   // Phase Distortion
//#include "plaits/dsp/engine2/wave_terrain_engine.h"      // Wave Terrain
//#include "plaits/dsp/engine2/string_machine_engine.h"    // String Machine / Chorus
//#include "plaits/dsp/engine2/chiptune_engine.h"          // Chiptune
//#include "plaits/dsp/engine2/virtual_analog_vcf_engine.h" // VA + VCF

// ==================== FM HELPERS ====================
#include "plaits/dsp/fm/operator.h"
#include "plaits/dsp/fm/algorithms.h"
#include "plaits/dsp/fm/dx_units.h"
#include "plaits/dsp/fm/lfo.h"
#include "plaits/dsp/fm/patch.h"
#include "plaits/dsp/fm/voice.h"
#include "plaits/dsp/fm/envelope.h"

// ==================== CHORDS ====================
#include "plaits/dsp/chords/chord_bank.h"
#include "plaits/user_data.h"

