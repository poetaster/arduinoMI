// Copyright 2015 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// String synth part.

#include "rings/dsp/string_synth_part.h"

#include "rings/dsp/dsp.h"

namespace rings {

using namespace std;
using namespace stmlib;

void StringSynthPart::Init(uint16_t* reverb_buffer) {
    sr_ = Dsp::getSr();     // vb
    a3_ = Dsp::getA3();     // vb
  active_group_ = 0;
  acquisition_delay_ = 0;
  
  polyphony_ = 1;
  fx_type_ = FX_ENSEMBLE;

  for (int32_t i = 0; i < kStringSynthVoices; ++i) {
    voice_[i].Init();
  }
  
  for (int32_t i = 0; i < kMaxStringSynthPolyphony; ++i) {
    group_[i].tonic = 0.0f;
    group_[i].envelope.Init();
  }
  
  for (int32_t i = 0; i < kNumFormants; ++i) {
    formant_filter_[i].Init();
  }
  
  limiter_.Init();
  
  reverb_.Init(reverb_buffer);
  chorus_.Init(reverb_buffer);
  ensemble_.Init(reverb_buffer);
  
  note_filter_.Init(
      sr_ / kMaxBlockSize,
      0.001f,  // Lag time with a sharp edge on the V/Oct input or trigger.
      0.005f,  // Lag time after the trigger has been received.
      0.050f,  // Time to transition from reactive to filtered.
      0.004f); // Prevent a sharp edge to partly leak on the previous voice.
}

const int32_t kRegistrationTableSize = 11;
const float registrations[kRegistrationTableSize][kNumHarmonics * 2] = {
  { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f },
  { 1.0f, 0.1f, 0.0f, 0.0f, 1.0f, 0.0f },
  { 1.0f, 0.5f, 1.0f, 0.0f, 1.0f, 0.0f },
  { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
  { 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f },
  { 0.0f, 0.5f, 1.0f, 0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f },
};

void StringSynthPart::ComputeRegistration(
    float gain,
    float registration,
    float* amplitudes) {
  registration *= (kRegistrationTableSize - 1.001f);
  MAKE_INTEGRAL_FRACTIONAL(registration);
  float total = 0.0f;
  for (int32_t i = 0; i < kNumHarmonics * 2; ++i) {
    float a = registrations[registration_integral][i];
    float b = registrations[registration_integral + 1][i];
    amplitudes[i] = a + (b - a) * registration_fractional;
    total += amplitudes[i];
  }
  for (int32_t i = 0; i < kNumHarmonics * 2; ++i) {
    amplitudes[i] = gain * amplitudes[i] / total;
  }
}

void StringSynthPart::ProcessEnvelopes(
    float shape,
    uint8_t* flags,
    float* values) {
  float decay = shape;
  float attack = 0.0f;
  if (shape < 0.5f) {
    attack = 0.0f;
  } else {
    attack = (shape - 0.5f) * 2.0f;
  }
  
  // Convert the arbitrary values to actual units.
  float period = sr_ / kMaxBlockSize;
  float attack_time = SemitonesToRatio(attack * 96.0f) * 0.005f * period;
  // float decay_time = SemitonesToRatio(decay * 96.0f) * 0.125f * period;
  float decay_time = SemitonesToRatio(decay * 84.0f) * 0.180f * period;
  float attack_rate = 1.0f / attack_time;
  float decay_rate = 1.0f / decay_time;
  
  for (int32_t i = 0; i < polyphony_; ++i) {
    float drone = shape < 0.98f ? 0.0f : (shape - 0.98f) * 55.0f;
    if (drone >= 1.0f) drone = 1.0f;

    group_[i].envelope.set_ad(attack_rate, decay_rate);
    float value = group_[i].envelope.Process(flags[i]);
    values[i] = value + (1.0f - value) * drone;
  }
}

const int32_t kFormantTableSize = 5;
const float formants[kFormantTableSize][kNumFormants] = {
  { 700, 1100, 2400 },
  { 500, 1300, 1700 },
  { 400, 2000, 2500 },
  { 600, 800, 2400 },
  { 300, 900, 2200 },
};

void StringSynthPart::ProcessFormantFilter(
    float vowel,
    float shift,
    float resonance,
    float* out,
    float* aux,
    size_t size) {
  for (size_t i = 0; i < size; ++i) {
    filter_in_buffer_[i] = out[i] + aux[i];
  }
  fill(&out[0], &out[size], 0.0f);
  fill(&aux[0], &aux[size], 0.0f);

  vowel *= (kFormantTableSize - 1.001f);
  MAKE_INTEGRAL_FRACTIONAL(vowel);
  
  for (int32_t i = 0; i < kNumFormants; ++i) {
    float a = formants[vowel_integral][i];
    float b = formants[vowel_integral + 1][i];
    float f = a + (b - a) * vowel_fractional;
    f *= shift;
    formant_filter_[i].set_f_q<FREQUENCY_DIRTY>(f / sr_, resonance);
    formant_filter_[i].Process<FILTER_MODE_BAND_PASS>(
        filter_in_buffer_,
        filter_out_buffer_,
        size);
    const float pan = i * 0.3f + 0.2f;
    for (size_t j = 0; j < size; ++j) {
      out[j] += filter_out_buffer_[j] * pan * 0.5f;
      aux[j] += filter_out_buffer_[j] * (1.0f - pan) * 0.5f;
    }
  }
}

struct ChordNote {
  float note;
  float amplitude;
};

void StringSynthPart::Process(
    const PerformanceState& performance_state,
    const Patch& patch,
    const float* in,
    float* out,
    float* aux,
    size_t size) {
  // Assign note to a voice.
  uint8_t envelope_flags[kMaxStringSynthPolyphony];
  
  fill(&envelope_flags[0], &envelope_flags[polyphony_], 0);
  note_filter_.Process(performance_state.note, performance_state.strum);
  if (performance_state.strum) {
    group_[active_group_].tonic = note_filter_.stable_note();
    envelope_flags[active_group_] = ENVELOPE_FLAG_FALLING_EDGE;
    active_group_ = (active_group_ + 1) % polyphony_;
    envelope_flags[active_group_] = ENVELOPE_FLAG_RISING_EDGE;
    acquisition_delay_ = 3;
  }
  if (acquisition_delay_) {
    --acquisition_delay_;
  } else {
    group_[active_group_].tonic = note_filter_.note();
    group_[active_group_].chord = performance_state.chord;
    group_[active_group_].structure = patch.structure;
    envelope_flags[active_group_] |= ENVELOPE_FLAG_GATE;
  }

  // Process envelopes.
  float envelope_values[kMaxStringSynthPolyphony];
  ProcessEnvelopes(patch.damping, envelope_flags, envelope_values);
  
    // vb, do we need to do this?
  copy(&in[0], &in[size], &aux[0]);
  copy(&in[0], &in[size], &out[0]);
  int32_t chord_size = min(kStringSynthVoices / polyphony_, kMaxChordSize);
  for (int32_t group = 0; group < polyphony_; ++group) {
    ChordNote notes[kMaxChordSize];
    float harmonics[kNumHarmonics * 2];
    
    ComputeRegistration(
        envelope_values[group] * 0.25f,
        patch.brightness,
        harmonics);
    
    // Note enough polyphony for smooth transition between chords.
    for (int32_t i = 0; i < chord_size; ++i) {
      float n = chords[polyphony_ - 1][group_[group].chord][i];
      notes[i].note = n;
      notes[i].amplitude = n >= 0.0f && n <= 17.0f ? 1.0f : 0.7f;
    }

    for (int32_t chord_note = 0; chord_note < chord_size; ++chord_note) {
      float note = 0.0f;
      note += group_[group].tonic;
      note += performance_state.tonic;
      note += performance_state.fm;
      note += notes[chord_note].note;
      
      float amplitudes[kNumHarmonics * 2];
      for (int32_t i = 0; i < kNumHarmonics * 2; ++i) {
        amplitudes[i] = notes[chord_note].amplitude * harmonics[i];
      }
      
      // Fold truncated harmonics.
      size_t num_harmonics = polyphony_ >= 2 && chord_note < 2
          ? kNumHarmonics - 1
          : kNumHarmonics;
      for (int32_t i = num_harmonics; i < kNumHarmonics; ++i) {
        amplitudes[2 * (num_harmonics - 1)] += amplitudes[2 * i];
        amplitudes[2 * (num_harmonics - 1) + 1] += amplitudes[2 * i + 1];
      }

      float frequency = SemitonesToRatio(note - 69.0f) * a3_;
      voice_[group * chord_size + chord_note].Render(
          frequency,
          amplitudes,
          num_harmonics,
          (group + chord_note) & 1 ? out : aux,
          size);
    }
  }
  
  if (clear_fx_) {
    reverb_.Clear();
    clear_fx_ = false;
  }
  
  switch (fx_type_) {
    case FX_FORMANT:
    case FX_FORMANT_2:
      ProcessFormantFilter(
          patch.position,
          fx_type_ == FX_FORMANT ? 1.0f : 1.1f,
          fx_type_ == FX_FORMANT ? 25.0f : 10.0f,
          out,
          aux,
          size);
      break;

    case FX_CHORUS:
      chorus_.set_amount(patch.position);
      chorus_.set_depth(0.15f + 0.5f * patch.position);
      chorus_.Process(out, aux, size);
      break;
    
    case FX_ENSEMBLE:
      ensemble_.set_amount(patch.position * (2.0f - patch.position));
      ensemble_.set_depth(0.2f + 0.8f * patch.position * patch.position);
      ensemble_.Process(out, aux, size);
      break;
  
    case FX_REVERB:
    case FX_REVERB_2:
      reverb_.set_amount(patch.position * 0.5f);
      reverb_.set_diffusion(0.625f);
      reverb_.set_time(fx_type_ == FX_REVERB
        ? (0.5f + 0.49f * patch.position)
        : (0.3f + 0.6f * patch.position));
      reverb_.set_input_gain(0.2f);
      reverb_.set_lp(fx_type_ == FX_REVERB ? 0.3f : 0.6f);
      reverb_.Process(out, aux, size);
      break;
    
    default:
      break;
  }

  // Prevent main signal cancellation when EVEN gets summed with ODD through
  // normalization.
  for (size_t i = 0; i < size; ++i) {
    aux[i] = -aux[i];
  }
  limiter_.Process(out, aux, size, 1.0f);
}

}  // namespace rings
