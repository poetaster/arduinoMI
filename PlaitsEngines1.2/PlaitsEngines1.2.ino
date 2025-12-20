/*
  (c) 2025 blueprint@poetaster.de
  GPLv3 the libraries are MIT as the originals for STM from MI were also MIT.

  some parts adapted from ledlaux
  Copyright (c) 2025 Vadims Maksimovs <ledlaux@gmail.com>
  MIT licence


  Change the PWMOUT number to the pin you are doing pwm on.
  Works fine 133mHz -O2 on an RP2350

  BE CAREFULL, can be loud or high pitched. Or BOTH!

*/

bool debug = true;

#include <Arduino.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include <hardware/pwm.h>

#include <PWMAudio.h>
#define SAMPLERATE 48000
#define PWMOUT 22
PWMAudio DAC(PWMOUT);  // 16 bit PWM audio

// plaits dsp
#include <STMLIB.h>
#include <PLAITS.h>

#define WORKSPACE_SIZE 65536
constexpr int AUDIO_BLOCK = plaits::kBlockSize;

// ==================== MEMORY BUFFERS ====================
static uint8_t shared_plaits_workspace[WORKSPACE_SIZE] __attribute__((aligned(4)));
static uint16_t clouds_buffer[65536];
static uint16_t oliverb_buffer[131072];

int16_t left[AUDIO_BLOCK];
int16_t right[AUDIO_BLOCK];

const char* engine_names[] = {
  "Virtual Analog", "Waveshaping", "FM", "Grain", "Additive", "Wavetable", "Chord", "Speech",
  "Swarm", "Noise", "Particle", "String", "Modal", "Bass Drum", "Snare Drum", "Hi-Hat",
  "Six OP 0", "Six OP 1", "Six OP 2", "Six OP 3", "Virtual Analog VCF", "Phase Distortion", "Wave Terrain",
  "String Machine", "Chiptune"
};

const size_t   kBlockSize = plaits::kBlockSize;

struct Voice {
  plaits::Voice* voice_;
  plaits::Patch patch;
  plaits::Modulations modulations;
  plaits::Voice::Frame out_buffer[AUDIO_BLOCK];
  int pitch;
  bool last_trig;
  bool prev_trig;
  char                *shared_buffer;
};

Voice voice;

// Plaits modulation vars
float morph_in = 0.1f; // IN(4);
float trigger_in = 0.0f; //IN(5);
float level_in = 0.5f; //IN(6);
float harm_in = 0.1f;
float timbre_in = 0.1f;
int engine_in;

float fm_mod = 0.f ; //IN(7);
float timb_mod = 0.f; //IN(8);
float morph_mod = 0.f; //IN(9);
float decay_in = 0.5f; // IN(10);
float lpg_in = 0.2f ;// IN(11);
float pitch_in = 60.f;
float octave_in = 3.f;

int engineCount = 0;
int engineInc = 0;

// ==================== UTIL ====================

inline float softClip(float x) {
  if (x > 1.5f) x = 1.5f;
  if (x < -1.5f) x = -1.5f;
  return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}


// produce some random numbers in ranges.
double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

void changeEngine(uint8_t engine_idx) {
  if (debug) {
    Serial.print("Switching to Engine [");
    Serial.print(engine_idx);
    Serial.print("]: ");
    if (engine_idx < 25) Serial.println(engine_names[engine_idx]);
  }
  // Reuse workspace
  memset(shared_plaits_workspace, 0, WORKSPACE_SIZE);
  stmlib::BufferAllocator allocator(shared_plaits_workspace, WORKSPACE_SIZE);
  voice.voice_->Init(&allocator);

}
void updateAudio(uint8_t engine_idx, bool triggerNow, float master_volume = 0.4f) {

  bool is_drum = (engine_idx >= 13 && engine_idx <= 15);
  bool is_wave_terrain = (engine_idx == 22);
  bool is_string_machine = (engine_idx == 23);
  bool is_chiptune = (engine_idx == 24);

  voice.patch.engine = engine_idx;
  voice.patch.note = (float)pitch_in;

  // Default Parameter Settings
  voice.patch.harmonics = harm_in;
  voice.patch.timbre = timbre_in;
  voice.patch.morph = morph_in;

  if (is_string_machine) {
    //voice.patch.harmonics = 0.5f; // Selects Chord (Major/Minor/etc.)
    voice.patch.timbre = 0.2f;    // Filter Cutoff + Chorus Amount (Avoid 0.5 becouse it makes silence)
    voice.patch.morph = 0.7f;     // Chorus depth
    voice.patch.decay = 0.9f;     // Internal VCA/LPG release time
    voice.modulations.level = 1.0f;
    voice.modulations.level_patched = true;
    voice.modulations.trigger = triggerNow ? 1.0f : 0.0f;
    voice.modulations.trigger_patched = true;
  }
  else if (is_wave_terrain || is_chiptune || is_drum) {
    voice.patch.decay = (is_drum ? 0.6f : 0.7f);
    voice.modulations.trigger = triggerNow ? 1.0f : 0.0f;
    voice.modulations.trigger_patched = true;
    voice.modulations.level = 1.0f;
    voice.modulations.level_patched = true;
  }
  else {
    voice.patch.decay = 0.2f;
    voice.modulations.trigger = 1.0f;
    voice.modulations.trigger_patched = false;
    voice.modulations.level = 1.0f;
    voice.modulations.level_patched = true;
  }

}

void setup() {
  if (debug) {
    Serial.begin(57600);
    Serial.println(F("YUP"));
  }

  // set up Pico PWM audio output
  DAC.setBuffers(4, 32); // DMA buffers
  //DAC.onTransmit(cb);
  DAC.setFrequency(SAMPLERATE);
  DAC.begin();

  // thi is to switch to PWM for power to avoid ripple noise
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);

  voice.voice_ = new plaits::Voice;
  voice.pitch = 48;
  delay(4000);
  changeEngine(0);

}

void loop() {
  
    // first render some audio
    voice.voice_->Render(voice.patch, voice.modulations, voice.out_buffer, AUDIO_BLOCK);
    // push it to the dac.
    for (int i = 0; i < AUDIO_BLOCK; i++)
      DAC.write(voice.out_buffer[i].out);

}

// second core dedicated to display foo

void setup1() {
  delay (200); // wait for main core to start up perhipherals
}


// second core deals with ui / control rate updates
void loop1() {

  engineInc++ ;
  if (engineInc > 4) {
    engineCount ++; // don't switch engine so often :)
    engineInc = 0;
    voice.patch.engine = engineCount;
  }
  if (engineCount > 24) engineCount = 0;

  float trigger = randomDouble(0.0, 1.0); // Dust.kr( LFNoise2.kr(0.1).range(0.1, 7) );
  float harmonics = randomDouble(0.0, 0.8); // SinOsc.kr(0.03, 0, 0.5, 0.5).range(0.0, 1.0);
  float timbre = randomDouble(0.0, 0.8); //LFTri.kr(0.07, 0, 0.5, 0.5).range(0.0, 1.0);
  float morph = randomDouble(0.1, 0.8) ; //LFTri.kr(0.11, 0, 0.5, 0.5).squared;
  float pitch = randomDouble(42, 64); // TIRand.kr(24, 48, trigger);
  float octave = randomDouble(0.2, 0.4);
  float decay = randomDouble(0.1, 0.4);

  //octave_in = octave;
  pitch_in = pitch;
  harm_in = harmonics;
  morph_in = morph;
  timbre_in = timbre;

  changeEngine(engineCount);
  updateAudio(engineCount, (trigger_in > 0.1), 0.4f);
  
  delay(3000);


}
