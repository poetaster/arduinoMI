/* 
  This is example code of Mutable Instruments Plaits 1.2 and Clouds Parasites firmware reverb
  running on Raspberry Pi Pico RP2350 with I2S DAC.

  Program cycles through clean Plaits engines -> Clouds reverb -> Oliverb fx
  An additional DX7 SysEx bank has been added for the OP6 engine, making a total of 25 engines available.
  
  Set:
  
  CPU 276mhz Overclock
  Optimize Even More -O3
  
  Select TinyUSB stack in ArduinoIDE tools for easier adjusments test


  Copyright (c) 2025 Vadims Maksimovs 
  https://github.com/ledlaux
  MIT licence
  
  --------------------------------------------------------------------------------
  Arduino Port 
  --------------------------------------------------------------------------------
 
  This code is part of the porting project of Mutable Instruments eurorack modules to Arduino by Mark Washeim
  Main repository: https://github.com/poetaster/arduinoMI

  --------------------------------------------------------------------------------
  Original Mutable Instruments Code
  --------------------------------------------------------------------------------
  Copyright (c) 2020 Emilie O. Gillet 
  stmlib, Plaits, Clouds libraries
  MIT licence

  --------------------------------------------------------------------------------
  Clouds Parasites Firmware
  --------------------------------------------------------------------------------
  Authored by Matthias Puech
  https://mqtthiqs.github.io/
  MIT licence
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <I2S.h>
#include <STMLIB.h>
#include <PLAITS.h>
#include <CLOUDS.h>
#include <pico/stdlib.h>

#define I2S_DATA_PIN 9
#define I2S_BCLK_PIN 10
#define SAMPLE_RATE 32000
I2S i2s_output(OUTPUT);

#define WORKSPACE_SIZE 32768
constexpr int AUDIO_BLOCK = plaits::kBlockSize;

// ==================== MEMORY BUFFERS ====================
static uint8_t shared_plaits_workspace[WORKSPACE_SIZE] __attribute__((aligned(4)));
static uint16_t clouds_buffer[65536];
static uint16_t oliverb_buffer[131072];

int16_t left[AUDIO_BLOCK];
int16_t right[AUDIO_BLOCK];

clouds::Reverb clouds_fx;
clouds::Oliverb oliverb_fx;

const char* engine_names[] = {
  "Virtual Analog", "Waveshaping", "FM", "Grain", "Additive", "Wavetable", "Chord", "Speech",
  "Swarm", "Noise", "Particle", "String", "Modal", "Bass Drum", "Snare Drum", "Hi-Hat",
  "Six OP 0", "Six OP 1", "Six OP 2", "Six OP 3", "Virtual Analog VCF", "Phase Distortion", "Wave Terrain",
  "String Machine", "Chiptune"
};

struct Voice {
  plaits::Voice* voice_;
  plaits::Patch patch;
  plaits::Modulations modulations;
  plaits::Voice::Frame out_buffer[AUDIO_BLOCK];
  int pitch;
};

Voice voice;

// ==================== UTIL ====================

inline float softClip(float x) {
  if (x > 1.5f) x = 1.5f;
  if (x < -1.5f) x = -1.5f;
  return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

void applyFxSettings() {
  clouds_fx.set_amount(0.9f);
  clouds_fx.set_input_gain(0.3f);
  clouds_fx.set_time(0.6f);
  clouds_fx.set_diffusion(0.625f);
  clouds_fx.set_lp(0.7f);

  oliverb_fx.set_size(0.8f);
  oliverb_fx.set_decay(2.0f);
  oliverb_fx.set_lp(0.6f);
  oliverb_fx.set_hp(0.1f);
  oliverb_fx.set_input_gain(0.4f);
  oliverb_fx.set_diffusion(0.7f);
  oliverb_fx.set_pitch_shift_amount(0.5f);
  oliverb_fx.set_mod_amount(0.02f);
  oliverb_fx.set_mod_rate(0.1f);
}

void changeEngine(uint8_t engine_idx) {
  Serial.print("Switching to Engine [");
  Serial.print(engine_idx);
  Serial.print("]: ");
  if (engine_idx < 25) Serial.println(engine_names[engine_idx]);

  // Reuse workspace 
  memset(shared_plaits_workspace, 0, WORKSPACE_SIZE);
  stmlib::BufferAllocator allocator(shared_plaits_workspace, WORKSPACE_SIZE);
  voice.voice_->Init(&allocator);

  // Flush FX buffers to stop feedback/tails
  memset(clouds_buffer, 0, sizeof(clouds_buffer));
  memset(oliverb_buffer, 0, sizeof(oliverb_buffer));
  clouds_fx.Init(clouds_buffer);
  oliverb_fx.Init(oliverb_buffer);
  applyFxSettings();
}

void updateAudio(uint8_t engine_idx, bool applyClouds, bool applyOliverb, bool triggerNow, float master_volume = 0.3f) {
  static clouds::FloatFrame fxBuffer[AUDIO_BLOCK];

  bool is_drum = (engine_idx >= 13 && engine_idx <= 15);
  bool is_wave_terrain = (engine_idx == 22); 
  bool is_string_machine = (engine_idx == 23);
  bool is_chiptune = (engine_idx == 24);

  voice.patch.engine = engine_idx;
  voice.patch.note = (float)voice.pitch;
  
  // Default Parameter Settings
  voice.patch.harmonics = 0.4f;
  voice.patch.timbre = 0.4f;
  voice.patch.morph = 0.4f;

  if (is_string_machine) {
    voice.patch.harmonics = 0.5f; // Selects Chord (Major/Minor/etc.)
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

  voice.voice_->Render(voice.patch, voice.modulations, voice.out_buffer, AUDIO_BLOCK);

  for (int i = 0; i < AUDIO_BLOCK; i++) {
    fxBuffer[i].l = voice.out_buffer[i].out / 32768.0f;
    fxBuffer[i].r = voice.out_buffer[i].aux / 32768.0f;
  }

  if (applyClouds) clouds_fx.Process(fxBuffer, AUDIO_BLOCK);
  if (applyOliverb) oliverb_fx.Process(fxBuffer, AUDIO_BLOCK);

  for (int i = 0; i < AUDIO_BLOCK; i++) {
    left[i] = (int16_t)(softClip(fxBuffer[i].l * master_volume) * 32767.0f);
    right[i] = (int16_t)(softClip(fxBuffer[i].r * master_volume) * 32767.0f);
  }
}

void setup() {
  Serial.begin(115200);

  i2s_output.setFrequency(SAMPLE_RATE);
  i2s_output.setDATA(I2S_DATA_PIN);
  i2s_output.setBCLK(I2S_BCLK_PIN);
  i2s_output.setBitsPerSample(16);
  i2s_output.setBuffers(8, AUDIO_BLOCK);
  i2s_output.begin();
  
  voice.voice_ = new plaits::Voice;
  voice.pitch = 48;
  delay(4000);
  changeEngine(0);
  
}

void loop() {
  
  static const uint32_t fxDuration = 2000;
  static const uint32_t pauseDelay = 1000;

  static uint32_t lastTime = 0;
  static uint8_t playCount = 0;  // 0=Clean, 1=Clouds, 2=Oliverb
  static uint8_t engine_idx = 0;
  static bool inPause = false;
  static bool fxJustSwitched = true;

  uint32_t now = millis();

  if (inPause) {
    if (now - lastTime >= pauseDelay) {
      inPause = false;
      lastTime = now;
      fxJustSwitched = true;
    } else {
      for (int i = 0; i < AUDIO_BLOCK; i++) i2s_output.write16(0, 0);
      return;
    }
  }

  if (now - lastTime < fxDuration) {
    bool applyClouds = (playCount == 1);
    bool applyOliverb = (playCount == 2);
    updateAudio(engine_idx, applyClouds, applyOliverb, fxJustSwitched, 0.4f);
    fxJustSwitched = false;
  } else {
    lastTime = now;
    playCount++;
    if (playCount > 2) {
      playCount = 0;
      engine_idx++;
      if (engine_idx >= 25) engine_idx = 0;
      changeEngine(engine_idx);
    }
    inPause = true;
    fxJustSwitched = true;
  }

  for (int i = 0; i < AUDIO_BLOCK; i++)
    i2s_output.write16(left[i], right[i]);
}
