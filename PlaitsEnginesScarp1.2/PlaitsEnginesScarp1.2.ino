/*
  (c) 2024 blueprint@poetaster.de
  GPLv3

      Some sources, including the stmlib and plaits lib are
      MIT License
      Copyright (c)  2020 (emilie.o.gillet@gmail.com)
*/

bool debug = false;

#include <Arduino.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <hardware/pwm.h>

#include <PWMAudio.h>
#define SAMPLERATE 48000
#define PWMOUT 22
PWMAudio DAC(PWMOUT);  // 16 bit PWM audio

#include "Midier.h"
#include "names.h"

// button inputs
#define BUTTON0  0 // key1 input on schematic
#define BUTTON1  2
#define BUTTON2 4
#define BUTTON3 6
#define BUTTON4 8
#define BUTTON5 10
#define BUTTON6 12
#define BUTTON7 14  // for the Pico board
#define SHIFTBUTTON 28 // was 24 USR button on VCC-GND boards

#define LED0 1 // LED1 output on schematic
#define LED1 3
#define LED2 5
#define LED3 7
#define LED4 9
#define LED5 11
#define LED6 13
#define LED7 15
#define LED 15 // shows if MIDI is being recieved
// analog freq pins

#include <MIDI.h>
#include <Wire.h>

//MIDI_CREATE_DEFAULT_INSTANCE();

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

// ==================== UTIL ====================

inline float softClip(float x) {
  if (x > 1.5f) x = 1.5f;
  if (x < -1.5f) x = -1.5f;
  return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

// Plaits modulation vars
float morph_in = 0.7f; // IN(4);
float trigger_in = 0.0f; //IN(5);
float level_in = 0.0f; //IN(6);
float harm_in = 0.5f;
float timbre_in = 0.1f;
int engine_in;

float fm_mod = 0.0f ; //IN(7);
float timb_mod = 0.0f; //IN(8);
float morph_mod = 0.0f; //IN(9);
float decay_in = 0.5f; // IN(10);
float lpg_in = 0.2f ;// IN(11);
float pitch_in = 60.0f;

// clock timer  stuff
//unsigned int SWPin = CLOCKIN;

int pressedB = 0;

// GPIO, pots
const int INTS_PIN = 26; // set the analog input for fm_intensity
const int RATE_PIN = 2; // set the analog input for mod rate
const int MODR_PIN = 27; // set the analog input for mod ratio

// encoder related
#include <RotaryEncoder.h>
const int encoderA_pin = 19;
const int encoderB_pin = 18;
const int encoderSW_pin = 28;

RotaryEncoder encoder(encoderB_pin, encoderA_pin, RotaryEncoder::LatchMode::FOUR3);

void checkEncoderPosition() {
  encoder.tick();   // call tick() to check the state.
}

// display related
const int oled_sda_pin = 20;
const int oled_scl_pin = 21;
const int oled_i2c_addr = 0x3C;
const int dw = 128;
const int dh = 64;

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
//Adafruit_SSD1306 display(dw, dh, &Wire, OLED_RESET);
//#include <Adafruit_SSD1306.h>

#include <Adafruit_SH110X.h>
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire);
#define WHITE SH110X_WHITE

#include "font.h"
#include "helvnCB6pt7b.h"
#define myfont helvnCB6pt7b // Org_01 looks better but is small.
#include "display.h"

// variables for UI state management
int encoder_pos_last = 0;
int encoder_delta = 0;
uint32_t encoder_push_millis;
uint32_t step_push_millis;
bool encoder_held = false;


// buttons & knobs defines/functions
#include "control.h"

// midi related functions
#include "midi.h"

// utility
double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

// audio related defines


int current_track;
int bpm = 120;

uint16_t noteA;
uint16_t noteM;
uint16_t attackIn = 8; // up to 384
uint16_t durationIn = 200; // up to 384
uint16_t modIn = 200; // up to 2800
uint16_t modC = 0;


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
    voice.patch.decay = 0.9f;     // Internal VCA/LPG release time
    voice.modulations.level = 1.0f;
    voice.modulations.level_patched = true;
    voice.modulations.trigger = triggerNow ? 1.0f : 0.0f;
    voice.modulations.trigger_patched = true;
  }
  else if (is_wave_terrain || is_chiptune || is_drum) {
    voice.patch.decay = (is_drum ? 0.7f : 0.8f);
    voice.modulations.trigger = triggerNow ; // ? 1.0f : 0.0f;
    voice.modulations.trigger_patched = true;
    voice.modulations.level = 1.0f;
    voice.modulations.level_patched = true;
  }
  else {
    voice.patch.decay = 0.2f;
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
  DAC.setBuffers(4, 32); //plaits::kBlockSize * 4); // DMA buffers
  //DAC.onTransmit(cb);
  DAC.setFrequency(SAMPLERATE);
    // now start the dac
  DAC.begin();

  // 4096
  analogReadResolution(12);
  
  // Additions
  // ENCODER
  pinMode(encoderA_pin, INPUT_PULLUP);
  pinMode(encoderB_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA_pin), checkEncoderPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_pin), checkEncoderPosition, CHANGE);
  
  // DISPLAY
  Wire.setSDA(oled_sda_pin);
  Wire.setSCL(oled_scl_pin);
  Wire.begin();

  // SSD1306 --  or SH1106 in this case
  //if (!display.begin(SSD1306_SWITCHCAPVCC, oled_i2c_addr)) {
  if (!display.begin( oled_i2c_addr)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) ;  // Don't proceed, loop forever
  }
  displaySplash();

  pinMode(BUTTON0, INPUT_PULLUP);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(BUTTON4, INPUT_PULLUP);
  pinMode(BUTTON5, INPUT_PULLUP);
  pinMode(BUTTON6, INPUT_PULLUP);
  pinMode(BUTTON7, INPUT_PULLUP);
  pinMode(SHIFTBUTTON, INPUT_PULLUP);

  pinMode(26, INTS_PIN);
  pinMode(27, MODR_PIN);

  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);

  pinMode(LED, OUTPUT);

  pinMode(23, OUTPUT); // thi is to switch to PWM for power to avoid ripple noise
  digitalWrite(23, HIGH);

  // init the plaits voices

  // initialize a mode to play
  mode = midier::Mode::Ionian;
  makeScale( roots[scaleRoot], mode);
  
  voice.voice_ = new plaits::Voice;
  voice.pitch = 48;
  delay(2000);
  changeEngine(0);



}


void loop() {
    // first render some audio
    voice.voice_->Render(voice.patch, voice.modulations, voice.out_buffer, AUDIO_BLOCK);
    // push it to the dac.
    for (int i = 0; i < AUDIO_BLOCK; i++)
      DAC.write(voice.out_buffer[i].out);
}



void updateControl() {

  //MIDI.read();

  uint32_t now = millis();
  bool anybuttonpressed;
  anybuttonpressed = false;

  // use the debounced values from readpots
  int p1 = potvalue[0]; //analogRead(MODR_PIN); // value is 0-4065
  int p2 = potvalue[1]; // analogRead(INTS_PIN); // value is 0-4065


  morph_in = (float)p1 / 4095.0f; //map(p1, 0, 4065, 0.0, 1.0); // IN(2);
  harm_in = (float)p2 / 4095.0f; //map(p2, 0, 4065, 0.0, 1.0); //IN(3);
  CONSTRAIN(morph_in, 0.0f, 1.0f);
  CONSTRAIN(harm_in, 0.0f, 1.0f);
  scanbuttons();


  for (int i = 0; i < 9; ++i) { // scan all the buttons
    if (button[i]) {

      anybuttonpressed = true;
      if (i < 8) {
        digitalWrite(led[i] , HIGH);
        // a track button is pressed
        current_track = i; // keypress selects track we are working on

        if ( encoder_delta == 0) {
          aNoteOff(currentMode[i], 0);
          if (button[8]) scaleRoot = i; // change scaleroot if both encoder and another button is pressed.
          pitch_in = currentMode[i]; //freqs[i];
          aNoteOn( pitch_in, 100 );
        }
        pressedB = i;
      } else {
        modeIndex = modeIndex + 1;
        if (modeIndex == 8 ) modeIndex = 0;

        switch (modeIndex) {
          case 0:
            mode = midier::Mode::Ionian;
            break;
          case 1:
            mode = midier::Mode::Dorian;
            break;
          case 2:
            mode = midier::Mode::Phrygian;
            break;
          case 3:
            mode = midier::Mode::Lydian;
            break;
          case 4:
            mode = midier::Mode::Mixolydian;
            break;
          case 5:
            mode = midier::Mode::Aeolian;
            break;
          case 6:
            mode = midier::Mode::Locrian;
            break;
        }
        makeScale( roots[scaleRoot], mode);
      }
    }
  }
}


// second core dedicated to display foo

void setup1() {
  delay (1000); // wait for main core to start up perhipherals
}

int engineCount = 0;
int engineInc = 0;

// second core deals with ui / control rate updates
void loop1() {

  updateControl(); //pots values on this loop since the other delays

  uint32_t now = millis();
  bool anybuttonpressed;
  anybuttonpressed = false;

  // UI handlers
  // first encoder
  encoder.tick();

  int encoder_pos = encoder.getPosition();
  if ( (encoder_pos != encoder_pos_last )) {
    encoder_delta = encoder_pos - encoder_pos_last;
  }
  // set play mode 0 play 1 edit patterns, 3 FX?
  if (encoder_push_millis > 0 ) {
    if ((now - encoder_push_millis) > 25 && ! encoder_delta ) {

      if ( !encoder_held ) encoder_held = true;

      display_mode = display_mode + 1;
      if ( display_mode > 2) { // switched back to play mode
        display_mode = 0;
        //configure_sequencer();
      }
    }

    if (step_push_millis > 0) { // we're pushing a step key too
      if (encoder_push_millis < step_push_millis) {  // and encoder was pushed first
        //strcpy(seq_info, "saveseq");
      }
    }
  }



  for (int i = 0; i < 9; ++i) { // scan all the buttons
    if (button[i]) {

      anybuttonpressed = true;
      if (i < 8)  digitalWrite(led[i] , HIGH);
      /*
        // a track button is pressed
        current_track = i; // keypress selects track we are working on

        if ( encoder_delta == 0) {
        //if (pressedB != i) {
        // turn off the last note
        //aNoteOff(freqs[pressedB],0);
        //}
        aNoteOff(freqs[pressedB], 0);
        noteA = freqs[i];
        pitch_in = freqs[i] + 24.0f;
        voices[0].patch.note = 24.0f + pitch; // 60.f + pitch * 12.f;
        aNoteOn( freqs[i], 100 );
        }
        pressedB = i;
      */

      //  if ((!potlock[1]) || (!potlock[2])) seq[i].trigger=euclid(16,map(potvalue[1],POT_MIN,POT_MAX,0,MAX_SEQ_STEPS),map(potvalue[2],POT_MIN,POT_MAX,0,MAX_SEQ_STEPS-1));
      if ( i == 8 && encoder_delta != 0) {
        engineCount = engineCount + encoder_delta;
        if (engineCount > 24) engineCount = 0; // circle around
        if (engineCount < 0) engineCount = 24;
        if (engineCount == 23) CONSTRAIN(timbre_in, 0.f,0.33f); // string machine bug fix
        engine_in = engineCount; // ( engine +) % voices[0].voice_.GetNumEngines();
        changeEngine(engineCount);
      }

    } else {
      if (i < 8) digitalWrite(led[i] , LOW); // else the button is off.
    }
  }

  // now, after buttons check if only encoder moved and no buttons
  // this is broken by mozzi, sigh.
  if (! anybuttonpressed && encoder_delta) {
    //bpm = bpm + encoder_delta;
    //RATE_value = RATE_value + encoder_delta;
    //voices[0].patch.note = voices[0].patch.note + RATE_value;

    float turn = ( encoder_delta * 0.005f ) + timbre_in;
    CONSTRAIN(turn, 0.f, 1.0f)
    if (engineCount == 23) {
      CONSTRAIN(turn, 0.f,0.33f);
    }
    timbre_in = turn;
    //display_value(RATE_value - 50); // this is wrong, bro :)
  }


  /// only set new pos last after buttons have had a chance to use the delta
  encoder_pos_last = encoder_pos;
  encoder_delta = 0;  // we've used it

  // start tracking time encoder button held
  if ( ! digitalRead( SHIFTBUTTON ) ) {
    encoder_push_millis = now;
  } else {
    encoder_push_millis = 0;
    encoder_held = false;
  }

  // lock pot settings when no keys are pressed so it requires more movement to change value
  // this is so when we change tracks we don't immediately change the settings on the new track
  if (!anybuttonpressed) lockpots();

  // reading A/D seems to cause noise in the audio so don't do it too often
  if ((now - pot_timer) > POT_SAMPLE_TIME) {
    readpot(0);
    readpot(1);
    pot_timer = now;
  }

  displayUpdate();
  // don't know why here :)
  voice.patch.engine = engine_in;
  
  updateAudio(engine_in, (trigger_in > 0.1), 0.4f);


}



void automatic() {
  if (engineCount > 16) engineCount = 0;

  /*
    float trigger = randomDouble(0.0, 2.0); // Dust.kr( LFNoise2.kr(0.1).range(0.1, 7) );
    float harmonics = randomDouble(0.2, .7); // SinOsc.kr(0.03, 0, 0.5, 0.5).range(0.0, 1.0);
    float timbre = randomDouble(0.1, .6); //LFTri.kr(0.07, 0, 0.5, 0.5).range(0.0, 1.0);
    float morph = randomDouble(0.2, 0.6) ; //LFTri.kr(0.11, 0, 0.5, 0.5).squared;
    float pitch = abs(randomDouble(34, 60)); // TIRand.kr(24, 48, trigger);
    float octave = randomDouble(0.3, 0.5);
    float decay = randomDouble(0.05, 0.2);
    float fm_mod = randomDouble(0.05, 0.2);
    float lpg = randomDouble(0.05, 0.2);

    //voices[0].patch.frequency_modulation_amount = fm_mod;
    voices[0].patch.engine = engineCount;
    //voices[0].transposition_ = 0.;
    voices[0].octave_ = 0.3;
    //voices[0].patch.note = pitch;
    //voices[0].patch.harmonics = harmonics;
    //voices[0].patch.morph = morph;
    //voices[0].patch.timbre = timbre;
    voices[0].patch.decay = decay; //0.5f;
    voices[0].patch.lpg_colour = lpg;

    if (trigger > 0.2 ) {
    voices[0].modulations.trigger = trigger;
    voices[0].modulations.trigger_patched = true;
    } else {
    voices[0].modulations.trigger = 0.0f;
    voices[0].modulations.trigger_patched = false;
    }

    engineInc++ ;
    if (engineInc > 5) {
    engineCount ++; // don't switch engine so often :)
    engineInc = 0;
    }

  */
}
