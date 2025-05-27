/*
  (c) 2025 blueprint@poetaster.de GPLv3

   Based on  mi_Ugens, Copyright (c) 2020 Volker Böhm. All rights reserved. GPLv3
   https://vboehm.net

   Mutable Instruments sources, including the stmlib, tides, plaits elements and braids libs are
   MIT License
   Copyright (c)  2020 (emilie.o.gillet@gmail.com)
*/

bool debugging = true;

#include <Arduino.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <hardware/pwm.h>

#include <PWMAudio.h>
#define SAMPLERATE 32000
#define PWMOUT 22
PWMAudio DAC(PWMOUT);  // 16 bit PWM audio

#include "Midier.h"

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

// rings dsp

#include <STMLIB.h>
#include <ELEMENTS.h>

float elements::Dsp::kSampleRate = 32000.0f;
float elements::Dsp::kSrFactor = 32000.0f / kSampleRate;
float elements::Dsp::kIntervalCorrection = logf(kSrFactor) / logf(2.0f) * 12.0f;

const size_t kBlockSize = elements::kMaxBlockSize;


struct Unit {

  elements::Part              *part;
  elements::PerformanceState  ps;
  elements::Patch             *p;

  float               *blow_in;
  float               *strike_in;
  float               *silence;

  uint16_t            *reverb_buffer;
  float               *out, *aux;     // output buffers
  double              sr;
  long                sigvs;

  short               blockCount;

  // we're using the original rendering, not vbs
  int16_t                 obuff[kBlockSize]; // from float[]s below in update routine
  int16_t                 abuff[kBlockSize]; // ditto

};

struct Unit voices[1];

// Plaits modulation vars
float morph_in = 0.7f; // IN(4);
float trigger_in; //IN(5);
float level_in = 0.0f; //IN(6);
float harm_in = 0.1f;
float timbre_in = 0.1f;
int engine_in;

float fm_mod = 0.0f ; //IN(7);
float timb_mod = 0.0f; //IN(8);
float morph_mod = 0.0f; //IN(9);
float decay_in = 0.5f; // IN(10);
float lpg_in = 0.1f ;// IN(11);
int pitch_in = 60;


// Braids vars
//    float   voct_in = IN0(0);
//    float   timbre_in = IN0(1);
//    float   color_in = IN0(2);
//    float   model_in = IN0(3);
//    float   *trig_in = IN(4);

// clock timer  stuff

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     4

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RPi_Pico_TimerInterrupt.h"

//unsigned int SWPin = CLOCKIN;

#define TIMER0_INTERVAL_MS 31.25 // SAMPLERATE 
// \20.833333333333running at 48Khz
#define DEBOUNCING_INTERVAL_MS   2// 80
#define LOCAL_DEBUG              0

volatile int counter = 0;

// Init RPI_PICO_Timer, can use any from 0-15 pseudo-hardware timers
RPI_PICO_Timer ITimer0(0);

bool TimerHandler0(struct repeating_timer *t) {
  (void) t;
  bool sync = true;
  if ( DAC.availableForWrite() ) {
    //DAC.write( voices[0].obuff[counter] , sync );
    for (size_t i = 0; i < kBlockSize; i++) {
      DAC.write( voices[0].obuff[i] );
    }
    counter =  1;
  }
  return true;
}

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
#include "shapes.h" // char array of names of instruments.
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

//float freqs[12] = { 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f, 493.88f};
int freqs[12] = { 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64};
int carrier_freq;

int current_track;
int bpm = 120;

uint16_t noteA;
uint16_t noteM;
uint16_t attackIn = 8; // up to 384
uint16_t durationIn = 200; // up to 384
uint16_t modIn = 200; // up to 2800
uint16_t modC = 0;
int pressedB;
int RATE_value = 1 ; //was an adc in kevin's orig


void setup() {
  if (debugging) {
    Serial.begin(57600);
    Serial.println(F("YUP"));
  }
  // pwm timing setup
  // we're using a pseudo interrupt for the render callback since internal dac callbacks crash
  // Frequency in float Hz
  //ITimer0.attachInterrupt(TIMER_FREQ_HZ, TimerHandler0);
  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS, TimerHandler0)) // that's 48kHz
  {
    if (debugging) Serial.print(F("Starting  ITimer0 OK, millis() = ")); Serial.println(millis());
  }  else {
    if (debugging) Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
  }

  // set up Pico PWM audio output
  DAC.setBuffers(4, 32); // DMA buffers
  //DAC.onTransmit(cb);
  DAC.setFrequency(SAMPLERATE);
  DAC.begin();

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

  // init the elements voice
  initVoices();

  // initialize a mode to play
  mode = midier::Mode::Ionian;
  makeScale( roots[scaleRoot], mode);

  // Connect the HandleNoteOn function to the library, so it is called upon reception of a NoteOn.
  //MIDI.setHandleNoteOn(HandleNoteOn);  // Put only the name of the function
  //MIDI.setHandleNoteOff(HandleNoteOff);  // Put only the name of the function
  //MIDI.begin(MIDI_CHANNEL_OMNI);


}

// initialize voice parameters
void initVoices() {

  elements::Dsp::setSr(SAMPLERATE);


  // allocate memory
  voices[0].reverb_buffer = (uint16_t*)malloc(32768 * sizeof(uint16_t));

  if (voices[0].reverb_buffer == NULL) {
    Serial.println("MiElements ERROR: mem alloc failed!\n");
  }

  voices[0].out = (float *)malloc(kBlockSize * sizeof(float));
  voices[0].aux = (float *)malloc(kBlockSize * sizeof(float));
  voices[0].silence = (float *)malloc(kBlockSize * sizeof(float));

  memset(voices[0].silence, 0, kBlockSize * sizeof(float));


  // Init and seed the random parameters and generators with the serial number.
  voices[0].part = new elements::Part;
  memset(voices[0].part, 0, sizeof(*voices[0].part));

  voices[0].part->Init(voices[0].reverb_buffer);

  uint32_t mySeed = 0x1fff7a10;
  voices[0].part->Seed(&mySeed, 3);

  voices[0].part->set_easter_egg(false);
  voices[0].p = voices[0].part->mutable_patch();

  voices[0].blockCount = 0;

  voices[0].p->exciter_envelope_shape = 0.1f;
  voices[0].p->exciter_bow_level = 0.f;
  voices[0].p->exciter_blow_level = 0.f;
  voices[0].p->exciter_strike_level = 0.5f;
  voices[0].p->exciter_blow_meta = 0.0f;
  voices[0].p->exciter_strike_meta = 0.5f;
  voices[0].p->exciter_bow_timbre = 0.f;
  voices[0].p->exciter_blow_timbre = 0.f;
  voices[0].p->exciter_strike_timbre = 0.5f;
  voices[0].p->resonator_geometry = 0.2f;
  voices[0].p->resonator_brightness = 0.3f;
  voices[0].p->resonator_damping = 0.7f;
  voices[0].ps.strength = 0.5f;
  voices[0].ps.gate = 0;

  voices[0].part->set_resonator_model(elements::RESONATOR_MODEL_STRING);
  // generate some samples
  updateElementsAudio();

}

void cb() {
  if (DAC.availableForWrite() > 14) {
    for (int i = 0; i <  kBlockSize; i++) {
      // out = ;   // left channel called .aux
      DAC.write( voices[0].obuff[i] );
    }
  }
}

/*
  void audioOutput() {
  voices[0].voice_->Render(voices[0].patch, voices[0].modulations,  outputPlaits,  plaits::kBlockSize);

  for (int i = 0; i <  plaits::kBlockSize; i++) {
    uint16_t out = outputPlaits[i].out;
    if ( DAC.availableForWrite() ) DAC.write( out );
  }
  // write samples to DMA buffer - this is a blocking call so it stalls when buffer is full
  // f.l() + MOZZI_AUDIO_BIAS
  // left
  }

  bool canBufferAudioOutput() {

  if ( DAC.availableForWrite() > 32 ) {
    return true;
  }
  return false;
  }


  bool updateAudio() {
  if (canBufferAudioOutput()) {
    audioOutput();
    return true;
  }
  return false;

  }
*/

bool canBufferAudioOutput() {
  if ( DAC.availableForWrite() > 32 ) {
    return true;
  }
  return false;
}
void updateControl() {

  //MIDI.read();

  uint32_t now = millis();
  bool anybuttonpressed;
  anybuttonpressed = false;

  int p1 = potvalue[0]; //analogRead(MODR_PIN); // value is 0-4065
  int p2 = potvalue[1]; // analogRead(INTS_PIN); // value is 0-4065


  morph_in = (float)p1 / 1000.0f; //map(p1, 0, 4065, 0.0, 1.0); // IN(2);
  timbre_in = (float)p2 / 1000.0f; //map(p2, 0, 4065, 0.0, 1.0); //IN(3);
  CONSTRAIN(morph_in, 0.0f, 1.0f);
  CONSTRAIN(timbre_in, 0.0f, 1.0f);
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
          //noteA = freqs[i];
          if (button[8]) scaleRoot = i; // change scaleroot if both encoder and another button is pressed.
          
          pitch_in = currentMode[i] + 24; //freqs[i];
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





void updateElementsAudio() {
  float   *in0;// = IN(0);
  float   *in1;// = IN(1);
  float   *gate_in;// = IN(2);

  float   voct_in = pitch_in; // IN0(3);
  float   strength = 0.7f; //IN0(4);
  float   contour = 0.9f ; //harm_in; // IN0(5); // envelope shape
  float   bow_level = 0.f; // IN0(6);
  float   blow_level = 0.f; // IN0(7);
  float   strike_level = 0.9f; // IN0(8);
  float   flow = 0.f; //IN0(9); blow meta
  float   mallet = 0.5f ; //IN0(10); // strike meta
  float   bow_timbre = timbre_in; //IN0(11);
  float   blow_timbre = timbre_in; //IN0(12);
  float   strike_timbre = timbre_in; // IN0(13);
  float   geometry = harm_in + 0.5f; // IN0(14);
  float   brightness = 0.3f; // IN0(15);
  float   damping = harm_in + 0.1f; // IN0(16);
  float   position = 0.5f; // IN0(17);
  float   space = morph_in + 0.01f; //IN0(18);
  int     model = engine_in; //2.f; //IN0(19);
  bool    easter_egg = false; //0.f; //IN0(20) > 0.f;


  /*
      voices[0].p->exciter_envelope_shape = 0.1f;
      voices[0].p->exciter_bow_level = 0.f;
      voices[0].p->exciter_blow_level = 0.f;
      voices[0].p->exciter_strike_level = 0.5f;
      voices[0].p->exciter_blow_meta = 0.0f;
      voices[0].p->exciter_strike_meta = 0.5f;
      voices[0].p->exciter_bow_timbre = 0.f;
      voices[0].p->exciter_blow_timbre = 0.f;
      voices[0].p->exciter_strike_timbre = 0.5f;
      voices[0].p->resonator_geometry = 0.2f;
      voices[0].p->resonator_brightness = 0.3f;
      voices[0].p->resonator_damping = 0.7f;
      voices[0].ps.strength = 0.5f;
      voices[0].ps.gate = 0;*/

  size_t  size = elements::kMaxBlockSize;

  float   *blow_in = voices[0].blow_in;
  float   *strike_in = voices[0].strike_in;
  float   *out = voices[0].out;
  float   *aux = voices[0].aux;

  elements::PerformanceState ps = voices[0].ps;
  elements::Patch            *p = voices[0].p;
  
  // set resonator model:
  // RESONATOR_MODEL_MODAL, RESONATOR_MODEL_STRING, RESONATOR_MODEL_STRINGS
  CONSTRAIN(model, 0, 2);
  voices[0].part->set_resonator_model(static_cast<elements::ResonatorModel>(model));

  // set pitch
  CONSTRAIN(voct_in, 8.f, 120.f);
  ps.note = voct_in;

  CONSTRAIN(strength, 0.f, 1.f);
  ps.strength = strength;

  // set level params
  CONSTRAIN(contour, 0.0f, 0.9995f);
  p->exciter_envelope_shape = contour;

  CONSTRAIN(bow_level, 0.0f, 0.9995f);
  p->exciter_bow_level = bow_level;

  CONSTRAIN(blow_level, 0.0f, 0.9995f);
  p->exciter_blow_level = blow_level;

  CONSTRAIN(strike_level, 0.0f, 0.9995f);
  p->exciter_strike_level = strike_level;

  // set meta params
  CONSTRAIN(flow, 0.0f, 0.9995f);
  p->exciter_blow_meta = flow;

  CONSTRAIN(mallet, 0.0f, 0.9995f);
  p->exciter_strike_meta = mallet;

  // set timbre params
  CONSTRAIN(bow_timbre, 0.0f, 0.9995f);
  p->exciter_bow_timbre = bow_timbre;

  CONSTRAIN(blow_timbre, 0.0f, 0.9995f);
  p->exciter_blow_timbre = blow_timbre;

  // set resonator params
  CONSTRAIN(geometry, 0.0f, 0.9995f);
  p->resonator_geometry = geometry;

  CONSTRAIN(brightness, 0.0f, 0.9995f);
  p->resonator_brightness = brightness;

  CONSTRAIN(damping, 0.0f, 0.9995f);
  p->resonator_damping = damping;

  CONSTRAIN(position, 0.0f, 0.9995f);
  p->resonator_position = position;

  CONSTRAIN(space, 0.0f, 0.9995f);
  p->space = space ; //* 1.11f; // vb, use the last bit to trigger rev freeze


  voices[0].part->set_easter_egg(easter_egg);

  // gate input
  ps.gate = trigger_in > 0.f; // gate_in[0] > 0.f;
  /*
    if(INRATE(2) == calc_FullRate) {
      float sum = 0.f;
      for(int i=0; i<inNumSamples; ++i) {
          sum += gate_in[i];
      }
      ps.gate = sum > 0.001f;
    }
    else
      ps.gate = gate_in[0] > 0.f;
  */

  blow_in = voices[0].silence;
  strike_in= voices[0].silence;
  // check input rates
  /*
    if(INRATE(0) == calc_FullRate)
      blow_in = in0;
    else
      blow_in = voices[0].silence;

    if(INRATE(1) == calc_FullRate)
      strike_in = in1;
    else
      strike_in = voices[0].silence;
  */

  // input and output can't be the same arrays
/*
  for (size_t count = 0; count < size; count += size) {

    voices[0].part->Process(ps, blow_in + count, strike_in + count, out + count, aux + count, size);
  }*/
  // from rings
  int16_t *obuff = voices[0].obuff;
  int16_t *abuff = voices[0].abuff;


  voices[0].part->Process(ps, blow_in, strike_in, out, aux, size);


  // to pwm buffers
  
  for (size_t i = 0; i < size; ++i) {
    //obuff[i] = stmlib::SoftConvert(out[i]);
    //abuff[i] = stmlib::SoftConvert(aux[i]);
    obuff[i] = stmlib::Clip16(static_cast<int16_t>(out[i] * 32768.0f));
    //abuff[i] = stmlib::Clip16(static_cast<int16_t>(aux[i] * 32768.0f));

  }



}

void loop() {
  // when the osc buffer has been written to PWM buffer
  if ( counter > 0 ) {
    updateElementsAudio();
    counter = 0; // increments on each pass of the timer when the timer writes
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

      //  if ((!potlock[1]) || (!potlock[2])) seq[i].trigger=euclid(16,map(potvalue[1],POT_MIN,POT_MAX,0,MAX_SEQ_STEPS),map(potvalue[2],POT_MIN,POT_MAX,0,MAX_SEQ_STEPS-1));
      // look up drum trigger pattern encoder play modes

      if ( i == 8) {
        engineCount = engineCount + encoder_delta;
        CONSTRAIN(engineCount, 0, 47);
        engine_in = engineCount; // ( engine +) % voices[0].voice_.GetNumEngines();

      }

      if ( (encoder_pos != encoder_pos_last ) && i == 1  ) {
        engineCount = engineCount + encoder_delta;
        CONSTRAIN(engineCount, 0, 47);
        engine_in = engineCount;

      }

      // change pitch on pot 0
      if (display_mode == 0 ) { // change sample if pot has moved enough
        //noteA = (map(mozziAnalogRead(MODR_PIN), POT_MIN, POT_MAX, 200, 10000));
        //voice[current_track].sampleincrement = (uint16_t)(map(potvalue[0], POT_MIN, POT_MAX, 2048, 8192)); // change sample pitch if pot has moved enough
      }

      // change volume on pot 1
      if (display_mode == 0) {
        //mod_to_carrier_ratio = (map(mozziAnalogRead(AIN1), POT_MIN, POT_MAX, 1, 20));
        //voice[current_track].level = (int16_t)(map(potvalue[1], POT_MIN, POT_MAX, 0, 1000));
        // change sample volume level if pot has moved enough
      }

      if (!potlock[0] && display_mode == 1 ) {
        //filter_fc = potvalue[0] * (LPF_MAX + 10) / 4096;
      }

      // set track euclidean triggers if either pot has moved enough
      if (!potlock[1] && ! button[8] && display_mode == 1) {
        //  seq[i].fills = map(potvalue[1], POT_MIN, POT_MAX, 0, 16);
        //  seq[i].trigger->generateSequence(seq[i].fills, 15);
        //seq[i].trigger= drumpatterns[map(potvalue[1],POT_MIN,POT_MAX,0,NUMPATTERNS-1)];
      }

    } else {
      if (i < 8) digitalWrite(led[i] , LOW); // else the button is off.
    }
  }

  // now, after buttons check if only encoder moved and no buttons
  // this is broken by mozzi, sigh.
  if (! anybuttonpressed && encoder_delta) {
    float turn = encoder_delta * 0.01f;
    harm_in = harm_in + turn;
    
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

  //delay(3000);


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
