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
#define SAMPLERATE 48000
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

// braids dsp

//const uint16_t decimation_factors[] = { 1, 2, 3, 4, 6, 12, 24 };
const uint16_t bit_reduction_masks[] = {
  0xffff,
  0xfff0,
  0xff00,
  0xf800,
  0xf000,
  0xe000,
  0xc000
};

#define     MI_SAMPLERATE      96000.f
#define     BLOCK_SIZE          32      // --> macro_oscillator.h !
#define     SAMP_SCALE          (float)(1.0 / 32756.0)


#include <STMLIB.h>
#include <BRAIDS.h>

typedef struct
{
  braids::MacroOscillator *osc;

  float       samps[BLOCK_SIZE] ;
  int16_t     buffer[BLOCK_SIZE];
  uint8_t     sync_buffer[BLOCK_SIZE];

} PROCESS_CB_DATA ;

char shared_buffer[16384];


//float a0 = (440.0 / 8.0) / kSampleRate; //48000.00;
const size_t   kBlockSize = BLOCK_SIZE;


struct Unit {
  braids::Quantizer   *quantizer;
  braids::SignatureWaveshaper *ws;

  bool            last_trig;

  // resampler
  //SRC_STATE       *src_state;
  PROCESS_CB_DATA pd;
  float           *samples;
  float           ratio;
};

static long src_input_callback(void *cb_data, float **audio);

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

#define TIMER0_INTERVAL_MS 20.833333333333   // \20.833333333333running at 48Khz
#define DEBOUNCING_INTERVAL_MS   2// 80
#define LOCAL_DEBUG              0

volatile int counter = 0;

// Init RPI_PICO_Timer, can use any from 0-15 pseudo-hardware timers
RPI_PICO_Timer ITimer0(0);

bool TimerHandler0(struct repeating_timer *t) {
  (void) t;
  bool sync = true;
  if ( DAC.availableForWrite() ) {
    for (size_t i = 0; i < BLOCK_SIZE; i++) {
      DAC.write( voices[0].pd.buffer[i], sync); 
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
  DAC.setBuffers(4, 32); // plaits::kBlockSize); // DMA buffers
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

  // init the braids voices
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

  voices[0].ratio = 48000.f / MI_SAMPLERATE;

  // init some params
  voices[0].pd.osc = new braids::MacroOscillator;
  memset(voices[0].pd.osc, 0, sizeof(*voices[0].pd.osc));

  voices[0].pd.osc->Init(48000.f);
  voices[0].pd.osc->set_pitch((48 << 7));
  voices[0].pd.osc->set_shape(braids::MACRO_OSC_SHAPE_VOWEL_FOF);


  voices[0].ws = new braids::SignatureWaveshaper;
  voices[0].ws->Init(123774);

  voices[0].quantizer = new braids::Quantizer;
  voices[0].quantizer->Init();
  voices[0].quantizer->Configure(braids::scales[0]);

  //unit->jitter_source.Init();

  memset(voices[0].pd.buffer, 0, sizeof(int16_t)*BLOCK_SIZE);
  memset(voices[0].pd.sync_buffer, 0, sizeof(voices[0].pd.sync_buffer));
  memset(voices[0].pd.samps, 0, sizeof(float)*BLOCK_SIZE);

  voices[0].last_trig = false;

// get some samples initially
  updateBraidsAudio();


  // Initialize the sample rate converter
  //int error;
  //int converter = SRC_SINC_FASTEST;       //SRC_SINC_MEDIUM_QUALITY;

 /* if ((unit->src_state = src_callback_new(src_input_callback, converter, 1, &error, &unit->pd)) == NULL)
  {
  }
  */
  
  //voices[0]->samples = (float *)RTAlloc(unit->mWorld, 1024 * sizeof(float));
  
  /*
         // check resample flag
      int resamp = (int)IN0(5);
      CONSTRAIN(resamp, 0, 2);
      switch(resamp) {
          case 0:
              SETCALC(MiBraids_next);
              //Print("resamp: OFF\n");
              break;
          case 1:
              unit->pd.osc->Init(MI_SAMPLERATE);
              SETCALC(MiBraids_next_resamp);
              Print("MiBraids: internal sr: 96kHz - resamp: ON\n");
              break;
          case 2:
              SETCALC(MiBraids_next_reduc);
              Print("MiBraids: resamp: OFF, reduction: ON\n");
              break;
      }
  */
}

void cb() {
  if (DAC.availableForWrite() > 14) {
    for (int i = 0; i <  BLOCK_SIZE; i++) {
      // out = ;   // left channel called .aux
      DAC.write( voices[0].pd.buffer[i] );
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

void updateBraidsAudio() {

  int16_t *buffer = voices[0].pd.buffer;
  uint8_t *sync_buffer = voices[0].pd.sync_buffer;
  size_t  size = BLOCK_SIZE;

  braids::MacroOscillator *osc = voices[0].pd.osc;

  // TODO: check setting pitch
  //CONSTRAIN(voct_in, 0.f, 127.f);
  //int pit = (int)voct_in;
  //float frac = voct_in - pit;
  //osc->set_pitch((pit << 7) + (int)(frac * 128.f));

  osc->set_pitch(pitch_in << 7);

  // set parameters
  CONSTRAIN(timbre_in, 0.f, 1.f);
  int16_t timbre = timbre_in * 32767.f;

  CONSTRAIN(morph_in, 0.f, 1.f);
  int16_t color = morph_in * 32767.f;
  osc->set_parameters(timbre, color);

  // set shape/model
  uint8_t shape = (int)(engine_in);
  if (shape >= braids::MACRO_OSC_SHAPE_LAST)
    shape -= braids::MACRO_OSC_SHAPE_LAST;
  osc->set_shape(static_cast<braids::MacroOscillatorShape>(shape));

  bool trigger = (trigger_in != 0.0);
  bool trigger_flag = (trigger && (!voices[0].last_trig));

  voices[0].last_trig = trigger;

  if (trigger_flag)
    osc->Strike();
    
  for (int count = 0; count < 32; count += size) {
    // render
    osc->Render(sync_buffer, buffer, size);

    /*for (int i = 0; i < size; ++i) {
      out[count + i] = buffer[i] * SAMP_SCALE;
    }*/
  }
  
}

void loop() {
  // when the osc buffer has been written to PWM buffer
  if ( counter > 0 ) {
    updateBraidsAudio();
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
