/*
  (c) 2024 blueprint@poetaster.de
  GPLv3

      Some sources, including the stmlib and plaits lib are
      MIT License
      Copyright (c)  2020 (emilie.o.gillet@gmail.com)
*/

bool debug = true;

#include <Arduino.h>
#include <math.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <hardware/pwm.h>

#include <iostream>
#include <vector>
#include <algorithm>

#include <PWMAudio.h>
#define SAMPLERATE 48000
#define PWMOUT 22
PWMAudio DAC(PWMOUT);  // 16 bit PWM audio

// utility
double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

double median(std::vector<int>& numbers) {
  std::sort(numbers.begin(), numbers.end());
  int n = numbers.size();
  return n % 2 == 0 ? (numbers[n / 2 - 1] + numbers[n / 2]) / 2.0 : numbers[n / 2];
}


// volts to octave for 3.3 volts
// based onhttps://little-scale.blogspot.com/2018/05/pitch-cv-to-frequency-conversion-via.html
float data;
float pitch;
float pitch_offset = 12;
float freq;

float max_voltage_of_adc = 3.3;
float voltage_division_ratio = 0.3333333333333;
float notes_per_octave = 12;
float volts_per_octave = 1;
float mapping_upper_limit = (max_voltage_of_adc / voltage_division_ratio) * notes_per_octave * volts_per_octave;


// encoder related // 2,3 8,9
#include "pio_encoder.h"

const int enc1A_pin = 18;
const int enc1B_pin = 19;
const int enc2A_pin = 2;
const int enc2B_pin = 3;
const int enc3A_pin = 8;
const int enc3B_pin = 9;

const int encoderSW_pin = 28;

PioEncoder enc1(18);
PioEncoder enc2(2);
PioEncoder enc3(8);


// cv input
#define CV1 (A0)
#define CV2 (A1)
#define CV3 (A2)
#define CV4 (A3)
int cvs_ins[4] = {CV1, CV2, CV3, CV4};
int cv_avg = 30;

// button inputs


#define SW1 6
#define SW2 17
#include <Bounce2.h>
Bounce2::Button btn_one = Bounce2::Button();
Bounce2::Button btn_two = Bounce2::Button();

// Generic pin state variable
byte pinState;

#define LED0 1 // LED1 output on schematic
#define LED1 3
#define LED2 5
#define LED3 7
#define LED4 9
#define LED5 11
#define LED6 13
#define LED7 15
// analog freq pins

// common output buffers
int16_t out_bufferL[32];
int16_t out_bufferR[32];

#include <Wire.h>
// we are reusing the plaits nomenclature for all modules
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

int max_engines = 16; // varies per backend

#include <STMLIB.h>

#include <PLAITS.h>
#include "plaits.h"


#include "Midier.h"
// midi related functions
#include "midi.h"

#include "names.h"


// clock timer  stuff

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     4

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RPi_Pico_TimerInterrupt.h"

//unsigned int SWPin = CLOCKIN;

#define TIMER0_INTERVAL_MS 20.833333333333 // running at 48Khz
// 32768.0f 30.517578125
// 24.390243902439025 // 44.1


#define DEBOUNCING_INTERVAL_MS   u2// 80
#define LOCAL_DEBUG              0

volatile int counter = 0;


// Init RPI_PICO_Timer, can use any from 0-15 pseudo-hardware timers
RPI_PICO_Timer ITimer0(0);

bool TimerHandler0(struct repeating_timer *t) {
  (void) t;
  bool sync = true;

  if ( DAC.availableForWrite()) {
    for (size_t i = 0; i < plaits::kBlockSize; i++) {
      DAC.write( out_bufferL[i]); // 244 is mozzi audio bias
    }

    counter = 1;
  }

  return true;
}

// callback for pwm method. we're not using it
void cb() {
  bool sync = true;
  if ( DAC.availableForWrite() > 32) {
    for (size_t i = 0; i < plaits::kBlockSize; i++) {
      DAC.write( outputPlaits[i].out); // 244 is mozzi audio bias
    }
    counter = 1;
  }
}


// variables for UI state management
int enc1_pos_last = 0;
int enc1_delta = 0;
int enc2_pos_last = 0;
int enc2_delta = 0;
int enc3_pos_last = 0;
int enc3_delta = 0;
;
uint32_t enc1_push_millis;
uint32_t step_push_millis;
bool encoder_held = false;


// display related
const int oled_sda_pin = 20;
const int oled_scl_pin = 21;
const int oled_i2c_addr = 0x3C;
const int dw = 128;
const int dh = 32;

#include <Adafruit_SSD1306.h>
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(dw, dh, &Wire, OLED_RESET);

#include "font.h"
#include "helvnCB6pt7b.h"
#define myfont helvnCB6pt7b // Org_01 looks better but is small.
#include "display.h"


// buttons & knobs defines/functions
//#include "control.h"


// audio related defines

//float freqs[12] = { 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f, 493.88f};
float freqs[12] = { 42.0f, 44.0f, 46.0f, 48.0f, 50.0f, 52.0f, 54.0f, 56.0f, 58.0f, 60.0f, 62.0f, 64.0f};
int carrier_freq;

int current_track;
int32_t update_timer;
int update_interval = 30;
int engineCount = 0;
bool button_state = true;

int32_t previous_pitch = 4000;


void setup() {
  if (debug) {
    Serial.begin(57600);
    Serial.println(F("YUP"));
  }
  // pwm timing setup
  // we're using a pseudo interrupt for the render callback since internal dac callbacks crash
  // Frequency in float Hz
  //ITimer0.attachInterrupt(TIMER_FREQ_HZ, TimerHandler0);

  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS, TimerHandler0)) // that's 48kHz
  {
    if (debug) Serial.print(F("Starting  ITimer0 OK, millis() = ")); Serial.println(millis());
  }  else {
    if (debug) Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
  }


  // set up Pico PWM audio output
  DAC.setBuffers(4, 32); //plaits::kBlockSize * 4); // DMA buffers
  //DAC.onTransmit(cb);
  DAC.setFrequency(SAMPLERATE);
  // now start the dac
  DAC.begin();

  // lets seee
  analogReadResolution(12);

  // ENCODER
  enc1.begin();
  enc2.begin();
  enc3.begin();
  enc3.flip();
  enc2.flip();
  enc1.flip();

  // this is only rp2040 relevant
  //pinMode(23, OUTPUT); // thi is to switch to PWM for power to avoid ripple noise
  //digitalWrite(23, HIGH);


  // CV
  pinMode(CV1, INPUT);
  pinMode(CV2, INPUT);
  pinMode(CV3, INPUT);
  pinMode(CV4, INPUT);

  // DISPLAY

  Wire.setSDA(oled_sda_pin);
  Wire.setSCL(oled_scl_pin);
  Wire.begin();

  // SSD1306 --  or SH1106 in this case
  if (!display.begin(SSD1306_SWITCHCAPVCC, oled_i2c_addr)) {
    //if (!display.begin( oled_i2c_addr)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) ;  // Don't proceed, loop forever
  }

  displaySplash();

  // buttons

  btn_one.attach( SW1 , INPUT_PULLUP);
  btn_one.interval(5);
  btn_one.setPressedState(LOW);
  
  btn_two.attach( SW2 , INPUT_PULLUP);
  btn_two.interval(5);
  btn_two.setPressedState(LOW);
  /*
      //sw2.attach( SW2 , INPUT);
      //sw2.interval(5);
      //sw2.setPressedState(LOW);
  */


  // initialize a mode to play
  //mode = midier::Mode::Ionian;
  //makeScale( roots[scaleRoot], mode);


// setup common output buffers
  //out_bufferL = (int16_t*)malloc(32768 * sizeof(int16_t));
  //memset(out_bufferL, 0, 32768 * sizeof(int16_t));
  //out_bufferR = (int16_t*)malloc(32768 * sizeof(int16_t));
  //memset(out_bufferR, 0, 32768 * sizeof(int16_t));

  
  // init the plaits voices

  initPlaits();

  // prefill buffer
  voices[0].voice_->Render(voices[0].patch, voices[0].modulations,  outputPlaits,  plaits::kBlockSize);

  // Initialize wave switch states
  update_timer = millis();

}



void loop() {

  if (counter == 1) {
    updatePlaitsAudio();
    counter = 0; // increments on each pass of the timer when the timer writes
  }
}



void setup1() {
  delay (1000); // wait for main core to start up perhipherals
}

// second core deals with ui / control rate updates
void loop1() {

  btn_one.update();
  read_buttons();

  int32_t now = millis();
  if ( now - update_timer > 10 ) {
    //if (debug) Serial.println(now - update_timer);

    voct_midi(CV1);
    read_trigger();
    read_cv();
    read_encoders();
    displayUpdate();
    update_timer = now;
    
    updatePlaitsControl();
  }

}

void read_trigger() {
  int16_t trig = analogRead(CV2);
  if (trig > 2048 ) {
    trigger_in = 1.0f;

  } else  {
    trigger_in = 0.0f;
  }
  

}
void read_buttons() {
  if (btn_one.pressed()) {
    if (debug) Serial.println("button");
    engineCount ++;
    if (engineCount > 16) {
      engineCount = 0;
    }
    engine_in = engineCount;

  }
  
}

void voct_midi(int cv_in) {
  int val = 0;
  for (int j = 0; j < cv_avg; ++j) val += analogRead(cv_in); // read the A/D a few times and average for a more stable value
  val = val / cv_avg;
  data = (float) val * 1.0f;
  pitch = pitch_offset + map(data, 0.0, 4095.0, mapping_upper_limit, 0.0); // convert pitch CV data value to a MIDI note number


  // well, it sucks :)
  if (pitch > 82) {
    pitch_in = pitch - 7;
  } else if (pitch < 38) {
    pitch_in = pitch - 9;
  } else {
    pitch_in = pitch - 8;
  }

  // this is a temporary move to get around clicking on trigger + note cv in
  previous_pitch = pitch;
  trigger_in = 1.0f;
  updateVoicetrigger();
  trigger_in = 0.0f;

}

void read_cv() {
  // CV updates
  // braids wants 0 - 32767, plaits 0-1

  int16_t timbre = analogRead(CV3);
  timb_mod = (float)timbre / 4095.0f;

  if (timb_mod > 0.1f) {
    if (debug) Serial.println(timb_mod);
    voices[0].modulations.timbre_patched = true;

  } else {
    voices[0].modulations.timbre_patched = false;
  }

  int16_t morph = analogRead(CV4) ;//, 0, 4095, 4095, 0));
  morph_mod = (float) morph / 4095.0f;

  if (morph_mod > 0.1f ) {
    if (debug) Serial.print(morph);
    if (debug) Serial.print(" : ");
    if (debug) Serial.println(morph_mod);
    voices[0].modulations.morph_patched = true;
  } else {
    voices[0].modulations.morph_patched = false;
  }


}


// either avg or median, both suck :)
int16_t avg_cv(int cv_in) {

  //std::vector<int> data;
  int16_t val = 0;

  //for (int j = 0; j < cv_avg; ++j) data.push_back(analogRead(cv_in)); // val += analogRead(cv_in); // read the A/D a few times and average for a more stable value
  for (int j = 0; j < cv_avg; ++j) val += analogRead(cv_in); // read the A/D a few times and average for a more stable value
  val = val / cv_avg;

  //return median(data);
  return val;
}


void read_encoders() {

  // first encoder
  int enc1_pos = enc1.getCount() / 4;

  if ( enc1_pos != enc1_pos_last ) {
    enc1_delta = (enc1_pos - enc1_pos_last) ;
  }

  if (enc1_delta) {
    float turn = ( enc1_delta * 0.01f ) + timbre_in;
    CONSTRAIN(turn, 0.f, 1.0f)
    if (debug) Serial.println(turn);
    timbre_in = turn;
  }

  /// only set new pos last after buttons have had a chance to use the delta
  enc1_delta = 0;
  enc1_pos_last = enc1_pos;


  // second encoder
  int enc2_pos = enc2.getCount() / 4;
  if ( enc2_pos != enc2_pos_last ) {
    enc2_delta = (enc2_pos - enc2_pos_last) ;
  }

  if (enc2_delta) {
    float turn = ( enc2_delta * 0.01f ) + morph_in;
    CONSTRAIN(turn, 0.f, 1.0f)
    if (debug) Serial.println(turn);
    morph_in = turn;

  }
  enc2_pos_last = enc2_pos;
  enc2_delta = 0;

  // third encoder

  int enc3_pos = enc3.getCount() / 4;

  if ( enc3_pos != enc3_pos_last ) {
    enc3_delta = (enc3_pos - enc3_pos_last);

  }

  if (enc3_delta) {
    float turn = ( enc3_delta * 0.01f ) + harm_in;
    CONSTRAIN(turn, 0.f, 1.0f)
    if (debug) Serial.println(turn);
    harm_in = turn;
  }
  enc3_pos_last = enc3_pos;
  enc3_delta = 0;

}
