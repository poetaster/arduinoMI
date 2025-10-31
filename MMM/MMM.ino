/*
  (c) 2024 blueprint@poetaster.de
  GPLv3

      Some sources, including the stmlib and plaits lib are
      MIT License
      Copyright (c)  2020 (emilie.o.gillet@gmail.com)
*/

bool debug = false;

#include <Arduino.h>
#include <math.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <hardware/pwm.h>

//#include <LittleFS.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <Wire.h>

#include <PWMAudio.h>
#define SAMPLERATE 48000
#define PWMOUT 22
#define PWMOUT2 30
PWMAudio DAC(PWMOUT);  // 16 bit PWM audio
//PWMAudio DAC2(PWMOUT2);

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

float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  float result;
  result = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  return result;
}


// volts to octave for 3.3 volts
// based onhttps://little-scale.blogspot.com/2018/05/pitch-cv-to-frequency-conversion-via.html
float data;
float pitch;
float pitch_offset = 36;
float freq;

float max_voltage_of_adc = 3.3;
float voltage_division_ratio = 0.3333333333333;
float notes_per_octave = 12;
float volts_per_octave = 1;
float mapping_upper_limit = 120; //(max_voltage_of_adc / voltage_division_ratio) * notes_per_octave * volts_per_octave;
float mapping_lower_limit = 0.0;


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
#define CV5 (44u)
#define CV6 (45u)
#define CV7 (46u)
#define CV8 (47u)

int cv_ins[8] = {CV1, CV2, CV3, CV4, CV5, CV6, CV7, CV8};
int cv_avg = 5;

// buffer for input to rings exciter
float CV1_buffer[32];

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

int voice_number = 0; // for switching  between modules

// we are reusing the plaits nomenclature for all modules
// Plaits modulation vars
float morph_in = 0.6f; // IN(4);
float trigger_in = 0.0f; //IN(5);
float level_in = 0.0f; //IN(6);
float harm_in = 0.5f;
float timbre_in = 0.5f;
int engine_in;
char engine_name;

// these are the last settings per voice
float plaits_morph = morph_in;
float plaits_harm = harm_in;
float plaits_timbre = timbre_in;
int   plaits_engine = 0;
float rings_morph = morph_in;
float rings_harm = harm_in;
float rings_timbre = timbre_in;
float rings_pos = 0.0f;
int   rings_engine = 0;
float braids_timbre = timbre_in;
float braids_morph = morph_in;
int   braids_engine = 0;


float fm_mod = 0.0f ; //IN(7);
float timb_mod = 0.0f; //IN(8);
float morph_mod = 0.0f; //IN(9);
float decay_in = 0.5f; // IN(10);
float lpg_in = 0.2f ;// IN(11);
float pitch_in = 44.0f;

int max_engines = 15; // varies per backend

// Rings modulation
float pos_mod = 0.25f; // position


#include <STMLIB.h>
#include <PLAITS.h>
#include "plaits.h"
#include <RINGS.h>
#include "rings.h"
#include <BRAIDS.h>
#include "braids.h"

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
volatile int repeat = 32;

// Init RPI_PICO_Timer, can use any from 0-15 pseudo-hardware timers
RPI_PICO_Timer ITimer0(0);

bool TimerHandler0(struct repeating_timer *t) {
  (void) t;
  bool sync = true;

  if ( DAC.availableForWrite()) {
    for (size_t i = 0; i < 32; i++) {
      DAC.write( out_bufferL[i]);
      //DAC2.write( out_bufferR[i]);
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
int addr = 0; // for writing to flash
int wrote = 0;
bool writing = false;
bool reading = false;


// called at voice chage to save current values of all voices.
// using the eeprom fake, we have the last 512 bytes, much more than we need
/*
  void writeSettings() {

  int val; // reuse
  addr = 0;
  writing = true; // to stop other actitiy
  // open for writing
  File settings = LittleFS.open(F("/settings.txt"), "w");

  // We simply print an int at a time, each on a new line.
  //plaits
  val = (int) plaits_morph * 100;
  settings.print(val);
  val =  (int) plaits_harm * 100;
  settings.print(val);
  val =  (int) plaits_timbre * 100;
  settings.print(val);
  val =  (int) plaits_engine;
  settings.print(val);


  //rings
  val =  (int) rings_morph * 100;
  settings.print(val);
  val =  (int) rings_harm * 100;
  settings.print(val);
  val =  (int) rings_timbre * 100;
  settings.print(val);
  val =  (int) rings_pos * 100;
  settings.print(val);
  val =  (int) rings_engine;
  settings.print(val);


  //braids
  val =  (int) braids_timbre * 100;
  settings.print(val);
  val =  (int) braids_morph * 100;
  settings.print(val);
  val =  (int) braids_engine;
  settings.print(val);


  //  finally, commit
  settings.close();
  writing = false;


  }

  // called at system setup to initialize from saved values.
  void readSettings() {

  int val; // reuse
  File settings = LittleFS.open(F("/settings.txt"), "r");

  // serial parseInt() calls will do an int a line at a time.

  //plaits
  val = settings.parseInt();

  // first check if the first values is set
  if (val > 1) {
    // we have a value, read the rest
    if (val > 0) plaits_morph = (float) val / 100;
    val = settings.parseInt();
    if (val > 0) plaits_harm = (float) val / 100;
    val = settings.parseInt();
    if (val > 0) plaits_timbre = (float) val / 100;
    val = settings.parseInt();
    plaits_engine = val;

    //rings
    val = settings.parseInt();
    if (val > 0) rings_morph = (float)val / 100;
    val = settings.parseInt();
    if (val > 0) rings_harm = (float)val / 100;
    val = settings.parseInt();
    if (val > 0) rings_timbre = (float)val / 100;
    val = settings.parseInt();
    if (val > 0) rings_pos = (float)val / 100;
    val = settings.parseInt();
    rings_engine = val;


    //braids
    val = settings.parseInt();
    if (val > 0) braids_timbre = (float)val / 100;
    val = settings.parseInt();
    if (val > 0) braids_morph = (float)val / 100;
    val = settings.parseInt();
    braids_engine = val;
  } else {
    writeSettings(); // initialize should only happen once.
  }

  }
*/

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
int32_t button_timer;
int update_interval = 30;
int engineCount = 0;
bool button_state = true;


// last time btn_one release
unsigned long btnOneLastTime;
unsigned long btnTwoLastTime;

int32_t previous_pitch = 40;

bool just_booting = false;

//File settingsFile;


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


  // set up Pico PWM audio output the DAC2 stereo approach works./
  DAC.setBuffers(4, 32); //plaits::kBlockSize * 4); // DMA buffers
  //DAC2.setBuffers(4,32);
  //DAC.onTransmit(cb);
  DAC.setFrequency(SAMPLERATE);
  //DAC2.setFrequency(SAMPLERATE);
  // now start the dac
  DAC.begin();
  //DAC2.begin();
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
  pinMode(CV5, INPUT);

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
  delay(100);
  // prefill buffer
  voices[0].voice_->Render(voices[0].patch, voices[0].modulations,  outputPlaits,  plaits::kBlockSize);
  delay(100);
  initRings();
  delay(100);
  initBraids();
  delay(100);
  // Initialize wave switch states
  update_timer = millis();
  button_timer = millis();
  just_booting = true;
  btn_one.update();
  btn_two.update();

  // let's see, seems to be too slow
  //LittleFS.begin();
  //readSettings(); // try to retrieve the voice settings from last session.

}



void loop() {

  if (counter == 1) {
    if (voice_number == 0) {
      updatePlaitsAudio();
    } else if (voice_number == 1) {
      updateRingsAudio();
    } else if (voice_number == 2) {
      updateBraidsAudio();
    }
    counter = 0; // increments on each pass of the timer when the timer writes
  }

  /*
    if (writing) {
      writeSettings(); // after switching, save this state to flash.
    }
  */
}



void setup1() {
  delay (1000); // wait for main core to start up perhipherals
}

// second core deals with ui / control rate updates
void loop1() {

  if (! writing) { // don't do shit when eeprom is being written

    // we need these on boot so the second loop can catch the startup button.
    btn_one.update();
    btn_two.update();

    // at boot permit octave down
    if (just_booting && btn_one.pressed()) {
      pitch_offset = 36;
    }

    just_booting = false;

    unsigned long now = millis();

    if ( now - update_timer > 5 ) {
      voct_midi(CV1);
      read_trigger();
      read_cv();
      read_encoders();
      displayUpdate();
      read_buttons();
      update_timer = now;
    }

    if (voice_number == 0) {
      updatePlaitsControl();
    } else if (voice_number == 1) {
      updateRingsControl();
    }
  }

}



void read_buttons() {


  bool doublePressMode = false;

  // if button one was held for more than 75 millis and we're in rings toggle easteregg
  if (btn_one.rose()) {
    btnOneLastTime = btn_one.previousDuration();
    if (btnOneLastTime > 250 && voice_number == 1) easterEgg = !easterEgg;
  }

  if (btn_two.rose()) {
    btnTwoLastTime = btn_two.previousDuration();
  }


  if (btn_one.pressed() && btn_two.pressed()) {
    // rings easter egg mode/ fm engine.
    easterEgg = !easterEgg;
    doublePressMode = true;
  }

  if (!doublePressMode) {
    // being tripple shure :)
    if (btn_one.pressed() && ! btn_two.pressed()) {
      engineCount ++;
      if (engineCount > max_engines) {
        engineCount = 0;
      }
      engine_in = engineCount;
    }
    if (btn_two.pressed() && ! btn_one.pressed() ) {

      // first record our last settings
      if (voice_number == 0) {
        plaits_morph = morph_in;
        plaits_timbre = timbre_in;
        plaits_harm = harm_in;
        plaits_engine = engine_in;
      }
      if (voice_number == 1) {
        rings_morph = morph_in;
        rings_timbre = timbre_in;
        rings_harm = harm_in;
        rings_pos = pos_mod;
        rings_engine = engine_in;
      }
      if (voice_number == 2) {
        braids_morph = morph_in;
        braids_timbre = timbre_in;
        braids_engine = engine_in;
      }

      voice_number++;

      if (voice_number > 2) voice_number = 0;

      if (voice_number == 0) {
        engine_in = plaits_engine; // engine_in % 17;
        max_engines = 15;
        morph_in = plaits_morph;
        timbre_in = plaits_timbre;
        harm_in = plaits_harm;

      } else if (voice_number == 1) {
        engine_in = rings_engine; // % 6;
        max_engines = 5;
        morph_in = rings_morph;
        harm_in = rings_harm;
        timbre_in = rings_timbre;
        //pos_mod = rings_pos;

      } else if (voice_number == 2 ) {
        engine_in = braids_engine; // engine_in % 46;
        max_engines = 45;
        morph_in = braids_morph;
        timbre_in = braids_timbre;
      }
      // sadly, this breaks badly
      //writing = true;
    }

  }
}

float voct_midiBraids(int cv_in) {

  int val = 0;
  for (int j = 0; j < cv_avg; ++j) val += analogRead(cv_in); // read the A/D a few times and average for a more stable value
  val = val / cv_avg;
  pitch = pitch_offset + map(val, 0.0, 4095.0, mapping_upper_limit, 0.0); // convert pitch CV data value to a MIDI note number
  return pitch - 37; // don't know why, probably tuned to A so -5 + -36 to drop two octaves
}

void voct_midi(int cv_in) {
  // this seems sufficient with 3 reads.
  int val = 0;
  for (int j = 0; j < cv_avg; ++j) val += analogRead(cv_in); // read the A/D a few times and average for a more stable value
  val = val / cv_avg;
  data = (float) val * 1.0f;
  pitch = map(data, 0.0, 4095.0, mapping_upper_limit, mapping_lower_limit); // convert pitch CV data value to a MIDI note number

  pitch = pitch - pitch_offset;

  pitch_in = pitch;

  // this is a temporary move to get around clicking on trigger + note cv in
  if (pitch != previous_pitch) {
    previous_pitch = pitch;
    // this is the plaits version

  }
}

void read_trigger() {
  int16_t trig = analogRead(CV2);
  if (trig > 2048 ) {
    trigger_in = 1.0f;
    if (voice_number == 0) updateVoicetrigger();

  } else  {
    //don't turn off here?
    trigger_in = 0.0f;
  }



}

void read_cv() {
  // CV updates
  // braids wants 0 - 32767, plaits 0-1

  // this should be worked out into calls for the engines instead of conditionals ....


  //plaits and rings cv
  int16_t timbre = analogRead(CV3);
  timb_mod = (float)timbre / 4095.0f;

  int16_t morph = analogRead(CV4) ;
  morph_mod = (float) morph / 4095.0f;

  
  // don't remember if this was important
  int16_t pos = analogRead(CV5) ; // f&d noise floor
  if (pos > 100) pos_mod = (float) pos / 4095.0f;
  
  int16_t lpgColor = (float) ( analogRead(CV6) ) / 4095.f ;
  lpg_in = lpgColor;

  if (voice_number == 0 || voice_number == 1) {
    // plaits

      timb_mod = mapf(timb_mod, 0.0f, 1.0f, 0.0f, 0.8f);
      if (debug) Serial.print(timb_mod);
      
      //voices[0].modulations.timbre_patched = true;
      //voices[0].modulations.timbre_patched = false;
      //morph_mod = mapf(morph_mod, 0.02f, 1.0f, -1.0f, 1.0f);
      morph_mod = mapf(morph_mod, 0.0f, 1.0f, 0.0f, 0.8f);
      //voices[0].modulations.morph_patched = true;

  }

  if (voice_number == 1 && timb_mod > 0.05f) {
    //rings
    for (size_t i = 0; i < 32; ++i) {
      CV1_buffer[i] = (float) ( analogRead(CV3) / 4095.0f) + 1.5f; // arbitrary +1 gain
    }
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

  if ( enc1_delta) {
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
