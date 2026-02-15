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
//#include <Wire.h>

#include <SoftwareSerial.h>

#include <MIDI.h>
// this method permits us to use arbitrary pins
// in pico land this is also a PIO statemachine (uart)

using Transport = MIDI_NAMESPACE::SerialMIDI<SoftwareSerial>;
int rxPin = 1;
int txPin = 3;
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin); //thruActivated
Transport serialMIDI(mySerial);
MIDI_NAMESPACE::MidiInterface<Transport> MIDI((Transport&)serialMIDI);


// start with CV input, switch on midi in or setting.
bool midi_switch = false;
bool midi_switch_setting = false;

#include <I2S.h>
#define SAMPLERATE 48000

#define pBCLK 8
#define pWS (pBCLK+1)
#define pDOUT 10
I2S DAC(OUTPUT, pBCLK, pDOUT);

// create ADSR env
#include "ADSR.h"
ADSR *env = new ADSR();
bool envRelease = false;
long envTimer = 0;



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
// based on https://little-scale.blogspot.com/2018/05/pitch-cv-to-frequency-conversion-via.html
float data;
float pitch;
float pitch_offset = 0; // 36 in mmm 10volt input
float freq;

float max_voltage_of_adc = 3.3;
float voltage_division_ratio = 0.3333333333333;
float notes_per_octave = 12;
float volts_per_octave = 1;
float mapping_upper_limit = 60.0; //(max_voltage_of_adc / voltage_division_ratio) * notes_per_octave * volts_per_octave;
float mapping_lower_limit = 0.0;


// trigger is on gp0

// cv input
#define CV1 (42u)
#define CV2 (43u)
#define CV3 (44u)
#define CV4 (45u)
#define CV5 (46u)
#define CV6 (47u)



int cv_ins[6] = {CV1, CV2, CV3, CV4, CV5, CV6};
int cv_avg = 20;

// buffer for input to rings exciter
float CV1_buffer[32];

// button inputs
#define SW1 34
#define SW2 35
#define SW3 2

#include <Bounce2.h>
Bounce2::Button btn_one = Bounce2::Button();
Bounce2::Button btn_two = Bounce2::Button();
Bounce2::Button btn_four = Bounce2::Button();

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

// sample buffer for clouds
int16_t sample_buffer[32]; // used while we play samples for demo

int voice_number = 0; // for switching  between modules

// we are reusing the plaits nomenclature for all modules
// Plaits modulation vars
float morph_in = 0.5f; // IN(4);
float trigger_in = 0.0f; //IN(5);
float level_in = 0.0f; //IN(6);
float harm_in = 0.3f;
float timbre_in = 0.3f;
float position_in = 0.5f;
float pos_mod = 0.0f;

int engine_in;
char engine_name;

// these are the last settings per voice
float plaits_morph = morph_in;
float plaits_harm = harm_in;
float plaits_timbre = timbre_in;
float plaits_position = position_in;
int   plaits_engine = 0;


// Rings modulation
float rings_position = 0.25f; // position
float rings_morph = morph_in;
float rings_harm = harm_in;
float rings_timbre = timbre_in;
int   rings_engine = 0;

float braids_timbre = timbre_in;
float braids_morph = morph_in;
int   braids_engine = 0;


float fm_mod = 0.0f ; //IN(7);
float timb_mod = 0.0f; //IN(8);
float morph_mod = 0.0f; //IN(9);
float decay_in = 0.5f; // IN(10);
float lpg_in = 0.2f ;// IN(11);
float pitch_in = 32.0f;


//clouds
float clouds_morph = morph_in;
float clouds_timbre = timbre_in;
float clouds_harm = harm_in;
float clouds_pos = position_in;
float clouds_mode = engine_in;
float clouds_dw_in = 1.0f;
float clouds_pos_in = 0.0f;
int   clouds_engine = 0;
bool  freeze_in = false;
int   voice_in = 4;

bool trigger_on = false;

int max_engines = 18; // varies per backend



#include <STMLIB.h> // 

#include <RINGS.h>
#include "rings.h"

#include <PLAITS.h>
#include "plaits.h"

#include <BRAIDS.h>
#include "braids.h"

// clouds dsp not used pushes ram
//#include <CLOUDS.h>
//#include "clouds.h"

#include "Midier.h"
// midi related functions
#include "midi.h"

#include "names.h"

volatile int counter = 0;
volatile int repeat = 32;

int addr = 0; // for writing to flash
int wrote = 0;
bool writing = false;
bool reading = false;

const int encoderSW_pin = 28;

// encoder related // 2,3 8,9


// ugly, but using both ranges does not work
#include "pio_encoder.h"
PioEncoder enc1(39, PIO pio1);
PioEncoder enc2(32, PIO pio1);
PioEncoder enc3(36, PIO pio1);

// PioEncoder enc4(6, PIO pio0);
// sadly we need a second approach.

#include <RotaryEncoder.h>
// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder enc4( 6,  7,  RotaryEncoder::LatchMode::TWO03);

int enc1_pos_last = 0;
int enc1_delta = 0;
int enc2_pos_last = 0;
int enc2_delta = 0;
int enc3_pos_last = 0;
int enc3_delta = 0;
int enc4_pos_last = 0;
int enc4_delta = 0;

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


uint32_t enc1_push_millis;
uint32_t step_push_millis;
bool encoder_held = false;


// display related
const int oled_sda_pin = 4;
const int oled_scl_pin = 5;
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
#define smallfont Org_01
#include "display.h"


// buttons & knobs defines/functions
//#include "control.h"


// audio related defines

int current_track;
int32_t update_timer;
int32_t button_timer;
int update_interval = 30;
int engineCount = 0;
bool button_state = true;

// last time btn_one release
unsigned long btnOneLastTime;
unsigned long btnTwoLastTime;
unsigned long btnFourLastTime;
int32_t previous_pitch = 40;

bool just_booting = false;

//File settingsFile;


void setup() {
  if (debug) {
    Serial.begin(57600);
    Serial.println(F("YUP"));
  }

  // start encoders MUST be first because of PIO init?
  //pio_set_gpio_base(PIO pio1, 16); this fails, do it in begin.
  //pio_set_gpio_base(PIO pio0, 0);

  enc1.begin();
  enc2.begin();
  enc3.begin();

  delay(100);

  // set up Pico PWM audio output the DAC2 stereo approach works./
  DAC.setBuffers(4, 32); //plaits::kBlockSize * 4); // DMA buffers
  //DAC.onTransmit(cb);
  DAC.setFrequency(SAMPLERATE);
  // now start the dac
  DAC.begin();

  delay(100);


  // doesn't really get us anything
  //analogReadResolution(12);

  // thi is to switch to PWM for power to avoid ripple noise
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);

  // mute
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);

  pinMode(LED_BUILTIN, OUTPUT);

  // CV
  pinMode(CV1, INPUT);
  pinMode(CV2, INPUT);
  pinMode(CV3, INPUT);
  pinMode(CV4, INPUT);
  pinMode(CV5, INPUT);
  pinMode(CV6, INPUT);

  // trigger in
  pinMode(0, INPUT);

  // DISPLAY

  Wire.setSDA(oled_sda_pin);
  Wire.setSCL(oled_scl_pin);
  Wire.begin();

  MIDI.setHandleNoteOn(HandleMidiNoteOn);  // Put only the name of the function
  MIDI.setHandleNoteOff(HandleMidiNoteOff);  // Put only the name of the function
  MIDI.setHandleControlChange(HandleControlChange);  // Put only the name of the function
  // Initiate MIDI communications, listen to all channels (not needed with Teensy usbMIDI)

  MIDI.begin(MIDI_CHANNEL_OMNI);

  // SSD1306 --  or SH1106 in this case
  if (!display.begin(SSD1306_SWITCHCAPVCC, oled_i2c_addr)) {
    //if (!display.begin( oled_i2c_addr)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) ;  // Don't proceed, loop forever
  }

  displaySplash();

  // buttons

  btn_four.attach( SW3 , INPUT_PULLUP);
  btn_four.interval(5);
  btn_four.setPressedState(LOW);

  btn_one.attach( SW1 , INPUT_PULLUP);
  btn_one.interval(5);
  btn_one.setPressedState(LOW);

  btn_two.attach( SW2 , INPUT_PULLUP);
  btn_two.interval(5);
  btn_two.setPressedState(LOW);


  // initialize enveloope settings
  
  env->setAttackRate(.05 * SAMPLERATE);  // .01 second
  env->setDecayRate(.3 * SAMPLERATE);
  env->setReleaseRate(5 * SAMPLERATE);
  env->setSustainLevel(.8);


  // initialize a mode to play
  //mode = midier::Mode::Ionian;
  //makeScale( roots[scaleRoot], mode);

  // init synth engines

  initPlaits();
  delay(50);
  initRings();
  delay(50);
  initBraids();
  //delay(100); not for now
  // initClouds();

  // Initialize wave switch states
  update_timer = millis();
  button_timer = millis();
  
  just_booting = true;
  btn_one.update();
  btn_two.update();
  btn_four.update();

  // let's see, seems to be too slow
  //LittleFS.begin();
  //readSettings(); // try to retrieve the voice settings from last session.

}


// main loop for audio rendering

void loop() {

  if ( DAC.availableForWrite()) {

    if (voice_number == 0) {
      
      updatePlaitsAudio();
      // now apply the envelope
      for (size_t i = 0; i < plaits::kBlockSize; ++i) {
        int16_t sampleL = (int16_t) ( (float) outputPlaits[i].out * env->process() ) ;
        int16_t sampleR = (int16_t) ( (float) outputPlaits[i].aux * env->process() ) ;
        //out_bufferL[i] = sample;
        DAC.write( sampleL );
        DAC.write( sampleR );
      }
      
    } else if (voice_number == 1) {
      // we're not doing stereo because we get neat poly output with note ins like this
      updateRingsAudio();
      for (size_t i = 0; i < 32; i++) {
        DAC.write( out_bufferL[i] );
        DAC.write( out_bufferL[i] );
      }
      
    } else if (voice_number == 2) {
      // just mono for now
      updateBraidsAudio();
      for (size_t i = 0; i < 32; i++) {
        int16_t sample =   (int16_t) ( (float) inst[0].pd.buffer[i] * env->process() ) ;
        DAC.write( sample );
        DAC.write( sample );
      }
    }



    //counter = 0; // increments on each pass of the timer when the timer writes
  }

  /*
    if (writing) {
      writeSettings(); // after switching, save this state to flash.
    }
  */

  /*
     disabled clouds
     else if (voice_number == 3) {

    // clouds, samplebuffer at same time
    // or braids into buffer directly.
    updateBraidsAudio();

    // copy the braids audio to the clouds input buffer
    clouds::FloatFrame  *input = cloud[0].input;

    for (int i = 0; i < 32; i++) {

    float sample = (float) ( inst[0].pd.buffer[i] / 32768.0f ) * 0.5f;
    //float sample = (float) ( analogRead(CV7) / 1023.0f ) * 0.9f;
    input[i].l = sample;
    input[i].r = sample;  // Mono input

    }
    //for (size_t i = 0; i < 32; ++i) {
    //  sample_buffer[i] = (float) ( analogRead(CV7) ); // arbitrary +1 gain
    //}

    updateCloudsAudio();
    }*/
}



void setup1() {
  delay (1000); // wait for main core to start up perhipherals
}



// second core deals with ui / control rate updates
void loop1() {

  if (! writing) { // don't do shit when eeprom is being written
    
    MIDI.read();
    
    // we need these on boot so the second loop can catch the startup button.
    btn_one.update();
    btn_two.update();
    btn_four.update();


    // at boot permit octave down
    if (just_booting && btn_one.pressed()) {
      pitch_offset = 36;
    }

    just_booting = false;

    unsigned long now = millis();

    // first control elements
    if (voice_number == 0) {
      updatePlaitsControl();
    } else if (voice_number == 1) {
      updateRingsControl();
    }
    // need faster updates
    read_encoders();

    if ( now - update_timer > 5 ) {

      if ( midi_switch == false && midi_switch_setting == false ) {
        voct_midi(CV1);
      }
      read_trigger();
      read_cv();
      read_buttons();
      update_timer = now;

      // display updates
      if (voice_number == 0) {
        displayPlaits();
      } else if (voice_number == 1) {
        displayRings();
      } else if (voice_number == 2) {
        displayBraids();
      } else {
        displayUpdate();
      }
    }
  }
}

void read_buttons() {

  bool doublePressMode = false;
  bool longPress = false;
  int oneState = btn_one.read();
  int twoState = btn_two.read();
  int fourState = btn_four.read();


  // if button one was held for more than 300 millis and we're in rings toggle easteregg
  if ( btn_one.rose() ) {

    btnOneLastTime = btn_one.previousDuration();
    if ( btnOneLastTime > 350 && ! btn_two.pressed()) {
      if ( voice_number == 1 ) {
        easterEgg = !easterEgg;
        longPress = true;
      }
      if ( voice_number == 3 && ! btn_two.pressed()) {
        freeze_in = !freeze_in;
        longPress = true;
      }
    } else {
      engineCount ++;
      if (engineCount > max_engines) {
        engineCount = 0;
      }
      engine_in = engineCount;
    }

  }

  if (btn_four.rose()) {

    // first record our last settings
    if (voice_number == 0) {
      plaits_morph = morph_in;
      plaits_timbre = timbre_in;
      plaits_harm = harm_in;
      plaits_engine = engine_in;
      plaits_position = position_in;
    }
    if (voice_number == 1) {
      rings_morph = morph_in;
      rings_timbre = timbre_in;
      rings_harm = harm_in;
      rings_position = position_in;
      rings_engine = engine_in;
    }
    if (voice_number == 2) {
      braids_morph = morph_in;
      braids_timbre = timbre_in;
      braids_engine = engine_in;
    }
    /* not used */
    if (voice_number == 3) {
      clouds_morph = morph_in;
      clouds_timbre = timbre_in;
      clouds_harm = harm_in;
      clouds_pos = position_in;
      clouds_engine = engine_in;
    }

    voice_number++;

    if (voice_number > 2) voice_number = 0;

    if (voice_number == 0) {
      engine_in = plaits_engine; // engine_in % 17;
      max_engines = 21; // was 15
      morph_in = plaits_morph;
      timbre_in = plaits_timbre;
      harm_in = plaits_harm;
      position_in = plaits_position;

    } else if (voice_number == 1) {
      engine_in = rings_engine; // % 6;
      max_engines = 5;
      morph_in = rings_morph;
      harm_in = rings_harm;
      timbre_in = rings_timbre;
      position_in = rings_position;

    } else if (voice_number == 2 ) {
      engine_in = braids_engine; // engine_in % 46;
      max_engines = 45;
      morph_in = braids_morph;
      timbre_in = braids_timbre;

    } else if (voice_number == 3 ) { // not used
      engine_in = clouds_engine; // engine_in % 46;
      max_engines = 3;
      morph_in = clouds_morph;
      timbre_in = clouds_timbre;
      harm_in = clouds_harm;

    }
  }

  if (btn_one.pressed() && btn_two.pressed()) {
    // rings easter egg mode/ fm engine.
    easterEgg = !easterEgg;
    doublePressMode = true;
  }

  if (!doublePressMode && !longPress) {
    // being tripple shure :)

  }
}

float voct_midiBraids(int cv_in) {

  int val = 0;
  for (int j = 0; j < cv_avg; ++j) val += analogRead(cv_in); // read the A/D a few times and average for a more stable value
  val = val / cv_avg;
  pitch = pitch_offset + map(val, 0.0, 1023.0, mapping_upper_limit, 0.0); // convert pitch CV data value to a MIDI note number
  return pitch - 37; // don't know why, probably tuned to A so -5 + -36 to drop two octaves
}


void voct_midi(int cv_in) {
  int val = 0;
  for (int j = 0; j < cv_avg; ++j) val += analogRead(cv_in); // read the A/D a few times and average for a more stable value
  val = val / cv_avg;

  data = (float) val * 1.0f;
  pitch = map(data, 0.0, 1023.0, mapping_upper_limit, mapping_lower_limit); // convert pitch CV data value to a MIDI note number

  pitch = pitch - pitch_offset;

  pitch_in = pitch;

  // this is a temporary move to get around clicking on trigger + note cv in
  if (pitch != previous_pitch) {
    previous_pitch = pitch;
    // this is the plaits version

  }
}



// in marvelous, moved to digital pin tx/gp0
void read_trigger() {
  int16_t trig = digitalRead(0);
  // tricky conflicts with default uarts
  if ( midi_switch == false && midi_switch_setting == false ) {

    if (trig  == HIGH ) {
      trigger_in = 1.0f;
      //trigger_on = true;
      //if (millis() - envTimer > 50) {
      envTimer = millis();
      env->gate(true);
      //}

    } else  {
      //don't turn off here?
      trigger_in = 0.0f;
      // don't retrigger ADSR too quickly
      //if (millis() - envTimer > 50) {
      envTimer = 0;
      env->gate(false);
      //}
      //trigger_on = false;
    }
    if (voice_number == 0) updateVoicetrigger();

  }

}

void read_cv() {
  // CV updates
  // braids wants 0 - 32767, plaits 0-1

  //plaits and rings cv
  int16_t timbre = avg_cv(CV2);
  timb_mod = (float)timbre;
  timb_mod = mapf( timb_mod, 180.0f, 1023.0f, 0.00f, 1.00f);

  int16_t morph = avg_cv(CV3) ;
  morph_mod = (float) morph;
  morph_mod = mapf ( (float) morph_mod, 180.0f, 1023.0f, 0.00f, 1.000f);

  // don't remember if this was important
  float pos = avg_cv(CV4) * 1.0f ; // f&d noise floor
  pos_mod = mapf (  pos, 180.0f, 1023.0f, 0.00f, 1.000f);
  pos_mod = constrain(pos, 0.00f, 1.00f);

  // plaits
  //int16_t lpgColor =  avg_cv(CV5);
  //lpg_in = mapf( (float) lpgColor, 180.0f, 1023.0f, 0.00f, 1.000f);

  if (voice_number == 0 || voice_number == 1) {
    // plaits

    timb_mod = mapf(timb_mod, 0.0f, 1.0f, 0.0f, 0.8f);

    //if (debug) Serial.print(timb_mod);

    //voices[0].modulations.timbre_patched = true;
    //voices[0].modulations.timbre_patched = false;
    //morph_mod = mapf(morph_mod, 0.02f, 1.0f, -1.0f, 1.0f);

    morph_mod = mapf(morph_mod, 0.0f, 1.0f, 0.0f, 0.8f);

    //voices[0].modulations.morph_patched = true;

  }
  if (voice_number == 3) {

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

  // enc4 is an exceptoin
  enc4.tick();

  // first encoder
  int enc1_pos = enc1.getCount() / 4;

  if ( enc1_pos != enc1_pos_last ) {
    enc1_delta = (enc1_pos - enc1_pos_last) ;
  }

  if ( enc1_delta) {
    float turn = ( enc1_delta * 0.003f ) + timbre_in;
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
    float turn = ( enc2_delta * 0.003f ) + morph_in;
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
    float turn = ( enc3_delta * 0.0031f ) + harm_in;
    CONSTRAIN(turn, 0.f, 1.0f)
    if (debug) Serial.println(turn);
    harm_in = turn;
  }
  enc3_pos_last = enc3_pos;
  enc3_delta = 0;


  int enc4_pos = enc4.getPosition();

  if ( enc4_pos != enc4_pos_last ) {
    engineCount =  (int) enc4.getDirection()  + engineCount ;
    if (engineCount > max_engines) {
      engineCount = 0;
    }
    if (engineCount < 0 ) {
      engineCount = max_engines;
    }
    // plaits is tricky
    if (voice_number == 0) {
      //changePlaitsEngine(engineCount);
    }
    engine_in = engineCount;
  }
  enc4_pos_last = enc4_pos;

}
