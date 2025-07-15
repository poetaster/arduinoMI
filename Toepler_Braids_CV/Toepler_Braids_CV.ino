/*
  (c) 2025 blueprint@poetaster.de
  GPLv3 the libraries are MIT as the originals for STM from MI were also MIT.
*/

/* toepler +
    // Toepler Plus pins
  #define OUT1 (0u)
  #define OUT2 (1u)
  #define SW1 (8u)
  #define CV1 (26u)
  #define CV2 (27u)
  #define CV3 (28u)

*/
bool debug = true;

#include <Arduino.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "potentiometer.h"

//#include <MIDI.h>
//#include <mozzi_midi.h>

long midiTimer;

float pitch;
//float pitch_offset = 4;
float pitch_offset =30;
float freq;

float max_voltage_of_adc = 3.3;
float voltage_division_ratio = 0.3333333333333;
float notes_per_octave = 12;
float volts_per_octave = 1;
float mapping_upper_limit = 120; // (max_voltage_of_adc / voltage_division_ratio) * notes_per_octave * volts_per_octave;
float mapping_lower_limit = 0.0;


/*
  struct Serial1MIDISettings : public midi::DefaultSettings
  {
  static const long BaudRate = 31250;
  static const int8_t TxPin  = 12u;
  static const int8_t RxPin  = 13u;
  };

  MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial1, MIDI, Serial1MIDISettings);
*/

#include <hardware/pwm.h>
#include <PWMAudio.h>

#define SAMPLERATE 48000
#define PWMOUT 0
#define BUTTON_PIN 8
#define LED 13

#include "utility.h"
#include <STMLIB.h>
#include <BRAIDS.h>
#include "braids.h"


#include <Bounce2.h>
Bounce2::Button button = Bounce2::Button();

PWMAudio DAC(PWMOUT);  // 16 bit PWM audio



int engineCount = 0;
int engineInc = 0;

// clock timer  stuff

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     4

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RPi_Pico_TimerInterrupt.h"

//unsigned int SWPin = CLOCKIN;

#define TIMER0_INTERVAL_MS 20.833333333333

//24.390243902439025 // 44.1
// \20.833333333333 running at 48Khz
// 10.416666666667  96kHz

#define DEBOUNCING_INTERVAL_MS   2// 80
#define LOCAL_DEBUG              0

volatile int counter = 0;

// Init RPI_PICO_Timer, can use any from 0-15 pseudo-hardware timers
RPI_PICO_Timer ITimer0(0);

bool TimerHandler0(struct repeating_timer *t) {
  (void) t;
  bool sync = true;
  if ( DAC.availableForWrite()) {
    for (size_t i = 0; i < BLOCK_SIZE; i++) {
      DAC.write( voices[0].pd.buffer[i], sync);
    }
    counter =  1;
  }

  return true;
}

void cb() {
  bool sync = true;
  if (DAC.availableForWrite() >= BLOCK_SIZE) {
    for (int i = 0; i <  BLOCK_SIZE; i++) {
      // out = ;   // left channel called .aux
      DAC.write( voices[0].pd.buffer[i]);
    }
  }
}

void HandleNoteOn(byte channel, byte note, byte velocity) {
  pitch_in = note << 7;
  trigger_in = velocity / 127.0;

  //aSin.setFreq(mtof(float(note)));
  //envelope.noteOn();
  //digitalWrite(LED, HIGH);
}
void HandleNoteOff(byte channel, byte note, byte velocity) {

  trigger_in = 0.0f;

  //aSin.setFreq(mtof(float(note)));
  //envelope.noteOn();
  //digitalWrite(LED, LOW);
}

void voct_midi(int cv_in) {
  

  pitch = map(potvalue[cv_in], 0.0, 4095.0, mapping_upper_limit, 0.0); // convert pitch CV data value to a MIDI note number
  
  pitch = pitch - pitch_offset; // pitch offset drops this octaves down

  if (debug) Serial.println(pitch);
  
  if (pitch > 63) pitch = pitch - 1; //adc correction
  
  pitch_in = pitch ;
  if (pitch != previous_pitch) { 
    trigger_in = 0.0f;  
    previous_pitch = pitch;
    trigger_in = 1.0f; //retain for cv only input?
  }

}




void setup() {

  if (debug) {
    Serial.begin(57600);
    Serial.println(F("YUP"));
  }
  //Serial1.begin(9600);
  
  analogReadResolution(12);
  // thi is to switch to PWM for power to avoid ripple noise
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);

  pinMode(AIN0, INPUT);
  pinMode(AIN1, INPUT);
  pinMode(AIN2, INPUT);
  //pinMode(13u, INPUT_PULLDOWN);
  
  //Serial1.setRX(13);
  
  pinMode(LED, OUTPUT);
  //MIDI.setHandleNoteOn(HandleNoteOn);  // Put only the name of the function
  //MIDI.setHandleNoteOff(HandleNoteOff);  // Put only the name of the function
  // Initiate MIDI communications, listen to all channels (not needed with Teensy usbMIDI)
  //MIDI.begin(MIDI_CHANNEL_OMNI);

  button.attach( BUTTON_PIN , INPUT);
  button.interval(5);
  button.setPressedState(LOW);

  // pwm timing setup, we're using a pseudo interrupt

  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS, TimerHandler0)) // that's 48kHz
  {
    if (debug) Serial.print(F("Starting  ITimer0 OK, millis() = ")); Serial.println(millis());
  }  else {
    if (debug) Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
  }


  // set up Pico PWM audio output
  DAC.setBuffers(4, 32); // plaits::kBlockSize); // DMA buffers
  //DAC.onTransmit(cb);
  DAC.setFrequency(SAMPLERATE);
  DAC.begin();


  // init the braids voices
  initVoices();

  // initial reading of the pots with debounce
  readpot(0);
  readpot(1);
  readpot(2);


  int16_t timbre = (map(potvalue[0], POT_MIN, POT_MAX, 0, 32767));
  timbre_in = timbre;

  int16_t morph = (map(potvalue[1], POT_MIN, POT_MAX, 0, 32767));
  morph_in = morph;

  // fm / pitch updates
  // int16_t  pitch = map(potvalue[2], POT_MIN, POT_MAX, 16383, 0); // convert pitch CV data value to valid range
  // pitch_in = pitch - 1638;
  // used to switch between FM and note on cv3

  midiTimer = millis();

}


void loop() {

  if ( counter > 0 ) {
    updateBraidsAudio();
    counter = 0; // increments on each pass of the timer after the timer writes samples
  }

}

// second core dedicated to display foo

void setup1() {
  delay (200); // wait for main core to start up perhipherals
}


char sBuffer[100];
int idx;


// second core deals with ui / control rate updates
void loop1() {

  //MIDI.read();
  uint32_t now = millis();

  int16_t timbre = (map(potvalue[0], POT_MIN, POT_MAX, 0, 32767));
  timbre_in = timbre;
  int16_t morph = (map(potvalue[1], POT_MIN, POT_MAX, 0, 32767));
  morph_in = morph;

  // fm / pitch updates
  //int16_t  pitch = map(potvalue[2], POT_MIN, POT_MAX, 127, 0); // convert pitch CV data value to valid range
 
  voct_midi(2);


  button.update();
  if ( button.pressed() ) {
    engineCount ++;
    if (engineCount > 46) {
      engineCount = 0;
    }
    engine_in = engineCount;
  }

  // reading A/D seems to cause noise in the audio so don't do it too often
  if ((now - pot_timer) > POT_SAMPLE_TIME) {
    readpot(0);
    readpot(1);
    readpot(2);



    pot_timer = now;
  }




}
