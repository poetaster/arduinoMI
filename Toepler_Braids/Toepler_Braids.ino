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
bool debug = false;

#include <Arduino.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "potentiometer.h"

#include <MIDI.h>
#include <mozzi_midi.h>

struct Serial1MIDISettings : public midi::DefaultSettings
{
  static const long BaudRate = 31250;
  static const int8_t TxPin  = 12u;
  static const int8_t RxPin  = 13u;
};

MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial1, MIDI, Serial1MIDISettings);

#include <hardware/pwm.h>
#include <PWMAudio.h>

#define SAMPLERATE 48000
#define PWMOUT 0
#define BUTTON_PIN 8
#define LED 13
// cheat. belongs in braids.h, but I need it forward
int32_t previous_pitch;

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
      DAC.write( voices[0].pd.buffer[i]);
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
  pitch_in = note;
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

void setup() {

  if (debug) {
    //Serial.begin(57600);
    //Serial.println(F("YUP"));
  }
  analogReadResolution(12);
  // thi is to switch to PWM for power to avoid ripple noise
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);

  pinMode(AIN0, INPUT);
  pinMode(AIN1, INPUT);
  pinMode(AIN2, INPUT);

  pinMode(LED, OUTPUT);
  MIDI.setHandleNoteOn(HandleNoteOn);  // Put only the name of the function
  MIDI.setHandleNoteOff(HandleNoteOff);  // Put only the name of the function
  // Initiate MIDI communications, listen to all channels (not needed with Teensy usbMIDI)
  MIDI.begin(MIDI_CHANNEL_OMNI);

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



  // init the braids voices
  initVoices();

  // initial reading of the pots with debounce
  readpot(0);
  readpot(1);
  DAC.begin();
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




// second core deals with ui / control rate updates
void loop1() {

  uint32_t now = millis();
  // pot updates
  // reading A/D seems to cause noise in the audio so don't do it too often
  if ((now - pot_timer) > POT_SAMPLE_TIME) {
    readpot(0);
    readpot(1);
    readpot(2);
    pot_timer = now;

    // 0, 1 are AIN0, AIN1 for timbre/color cv control
    if (!potlock[0]  ) { // change sample if pot has moved enough
      uint16_t timbre = (uint16_t)(map(potvalue[0], POT_MIN, POT_MAX, 0, 32767));
      timbre_in = timbre;
    }
    if (!potlock[1]  ) { // change sample if pot has moved enough
      uint16_t morph = (uint16_t)(map(potvalue[1], POT_MIN, POT_MAX, 0, 32767));
      morph_in = morph;
    }
    // fm / pitch updates
    int16_t pitch = map(potvalue[2], 0, 4095, 12, 127); // cv for pitch was midi note << 7
    pitch_in = pitch_in;
    MIDI.read();
    
  }
  




  /*
    if (digitalRead(13) == HIGH) {
       trigger_in = 1.0f;
       Serial.println("trigger high");
       //  Serial.println(pitch);
      } else {
       trigger_in = 0.0f;
      }
  */

  /* float trigger = randomDouble(0.0, 1.0); // Dust.kr( LFNoise2.kr(0.1).range(0.1, 7) );
    if (trigger > 0.2 ) {
    trigger_in = trigger;
    }
    float timbre = randomDouble(0.0, 1.0); //LFTri.kr(0.07, 0, 0.5, 0.5).range(0.0, 1.0);
    float morph = randomDouble(0.1, 0.8) ; //LFTri.kr(0.11, 0, 0.5, 0.5).squared;
    float pitch = randomDouble(38, 64); // TIRand.kr(24, 48, trigger);
    float decay = randomDouble(0.1, 0.4);

  */

  /* VB
    var sub = SinOsc.ar(pitch.midicps, 0, 0.1);
    var mi = MiPlaits.ar( pitch, engine, harmonics, timbre, morph,
     trigger: trigger, decay: 0.8, lpg_colour: 0.2, mul: 0.5);
    mi + sub
  */
  float harmonics = randomDouble(0.0, 1.0); // SinOsc.kr(0.03, 0, 0.5, 0.5).range(0.0, 1.0);
  harm_in = harmonics;

  button.update();
  if ( button.pressed() ) {
    engineCount ++;
    if (engineCount > 46) {
      engineCount = 0;
    }
    engine_in = engineCount;
  }
  /*
    engineInc++ ;
    if (engineInc > 3) {
      engineCount ++; // don't switch engine so often :)
      engineInc = 0;
    }
    if (engineCount > 46) engineCount = 0;
    engine_in = engineCount;
    delay(3000);
  */

}
