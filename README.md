# arduinoMI
Ports of mutable intstruments eurorack code to arduino.

Current status, 16.12.2025:
* braids works well on the RP2040/RP2350
* tides works well, or so I believe on RP2040/RP2350
* rings is working well with a polyphony of 4 on RP2350
* elements RP2350 only
* plaits works fine on the RP2350 only
* clouds works with reservations :) it works well, but running at 48kHz is pushing it

This port was possible because of the work of Volker Boehm to bring MI modules to supercollider: https://github.com/v7b1/mi-UGens Very cool.

A simple sketch with no other io other than PWM pin 22 is located in the examples folder of each submodule. It cycles through all voices with random parameters every 3 seoncds.

The examples are duplicated as directories in this 'monorepo' since working on help files requires saving, yadayada. So:
BraidsEngines, PlaitsEngines, RingsEngines, are just duplicates for ease of editing.

A number of sketches for the tonetoys scarp (my button punching pico project https://github.com/poetaster/scarp) are included here as it's convenient. For me :)

To use the sketch, copy (or clone the submodules) STMLIB and BRAIDS (for instance) directories to you ~/Arduino/libraries folder, open the BraidsEngines sketch change 

#define PWMOUT 22

to match whatever pin your using and

choose 250mHz overclocking (well, 200 will do it too, but, 250 is stable and some require all out 276mHz) and optimize -O2 (or -O3). Then install it on your pico. 

You should hear it step through the patches making small changes as it goes.


The STMLIB, BRAIDS and PLAITS directories can be placed in your Arduino/libraries/ folder and used as follows, for braids & plaits:
```
#pragma once

#include <STMLIB.h>
#include <BRAIDS.h>

#include "braids/envelope.h"
#include "braids/macro_oscillator.h"
#include "braids/quantizer.h"
#include "braids/signature_waveshaper.h"
#include "braids/quantizer_scales.h"
#include "braids/vco_jitter_source.h"


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


// plaits dsp
#include <STMLIB.h>
#include <PLAITS.h>

#include "plaits/dsp/dsp.h"
#include "plaits/dsp/voice.h"

plaits::Modulations modulations;
plaits::Patch patch;
plaits::Voice voice;

char shared_buffer[16384];
stmlib::BufferAllocator allocator;

//float a0 = (440.0 / 8.0) / kSampleRate; //48000.00;
const size_t   kBlockSize = plaits::kBlockSize;

plaits::Voice::Frame outputPlaits[ plaits::kBlockSize];

struct Unit {
  plaits::Voice       *voice_;
  plaits::Modulations modulations;
  plaits::Patch       patch;
  float               transposition_;
  float               octave_;
  short               trigger_connected;
  short               trigger_toggle;

  char                *shared_buffer;
  void                *info_out;
  bool                prev_trig;
  float               sr;
  int                 sigvs;
};

```

See the included sketches.

