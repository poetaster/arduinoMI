# arduinoMI
Ports of mutable intstruments eurorack code to arduino.

This port was possible because of the work of Volker Boehm to bring MI modules to supercollider: https://github.com/v7b1/mi-UGens Very cool.

Still need to implement proper audio rendering in plaits, but the engines from plaits all make sound :) And braids is already very clean!

I'll release basic sketches without all the scarp hardware when I have a bit of time. I pushed to scarp since it makes it easy to test with inputs. Which is why I built scarp after all :)

The STMLIB, BRAIDS and PLAITS directories can be placed in your Arduino/libraries/ folder and used as follows, for plaits:
```
// plaits dsp
#include <STMLIB.h>
#include <PLAITS.h>

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

Curently, plaits is still a bit noisy but braids is really, really nice. 
