// midier scale/mode generation

const midier::Degree scaleDegrees[] = { 1, 2, 3, 4, 5, 6, 7, 8 };

const midier::Note roots[] = {
  midier::Note::C, midier::Note::D,  midier::Note::E, midier::Note::F, midier::Note::G, midier::Note::A, midier::Note::B
};

midier::Mode mode;
int scaleRoot = 0; // start at c, yawn.

int modeIndex = 0;
int currentMode[8];
int octave = 3;

void makeScale(midier::Note root, midier::Mode mode) {

  // the root note of the scale
  const midier::Note scaleRoot = root;

  // we are playing ionian mode which is the major scale
  // if you are not familiar with modes, just know that "ionian" is the major scale and "aeolian" is the minor scale
  //const midier::Mode mode = midier::Mode::Ionian;


  for (midier::Degree scaleDegree : scaleDegrees)
  {
    // find out the interval to be added to the root note for this chord degree and chord quality
    const midier::Interval interval = midier::scale::interval(mode, scaleDegree);

    // calculate the note of this chord degree
    const midier::Note note = scaleRoot + interval;
    currentMode[ scaleDegree - 1 ] = midier::midi::number(note, octave);


  }
  //Serial.println();
}



// midi routines/callbacks

void HandleMidiNoteOn(byte channel, byte note, byte velocity) {
  pitch_in = note;
  trigger_in = velocity / 127.0;


  digitalWrite(LED_BUILTIN, HIGH);
  midi_switch = true;
  //env->reset();
  envTimer = millis();
  env->gate(true);
}
void HandleMidiNoteOff(byte channel, byte note, byte velocity) {

  trigger_in = 0.0f;

  //aSin.setFreq(mtof(float(note)));
  //envelope.noteOn();
  digitalWrite(LED_BUILTIN, LOW);
  envTimer = 0;
  env->gate(false);
}


/* we're default mapping the minilab 3

   knobs
  1 Brightness 74 (Filter cutoff frequency)
  2 Timbre 71 (Filter resonance)
  3 Time 76 (Sound control 7)
  4 Movement 77 (Sound control 8)
  5 FX A Dry/Wet 93 (Chorus level)
  6 FX B Dry/Wet 18 (General purpose)
  7 Delay Volume 19 (General purpose)
  8 Reverb Volume 16 (General purpose
   faders
  1 Bass 82 (General purpose 3)
  2 Midrange 83 (General purpose 4)
  3 Treble 85 (Undefined)
  4 Master Volume 17 (General purpose)
*/

float ccTofloat(byte value) {

  float val = (float) value / 127.0f;
  return constrain( val, 0.00f, 1.00f);
}

void HandleControlChange(byte channel, byte cc, byte value) {
  switch (cc) {
    case 82:
      position_in = ccTofloat(value) ;
      break;
    case 83:
      break;
    case 85:
      break;
    case 17:
      break;
    case 74:
      morph_in = ccTofloat(value) ;
      break;
    case 71:
      timbre_in = ccTofloat(value) ;
      break;
    case 76:
      harm_in = ccTofloat(value) ;
      break;
    case 77:
      position_in = ccTofloat(value) ;
      break;
    case 93:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case 18:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case 19:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case 16:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    default:
      digitalWrite(LED_BUILTIN, LOW);
  }


}



void aNoteOff( float note, int velocity) {
  trigger_in = 0.0f;
  //voices[0].modulations.trigger_patched = false;
  //voices[0].modulations.trigger = 0.f;
  //envelope.noteOff();
  //digitalWrite(LED, LOW);
}

void aNoteOn(float note, int velocity) {
  if (velocity == 0) {
    aNoteOff(note, velocity);
    trigger_in = 0.0f;
    return;
  };


  double trig = randomDouble(0.1, 0.9);
  bool trigger = (trig > 0.1);
  bool trigger_flag = (trigger && (!voices[0].last_trig));

  voices[0].last_trig = trigger;


  if (trigger_flag) {
    trigger_in = trig;
    //decay_in = randomDouble(0.05,0.3);
    voices[0].modulations.trigger_patched = true;
  } else {
    trigger_in = 0.0f;
    voices[0].modulations.trigger_patched = false;
  }

  //voices[0].patch.note = pitch;
  //carrier_freq = note;
  //envelope.noteOn();
  //digitalWrite(LED, HIGH);
}

/*using ErrorCallback                = void (*)(int8_t);
  using NoteOffCallback              = void (*)(Channel channel, byte note, byte velocity);
  using NoteOnCallback               = void (*)(Channel channel, byte note, byte velocity);
  using AfterTouchPolyCallback       = void (*)(Channel channel, byte note, byte velocity);
  using ControlChangeCallback        = void (*)(Channel channel, byte, byte);
  using ProgramChangeCallback        = void (*)(Channel channel, byte);
  using AfterTouchChannelCallback    = void (*)(Channel channel, byte);
  using PitchBendCallback            = void (*)(Channel channel, int);
  using SystemExclusiveCallback      = void (*)(byte * array, unsigned size);
  using TimeCodeQuarterFrameCallback = void (*)(byte data);
  using SongPositionCallback         = void (*)(unsigned beats);
  using SongSelectCallback           = void (*)(byte songnumber);
  using TuneRequestCallback          = void (*)(void);
  using ClockCallback                = void (*)(void);
  using StartCallback                = void (*)(void);
  using TickCallback                 = void (*)(void);
  using ContinueCallback             = void (*)(void);
  using StopCallback                 = void (*)(void);
  using ActiveSensingCallback        = void (*)(void);
  using SystemResetCallback          = void (*)(void);

*/
