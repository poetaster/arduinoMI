// midier scale/mode generation

const midier::Degree scaleDegrees[] = { 1, 2, 3, 4, 5, 6, 7, 8 };

const midier::Note roots[] = {
  midier::Note::C, midier::Note::D,  midier::Note::E, midier::Note::F, midier::Note::G, midier::Note::A, midier::Note::B
};

midier::Mode mode;
int scaleRoot = 0; // start at c, yawn.

int modeIndex = 0;
int currentMode[8];


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
    currentMode[ scaleDegree - 1 ] = midier::midi::number(note, 4);


  }
  //Serial.println();
}



void HandleNoteOff(byte channel, byte note, byte velocity) {
  //envelope.noteOff();

  //digitalWrite(LED, LOW);
}

void HandleNoteOn(byte channel, byte note, byte velocity) {
  if (velocity == 0) {
    HandleNoteOff(channel, note, velocity);
    return;
  }
  //carrier_freq = mtof(note);
  //envelope.noteOn();
  digitalWrite(LED, HIGH);
}

void aNoteOff( float note, int velocity) {
  trigger_in = 0.0;
}

void aNoteOn(float note, int velocity) {
  trigger_in = randomDouble(0.0, 1.0);
  if (trigger_in < 0.1) {
    trigger_in = 0.0f;
  }
}
