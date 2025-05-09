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
  //voices[0].modulations.trigger_patched = false;
  //voices[0].modulations.trigger = 0.f;
  //envelope.noteOff();
  //digitalWrite(LED, LOW);
}

void aNoteOn(float note, int velocity) {
  if (velocity == 0) {
    aNoteOff(note, velocity);
    trigger_in = 0.0;
    return;
  };

  trigger_in = randomDouble(0.0, 1.0);
  if (trigger_in < 0.3) {
    trigger_in = 0.0;
  }


  //voices[0].patch.note = pitch;
  //carrier_freq = note;
  //envelope.noteOn();
  //digitalWrite(LED, HIGH);
}
