
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
