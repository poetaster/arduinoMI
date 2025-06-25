
// plaits dsp

plaits::Modulations modulations;
plaits::Patch patch;
plaits::Voice voice;

//Settings settings;
//Ui ui;
//UserData user_data;
//UserDataReceiver user_data_receiver;

stmlib::BufferAllocator allocator;

//float a0 = (440.0 / 8.0) / kSampleRate; //48000.00;
const size_t   kBlockSize = plaits::kBlockSize;

plaits::Voice::Frame outputPlaits[plaits::kBlockSize];
//plaits::Voice::out_buffer renderBuffer;

struct Unit {
  plaits::Voice       *voice_;
  plaits::Modulations modulations;
  plaits::Patch       patch;
  float               transposition_;
  float               octave_;
  short               trigger_connected;
  short               trigger_toggle;
  bool                last_trig; // from braids

  char                *shared_buffer;
  void                *info_out;
  bool                prev_trig;
  float               sr;
  int                 sigvs;
};

struct Unit voices[1];



// initialize voice parameters
void initPlaits() {
  // init some params
  //voices[0] = {};
  //voices[0].modulations = modulations;
  voices[0].modulations.engine = 0;
  voices[0].patch = patch;
  voices[0].patch.engine = 0;
  voices[0].transposition_ = 0.;
  voices[0].patch.decay = decay_in; //0.5f;
  voices[0].patch.lpg_colour = lpg_in;

  voices[0].patch.note = 48.0;
  voices[0].patch.harmonics = 0.5;
  voices[0].patch.morph = 0.3;
  voices[0].patch.timbre = 0.3;
  voices[0].last_trig = false;

  voices[0].shared_buffer = (char*)malloc(32756);
  // init with zeros
  memset(voices[0].shared_buffer, 0, 32756);

  stmlib::BufferAllocator allocator(voices[0].shared_buffer, 32756);

  voices[0].voice_ = new plaits::Voice;
  voices[0].voice_->Init(&allocator);

  memset(&voices[0].patch, 0, sizeof(voices[0].patch));
  memset(&voices[0].modulations, 0, sizeof(voices[0].modulations));

  // start with no CV input
  voices[0].prev_trig = false;
  voices[0].modulations.timbre_patched = false;  //(INRATE(3) != calc_ScalarRate);
  voices[0].modulations.morph_patched = false;   // (INRATE(4) != calc_ScalarRate);
  voices[0].modulations.trigger_patched = false; //(INRATE(5) != calc_ScalarRate);
  voices[0].modulations.level_patched = false;   // (INRATE(6) != calc_ScalarRate);
  // TODO: we don't have an fm input yet.
  voices[0].modulations.frequency_patched = false;

}

void updatePlaitsAudio() {
  voices[0].voice_->Render(voices[0].patch, voices[0].modulations,  outputPlaits,  plaits::kBlockSize);
  for (size_t i = 0; i < plaits::kBlockSize; i++) {
    out_bufferL[i] = outputPlaits[i].out ;
  }
}

void updatePlaitsControl() {
  voices[0].patch.engine = engine_in;
  voices[0].patch.note = pitch_in;
  voices[0].patch.harmonics = harm_in;
  voices[0].patch.morph = morph_in;
  voices[0].patch.timbre = timbre_in;

  voices[0].patch.timbre_modulation_amount = timb_mod;
  voices[0].patch.morph_modulation_amount = morph_mod;

  /*
    voices[0].octave_ = octave_in;
     voices[0].patch.decay = 0.5f;
    voices[0].patch.lpg_colour = 0.2;
  */
}

void updateVoicetrigger() {

  //trigger_in = 1.0f; //retain for cv only input?
  //if (debug) Serial.println(pitch);

  bool trigger = (trigger_in == 1.0f);
  bool trigger_flag = (trigger && (!voices[0].last_trig));
  voices[0].last_trig = trigger;

  if (trigger_flag) {
    voices[0].modulations.trigger_patched = true;
  } else {
    voices[0].modulations.trigger_patched = false;
  }
  //trigger_in = 0.0f;
}
