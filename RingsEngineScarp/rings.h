#include "rings/dsp/part.h"
#include "rings/dsp/strummer.h"
#include "rings/dsp/string_synth_part.h"
#include "rings/dsp/dsp.h"

const size_t kBlockSize = rings::kMaxBlockSize;

float rings::Dsp::sr = 48000.0f;
float rings::Dsp::a3 = 440.0f / 48000.0f;


struct Ring {
  rings::Part             part;
  rings::StringSynthPart  string_synth;
  rings::Strummer         strummer;
  rings::PerformanceState performance_state;
  rings::Patch            patch;

  uint16_t                reverb_buffer[32768];

  float                   *silence;
  float                   *out, *aux;     // output buffers
  float                   *input; // input buffer

  bool                    prev_trig;
  int                     prev_poly;
  
    // we're using the original rendering, not vbs
  int16_t                 obuff[kBlockSize]; // from float[]s below in update routine
  int16_t                 abuff[kBlockSize]; // ditto

};

struct Ring voices[1];

void updateRingsAudio() {

  rings::Patch *patch = &voices[0].patch;
  rings::PerformanceState *ps = &voices[0].performance_state;

  size_t  size = rings::kMaxBlockSize;

  // only do trigger actions if we are in focus.
  bool trigger = (trigger_in > 0.0f);
  bool trigger_flag = (trigger && (!voices[0].prev_trig));
  if ( trigger_flag) {
    ps->strum = true;
  } else {
    ps->strum = false;
  }
  voices[0].prev_trig = trigger;

  if (easterEgg) {
    voices[0].strummer.Process(NULL, size, ps);
    voices[0].string_synth.Process(*ps, *patch, voices[0].silence, voices[0].out, voices[0].aux, size);
  }
  else {
    voices[0].strummer.Process(voices[0].input, size, ps);
    voices[0].part.Process(*ps, *patch, voices[0].input, voices[0].out, voices[0].aux, size);
  }
  float gain;
  if (engine_in !=0 && engine_in !=3) {
    gain = 1.0;
  } else if (engine_in == 5) {
    gain = 1.1;
  } else {
    gain = 1.0;
  }

  for (size_t i = 0; i < size; ++i) {
    // we're reducing to mono for now. the stereo below does work..
    voices[0].obuff[i] =   stmlib::Clip16(static_cast<int32_t>(  (voices[0].out[i]  + voices[0].aux[i]  ) * 32768.0f));
      
    //out_bufferR[i] = stmlib::Clip16(static_cast<int32_t>((voices[0].aux[i]+.11) * 32768.0f)); // the .11 is gainwhich should be done by calibration.
    //out_bufferL[i] = stmlib::Clip16(static_cast<int32_t>((voices[0].out[i]+.11) * 32768.0f)); // was obuff

  }


}

void updateRingsControl() {
  float   *trig_in; // = IN(1);
  float   voct_in = pitch_in ;//* 1.0f;

  float   struct_in = harm_in;
  float   bright_in = timbre_in;
  float   damp_in = morph_in;
  float   pos_in = 0.25f ;

  short   model = engine_in;
  short   polyphony = 4;
  bool    intern_exciter = false;
  bool    easter_egg = easterEgg;
  bool    bypass = false;

  rings::Patch *patch = &voices[0].patch;
  rings::PerformanceState *ps = &voices[0].performance_state;

  // float   *in = voices[0].input;
  size_t  size = rings::kMaxBlockSize;

  // check input rates for excitation input
/*
  if ( timb_mod > 0.08f ) {
    // input on CV3
    // intern_exciter should be off, but user can override
    voices[0].input = CV1_buffer;
    ps->internal_exciter = intern_exciter;
  } else {
    // if there's no audio input, set input to zero...
    voices[0].input = voices[0].silence;
    // ... and use internal exciter!
    ps->internal_exciter = true;
  }*/

    // if there's no audio input, set input to zero...
    voices[0].input = voices[0].silence;
    // ... and use internal exciter!
    ps->internal_exciter = true;
    
  /* from the original with gain foo
    for (size_t i = 0; i < size; ++i) {
    float in_sample = static_cast<float>(input[i].r) / 32768.0f;
    float error, gain;
    error = in_sample * in_sample - in_level;
    in_level += error * (error > 0.0f ? 0.1f : 0.0001f);
    gain = in_level <= kNoiseGateThreshold
         ? (1.0f / kNoiseGateThreshold) * in_level : 1.0f;
    in[i] = gain * in_sample;
    }*/


  if (engine_in == 3 && !easterEgg) polyphony = 2; // the 2op fm freaks out with higher polyphony

  // set resonator model
  CONSTRAIN(model, 0, 5);
  voices[0].part.set_model(static_cast<rings::ResonatorModel>(model));
  voices[0].string_synth.set_fx(static_cast<rings::FxType>(model));

  // set polyphony
  if (polyphony != voices[0].prev_poly) {
    CONSTRAIN(polyphony, 1, 4);
    voices[0].part.set_polyphony(polyphony);
    voices[0].string_synth.set_polyphony(polyphony);
    voices[0].prev_poly = polyphony;
  }

  // set pitch
  CONSTRAIN(voct_in, 0.f, 114.f);
  ps->tonic = 12.f;
  ps->note = voct_in;

  // set params
  CONSTRAIN(struct_in, 0.0f, 1.0f);    //0.9995f
  patch->structure = struct_in;

  float chord = struct_in * (rings::kNumChords - 1);
  voices[0].performance_state.chord = roundf(chord);

  CONSTRAIN(bright_in, 0.0f, 1.0f);
  patch->brightness = bright_in;

  CONSTRAIN(damp_in, 0.0f, 1.0f);
  patch->damping = damp_in;

  CONSTRAIN(pos_in, 0.0f, 1.0f);
  patch->position = pos_in;

  // check trigger input
  //
  voices[0].part.set_bypass(bypass);

}



// initialize voice parameters
void initRings() {

  //rings::Dsp::setSr(SAMPLERATE); // we removed this

  // allocate memory + init with zeros
  //voices[0].reverb_buffer = (uint16_t*)malloc(32768 * sizeof(uint16_t));
  //memset(voices[0].reverb_buffer, 0, 32768 * sizeof(uint16_t));

  voices[0].silence = (float*)malloc(rings::kMaxBlockSize * sizeof(float));
  memset(voices[0].silence, 0, rings::kMaxBlockSize * sizeof(float));

  voices[0].input = (float*)malloc(rings::kMaxBlockSize * sizeof(float));
  memset(voices[0].input, 0, rings::kMaxBlockSize * sizeof(float));

  voices[0].out = (float *)malloc(rings::kMaxBlockSize * sizeof(float));
  voices[0].aux = (float *)malloc(rings::kMaxBlockSize * sizeof(float));

  // zero out...
  memset(&voices[0].strummer, 0, sizeof(voices[0].strummer));
  memset(&voices[0].part, 0, sizeof(voices[0].part));
  memset(&voices[0].string_synth, 0, sizeof(voices[0].string_synth));

  voices[0].strummer.Init(0.01, 48000.0f / rings::kMaxBlockSize);
  voices[0].part.Init(voices[0].reverb_buffer);
  voices[0].string_synth.Init(voices[0].reverb_buffer);

  voices[0].part.set_polyphony(4);
  voices[0].part.set_model(rings::RESONATOR_MODEL_MODAL);

  voices[0].string_synth.set_polyphony(4);
  voices[0].string_synth.set_fx(rings::FX_FORMANT);
  voices[0].prev_poly = 1;

  voices[0].performance_state.fm = 0.f;       // TODO: fm not used, maybe later...
  voices[0].prev_trig = false;

  // we're fixing this for the moment until we can test with input
  voices[0].performance_state.internal_exciter = true;

  // let's see
  voices[0].performance_state.internal_strum = false;

  updateRingsAudio() ;

  // check input rates
  /*
    if(INRATE(0) == calc_FullRate)
      voices[0].performance_state.internal_exciter = false;
    else
      voices[0].performance_state.internal_exciter = true;

    if(INRATE(1) == calc_ScalarRate)
      voices[0].performance_state.internal_strum = true;
    else
      voices[0].performance_state.internal_sturum = false;

    if(INRATE(2) == calc_ScalarRate)
      voices[0].performance_state.internal_note = true;
    else
      voices[0].performance_state.internal_note = false;
  */

}
