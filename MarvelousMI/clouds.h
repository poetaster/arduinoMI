
// the following is largely from the mi-Ugens sourec MiClouds.cpp
#include "clouds/dsp/granular_processor.h"
#include "clouds/resources.h"
#include "clouds/dsp/audio_buffer.h"
#include "clouds/dsp/mu_law.h"
#include "clouds/dsp/sample_rate_converter.h"



const uint16_t cloudsAudioBlockSize = 32;        // sig vs can't be smaller than this!
const uint16_t kNumArgs = 14;

enum ModParams {
  PARAM_PITCH,
  PARAM_POSITION,
  PARAM_SIZE,
  PARAM_DENSITY,
  PARAM_TEXTURE,
  PARAM_DRYWET,
  PARAM_CHANNEL_LAST
};

struct Cloud {

  clouds::GranularProcessor   *processor;

  // buffers
  //uint8_t     *large_buffer;
  //uint8_t     *small_buffer;
  // Pre-allocate big blocks in main memory and CCM. No malloc here.
  uint8_t     large_buffer[118784];
  uint8_t     small_buffer[65536 - 128];

  // parameters
  float       in_gain;
  bool        freeze;
  bool        trigger, previous_trig;
  bool        gate;

  float       pot_value_[PARAM_CHANNEL_LAST];
  float       smoothed_value_[PARAM_CHANNEL_LAST];
  float       coef;      // smoothing coefficient for parameter changes

  clouds::FloatFrame  input[cloudsAudioBlockSize];
  clouds::FloatFrame  output[cloudsAudioBlockSize];

  float       sr;
  long        sigvs;

  bool        gate_connected;
  bool        trig_connected;
  uint32_t      pcount;


  clouds::SampleRateConverter < -clouds::kDownsamplingFactor, 45, clouds::src_filter_1x_2_45 > src_down_;
  clouds::SampleRateConverter < +clouds::kDownsamplingFactor, 45, clouds::src_filter_1x_2_45 > src_up_;

};

struct Cloud cloud[1];

void initClouds() {

  int largeBufSize = 118784;
  int smallBufSize =  65536 - 128;

  // we fixed it with static inits in the unit
  //cloud[0].large_buffer = (uint8_t*)malloc(largeBufSize * sizeof(uint8_t));
  //cloud[0].small_buffer = (uint8_t*)malloc(smallBufSize * sizeof(uint8_t));

  cloud[0].sr = 32000.0f ;// SAMPLERATE;
  cloud[0].processor = new clouds::GranularProcessor;
  memset(cloud[0].processor, 0, sizeof(*cloud[0].processor));

  cloud[0].processor->Init(cloud[0].large_buffer, largeBufSize, cloud[0].small_buffer, smallBufSize);
  cloud[0].processor->set_sample_rate(cloud[0].sr);
  cloud[0].processor->set_num_channels(2);       // always use stereo setup
  cloud[0].processor->set_low_fidelity(false);
  cloud[0].processor->set_playback_mode(clouds::PLAYBACK_MODE_GRANULAR);

  // init values
  cloud[0].pot_value_[PARAM_PITCH] = cloud[0].smoothed_value_[PARAM_PITCH] = 0.f;
  cloud[0].pot_value_[PARAM_POSITION] = cloud[0].smoothed_value_[PARAM_POSITION] = 0.f;
  cloud[0].pot_value_[PARAM_SIZE] = cloud[0].smoothed_value_[PARAM_SIZE] = 0.5f;
  cloud[0].pot_value_[PARAM_DENSITY] = cloud[0].smoothed_value_[PARAM_DENSITY] = 0.1f;
  cloud[0].pot_value_[PARAM_TEXTURE] = cloud[0].smoothed_value_[PARAM_TEXTURE] = 0.5f;
  cloud[0].pot_value_[PARAM_DRYWET] = cloud[0].smoothed_value_[PARAM_DRYWET] = 1.f;
  cloud[0].processor->mutable_parameters()->stereo_spread = 0.5f;
  cloud[0].processor->mutable_parameters()->reverb = 0.f;
  cloud[0].processor->mutable_parameters()->feedback = 0.f;
  cloud[0].processor->mutable_parameters()->freeze = false;
  cloud[0].in_gain = 1.0f;
  cloud[0].coef = 0.1f;
  cloud[0].previous_trig = false;
  cloud[0].src_down_.Init();
  cloud[0].src_up_.Init();
  cloud[0].pcount = 0;
  // have to think about this :)
  //uint16_t numAudioInputs = cloud[0].mNumInputs - kNumArgs;
  if (debug) Serial.println(F("INIT DONE"));



}


// main audio called from loop, cpu 2)
void updateCloudsAudio() {

  //  MiClouds.ar(input, pit: -15.0, pos: 0.5, size: 0.25,  dens: dens, tex: 0.5, drywet: 1, mode: 0);
  float timbre = (timbre_in + timb_mod);
  float morph = (morph_in + morph_mod);
    
  float modulation;
  

  if (pos_mod < 0.05) {
    modulation = 1.0f;
  } else {
    modulation = pos_mod;
  }
  float harm = (harm_in + pos_mod);
  
  CONSTRAIN(harm, 0.0f, 1.0f);
  float   pitch =   mapf(pitch_in, 0.0, 60.0, -48.0, 48.0);
  float   in_gain = 0.5f; // harm_in; //IN0(6);
  float   spread = 0.5f;// IN0(7);
  float   reverb = 0.4f; // IN0(8);
  float   fb =  0.3f; // IN0(9);
  float   siz = constrain(harm, 0.f, 1.0f) ;// 0.35f;
  float   dens = constrain(morph, 0.f, 1.0f);;
  float   tex = constrain (timbre, 0.f, 1.0f) ;
  float   posi = constrain(pos_mod, 0.f, 1.f);
  float   drywet = 1.0f; // constrain(clouds_dw_in, 0.3f, 1.0f);
  bool    freeze = freeze_in;
  short   engine = constrain(engine_in, 0, 3); // 0 -3
  bool    lofi = 0; // IN0(12) > 0.f;

  int vs = 32; //inNumSamples; // hmmmm

  clouds::FloatFrame  *input = cloud[0].input;
  clouds::FloatFrame  *output = cloud[0].output;

  float       *smoothed_value = cloud[0].smoothed_value_;
  float       coef = cloud[0].coef;
  clouds::GranularProcessor   *gp = cloud[0].processor;
  clouds::Parameters          *p = gp->mutable_parameters();

  smoothed_value[PARAM_PITCH] += coef * (pitch - smoothed_value[PARAM_PITCH]);
  p->pitch =  smoothed_value[PARAM_PITCH];

  /*
    float note = calibration_data_->pitch_offset;
    note += smoothed_adc_value_[ADC_V_OCT_CV] * calibration_data_->pitch_scale;
    if (fabs(note - note_) > 0.5f) {
      note_ = note;
    } else {
      ONE_POLE(note_, note, 0.2f)
    }

    parameters->pitch += note_;
    CONSTRAIN(parameters->pitch, -48.0f, 48.0f);

  */
  
  cloud[0].pot_value_[PARAM_DRYWET] = cloud[0].smoothed_value_[PARAM_DRYWET] = drywet;

  /* this was from the original cv input
    for (int i = 1; i < PARAM_CHANNEL_LAST; ++i) {
    float value = 0.5f; // 0.0f; //IN0(i);
    value = constrain(value, 0.0f, 1.0f);

    smoothed_value[i] += coef * (value - smoothed_value[i]);
    }*/

  smoothed_value[PARAM_POSITION] += coef * (posi - smoothed_value[PARAM_POSITION]);
  p->position = smoothed_value[PARAM_POSITION];

  smoothed_value[PARAM_SIZE] += coef * (siz - smoothed_value[PARAM_SIZE]);
  p->size = smoothed_value[PARAM_SIZE];

  smoothed_value[PARAM_DENSITY] += coef * (dens - smoothed_value[PARAM_DENSITY]);
  p->density = smoothed_value[PARAM_DENSITY];


  smoothed_value[PARAM_TEXTURE] += coef * (tex - smoothed_value[PARAM_TEXTURE]);
  p->texture = smoothed_value[PARAM_TEXTURE];

  p->dry_wet = smoothed_value[PARAM_DRYWET];
  p->stereo_spread = spread;
  p->reverb = reverb;
  p->feedback = fb;
  gp->set_low_fidelity(lofi);
  gp->set_freeze(freeze);
  gp->set_playback_mode(static_cast<clouds::PlaybackMode>(engine));

  // uint16_t trig_rate = INRATE(13); // A non-positive to positive transition causes a trigger to happen.
  // currently writing the input buffer in the loop
 /* for (int i = 0; i < cloudsAudioBlockSize; ++i) {
    
    input[i].l = (float) ( ( sample_buffer[i] / 4095.0f ) *  in_gain) ;
    input[i].r = input[i].l;
  }*/

  bool trigger = false;
  if (trigger_in == 1.0f) {
    trigger = true;
  }

  p->trigger = (trigger && !cloud[0].previous_trig);
  cloud[0].previous_trig = trigger;

  gp->Process(input, output, cloudsAudioBlockSize);
  gp->Prepare();      // why here?

  if (p->trigger)
    p->trigger = false;

  for (int i = 0; i < cloudsAudioBlockSize; ++i) {
    out_bufferL[i] = stmlib::Clip16( static_cast<int32_t>(  (  output[i].l )  * 32768.0f  ) ) ;

  }


}
