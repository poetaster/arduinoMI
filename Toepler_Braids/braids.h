#pragma once

// braids dsp

//const uint16_t decimation_factors[] = { 1, 2, 3, 4, 6, 12, 24 };
const uint16_t bit_reduction_masks[] = {
  0xffff,
  0xfff0,
  0xff00,
  0xf800,
  0xf000,
  0xe000,
  0xc000
};

#define     MI_SAMPLERATE      96000.f
#define     BLOCK_SIZE          32      // --> macro_oscillator.h !
#define     SAMP_SCALE          (float)(1.0 / 32756.0)



typedef struct
{
  braids::MacroOscillator *osc;

  float       samps[BLOCK_SIZE] ;
  int16_t     buffer[BLOCK_SIZE];
  uint8_t     sync_buffer[BLOCK_SIZE];

} PROCESS_CB_DATA ;

char shared_buffer[16384];

//float a0 = (440.0 / 8.0) / kSampleRate; //48000.00;
const size_t   kBlockSize = BLOCK_SIZE;


struct Unit {
  braids::Quantizer   *quantizer;
  braids::SignatureWaveshaper *ws;
  braids::Envelope *envelope;

  bool            last_trig;
  // resampler
  //SRC_STATE       *src_state;

  PROCESS_CB_DATA pd;
  float           *samples;
  float           ratio;
};

static long src_input_callback(void *cb_data, float **audio);

struct Unit voices[1];

// Plaits modulation vars, reusing names
int16_t morph_in = 1000; // IN(4);
float trigger_in; //IN(5);
float level_in = 0.0f; //IN(6);
float harm_in = 0.1f;
int16_t timbre_in = 1000;
int engine_in = 11;

float fm_mod = 0.0f ; //IN(7);
float timb_mod = 0.0f; //IN(8);
float morph_mod = 0.0f; //IN(9);
float decay_in = 0.5f; // IN(10);
float lpg_in = 0.1f ;// IN(11);
int pitch_in = 48;
int16_t pitch_fm = 0;


// Braids vars
//    float   voct_in = IN0(0);
//    float   timbre_in = IN0(1);
//    float   color_in = IN0(2);
//    float   model_in = IN0(3);
//    float   *trig_in = IN(4);



void updateBraidsAudio() {

  int16_t *buffer = voices[0].pd.buffer;
  uint8_t *sync_buffer = voices[0].pd.sync_buffer;
  size_t  size = BLOCK_SIZE;

  braids::MacroOscillator *osc = voices[0].pd.osc;
  /*
    braids::Envelope *envelope = voices[0].envelope;
    envelope->Update(
    braids::settings.GetValue(braids::SETTING_AD_ATTACK) * 8,
    braids::settings.GetValue(braids::SETTING_AD_DECAY) * 8);


    uint32_t ad_value = envelope->Render();
  */

  // set parameters
  // CONSTRAIN(timbre_in, 0.f, 1.f);
  // CONSTRAIN(morph_in, 0.f, 1.f);

  int16_t timbre = timbre_in ; // * 32767.f; we are now mapping directly in loop
  int16_t color = morph_in ; //* 32767.f; we are now mapping directly

  osc->set_parameters(timbre, color);

  // set shape/model
  //uint8_t shape = (int)(engine_in);
  //if (shape >= braids::MACRO_OSC_SHAPE_LAST)
  //  shape -= braids::MACRO_OSC_SHAPE_LAST;

  // engine_in is simply from button
  osc->set_shape(static_cast<braids::MacroOscillatorShape>(engine_in));

  // TODO: check setting pitch VB
  //CONSTRAIN(voct_in, 0.f, 127.f);
  //int pit = (int)voct_in;
  //float frac = voct_in - pit;
  //osc->set_pitch((pit << 7) + (int)(frac * 128.f));
  // MI
  // Apply hysteresis to ADC reading to prevent a single bit error to move
  // the quantized pitch up and down the quantization boundary.

/*
  int32_t pitch = voices[0].quantizer->Process(
                    braids::settings.adc_to_pitch(pitch_in),
                    (60 + 0 ) << 7); // settings.quantizer_root() where zero
*/
  // Check if the pitch has changed to cause an auto-retrigger

  bool trigger_flag ;//= trigger_in > 0.0f; // = (trigger && (!voices[0].last_trig));
  
  int32_t pitch_delta = pitch_in - previous_pitch;
  
  if ((pitch_delta >= 0x40 || -pitch_delta >= 0x40)) {
    trigger_flag = true;
  }

  previous_pitch = pitch_in;
  
  osc->set_pitch(pitch_in << 7);
    
  voices[0].last_trig = trigger_flag;

  if (trigger_flag) {
    osc->Strike();
    trigger_flag = false;
  }
  // render
  for (int count = 0; count < 32; count += size) {

    osc->Render(sync_buffer, buffer, size);
  }
}

// initialize macro osc
void initVoices() {

  voices[0].ratio = 48000.f / MI_SAMPLERATE;

  // init some params
  voices[0].pd.osc = new braids::MacroOscillator;
  memset(voices[0].pd.osc, 0, sizeof(*voices[0].pd.osc));

  voices[0].pd.osc->Init(48000.f);
  voices[0].pd.osc->set_pitch((48 << 7));
  voices[0].pd.osc->set_shape(braids::MACRO_OSC_SHAPE_VOWEL_FOF);


  voices[0].ws = new braids::SignatureWaveshaper;
  voices[0].ws->Init(123774);

  voices[0].quantizer = new braids::Quantizer;
  voices[0].quantizer->Init();
  voices[0].quantizer->Configure(braids::scales[0]);

  //unit->jitter_source.Init();

  memset(voices[0].pd.buffer, 0, sizeof(int16_t)*BLOCK_SIZE);
  memset(voices[0].pd.sync_buffer, 0, sizeof(voices[0].pd.sync_buffer));
  memset(voices[0].pd.samps, 0, sizeof(float)*BLOCK_SIZE);

  voices[0].last_trig = false;

  //voices[0].envelope = new braids::Envelope;
  //voices[0].envelope->Init();

  // get some samples initially
  updateBraidsAudio();

  /*
    // Initialize the sample rate converter
    int error;
    int converter = SRC_SINC_FASTEST;       //SRC_SINC_MEDIUM_QUALITY;


         // check resample flag
      int resamp = (int)IN0(5);
      CONSTRAIN(resamp, 0, 2);
      switch(resamp) {
          case 0:
              SETCALC(MiBraids_next);
              //Print("resamp: OFF\n");
              break;
          case 1:
              unit->pd.osc->Init(MI_SAMPLERATE);
              SETCALC(MiBraids_next_resamp);
              Print("MiBraids: internal sr: 96kHz - resamp: ON\n");
              break;
          case 2:
              SETCALC(MiBraids_next_reduc);
              Print("MiBraids: resamp: OFF, reduction: ON\n");
              break;
      }
  */
}

const braids::SettingsData kInitSettings = {
  braids::MACRO_OSC_SHAPE_CSAW,

  braids::RESOLUTION_16_BIT,
  braids::SAMPLE_RATE_96K,

  0,  // AD->timbre
  true,  // Trig source auto trigger
  1,  // Trig delay
  false,  // Meta modulation

  braids::PITCH_RANGE_EXTERNAL,
  2,
  0,  // Quantizer is off
  false,
  false,
  false,

  2,  // Brightness
  0,  // AD attack
  5,  // AD decay
  0,  // AD->FM
  0,  // AD->COLOR
  0,  // AD->VCA
  0,  // Quantizer root

  50,
  15401,
  2048,

  { 0, 0 },
  { 32768, 32768 },
  "GREETINGS FROM MUTABLE INSTRUMENTS *EDIT ME*",
  };
