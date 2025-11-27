// CLOUDS.h - Arduino wrapper for Mutable Instruments CLOUDS
// Note all the originals sources, .cc files, are retained and a copy with .inc extension has been made
//
#ifndef CLOUDS_ARDUINO_H_
#define CLOUDS_ARDUINO_H_

#include "clouds/dsp/audio_buffer.h"
#include "clouds/dsp/correlator.h"
#include "clouds/dsp/frame.h"

#include "clouds/dsp/fx/diffuser.h"
#include "clouds/dsp/fx/fx_engine.h"
#include "clouds/dsp/fx/pitch_shifter.h"
#include "clouds/dsp/fx/reverb.h"

#include "clouds/dsp/grain.h"
#include "clouds/dsp/granular_processor.h"
#include "clouds/dsp/granular_sample_player.h"
#include "clouds/dsp/looping_sample_player.h"
#include "clouds/dsp/mu_law.h"
#include "clouds/dsp/parameters.h"

#include "clouds/dsp/pvoc/frame_transformation.h"
#include "clouds/dsp/pvoc/phase_vocoder.h"
#include "clouds/dsp/pvoc/stft.h"

#include "clouds/dsp/sample_rate_converter.h"
#include "clouds/dsp/window.h"
#include "clouds/dsp/wsola_sample_player.h"
#include "clouds/resources.h"

#endif // CLOUDS_ARDUINO_H_
