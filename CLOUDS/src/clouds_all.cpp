// clouds_all.cpp - Single compilation unit for clouds
// Only includes the .inc files that exist in poetaster's arduinoMI port

#include <STMLIB.h>

#include "clouds/dsp/correlator.cc"
#include "clouds/dsp/granular_processor.cc"
#include "clouds/dsp/mu_law.cc"
#include "clouds/dsp/pvoc/frame_transformation.cc"
#include "clouds/dsp/pvoc/phase_vocoder.cc"
#include "clouds/dsp/pvoc/stft.cc"
#include "clouds/resources.cc"
