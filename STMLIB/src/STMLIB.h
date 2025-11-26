// STMLIB.h - Arduino wrapper for Mutable Instruments stmlib
// Fixed version: headers only, no .cc includes, cc to .inc 
#ifndef STMLIB_ARDUINO_H_
#define STMLIB_ARDUINO_H_

// Core stmlib header
#include "stmlib/stmlib.h"

// DSP modules - headers only
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/limiter.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/polyblep.h"
#include "stmlib/dsp/units.h"
#include "stmlib/dsp/atan.h"
#include "stmlib/dsp/rsqrt.h"
#include "stmlib/dsp/cosine_oscillator.h"

// Utils - headers only
#include "stmlib/utils/random.h"
#include "stmlib/utils/buffer_allocator.h"
#include "stmlib/utils/ring_buffer.h"

#endif // STMLIB_ARDUINO_H_
