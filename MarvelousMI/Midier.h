#pragma once

namespace midier
{

using Degree = unsigned char;
using Octave = unsigned char;



enum class Interval : char
{
  // the values are the corresponding numbers of semitones

  P1 = 0,
  m2 = 1,
  M2 = 2,
  m3 = 3,
  M3 = 4,
  P4 = 5,
  A4 = 6,
  d5 = 6,
  P5 = 7,
  A5 = 8,
  m6 = 8,
  M6 = 9,
  m7 = 10,
  M7 = 11,
  P8 = 12,
};

inline Interval operator+(Interval lhs, Interval rhs)
{
  return (Interval)((char)lhs + (char)rhs);
}
enum class Note : char
{
  // the values are the corresponding number of semitones from 'C' in
  // order to easily convert a note to its respective MIDI note number

  C = 0,
  D = 2,
  E = 4,
  F = 5,
  G = 7,
  A = 9,
  B = 11,

  // we might represent other values, in run-time, as notes as well, e.g. 'C = 12', 'G# = 20'
};

enum class Accidental : short
{
  // the values are the number of semitones (and direction)
  // the note should be modified by this accidental

  Flat    = -1,
  Natural =  0,
  Sharp   = +1,
};

inline Note operator+(Note note, Interval interval)
{
  return (Note)((char)note + (char)interval);
}

inline Note operator+(Note note, Accidental accidental)
{
  return (Note)((char)note + (char)accidental);
}


enum class Quality : char
{
  // in order to support both 7-th chords (4 notes) and triads (3 notes), we define
  // 3-note qualities as an alias to a 4-note quality with the same first 3 notes.
  // currently, qualities are used only to retrieve the interval of a chord degree
  // and there's no reason to be able to distinguish between 3-note qualities and
  // their matching 4-note qualities by having different enum values

  // seventh chords (4-note qualities)
  m7b5,   // half-diminished
  m7,     // minor seventh
  dom7,   // dominant seventh
  maj7,   // major seventh
  aug7,   // augmented seventh

  // triads (3-note qualities)
  dim = m7b5,     // diminished
  minor = m7,     // minor
  major = dom7,   // major
  aug = aug7,     // augmented
};

enum class Mode : char
{
  Ionian,
  Dorian,
  Phrygian,
  Lydian,
  Mixolydian,
  Aeolian,
  Locrian,

  // last element to indicate the # of actual elements
  Count,
};

namespace mode
{

using Name = char[11]; // "mixolydian" + '\0'

void name(Mode mode, /* out */ Name & name);

const Name PROGMEM __ionian = "Ionian";
const Name PROGMEM __dorian = "Dorian";
const Name PROGMEM __phrygian = "Phrygian";
const Name PROGMEM __lydian = "Lydian";
const Name PROGMEM __mixolydian = "Mixolydian";
const Name PROGMEM __aeolian = "Aeolian";
const Name PROGMEM __locrian = "Locrian";

char const * const __names[] PROGMEM = {
  __ionian,
  __dorian,
  __phrygian,
  __lydian,
  __mixolydian,
  __aeolian,
  __locrian,
};

static_assert(sizeof(__names) / sizeof(__names[0]) == (unsigned)Mode::Count, "Unexpected number of names declared");



void name(Mode mode, /* out */ Name & name)
{
  strcpy_P(
    /* out */ name,
    (char const *)pgm_read_ptr(&(__names[(unsigned)mode]))
  );
}

} // mode



namespace scale
{

Interval interval(Mode mode, Degree degree);
Quality  quality (Mode mode, Degree degree);

#define ASSERT(mode, expected) static_assert(static_cast<int>(mode) == (expected), "Expected midier::" #mode " to be equal to " #expected);

ASSERT(Mode::Ionian,     0);
ASSERT(Mode::Dorian,     1);
ASSERT(Mode::Phrygian,   2);
ASSERT(Mode::Lydian,     3);
ASSERT(Mode::Mixolydian, 4);
ASSERT(Mode::Aeolian,    5);
ASSERT(Mode::Locrian,    6);

Interval interval(Mode mode, Degree degree)
{
  static Interval const PROGMEM __ionian     [] = { Interval::P1, Interval::M2, Interval::M3, Interval::P4, Interval::P5, Interval::M6, Interval::M7 };
  static Interval const PROGMEM __dorian     [] = { Interval::P1, Interval::M2, Interval::m3, Interval::P4, Interval::P5, Interval::M6, Interval::m7 };
  static Interval const PROGMEM __phrygian   [] = { Interval::P1, Interval::m2, Interval::m3, Interval::P4, Interval::P5, Interval::m6, Interval::m7 };
  static Interval const PROGMEM __lydian     [] = { Interval::P1, Interval::M2, Interval::M3, Interval::A4, Interval::P5, Interval::M6, Interval::M7 };
  static Interval const PROGMEM __mixolydian [] = { Interval::P1, Interval::M2, Interval::M3, Interval::P4, Interval::P5, Interval::M6, Interval::m7 };
  static Interval const PROGMEM __aeolian    [] = { Interval::P1, Interval::M2, Interval::m3, Interval::P4, Interval::P5, Interval::m6, Interval::m7 };
  static Interval const PROGMEM __locrian    [] = { Interval::P1, Interval::m2, Interval::m3, Interval::P4, Interval::d5, Interval::m6, Interval::m7 };

  static Interval const * const __all[] PROGMEM =
  {
    __ionian,
    __dorian,
    __phrygian,
    __lydian,
    __mixolydian,
    __aeolian,
    __locrian,
  };

  static_assert(sizeof(__all) / sizeof(__all[0]) == (unsigned)Mode::Count, "Unexpected number of modes declared");
  static_assert(sizeof(Interval) == sizeof(byte), "Unexpected size of 'Interval'");

  Interval octaver = Interval::P1;

  while (degree > 7)
  {
    degree -= 7;
    octaver = octaver + Interval::P8;
  }

  return octaver + (Interval)pgm_read_byte(pgm_read_ptr(__all + (unsigned)mode) + (degree - 1));
}

Quality quality(Mode mode, Degree degree)
{
  static Quality const PROGMEM __qualities[] = { Quality::maj7, Quality::m7, Quality::m7, Quality::maj7, Quality::dom7, Quality::m7, Quality::m7b5 };

  constexpr auto __count = sizeof(__qualities) / sizeof(__qualities[0]);

  static_assert(__count == 7, "Expected 7 qualities to be declared");
  static_assert(sizeof(Quality) == sizeof(byte), "Unexpected size of 'Quality'");

  return (Quality)pgm_read_byte(__qualities + ((degree - 1 + (unsigned)mode) % __count));
}
} // scale

namespace triad
{

Interval interval(Quality quality, Degree degree);


#define ASSERT(quality, expected) static_assert((int)(quality) == (int)(expected), "Expected " #quality " to be equal to " #expected);

ASSERT(Quality::m7b5,   0);
ASSERT(Quality::m7,     1);
ASSERT(Quality::dom7,   2);
ASSERT(Quality::maj7,   3);
ASSERT(Quality::aug7,   4);

ASSERT(Quality::dim,    Quality::m7b5);
ASSERT(Quality::minor,  Quality::m7);
ASSERT(Quality::major,  Quality::dom7);
ASSERT(Quality::aug,    Quality::aug7);

static const Interval __intervals[][4] =
{
  { Interval::P1, Interval::m3, Interval::d5, Interval::m7 }, // half-diminished chord (m7b5) and diminished triad (dim)
  { Interval::P1, Interval::m3, Interval::P5, Interval::m7 }, // minor 7th chord (m7) and minor triad (minor)
  { Interval::P1, Interval::M3, Interval::P5, Interval::m7 }, // dominanth 7th chord (dom7) and major triad (major)
  { Interval::P1, Interval::M3, Interval::P5, Interval::M7 }, // major 7th (maj7)
  { Interval::P1, Interval::M3, Interval::A5, Interval::m7 }, // augmented 7th chord (aug7) and augmented triad (aug)
};

static_assert(sizeof(__intervals) / sizeof(__intervals[0]) == 5, "Expected 5 qualities to be declared");

Interval interval(Quality quality, Degree degree)
{
  Interval octaver = Interval::P1;

  if (degree > 7)
  {
    degree -= 7;
    octaver = Interval::P8;
  }

  return octaver + __intervals[(unsigned)quality][degree >> 1]; // transforming 'degree' from {1,3,5,7} to {0,1,2,3}, respectively
}

} // triad



namespace midi
{

enum class Velocity : char
{
  // values are MIDI velocity

  High = 127, // maximum velocity
  Low  = 75,
};

// represents a MIDI note number
using Number = unsigned char;

// calculate MIDI note number from a musical note and an octave
Number number(Note note, Octave octave);

// send a 'NOTE_ON' MIDI command
void on(Number number, Velocity velocity = Velocity::High); // by default max velocity

// send a 'NOTE_OFF' MIDI command
void off(Number number);

// play a musical note for a specific duration of time (in ms)
void play(Note note,                unsigned duration = 200);
void play(Note note, Octave octave, unsigned duration = 200);

void send(byte command, byte data1, byte data2)
{
  constexpr auto channel = 0;

#ifndef DEBUG
  Serial.write((command & 0xF0) | (channel & 0x0F));
  Serial.write(data1 & 0x7F);
  Serial.write(data2 & 0x7F);
#else
  // TRACE_4(F("Sending MIDI command NOTE-"), command == 0x90 ? F("ON") : F("OFF"), " #", (int)data1);
#endif
}

Number number(Note note, Octave octave)
{
  return 24 + (12 * (octave - 1)) + (char)note;
}

void on(Number number, Velocity velocity)
{
  send(0x90, number, (char)velocity);
}

void off(Number number)
{
  send(0x80, number, 0);
}

void play(Note note, unsigned duration)
{
  play(note, 3, duration); // playing notes in octave 3 by default
}

void play(Note note, Octave octave, unsigned duration)
{
  const auto no = number(note, octave);

  on(no);
  delay(duration);
  off(no);
}

} // midi

namespace style
{

Degree degree(unsigned n, unsigned r, unsigned index);
unsigned count(unsigned n);

using Description = char[13]; // 12 + '\0'
void description(unsigned n, unsigned r, /* out */ Description & desc);

namespace
{

inline unsigned __factorial(unsigned x)
{
  if      (x == 0) {
    return 1;
  }
  else if (x == 1) {
    return 1;
  }
  else if (x == 2) {
    return 2;
  }
  else if (x == 3) {
    return 6;
  }
  else if (x == 4) {
    return 24;
  }
  else if (x == 5) {
    return 120;
  }
  else if (x == 6) {
    return 720;
  }
}

// the algorithm used for permutation generation is based on algorithm 2.16 in the book
// "Combinatorial Algorithms: Generation, Enumeration, and Search" by Donald L. Kreher, Douglas R. Stinson
// the algorithm can be seen (Jan 2020) at https://cw.fel.cvut.cz/old/_media/courses/be4m33pal/lesson05_pp52_57.pdf
// book references:
//   1) https://www.amazon.com/Combinatorial-Algorithms-Enumeration-Mathematics-Applications/dp/084933988X
//   2) https://www.crcpress.com/Combinatorial-Algorithms-Generation-Enumeration-and-Search/Kreher-Stinson/p/book/9780367400156
//
void __algorithm(unsigned n, unsigned r, /* out */ unsigned * pi)
{
  pi[n - 1] = 0;

  for (unsigned j = 1; j < n; ++j)
  {
    const unsigned d = (r % __factorial(j + 1)) / __factorial(j);

    r -= (d * __factorial(j));
    pi[n - j - 1] = d;

    for (unsigned i = n - j; i < n; ++i)
    {
      if (pi[i] >= d)
      {
        ++pi[i];
      }
    }
  }
  // the algorithm has generated a permutation of indexes between [0,n]
  // let's now convert those indexes into chord degrees

  for (unsigned i = 0; i < n; ++i)
  {
    pi[i] = pi[i] << 1; // {0,1,2,3,..,n} -> {0,2,4,6,..,2n}

    if (pi[i] < 7)
    {
      ++pi[i]; // {0,2,4,6} -> {1,3,5,7}
    }
    else
    {
      // keep {8,10,12,..} the same as they represent {1,3,5,..}
      // but an octave higher (can be retrieved by "x-7")
    }
  }
}
Degree degree(unsigned n, unsigned r, unsigned index)
{
  unsigned pi[n];

  __algorithm(n, r, /* out */ pi);

  return pi[index];
}

unsigned count(unsigned n)
{
  return __factorial(n);
}

void description(unsigned n, unsigned r, /* out */ Description & desc)
{
  unsigned pi[n];

  __algorithm(n, r, /* out */ pi);

  char * pos = desc;

  for (unsigned i = 0; i < n; ++i)
  {
    if (i > 0)
    {
      *pos++ = ' ';
    }

    itoa(pi[i], pos++, 10); // places '\0'

    if (pi[i] >= 10) // 2 digits
    {
      ++pos;
    }
  }
}

} // style
} // style

} // midier
