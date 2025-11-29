// sample structure built by wav2header based on wav2sketch by Paul Stoffregen

struct sample_t {
  const int16_t * samplearray; // pointer to sample array
  uint32_t samplesize; // size of the sample array
  uint32_t sampleindex; // current sample array index when playing. index at last sample= not playing
  uint8_t MIDINOTE;  // MIDI note on that plays this sample
  uint8_t play_volume; // play volume 0-127
  char sname[20];        // sample name
} sample[] = {

	Eight,	// pointer to sample array
	Eight_SIZE,	// size of the sample array
	Eight_SIZE,	//sampleindex. if at end of sample array sound is not playing
	35,	// MIDI note on that plays this sample
	127,	// play volume 0-127
	"Eight",	// sample name

	Five,	// pointer to sample array
	Five_SIZE,	// size of the sample array
	Five_SIZE,	//sampleindex. if at end of sample array sound is not playing
	36,	// MIDI note on that plays this sample
	127,	// play volume 0-127
	"Five",	// sample name

	Four,	// pointer to sample array
	Four_SIZE,	// size of the sample array
	Four_SIZE,	//sampleindex. if at end of sample array sound is not playing
	37,	// MIDI note on that plays this sample
	127,	// play volume 0-127
	"Four",	// sample name

	One,	// pointer to sample array
	One_SIZE,	// size of the sample array
	One_SIZE,	//sampleindex. if at end of sample array sound is not playing
	38,	// MIDI note on that plays this sample
	127,	// play volume 0-127
	"One",	// sample name

	Seven,	// pointer to sample array
	Seven_SIZE,	// size of the sample array
	Seven_SIZE,	//sampleindex. if at end of sample array sound is not playing
	39,	// MIDI note on that plays this sample
	127,	// play volume 0-127
	"Seven",	// sample name

	Six,	// pointer to sample array
	Six_SIZE,	// size of the sample array
	Six_SIZE,	//sampleindex. if at end of sample array sound is not playing
	40,	// MIDI note on that plays this sample
	127,	// play volume 0-127
	"Six",	// sample name

	Two,	// pointer to sample array
	Two_SIZE,	// size of the sample array
	Two_SIZE,	//sampleindex. if at end of sample array sound is not playing
	41,	// MIDI note on that plays this sample
	127,	// play volume 0-127
	"Two",	// sample name

};
