// pots
#define AIN0  26
//CV1
#define AIN1  27
//CV2
#define AIN2  28
//CV3
//#define AIN3  29 // not available on standard Pico board

#define POT_SAMPLE_TIME 5 // delay time between pot reads
#define MIN_POT_CHANGE 1 // locked pot reading must change by this in order to register
#define MIN_COUNTS 1  // unlocked pot must change by this in order to register
#define POT_AVERAGING 15 // analog sample averaging count 
#define POT_MIN 0   // A/D may not read min value of 0 so use a bit larger value for map() function
#define POT_MAX 4095 // A/D may not read max value of 1023 so use a bit smaller value for map() function

#define NPOTS 3 // number of pots

uint16_t potvalue[NPOTS]; // pot readings
uint16_t lastpotvalue[NPOTS]; // old pot readings
bool potlock[NPOTS]; // when pots are locked it means they must change by MIN_POT_CHANGE to register
uint32_t pot_timer; // reading pots too often causes noise

// flag all pot values as locked ie they have to change more than MIN_POT_CHANGE to register
void lockpots(void) {
  for (int i = 0; i < NPOTS; ++i) potlock[i] = 1;
}


// sample analog pot input and do filtering.
// if pots are locked, change value only if movement is greater than MIN_POT_CHANGE
uint16_t readpot(uint8_t potnum) {
  int val = 0;
  int input;
  if (potnum == 0) {
    input = AIN0;
  } else if (potnum == 1 ){
    input = AIN1;
  } else {
    input = AIN2;
  }
  
  // note that Pikocore pots are wired "backwards" - max voltage is full ccw

  for (int j = 0; j < POT_AVERAGING; ++j) { 
    int readA = analogRead(input); // read the A/D a few times and average for a more stable value
    if (readA > 1904) readA = readA - 10;
    val+= readA;
  }
  
  val = val / POT_AVERAGING;
  
  if (potlock[potnum]) {
    int delta = lastpotvalue[potnum] - val; // this needs to be done outside of the abs() function - see arduino abs() docs
    if (abs(delta) > MIN_POT_CHANGE) {
      potlock[potnum] = 0; // flag pot no longer locked
      potvalue[potnum] = lastpotvalue[potnum] = val; // save the new reading
    }
    else val = lastpotvalue[potnum];
  }
  else {
    if (abs(lastpotvalue[potnum] - val) > MIN_COUNTS) lastpotvalue[potnum] = val; // even if pot is unlocked, make sure pot has moved at least MIN_COUNT counts so values don't jump around
    else val = lastpotvalue[potnum];
    potvalue[potnum] = val; // pot is unlocked so save the reading
  }
  
  return val;
}
