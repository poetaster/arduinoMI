// the somewhat flake code from my pikobeats version
// pot locking and sample averaging
// button handling / led handling


// pots
#define NPOTS 2 // number of pots
uint16_t potvalue[NPOTS]; // pot readings
uint16_t lastpotvalue[NPOTS]; // old pot readings
bool potlock[NPOTS]; // when pots are locked it means they must change by MIN_POT_CHANGE to register
uint32_t pot_timer; // reading pots too often causes noise
#define POT_SAMPLE_TIME 30 // delay time between pot reads
#define MIN_POT_CHANGE 25 // locked pot reading must change by this in order to register
#define MIN_COUNTS 8  // unlocked pot must change by this in order to register
#define POT_AVERAGING 20 // analog sample averaging count 
#define POT_MIN 4   // A/D may not read min value of 0 so use a bit larger value for map() function
#define POT_MAX 1019 // A/D may not read max value of 1023 so use a bit smaller value for map() function


#define NUM_BUTTONS 9 // 8 buttons plus USR button on VCC-GND board
#define SHIFT 8 // index of "shift" USR button 
uint8_t debouncecnt[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // debounce counters
bool button[NUM_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // key active flags

int buttons[NUM_BUTTONS] = {BUTTON0, BUTTON1, BUTTON2, BUTTON3, BUTTON4, BUTTON5, BUTTON6, BUTTON7, SHIFT};

// flag all pot values as locked ie they have to change more than MIN_POT_CHANGE to register
void lockpots(void) {
  for (int i = 0; i < NPOTS; ++i) potlock[i] = 1;
}


// sample analog pot input and do filtering.
// if pots are locked, change value only if movement is greater than MIN_POT_CHANGE
uint16_t readpot(uint8_t potnum) {
  int val = 0;
  int input;
  switch (potnum) { // map potnum to RP2040 pin
    case 0:
      input = 26;
      break;
    case 1:
      input = 27;
      break;
    case 2:
    default:   // shouldn't happen
      input = INTS_PIN;
      break;
  }
  // note that Pikocore pots are wired "backwards" - max voltage is full ccw
  for (int j = 0; j < POT_AVERAGING; ++j) val += (1024 - analogRead(input)); // read the A/D a few times and average for a more stable value
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

// scan buttons
bool scanbuttons(void)
{
  bool pressed;
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    switch (i) { // sample gate inputs
      case 0:
        pressed = !digitalRead(BUTTON0); // active low key inputs
        break;
      case 1:
        pressed = !digitalRead(BUTTON1);
        break;
      case 2:
        pressed = !digitalRead(BUTTON2);
        break;
      case 3:
        pressed = !digitalRead(BUTTON3);
        break;
      case 4:
        pressed = !digitalRead(BUTTON4);
        break;
      case 5:
        pressed = !digitalRead(BUTTON5);
        break;
      case 6:
        pressed = !digitalRead(BUTTON6);
        break;
      case 7:
        pressed = !digitalRead(BUTTON7);
        break;
      case 8:
        pressed = !digitalRead(SHIFTBUTTON);
        break;
    }

    if (pressed) {
      if (debouncecnt[i] <= 3) ++debouncecnt[i];
      if (debouncecnt[i] == 2) { // trigger on second sample of key active
        button[i] = 1;
      }
    }
    else {
      debouncecnt[i] = 0;
      button[i] = 0;
    }
  }
  if (pressed) return true;
  else return false;
}
