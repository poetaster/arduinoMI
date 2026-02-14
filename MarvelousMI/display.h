// display functions/defines

enum {
  MODE_PLAY = 0,
  MODE_CONFIG,
  MODE_COUNT   // how many modes we got
};

int display_mode = 0;
uint8_t display_repeats = 0;

int32_t display_timer;

// display functions
int led[8] = {LED0, LED1, LED2, LED3, LED4, LED5, LED6, LED7};
// show a number in binary on the LEDs
void display_value(int16_t value) {
  for (int i = 7; i >= 0; i--) { // NOPE + 1 can loop this way because port assignments are sequential
    digitalWrite(led[i], value & 1);
    value = value >> 1;
  }
  display_timer = millis();
}


typedef struct {
  int x;
  int y;
  const char* str;
} pos_t;

//// {x,y} locations of play screen items
const int step_text_pos[] = { 0, 15, 16, 15, 32, 15, 48, 15, 64, 15, 80, 15, 96, 15, 112, 15 };

const pos_t line_1_1    = {.x = 0,  .y = 8, .str = "bpm:%3d" };
const pos_t line_1_2  = {.x = 46, .y = 8, .str = "trs:%+2d" };
const pos_t line_1_3  = {.x = 90, .y = 8, .str = "seq:%d" };
const pos_t line_2_1    = {.x = 0,  .y = 18, .str = "" };
const pos_t line_2_2   = {.x = 46, .y = 18, .str = "" };
const pos_t line_2_3   = {.x = 90, .y = 18, .str = "" };
const pos_t line_3_1 = { .x = 0, .y = 24,  .str = "" };
const pos_t line_3_2 = { .x = 46, .y = 24, .str = "" };
const pos_t line_3_3 = { .x = 90, .y = 24,  .str = "" };

const pos_t oct_text_offset = { .x = 3, .y = 10,  .str = "" };
const pos_t gate_bar_offset = { .x = 0, .y = -15, .str = "" };
const pos_t edit_text_offset = { .x = 3, .y = 22,  .str = "" };

const int gate_bar_width = 14;
const int gate_bar_height = 4;

void displayPlaits() {
  display.clearDisplay();
  // // name
  display.setCursor(line_1_1.x, line_1_1.y);
  display.print(oscnames[engine_in]);
  display.print(" ");
 // display.print(midi_switch_setting);
  display.setCursor(line_1_3.x, line_1_3.y);
  display.print("P:");
  display.print(position_in);
  
  //display.setCursor(line_1_2.x, line_1_2.y);
  //display.print(oscnames[engine_in]);
  // morph
  display.setCursor(line_2_1.x, line_2_1.y);
  display.print("M:");
  display.print(morph_in);
  // harmonics
  display.setCursor(line_2_2.x, line_2_2.y);
  display.print("H:");
  display.print(harm_in);  // user sees 1-8
  // timber
  display.setCursor(line_2_3.x, line_2_3.y);
  display.print("T:");
  display.print(timbre_in);


  //display.print(voices[0].patch.note);

  // play/pause
  //display.setCursor(line_2_3.x, line_2_3.y);
  //display.print("m: ");
  //display.print(display_mode);

  display.display();
}

void displayRings() {
  display.clearDisplay();
  // // name
  display.setCursor(line_1_1.x, line_1_1.y);
  if (easterEgg) {
    display.print(FXnames[engine_in]);
  } else {
    display.print(modelnames[engine_in]);
  }
  display.print(" ");
  //display.print(midi_switch_setting);
  display.setCursor(line_1_3.x, line_1_3.y);
  display.print("p:");
  display.print(position_in);
  
  //display.setCursor(line_1_2.x, line_1_2.y);
  //display.print(oscnames[engine_in]);
  // morph
  display.setCursor(line_2_1.x, line_2_1.y);
  display.print("M: ");
  display.print(morph_in);
  // harmonics
  display.setCursor(line_2_2.x, line_2_2.y);
  display.print("H: ");
  display.print(harm_in);  // user sees 1-8
  // timber
  display.setCursor(line_2_3.x, line_2_3.y);
  display.print("T: ");
  display.print(timbre_in);

  //display.setCursor(line_1_2.x, line_2_1.y);
  //display.print("P:");
  //display.print(voices[0].patch.note);

  // play/pause
  //display.setCursor(line_2_3.x, line_2_3.y);
  //display.print("m: ");
  //display.print(display_mode);

  display.display();
}

/* original generic */
void displayBraids() {
  display.clearDisplay();
  // // name
  display.setCursor(line_1_1.x, line_1_1.y);
  display.print(engine_in);
  display.print(" ");
  display.print(braidsnames[engine_in]);
 //display.setCursor(line_1_2.x, line_1_2.y);
  //display.print(oscnames[engine_in]);
  // morph
  display.setCursor(line_2_1.x, line_2_1.y);
  display.print("C: ");
  display.print(morph_in);
  // harmonics
  //display.setCursor(line_2_2.x, line_2_2.y);
  //display.print("H: ");
  //display.print(harm_in);  // user sees 1-8
  // timber
  display.setCursor(line_2_3.x, line_2_3.y);
  display.print("T: ");
  display.print(timbre_in);
  //display.setCursor(line_1_2.x, line_2_1.y);
  //display.print("P:");
  //display.print(voices[0].patch.note);
  // play/pause
  //display.setCursor(line_2_3.x, line_2_3.y);
  //display.print("m: ");
  //display.print(display_mode);

  display.display();
}

/* original generic */
void displayUpdate() {
  display.clearDisplay();
  // // name
  display.setCursor(line_1_1.x, line_1_1.y);
  display.print(engine_in);
  display.print(" ");

  if (voice_number == 0) {
    display.print(oscnames[engine_in]);
  } else if (voice_number == 1) {
    if (easterEgg) {
      display.print(FXnames[engine_in]);
    } else {
      display.print(modelnames[engine_in]);
    }
  } else if (voice_number == 2) {
    display.print(braidsnames[engine_in]);
  } else if (voice_number == 3) {
    display.print(cloudnames[engine_in]);
  }

  //display.setCursor(line_1_2.x, line_1_2.y);
  //display.print(oscnames[engine_in]);

  // morph
  display.setCursor(line_2_1.x, line_2_1.y);
  display.print("M: ");
  display.print(morph_in);

  // harmonics
  display.setCursor(line_2_2.x, line_2_2.y);
  display.print("H: ");
  display.print(harm_in);  // user sees 1-8

  // timber
  display.setCursor(line_2_3.x, line_2_3.y);
  display.print("T: ");
  display.print(timbre_in);

  //display.setCursor(line_1_2.x, line_2_1.y);
  //display.print("P:");
  //display.print(voices[0].patch.note);

  // play/pause
  //display.setCursor(line_2_3.x, line_2_3.y);
  //display.print("m: ");
  //display.print(display_mode);

  display.display();
}

void displaySplash() {
  display.clearDisplay();
  display.setFont(&myfont);
  display.setTextColor(WHITE, 0);
  display.drawRect(0, 0, dw - 1, dh - 1, WHITE);
  display.setCursor(15, 32);
  display.print("MarvelousMI");
  display.display();
}
