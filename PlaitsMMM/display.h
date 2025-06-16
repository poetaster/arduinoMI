// display functions/defines

#define DISPLAY_TIME 2000 // time in ms to display numbers on LEDS

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

const pos_t bpm_text_pos    = {.x = 0,  .y = 5, .str = "bpm:%3d" };
const pos_t trans_text_pos  = {.x = 46, .y = 5, .str = "trs:%+2d" };
const pos_t seqno_text_pos  = {.x = 90, .y = 5, .str = "seq:%d" };
const pos_t seq_info_pos    = {.x = 0, .y = 20, .str = "" };
const pos_t mode_text_pos   = {.x = 46, .y = 20, .str = "" };
const pos_t play_text_pos   = {.x = 120, .y = 32, .str = "" };

const pos_t oct_text_offset = { .x = 3, .y = 10,  .str = "" };
const pos_t gate_bar_offset = { .x = 0, .y = -15, .str = "" };
const pos_t edit_text_offset = { .x = 3, .y = 22,  .str = "" };
const int gate_bar_width = 14;
const int gate_bar_height = 4;

void displayUpdate() {
  display.clearDisplay();
  //display.setFont(&myfont); don't need to call this every time!
  //display.setTextColor(WHITE, 0);


  // display.setFont(&myfont2);

  // bpm
  display.setCursor(bpm_text_pos.x, bpm_text_pos.y);
  display.print(voices[0].patch.engine);
  display.print(" ");
  display.print(oscnames[engine_in]); 
  // name
  //display.setCursor(trans_text_pos.x, trans_text_pos.y);
  //display.print(oscnames[engine_in]); 
  
  // harmonics
  display.setCursor(seqno_text_pos.x, seqno_text_pos.y);
  display.print("h: ");
  display.print(voices[0].patch.harmonics);  // user sees 1-8

  // seq info / meta
  display.setCursor(seq_info_pos.x, seq_info_pos.y);
  display.print("m: ");
  display.print(morph_in);
  
  display.setCursor(trans_text_pos.x, seq_info_pos.y);
  display.print("p:");
  display.print(voices[0].patch.note);

  display.setCursor(mode_text_pos.x, play_text_pos.y);
  display.print("t: ");
  display.print(voices[0].patch.timbre);
  
  // play/pause
  //display.setCursor(play_text_pos.x, play_text_pos.y);
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
  display.print("MMM Plaits");
  display.display();
}
