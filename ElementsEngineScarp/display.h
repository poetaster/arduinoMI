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
const pos_t bpm_text_pos    = {.x = 0,  .y = 15, .str = "bpm:%3d" };
const pos_t trans_text_pos  = {.x = 46, .y = 15, .str = "trs:%+2d" };
const pos_t seqno_text_pos  = {.x = 90, .y = 15, .str = "seq:%d" };
const pos_t seq_info_pos    = {.x = 0, .y = 35, .str = "" };
const pos_t mode_text_pos   = {.x = 0, .y = 55, .str = "" };
const pos_t play_text_pos   = {.x = 90, .y = 55, .str = "" };

const pos_t oct_text_offset = { .x = 3, .y = 10,  .str = "" };
const pos_t gate_bar_offset = { .x = 0, .y = -15, .str = "" };
const pos_t edit_text_offset = { .x = 3, .y = 22,  .str = "" };
const int gate_bar_width = 14;
const int gate_bar_height = 4;

void displayUpdate() {
  display.clearDisplay();
  //display.setFont(&myfont); don't need to call this every time!
  //display.setTextColor(WHITE, 0);
  /*
    for (int i = 0; i < 8; i++) {
    Step s = seqr.steps[i];
    const char* nstr = notenum_to_notestr(s.note);
    int o = notenum_to_oct(s.note);
    int x = step_text_pos[i * 2], y = step_text_pos[i * 2 + 1];
    display.setCursor(x, y);
    display.print(nstr);
    display.setCursor(x + oct_text_offset.x, y + oct_text_offset.y);
    display.printf("%1d", o);
    display.setCursor(x + edit_text_offset.x, y + edit_text_offset.y);
    display.print((i == selected_step) ? '^' : (s.on) ? ' '
                                                      : '*');
    int gate_w = 1 + (s.gate * gate_bar_width / 16);
    display.fillRect(x + gate_bar_offset.x, y + gate_bar_offset.y, gate_w, gate_bar_height, WHITE);
    }
  */

  // display.setFont(&myfont2);


  /*
    const pos_t bpm_text_pos    = {.x=0,  .y=57, .str="bpm:%3d" };
    const pos_t trans_text_pos  = {.x=55, .y=57, .str="trs:%+2d" };
    const pos_t seqno_text_pos  = {.x=0,  .y=45, .str="seq:%d" };
    const pos_t seq_info_pos    = {.x=60, .y=45, .str="" };
    const pos_t play_text_pos   = {.x=110,.y=57, .str="" };
  */
  // bpm
  display.setCursor(bpm_text_pos.x, bpm_text_pos.y);
  display.print("r: ");
  display.print(engine_in);//SETTING_OSCILLATOR_SHAPE

  // transpose
  display.setCursor(trans_text_pos.x, trans_text_pos.y);
  display.print("p:");
  display.print(pitch_in);

  // seqno
  display.setCursor(seqno_text_pos.x, seqno_text_pos.y);
  display.print("h: ");
  display.print(harm_in);  // user sees 1-8

  // name
  display.setCursor(seq_info_pos.x, seq_info_pos.y);
  display.print("P: ");
  display.print(oscnames[engine_in]);
  
  // color
  display.setCursor(trans_text_pos.x, play_text_pos.y);
  display.print("c: ");
  display.print(morph_in);


  display.setCursor(mode_text_pos.x, play_text_pos.y);
  display.print("t: ");
  display.print(timbre_in);
  
  // play/pause
  display.setCursor(play_text_pos.x, play_text_pos.y);
  display.print("m: ");
  display.print(display_mode);

  display.display();
}

void displaySplash() {
  display.clearDisplay();
  display.setFont(&myfont);
  display.setTextColor(WHITE, 0);
  display.drawRect(0, 0, dw - 1, dh - 1, WHITE);
  display.setCursor(25, 32);
  display.print("PikoBeatBox");
  display.display();
  // a little LED dance
  /*
    for( int i=0; i<1000; i++) {
    for( int j=0; j<8; j++) {
      int v = 30 + 30 * sin( (j*6.2 / 8 ) + i/50.0 ) ;
      analogWrite( led_pins[j], v);
    }
    delay(1);
    }*/
}
