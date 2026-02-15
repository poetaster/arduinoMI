/*
 * We don't actually use eeprom in this case since it causes
 * glitches. here for reference on how it might, naively, be done
 */
void writeEEPROMSettings() {

  int val; // reuse
  addr = 0;

  //plaits
  val = (int) plaits_morph * 100;
  EEPROM.write(addr, val);
  addr = addr + 1;
  val =  (int) plaits_harm * 100;
  EEPROM.write(addr, val);
  addr = addr + 1;
  val =  (int) plaits_timbre * 100;
  EEPROM.write(addr, val);
  addr = addr + 1;
  val =  (int) plaits_engine;
  EEPROM.write(addr, val);
  addr = addr + 1;

  //rings
  val =  (int) rings_morph * 100;
  EEPROM.write(addr, val);
  addr = addr + 1;
  val =  (int) rings_harm * 100;
  EEPROM.write(addr, val);
  addr = addr + 1;
  val =  (int) rings_timbre * 100;
  EEPROM.write(addr, val);
  addr = addr + 1;
  val =  (int) rings_pos * 100;
  EEPROM.write(addr, val);
  addr = addr + 1;
  val =  (int) rings_engine;
  EEPROM.write(addr, val);
  addr = addr + 1;

  //braids
  val =  (int) braids_timbre * 100;
  EEPROM.write(addr, val);
  addr = addr + 1;
  val =  (int) braids_morph * 100;
  EEPROM.write(addr, val);
  addr = addr + 1;
  val =  (int) braids_engine;
  EEPROM.write(addr, val);
  addr = addr + 1;

  //  finally, commit
  writing = true;
  


  if (EEPROM.commit()) {
    wrote = 1;
  } else {
    wrote = 0;
  }
  
  
  writing = false;

  addr = 0; // reset to start

}

// called at system setup to initialize from saved values.
void readEEPROMSettings() {

  int val; // reuse
  addr = 0; // reset to start
  File settings = LittleFS.open(F("/settings.txt"), "r");
 
  //plaits
  val = EEPROM.read(addr);
  if (val > 0) plaits_morph = (float) val / 100;
  addr = addr + 1;
  val = EEPROM.read(addr);
  if (val > 0) plaits_harm = (float) val / 100;
  addr = addr + 1;
  val = EEPROM.read(addr);
  if (val > 0) plaits_timbre = (float) val / 100;
  addr = addr + 1;
  val = EEPROM.read(addr);
  plaits_engine = val;
  addr = addr + 1;

  //rings
  val = EEPROM.read(addr);
  if (val > 0) rings_morph = (float)val / 100;
  addr = addr + 1;
  val = EEPROM.read(addr);
  if (val > 0) rings_harm = (float)val / 100;
  addr = addr + 1;
  val = EEPROM.read(addr);
  if (val > 0) rings_timbre = (float)val / 100;
  addr = addr + 1;
  val = EEPROM.read(addr);
  if (val > 0) rings_pos = (float)val / 100;
  addr = addr + 1;
  val = EEPROM.read(addr);
  rings_engine = val;
  addr = addr + 1;

  //braids
  val = EEPROM.read(addr);
  if (val > 0) braids_timbre = (float)val / 100;
  addr = addr + 1;
  val = EEPROM.read(addr);
  if (val > 0) braids_morph = (float)val / 100;
  addr = addr + 1;
  val = EEPROM.read(addr);
  braids_engine = val;

  addr = 0; //reset to start

}
