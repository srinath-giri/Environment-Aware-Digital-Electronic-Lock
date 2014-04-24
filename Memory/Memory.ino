#include <EEPROM.h>

#define LOCKED 0xFF
#define UNLOCKED 0x00

#define HASH_SIZE 8
#define HASH_ADDR 0x20
#define LOCK_ADDR 0x10

void setup() {
  clearMem();
  //randomizeMem();
}  

void clearMem() {
  for (unsigned int i = 0; i < HASH_SIZE; i++)
    EEPROM.write(HASH_ADDR, 0);
  EEPROM.write(LOCK_ADDR, UNLOCKED);
}

void randomizeMem() {
  randomSeed(analogRead(0));
  
  for (unsigned int i = 0; i < 512; i++) {
    byte randNumber = random(255);
    EEPROM.write(i, randNumber);
  }

}

void loop() {

}
