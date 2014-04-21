
/*
Project 3 - srinathg@andrew.cmu.edu
 */

#include <EEPROM.h>
#include <Servo.h>
#include <SipHash_2_4.h>

#define LOCKED 0xFF
#define UNLOCKED 0x00
#define LOCK_KEY '#'
#define UNLOCK_KEY '*'

#define HASH_SIZE 8
#define HASH_ADDR 0x20
#define LOCK_ADDR 0x10

const int lightpin = A6;
const int temperaturepin = A5;
const int pressurepin = A4;
const int gpspin = A3;
const int greenLED = 2;
const int redLED = 3;

const int LOCK_POSITION = 0;
const int UNLOCK_POSITION = 90;
Servo servoMotor;

void setup() {
  Serial.begin(9600);
  servoMotor.attach(9);
  unsigned char key[] PROGMEM = {0xAF, 0xF3, 0xA2, 0x1C, 0x04, 0x5E, 0xFF, 0x98,
                                0x34, 0xDC, 0xE2, 0xAA, 0x7B, 0x14, 0x45, 0xC6};
  sipHash.initFromPROGMEM(key);
  delay(300);
}

char getKeyPressed() {
  return ' ';
}

void loop() {
  char keyPressed = getKeyPressed();
  byte PIN[] = { 6, 6, 6, 6 }; 
  byte PINSize = 4;
  //switch(keyPressed) {
  //  case 
  //}
  
  if(keyPressed == LOCK_KEY)
    lock(PIN, PINSize);
  else if(keyPressed == UNLOCK_KEY)
    unlock(PIN, PINSize);
}


void lock(byte* PIN, byte PINSize) {
  if(isLocked()) return;
  
  servoMotor.write(LOCK_POSITION);
  delay(20);
  
  save(PIN, PINSize);
  ledRed(false);
  ledGreen(true);
  
}

void unlock(byte* PIN, byte PINSize) {
  if(!isLocked()) return;
  
  if(!validatePINS(PIN, PINSize)) return;

  servoMotor.write(UNLOCK_POSITION);
  delay(20);

  reset();  
  ledGreen(false);
  ledRed(true);

}

boolean isLocked() {
  return (EEPROM.read(LOCK_ADDR) == LOCKED);
}

boolean validatePINS(byte* PIN, byte PINSize) {
  byte measuredLight = measureLight();
  byte measuredTemperature = measureTemperature();
  byte measuredPressure = measurePressure();
  byte measuredLocation = measureLocation();

 byte stored_hash[HASH_SIZE];
 for (unsigned int i = 0; i < HASH_SIZE; i++) {
    stored_hash[i] = EEPROM.read(HASH_ADDR + i);
 }
 
 byte* computed_hash = hashPINS(PIN, PINSize, measuredLight, measuredTemperature, measuredPressure, measuredLocation);
 
 for (unsigned int i = 0; i < HASH_SIZE; i++) {
    if( stored_hash[i] != computed_hash[i] )
       return false;
 }

 return true;
}

void save(byte* PIN, byte PINSize) {
 
  byte measuredLight = measureLight();
  byte measuredTemperature = measureTemperature();
  byte measuredPressure = measurePressure();
  byte measuredLocation = measureLocation();
  
  byte *hash = hashPINS(PIN, PINSize, measuredLight, measuredTemperature, measuredPressure, measuredLocation);
  
  for (unsigned int i = 0; i < HASH_SIZE; i++)
    EEPROM.write(HASH_ADDR, hash[i]);
  
  EEPROM.write(LOCK_ADDR, LOCKED);
  
}

void reset() {

  for (unsigned int i = 0; i < HASH_SIZE; i++)
    EEPROM.write(HASH_ADDR, 0);

  EEPROM.write(LOCK_ADDR, UNLOCKED);
  
}

byte* hashPINS(byte* PIN, byte PINSize, byte measuredLight, byte measuredTemperature, byte measuredPressure, byte measuredLocation) {
                                
 byte msg[PINSize + 4];                                
 unsigned int msgLen = PINSize + 4;
 
 for (unsigned int i=0; i<PINSize;i++)
   msg[i] = PIN[i];

 for (unsigned int i=0; i<msgLen;i++) {
   sipHash.updateHash((byte)msg[i]); 
 }

 sipHash.finish(); 

 return (byte*) sipHash.result;

}

byte measureLight() {
  byte luminosity = analogRead(lightpin) >> 7;
  log("Luminosity:",luminosity);
  return luminosity;
}

byte measureTemperature() {
  byte temperature = analogRead(temperaturepin) >> 7;
  log("Temperature:",temperature);
  return temperature;
}

byte measurePressure() {
  byte pressure = analogRead(pressurepin) >> 7;
  log("Pressure:",pressure);
  return pressure;
}

byte measureLocation() {
  byte location = analogRead(gpspin) >> 7;
  log("Location:",location);
  return location;
}

void ledGreen(boolean on) {
  if(on)
   digitalWrite(greenLED,HIGH);
  else
   digitalWrite(greenLED,LOW);  
}


void ledRed(boolean on) {
  if(on)
   digitalWrite(redLED,HIGH);
  else
   digitalWrite(redLED,LOW);  
}

void log(const char *name, int value) {
  Serial.print(name);
  Serial.print("\t");
  Serial.print(value);
  Serial.print("\n");
}

