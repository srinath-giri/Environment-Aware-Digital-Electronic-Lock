
/*
Project 3 - srinathg@andrew.cmu.edu
*/

#include <EEPROM.h>
#include <Servo.h>
#include <SipHash_2_4.h>
#include <Keypad.h>

#define LOCKED 0xFF
#define UNLOCKED 0x00
#define LOCK_KEY '#'
#define UNLOCK_KEY '*'

#define MIN_PIN_SIZE 4
#define MAX_PIN_SIZE 6
#define HASH_SIZE 8
#define HASH_ADDR 0x20
#define LOCK_ADDR 0x10
#define MAX_INVALID_TRIALS 5

const int lightpin = A6;
const int temperaturepin = A5;
const int pressurepin = A4;
const int gpspin = A3;
const int greenLED = 2;
const int redLED = 3;

const unsigned char key[] PROGMEM = {
  0xAF, 0xF3, 0xA2, 0x1C, 0x04, 0x5E, 0xFF, 0x98,
  0x34, 0xDC, 0xE2, 0xAA, 0x7B, 0x14, 0x45, 0xC6
};

const int LOCK_POSITION = 0;
const int UNLOCK_POSITION = 90;
Servo servoMotor;

// 12-Key Keypad                                                               
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  { '1','2','3'  },
  { '4','5','6'  },
  { '7','8','9'  },
  { '#','0','*'  }
};
byte rowPins[ROWS] = { 5, 4, 3, 2};
byte colPins[COLS] = { 8, 7, 6};
//Keypad keypad;


void setup() {
  Serial.begin(9600);
  pinMode(greenLED, OUTPUT); 
  pinMode(redLED, OUTPUT); 
  //keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
  //keypad.addEventListener(keypadEvent);
  servoMotor.attach(9);  
  sipHash.initFromPROGMEM(key);
  delay(300);
}

void loop() {

  //  char keyPressed = keypad.getKey();
  //  if (key != NO_KEY){
  //     log("Key Pressed:",println(key);
  //     return;
  //  }
  delay(10);
}

/*
void keypadEvent(KeypadEvent key) {

  static byte PIN[MAX_PIN_SIZE] = { 0 }; 
  static byte PINSize = 0;

  if (keypad.getState() == RELEASED){
    switch (key){
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      PIN[PINSize++] = key;
      break;
    case LOCK_KEY:   
      if(isPINValid(PIN, PINSize))
        lock(PIN, PINSize);   
      else 
        PINSize = 0;
      break;
    case UNLOCK_KEY: 
      if(isPINValid(PIN, PINSize))
        unlock(PIN, PINSize); 
      else 
        PINSize = 0;
      break;
    }
  }
}
*/

boolean isPINValid(byte* PIN, byte PINSize) {
  return PINSize >= MIN_PIN_SIZE && PINSize <= MAX_PIN_SIZE;
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
  
  static int invalidTrials = 0;  
  
  if(!isLocked()) 
    return;
  if(invalidTrials > MAX_INVALID_TRIALS) 
    return;
  if(!validatePINS(PIN, PINSize)) {
    invalidTrials++;
    return;
  }
  else invalidTrials = 0;
  
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
  byte *new_hash = hashPINS(PIN, PINSize, measuredLight, measuredTemperature, measuredPressure, measuredLocation);

  for (unsigned int i = 0; i < HASH_SIZE; i++)
    EEPROM.write(HASH_ADDR + i, new_hash[i]);
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

  for (unsigned int i=0; i<PINSize; i++)
    msg[i] = PIN[i];

  msg[PINSize    ] = measuredLight;
  msg[PINSize + 1] = measuredTemperature;
  msg[PINSize + 2] = measuredPressure;
  msg[PINSize + 3] = measuredLocation;

  for (unsigned int i=0; i<msgLen; i++) {
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
  on ? digitalWrite(greenLED,HIGH):digitalWrite(greenLED,LOW);  
}

void ledRed(boolean on) {
  on ? digitalWrite(redLED,HIGH):digitalWrite(redLED,LOW);  
}

void log(const char *name, int value) {
  Serial.print(name);
  Serial.print("\t");
  Serial.print(value);
  Serial.print("\n");
}


