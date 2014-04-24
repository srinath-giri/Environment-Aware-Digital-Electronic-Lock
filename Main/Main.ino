
/*
Project 3 - srinathg@andrew.cmu.edu
 */

#include <EEPROM.h>
#include <Servo.h>
#include <SipHash_2_4.h>
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <TinyGPS.h>

#define LOCKED 0xFF
#define UNLOCKED 0x00
#define LOCK_KEY '*'
#define UNLOCK_KEY '#'

#define MIN_PIN_SIZE 4
#define MAX_PIN_SIZE 6
#define HASH_SIZE 8
#define HASH_ADDR 0x20
#define LOCK_ADDR 0x10
#define MAX_INVALID_TRIALS 5

// Lock
const int LOCK_POSITION = 90;
const int UNLOCK_POSITION = 0;
const int servoPin = 9;
Servo servoMotor;

// 12-Key Keypad                                                               
//const byte ROWS = 4;
//const byte COLS = 3;
//char keys[ROWS][COLS] = {
//  { '1','2','3'  },
//  { '4','5','6'  },
//  { '7','8','9'  },
//  { '#','0','*'  }
//};
//byte rowPins[ROWS] = { 7, 6, 5, 4 };
//byte colPins[COLS] = { 8, 10, 13 };
//Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );;

// 8x2 LCD Output
LiquidCrystal lcd(12, 11, 4, 5, 6, 7);

// Lock / Unlock LED
const int greenLED = 2;
const int redLED = 3;

// PINS
const int lightpin = A6;
const int temperaturepin = A5;
const int pressurepin = A4;
TinyGPS gps;

const int lockpin = A0;
const int unlockpin = A1;
const unsigned char hashKey[16] = {
  0xAF, 0xF3, 0xA2, 0x1C, 0x04, 0x5E, 0xFF, 0x98,
  0x34, 0xDC, 0xE2, 0xAA, 0x7B, 0x14, 0x45, 0xC6
};

void setup() {
  Serial.begin(57600);
  
  pinMode(lockpin, INPUT); 
  pinMode(unlockpin, INPUT);   

  pinMode(greenLED, OUTPUT); 
  pinMode(redLED, OUTPUT); 

  ledLocked(isLocked());
  
  lcd.begin(8, 2);
  lcd.home();
  lcd.noAutoscroll();
  lcd.noCursor();

  servoMotor.attach(servoPin);
  delay(500);  
}

void loop() { 
  
  if(switch_is_pressed(lockpin))
    keypadEvent(LOCK_KEY);
  if(switch_is_pressed(unlockpin))
    keypadEvent(UNLOCK_KEY);
  delay(50);
}

boolean switch_is_pressed(const int switchno) {
  return (digitalRead(switchno) == HIGH) ;
}

void keypadEvent(KeypadEvent key) {

  logc("Key Pressed:",key);       

  const byte PIN[MAX_PIN_SIZE] = { 8,8,8,8 }; 
  const byte PINSize = 4;

  switch (key){
  case LOCK_KEY:   
    if(isPINValid(PIN, PINSize))
    {
      lock(PIN, PINSize);
      showMessage("Locked");
    }
    else 
    {        
      //PINSize = 0;
      showMessage("Invalid");
    }
    break;
  case UNLOCK_KEY: 
    if(isPINValid(PIN, PINSize))
    {
      if(unlock(PIN, PINSize)) 
        showMessage("UnLocked");        
    }
    else 
    {
      //PINSize = 0;
      showMessage("Invalid");
    }
    break;
  }
}

boolean isPINValid(const byte* PIN, const byte PINSize) {
  return PINSize >= MIN_PIN_SIZE && PINSize <= MAX_PIN_SIZE;
}

void showPIN(const byte* PIN,const byte PINSize) {
  lcd.clear();
  for (unsigned int i=0; i<PINSize; i++) {
    lcd.setCursor(i, 1);
    lcd.write(PIN[i]);
    log("PIN digit",PIN[i]);
  }
}

void showMessage(const char* msg) {
  lcd.clear();
  lcd.noAutoscroll();
  lcd.noCursor();
  for (unsigned int i=0; i<8; i++) {
    lcd.setCursor(i, 1);
    lcd.write(msg[i]);
  }  
  log(msg,0);
}

void lock(const byte* PIN, const byte PINSize) {
  if(isLocked()) return;
  servoLock(true);
  save(PIN, PINSize);
  showPIN(PIN, PINSize);
  ledLocked(true);
}

boolean unlock(const byte* PIN, const byte PINSize) {

  static int invalidTrials = 0;  

  if(!isLocked()) 
    return false;
  if(invalidTrials > MAX_INVALID_TRIALS) 
    return false;
  if(!validatePINS(PIN, PINSize)) {
    invalidTrials++;
    showMessage("InvalidT");
    return false;
  }
  else invalidTrials = 0;

  servoLock(false);
  reset();  
  ledLocked(false);
  return true;
}

boolean isLocked() {
  return (EEPROM.read(LOCK_ADDR) == LOCKED);
}

boolean validatePINS(const byte* PIN,const byte PINSize) {
  byte measuredLight = measureLight();
  byte measuredTemperature = measureTemperature();
  byte measuredPressure = measurePressure();
  long lat = 0L, lon = 0L; 
  measureLocation(&lat, &lon);

  byte stored_hash[HASH_SIZE];
  for (unsigned int i = 0; i < HASH_SIZE; i++) {
    stored_hash[i] = EEPROM.read(HASH_ADDR + i);
  }

  showPIN(PIN, PINSize);
  byte* computed_hash = hashPINS(PIN, PINSize, measuredLight, measuredTemperature, measuredPressure, lat, lon);

  logh(stored_hash, computed_hash);
  
  for (unsigned int i = 0; i < HASH_SIZE; i++) {
    if( stored_hash[i] != computed_hash[i] )
      return false;
  }

  return true;
}

void servoLock(boolean lock) {
  if(lock)
    servoMotor.write(LOCK_POSITION);
  else
    servoMotor.write(UNLOCK_POSITION);
  delay(100);
}

void save(const byte* PIN,const byte PINSize) {
  byte measuredLight = measureLight();
  byte measuredTemperature = measureTemperature();
  byte measuredPressure = measurePressure();
  long lat = 0L, lon = 0L; 
  measureLocation(&lat, &lon);
  byte *new_hash = hashPINS(PIN, PINSize, measuredLight, measuredTemperature, measuredPressure, lat, lon);

  for (unsigned int i = 0; i < HASH_SIZE; i++)
    EEPROM.write(HASH_ADDR + i, new_hash[i]);
  EEPROM.write(LOCK_ADDR, LOCKED);
}

void reset() {
  for (unsigned int i = 0; i < HASH_SIZE; i++)
    EEPROM.write(HASH_ADDR, 0);
  EEPROM.write(LOCK_ADDR, UNLOCKED);
}

byte* hashPINS(const byte* PIN, const byte PINSize, const byte measuredLight, const byte measuredTemperature, const byte measuredPressure, const long lat, const long lon) {

  byte msg[PINSize + 3 + 8];                                
  unsigned int msgLen = PINSize + 3 + 8;

  for (unsigned int i=0; i<PINSize; i++)
    msg[i] = PIN[i];

  msg[PINSize    ] = measuredLight;
  msg[PINSize + 1] = measuredTemperature;
  msg[PINSize + 2] = measuredPressure;
  msg[PINSize + 3] = (byte) lat;
  msg[PINSize + 4] = (byte) lat >> 8;
  msg[PINSize + 5] = (byte) lat >> 16;
  msg[PINSize + 6] = (byte) lat >> 24;
  msg[PINSize + 7] = (byte) lon;
  msg[PINSize + 8] = (byte) lon >> 8;
  msg[PINSize + 9] = (byte) lon >> 16;
  msg[PINSize + 10] = (byte) lon >> 32;

  sipHash.initFromRAM(hashKey);  
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
  byte temperature = 0;
  //byte temperature = analogRead(temperaturepin) >> 7;
  log("Temperature:",temperature);
  return temperature;
}

byte measurePressure() {
  byte pressure = 0;
  //byte pressure = analogRead(pressurepin) >> 7;
  log("Pressure:",pressure);
  return pressure;
}

boolean measureLocation(long* lat, long* lon) {
    
  boolean validPosition = false;
  unsigned long fix_age = 0L;
  for (unsigned long start = millis(); millis() - start < 2000;) 
  {
  while(Serial.available())
  {
    char c = Serial.read();
    if (gps.encode(c))
      validPosition = true;
  }  
  }
  
  if(validPosition)
    { 
      gps.get_position(lat, lon, &fix_age);    
      *lat = *lat / 10000;
      *lon = *lon / 10000;
    }
  log("Latitude:",*lat);
  log("Longditude:",*lon);
  
  return validPosition;
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

void log(const char *name, long value) {
  Serial.print(name);
  Serial.print("\t");
  Serial.print(value);
  Serial.print("\n");
}

void logc(const char *name, char value) {
  Serial.print(name);
  Serial.print("\t");
  Serial.print(value);
  Serial.print("\n");
}

void logh(const byte* stored_hash, const byte* computed_hash) {
  Serial.println("Hash Comparison");
  for (unsigned int i = 0; i < HASH_SIZE; i++) {
    Serial.print(stored_hash[i]); 
    Serial.print("-");
    Serial.print(computed_hash[i]);
    Serial.print("\n");
  }
}

void ledLocked(boolean isLocked) {
  if(isLocked) {
    ledGreen(true);
    ledRed(false);
  }
  else {
    ledGreen(false);
    ledRed(true);
  }
}

