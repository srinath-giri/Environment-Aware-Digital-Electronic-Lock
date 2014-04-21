
/*
Project 3 - srinathg@andrew.cmu.edu
 */

#include <Servo.h>
#include <SipHash_2_4.h>

Servo servoMotor;
int lightpin = A6;

const int LOCK_POSITION = 0;
const int UNLOCK_POSITION = 90;

void setup() {
  Serial.begin(9600);
  servoMotor.attach(9);
}

void loop() {
  char keyPressed = getKeyPressed();
  if(keyPressed == LOCK_KEY)
    lock();
  else if((keyPressed == UNLOCK_KEY))
    unlock();
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
  byte location = analogRead(locationpin) >> 7;
  log("Location:",location);
  return location;
}

void lock() {
  if(isLocked()) return;
  
  servoMotor.write(LOCK_POSITION);
  delay(20);
  save();
}

void unlock() {
  if(!isLocked()) return;

  servoMotor.write(UNLOCK_POSITION);
  delay(20);
}

boolean isLocked() {
  return false;
}

void save() {
  
  measureLight();
  
  EEPROM.write(LOCK_ADDR, LOCKED);
  
}

void log(char name[], int value) {
  Serial.print(name);
  Serial.print("\t");
  Serial.print(value);
  Serial.print("\n");
}

