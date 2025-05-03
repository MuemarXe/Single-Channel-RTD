/*

Author: Evans Muema
GitHUb: MuemarXe
Date: 1/2/2025
This is a firmware code for a single channel RTD model that supports both Pt100 and Pt1000.
It is written for an ESP32 S3 microcontroller. It uses SPI pins for communication.
GPIO10	CS	Chip Select
GPIO11	SDI	MOSI(Master Out, Slave In)
GPIO13	SDO	MISO (Master In, Slave Out)
GPIO12	SCK	SPI Clock

GPIO7	S1	MUX Select (ref switch control)
GPIO6	LED	Debug LED (blinks during reading) 
GPIO4	Pt100 LED (Green)
GPIO5	Pt1000 LED (Blue)

*/

#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>

// SPI and Control Pin Definitions
#define CS_PIN        10  // Chip Select
#define MUX_SEL_PIN    7  // GPIO to control 74HC4053 MUX
#define DEBUG_LED_PIN  6  // GPIO connected to a debug LED
#define PT100_LED_PIN  4  // Green LED for Pt100
#define PT1000_LED_PIN 5  // Blue LED for Pt1000

// RTD and temperature conversion variables
double resistance;
uint8_t reg1, reg2;
uint16_t fullreg;
double temperature;

double Z1, Z2, Z3, Z4, Rt;
double RTDa = 3.9083e-3;
double RTDb = -5.775e-7;
double rpoly = 0;

void setup() {
  SPI.begin();
  Serial.begin(115200);
  pinMode(CS_PIN, OUTPUT);
  pinMode(MUX_SEL_PIN, OUTPUT);
  pinMode(DEBUG_LED_PIN, OUTPUT);
  pinMode(PT100_LED_PIN, OUTPUT);
  pinMode(PT1000_LED_PIN, OUTPUT);

  // Initially assume Pt100 (set MUX to 430 ohm ref)
  digitalWrite(MUX_SEL_PIN, LOW);
  digitalWrite(DEBUG_LED_PIN, HIGH); 
  digitalWrite(PT100_LED_PIN, LOW);
  digitalWrite(PT1000_LED_PIN, LOW);
  Serial.println("Initial RTD type assumed: Pt100");
}

void loop() {
  digitalWrite(DEBUG_LED_PIN, LOW); // blink during read
  delay(500);
  digitalWrite(DEBUG_LED_PIN, HIGH);

  readRegister();
  selectRTDType();
  convertToTemperature();

  delay(1000); 
}

void convertToTemperature() {
  Rt = resistance;
  Rt /= 32768.0;
  Rt *= (digitalRead(MUX_SEL_PIN) == LOW ? 430.0 : 4300.0);

  Z1 = -RTDa;
  Z2 = RTDa * RTDa - (4 * RTDb);
  Z3 = (4 * RTDb) / 100;
  Z4 = 2 * RTDb;

  temperature = Z2 + (Z3 * Rt);
  temperature = (sqrt(temperature) + Z1) / Z4;

  if (temperature >= 0) {
    Serial.print("Temperature: ");
    Serial.println(temperature);
    return;
  } else {
    Rt /= 100.0;
    rpoly = Rt;
    temperature = -242.02 + 2.2228 * rpoly;
    rpoly *= Rt;
    temperature += 2.5859e-3 * rpoly;
    rpoly *= Rt;
    temperature -= 4.8260e-6 * rpoly;
    rpoly *= Rt;
    temperature -= 2.8183e-8 * rpoly;
    rpoly *= Rt;
    temperature += 1.5243e-10 * rpoly;

    Serial.print("Temperature: ");
    Serial.println(temperature);
  }
}

void selectRTDType() {
  double estimatedResistance = resistance * 430.0 / 32768.0; // assume 430 by default
  if (estimatedResistance > 300.0) {
    digitalWrite(MUX_SEL_PIN, HIGH); // Switch to 4300 ohm (Pt1000)
    digitalWrite(PT100_LED_PIN, LOW);
    digitalWrite(PT1000_LED_PIN, HIGH);
    Serial.println("RTD type: Pt1000 selected");
  } else {
    digitalWrite(MUX_SEL_PIN, LOW);  // Switch to 430 ohm (Pt100)
    digitalWrite(PT100_LED_PIN, HIGH);
    digitalWrite(PT1000_LED_PIN, LOW);
    Serial.println("RTD type: Pt100 selected");
  }
}

void readRegister() {
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x90); // 4-wire mode Config reg
  SPI.transfer(0xB0); // Enable bias, 1-shot, etc.
  digitalWrite(CS_PIN, HIGH);

  delay(10); 

  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x01); // Read RTD MSB
  reg1 = SPI.transfer(0xFF);
  reg2 = SPI.transfer(0xFF);
  digitalWrite(CS_PIN, HIGH);

  fullreg = reg1;
  fullreg <<= 8;
  fullreg |= reg2;
  fullreg >>= 1;
  resistance = fullreg;

  SPI.transfer(0x80);
  SPI.transfer(0x90);
  SPI.endTransaction();

  Serial.print("Raw Resistance: ");
  Serial.println(resistance);
}
