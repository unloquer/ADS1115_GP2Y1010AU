/*
This sketch tests the library, outputting the values ready to Serial.
Only one gain setting is tested at a time.  If the gain is changed to
GAIN_SIXTEEN then some of the read voltagees exceed the gain range and should
read as the max value.  With the resistors below all other gains should 
produce correct results (within a reasonable tolerance). Note that reading
A0 (GND) may produce a slightly negative voltage

Sketch and circuit by soligen2010, Feb 12, 2016

Below are the components and connections for the test circuit for the test:

Components:

ADS1115
D1 1N4004 (Cathode to A0, Anode to Pin 3 for arduino - choose different pin for ESP8266)
R1 3k3    (VDD to A0)
R2 240R   (A0 to A1)
R3 1K     (A1 to A2)
R4 1K     (A2 to A3)

Connections:

VDD ---- +5, R1 (VDD to A0)
GND ---- Gound
SCL ---- Arduino Uno A5 or ESP8266 Pin 5
SDA ---- Arduino Uno A4 or ESP8266 Pin 4
ADDR --- GND
ALRT --- Digital Pin 2
A0 ----- R1, D1(Cathode to A0, Anode to Pin 3), R2
A1 ----- R2, R3
A2 ----- R3, R4
A3 ----- R4, GND

*/

#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use thi for the 12-bit version */

int ledPower = 10;   //Connect 3 led driver pins of dust sensor to Arduino D6
 
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
 
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

float multiplier;

void setup(void) 
{
  pinMode(ledPower,OUTPUT);
  Serial.begin(115200);
  Serial.println("Hello!");

  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  //ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  ads.begin();
  multiplier = ads.voltsPerBit()*1000.0F;           // Gets the millivolts per bit
}

void loop(void)
{
  int16_t adc0;

  //adc0 = ads.readADC_SingleEnded(0);
  //Serial.print("Single ADC 0: "); Serial.print(adc0); Serial.print("("); Serial.print(adc0 * multiplier); Serial.println("mV)");
  //Serial.println(1000.0F*ads.readADC_SingleEnded_V(0));

  digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(samplingTime);

  voMeasured = 1000.0F*ads.readADC_SingleEnded_V(0);// read the dust value
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH); // turn the LED off
  delayMicroseconds(sleepTime);

  // // 0 - 5V mapped to 0 - 32767 integer values
  // // recover voltage
  calcVoltage = voMeasured; //EL valor de la lectura del pin análogo conectado a 5v

  // // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // // Chris Nafis (c) 2012
  dustDensity = 0.17 * calcVoltage - 0.1;

  //Serial.print("Raw Signal Value (0-65536): ");
  //Serial.print(adc0);

  Serial.print("Voltage: ");
  Serial.print(calcVoltage);
  Serial.print("mV)");

  Serial.print(" - Dust Density: ");
  Serial.print(dustDensity);
  Serial.println(" mg/m3");

  // Serial.print(" GAS : ");
  // Serial.println(adc1);

  delay(1000);
}
