#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#define DISPLAY1_ADDRESS   0x70
#define DISPLAY2_ADDRESS   0x71
#define DISPLAY3_ADDRESS   0x73

#define STRIP_PIN 7
#define LED_COUNT 28

#define TEMP_PIN A8

#define CURRENT_PIN_CC  A5
#define CURRENT_PIN_BATT A7
#define VOLTAGE_PIN_BATT A3
#define VOLTAGE_PIN_CC A1


Adafruit_7segment segment1 = Adafruit_7segment();
Adafruit_7segment segment2 = Adafruit_7segment();
Adafruit_7segment segment3 = Adafruit_7segment();

LiquidCrystal_I2C lcd1 = LiquidCrystal_I2C(0x3E, 20, 4); // Change to (0x27,16,2) for 16x2 LCD.
LiquidCrystal_I2C lcd2 = LiquidCrystal_I2C(0x27, 20, 4); // Change to (0x27,16,2) for 16x2 LCD.

Adafruit_NeoPixel strip(LED_COUNT, STRIP_PIN, NEO_GRB + NEO_KHZ800);

struct sensor {
  double current;
  double voltage;
  double power;
};

struct sensor charge;
struct sensor battery;

int displayValue = 432;
float temp;

const float FACTOR = 40.0/1000; // current sensor sensitivity is 40mV/A
const float QOV = 0.5*5.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  lcd1.init();
  lcd1.backlight();
  lcd2.init();
  lcd2.backlight();

  // Setup the display.
  segment1.begin(DISPLAY1_ADDRESS);
  segment2.begin(DISPLAY2_ADDRESS);
  segment3.begin(DISPLAY3_ADDRESS);

  //set the brightness (0-15)
  segment1.setBrightness(15);
  segment2.setBrightness(15);
  segment3.setBrightness(15);
  
  //segment1.print(displayValue, DEC);
  //segment1.printError();


  lcd1.setCursor(0,0);
  lcd1.print("batt voltage:");
  lcd1.setCursor(0,1);
  lcd1.print("batt current:");
  lcd1.setCursor(0,2);
  lcd1.print("Box Temp:");

  lcd2.setCursor(0,0);
  lcd2.print("CC voltage:");
  lcd2.setCursor(0,2);
  lcd2.print("CC current:");


  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

}

void loop() {

  battery.voltage = readVoltage(VOLTAGE_PIN_BATT,23.261);
  battery.voltage = 1.1833*battery.voltage+0.1535; //linear offset for voltage sensor
  battery.current = readCurrent(CURRENT_PIN_BATT);
  battery.power = computePower(battery);
  
  charge.voltage = readVoltage(VOLTAGE_PIN_CC,25);
  charge.current = readCurrent(CURRENT_PIN_CC);
  charge.power = computePower(charge);

  temp = readTemp(TEMP_PIN);
  
  printDisplays(battery);

  //Serial.println(charge.current,3);
  lcd2.setCursor(0,1);
  lcd2.print(charge.voltage);
  lcd2.setCursor(0,3);
  lcd2.print(charge.current);

  lcd1.setCursor(15,0);
  lcd1.print(battery.voltage);
  lcd1.setCursor(15,1);
  lcd1.print(battery.current);
  lcd1.setCursor(15,2);
  lcd1.print(temp);

  //Serial.println(battery.voltage);
  
  disp_batt_level();

 delay(500);
}


float computePower(struct sensor sens)
{
  return sens.voltage * sens.current;
}

float readCurrent(int pin)
{
  float raw_voltage = (5.0/1023.0)*analogRead(pin);
  float voltage = raw_voltage - QOV;
  float current = voltage / FACTOR;
  return current;
}

float readVoltage(int pin, float maximum)
{
  float input = analogRead(pin); //number from 0 to 1023 based on input voltage
  float voltage = MyMap(input,0,1023,0,maximum);
  return voltage;
  
}

float readTemp(int pin)
{
  float raw_voltage = (5.0/1023.0)*analogRead(pin);
  float temp = (raw_voltage-1.375)/(0.0225);
  return temp;
}

//maping function that does not truncate decimal (old map() type casts to interger format, here we use double)
double MyMap(double x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void disp_batt_level()//function to display battery level on the lower left neopixel bar
{
  int len = LED_COUNT/2;
  float float_num = MyMap(battery.voltage,12,16,0,len);
  Serial.println(float_num);
  int num_lit = (int) float_num;
  int R = 255, G = 0, B = 0;
  int interval = 255/len;
  int counter = 0;
  for(int i = LED_COUNT-1; i > len-1; i--)
  {
    if(counter < num_lit)
    {
      strip.setPixelColor(i,R,G,B);
      R -= interval;
      G += interval;
    }
    else
    {
      strip.setPixelColor(i,0,0,0);
    }
    counter++;
  }
  strip.show();
}

void printDisplays(sensor x)
{
  segment1.printFloat(x.voltage,3);
  segment2.printFloat(x.current,3);
  segment3.printFloat(x.power,3);
  segment1.writeDisplay();
  segment2.writeDisplay();
  segment3.writeDisplay();
}
