#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#define DISPLAY1_ADDRESS   0x70
#define DISPLAY2_ADDRESS   0x71
#define DISPLAY3_ADDRESS   0x73


Adafruit_7segment segment1 = Adafruit_7segment();
Adafruit_7segment segment2 = Adafruit_7segment();
Adafruit_7segment segment3 = Adafruit_7segment();

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x3F, 20, 4); // Change to (0x27,16,2) for 16x2 LCD.

struct sensor {
  double current;
  double voltage;
  double power;
};

struct sensor sensor1;

int displayValue = 432;
double d1 = 0.123;
double d2 = 0.456;
double d3 = 0.789;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

  // Setup the display.
  segment1.begin(DISPLAY1_ADDRESS);
  segment2.begin(DISPLAY2_ADDRESS);
  segment3.begin(DISPLAY3_ADDRESS);

  //set the brightness (0-15)
  segment1.setBrightness(1);
  segment2.setBrightness(1);
  segment3.setBrightness(1);
  
  //segment1.print(displayValue, DEC);
  //segment1.printError();
  segment1.printFloat(d1,3);
  segment2.printFloat(d2,3);
  segment3.printFloat(d3,3);
  
  segment1.writeDisplay();
  segment2.writeDisplay();
  segment3.writeDisplay();

  lcd.setCursor(0,0);
  lcd.print("hello team P21462");


}

void loop() {
  // put your main code here, to run repeatedly:
  sensor1.current = analogRead(A0);
  sensor1.voltage = analogRead(A1);
  sensor1.current = MyMap(sensor1.current, 0, 1023, 0, 50); //0-50 range for 50-amp sensor
  sensor1.voltage = MyMap(sensor1.voltage, 0, 1023, 0, 12); 
  computePower(sensor1);


}

void computePower(struct sensor sens)
{
  sens.power = sens.voltage * sens.current;
}

//maping function that does not truncate decimal (old map() type casts to interger format, here we use double)
double MyMap(double x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
