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

#define CURRENT_PIN_CC  A2
#define CURRENT_PIN_BATT A0
#define VOLTAGE_PIN_BATT A3


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

const float FACTOR = 40.0/1000;
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
  lcd1.print("battery voltage:");
  lcd1.setCursor(0,2);
  lcd1.print("battery current:");

  lcd2.setCursor(0,0);
  lcd2.print("CC voltage:");
  lcd2.setCursor(0,2);
  lcd2.print("CC current:");


  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)


}

void loop() {
// put your main code here, to run repeatedly:
//  sensor1.current = analogRead(A0);
//  sensor1.voltage = analogRead(A1);
//  sensor1.current = MyMap(sensor1.current, 0, 1023, 0, 50); //0-50 range for 50-amp sensor
//  sensor1.voltage = MyMap(sensor1.voltage, 0, 1023, 0, 12); 
//  computePower(sensor1);

  battery.voltage = readVoltage(VOLTAGE_PIN_BATT,16);
  battery.voltage -= 0.4; //manual compensation for incorrect resistor
  battery.current = readCurrent(CURRENT_PIN_BATT);
  battery.power = computePower(battery);

  segment1.printFloat(battery.voltage,3);
  segment2.printFloat(battery.current,3);
  segment3.printFloat(battery.power,3);
  segment1.writeDisplay();
  segment2.writeDisplay();
  segment3.writeDisplay();
  
  charge.current = readCurrent(CURRENT_PIN_CC);
//  segment2.printFloat(charge.current, 3);
//  segment2.writeDisplay();
  //Serial.println(charge.current,3);
  lcd2.setCursor(0,1);
  lcd2.print("---");
  lcd2.setCursor(0,3);
  lcd2.print(charge.current);

  lcd1.setCursor(0,1);
  lcd1.print(battery.voltage);
  lcd1.setCursor(0,3);
  lcd1.print(battery.current);
  
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

//maping function that does not truncate decimal (old map() type casts to interger format, here we use double)
double MyMap(double x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void disp_batt_level()//function to display battery level on the lower left neopixel bar
{
  int len = strip.numPixels()/2;
  int num_lit = map(battery.voltage,0,16,0,len);
  int R = 255, G = 0, B = 0;
  int interval = 255/len;
  int counter = 0;
  for(int i = LED_COUNT-1; i > len; i--)
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

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}
