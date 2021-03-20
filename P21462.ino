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

struct sensor sensor1;

int displayValue = 432;
double d1 = 0.123;
double d2 = 0.456;
double d3 = 0.789;

const float FACTOR = 40.0/1000;
const float QOV = 0.5*5.0;
float voltage;

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
  segment1.printFloat(d1,3);
  segment2.printFloat(d2,3);
  segment3.printFloat(d3,3);
  
  segment1.writeDisplay();
  segment2.writeDisplay();
  segment3.writeDisplay();

  lcd1.setCursor(0,0);
  lcd1.print("hello team P21462");

  lcd2.setCursor(0,0);
  lcd2.print("greetings team P21462");


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

  float raw_voltage = (5.0/1023.0)*analogRead(CURRENT_PIN_CC);
  raw_voltage = raw_voltage - QOV + 0.007;
  voltage = raw_voltage / FACTOR;
  segment2.printFloat(voltage, 3);
  segment2.writeDisplay();
  Serial.println(voltage,3);
  
  delay(500);
  //rainbow(1);             // Flowing rainbow cycle along the whole strip

}

void computePower(struct sensor sens)
{
  sens.power = sens.voltage * sens.current;
}

//maping function that does not truncate decimal (old map() type casts to interger format, here we use double)
double MyMap(double x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
