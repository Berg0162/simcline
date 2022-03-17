 /* This example demonstrates how to use interleaved mode to
take continuous range and ambient light measurements. The
datasheet recommends using interleaved mode instead of
running "range and ALS continuous modes simultaneously (i.e.
asynchronously)".

In order to attain a faster update rate (10 Hz), the max
convergence time for ranging and integration time for
ambient light measurement are reduced from the normally
recommended defaults. See section 2.4.4 ("Continuous mode
limits") and Table 6 ("Interleaved mode limits (10 Hz
operation)") in the VL6180X datasheet for more details.

Raw ambient light readings can be converted to units of lux
using the equation in datasheet section 2.13.4 ("ALS count
to lux conversion").

Example: A VL6180X gives an ambient light reading of 613
with the default gain of 1 and an integration period of
50 ms as configured in this sketch (reduced from 100 ms as
set by configureDefault()). With the factory calibrated
resolution of 0.32 lux/count, the light level is therefore
(0.32 * 613 * 100) / (1 * 50) or 392 lux.

The range readings are in units of mm. */

#include <Wire.h>
#include <VL6180X.h>
#include <MovingAverageFilter.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1         // No reset pin on this OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

VL6180X sensor;

// Declare our filter
MovingAverageFilter movingAverageFilter_Range(10);     // sampling is at about 10 Hz --> 10 VL6180X-RANGE-readings per second

int16_t RawRange;
int16_t AverageRange;

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for feather nrf52 with native usb

// Init OLED Display
   if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
   { // Address 0x3C for 128x64
   Serial.println("SSD1306 allocation failed");
   }
  else
  {
  // Show Oled with initial display buffer contents on the screen --
  // the library initializes this with a Adafruit splash screen (edit the splash.h in the library).
  display.display();
  delay(2000);                    // Pause some time
  }
  
  Wire.begin();
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(3);        // !!!!!!!!!!!!!!!!!!!!!!

  // Reduce range max convergence time and the inter-measurement
  // -time to 30 ms and 50 ms, respectively, to allow 10 Hz
  // operation. Somewhat more power consumption but higher accuracy!
  sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor.writeReg(VL6180X::SYSRANGE__INTERMEASUREMENT_PERIOD, 50);
 
  sensor.setTimeout(500);

   // stop continuous mode if already active
  sensor.stopContinuous();
  // in case stopContinuous() triggered a single-shot
  // measurement, wait for it to complete
  delay(300);
  // start range continuous mode with period of 100 ms
  sensor.startRangeContinuous(100);
  
  Serial.print("Test VL6180X Range Continuous with Scaling = ");
  Serial.print(sensor.getScaling());
  Serial.println("x");

  Serial.print("Setting READOUT__AVERAGING_SAMPLE_PERIOD: ");
  Serial.println(sensor.readReg(VL6180X::READOUT__AVERAGING_SAMPLE_PERIOD));

  // Clear and set Oled display for further use
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(16,22);
  display.print("SIMCLINE"); 
  display.display(); 
  delay(1000);

}

void loop()
{
  Serial.print("Raw Range: ");
  RawRange = sensor.readRangeContinuousMillimeters();
  Serial.print(RawRange);
  AverageRange = movingAverageFilter_Range.process(RawRange);
  Serial.print(" Running average: "); Serial.print(AverageRange);
    
  // Show on OLED
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);  // Large characters
  display.setCursor(36,2);
  display.print(sensor.getScaling());
  display.print("x");
  display.setCursor(32,22);
  display.print(sensor.readRangeContinuousMillimeters()); 
  display.display(); 

 
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();


}
