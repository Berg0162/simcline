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

VL6180X sensor;

// Declare our filter
MovingAverageFilter movingAverageFilter_Range(10);     // sampling is at about 10 Hz --> 10 VL6180X-RANGE-readings per second

int16_t RawRange;
int16_t AverageRange;

void setup()
{
  Serial.begin(115200);
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

  Serial.print("READOUT__AVERAGING_SAMPLE_PERIOD: ");
  Serial.println(sensor.readReg(VL6180X::READOUT__AVERAGING_SAMPLE_PERIOD));
 

}

void loop()
{
  Serial.print("Range: ");
  RawRange = sensor.readRangeContinuousMillimeters();
  Serial.print(RawRange);
  
  AverageRange = movingAverageFilter_Range.process(RawRange);
  
  Serial.print(" Running average: "); Serial.print(AverageRange);
 
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();


}
