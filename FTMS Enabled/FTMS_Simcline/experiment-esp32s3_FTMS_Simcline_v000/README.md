# <img src="https://github.com/Berg0162/simcline/blob/master/images/SC_logo.png" width="64" height="64" alt="SIMCLINE Icon"> &nbsp; SIMCLINE for FTMS enabled Trainers <br> based on the LILYGO ESP32S3 T-Display board <br>(experiment in progress!)

## Description
T-Display-S3 is a ESP32-S3 development board. It is equipped with a color 1.9" LCD screen and two programmable buttons. Communication using I8080 interface. Retains the same layout design as T-Display. You can directly use ESP32S3 for USB communication or programming.

## LilyGo ESP32S3 T-display
This board and ESP32S3 processor was selected for its excellent specifications crisp and colorful display. Aside of gaining experience with the [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) library, creating a much richer visual user interface (than a SSD1306 0,96 Inch) was the dominant incentive. A lot of inspiration and practical knowledge was obtained by studying on Youtube: [the Volos Projects](https://www.youtube.com/c/VolosProjects). This ended up sofar in a good working and visually attractive Simcline 2.0 interface! What's more, in the experiments the processor was behaving like to be expected (despite the high display interface load) and I did not experience (during operation) any response difference with other ESP32 boards (with minimal display interface load) and that have been tested thoroughly...<br>
Unfortunately the LilyGo board does <b>NOT fit the original components box</b>, so that needs extra attention in the near future. Technically there is no reason sofar not to like this board!!!

## Arduino IDE 2.2
The present code is developed on Arduino IDE 2.2. Notice that you will need an Arduino IDE that is tailored for this specific <b>ESP32S3</b> processor!<br>

## Before you start
Please follow the installation instructions for the [ESP32S3 T-display](https://github.com/Xinyuan-LilyGO/T-Display-S3)<br>
If you select in the menu bar of Arduino IDE 2.2 <b>Tools</b>, the settings for the <b>ESP32S3</b> processor and the project are the following:
<p align=center>
<img src="https://github.com/Berg0162/s3-switch/blob/main/images/LilyGO_ESP32S3_Tools_Settings.png" width="350" height="400" alt="S3-Switch">
</p>
<br clear="left">

## T-Display-S3 Pinout
<img src="https://github.com/Berg0162/simcline/blob/master/images/T-Display-S3-pinout.png">

## Quick Specs
- MCU: ESP32-S3R8 Dual-core LX7 microprocessor
- Wireless Connectivity: Wi-Fi 802.11, BLE 5 + BT mesh
- Programming Platform: Arduino-ide, Micropython
- Flash: 16MB
- PSRAM: 8MB
- Bat Voltage Detection: IO 04
- Onboard Functions: Boot + Reset + IO 14 Button
- LCD: 1.9" diagonal, Full-color TFT Display
- Drive Chip: ST7789V
- Resolution: 170(H)RGB x320(V) 8-Bit Parallel Interface
- Working Power Supply: 3.3V
- Supports: STEMMA QT / Qwiic
- Connector: JST-SH 1.0mm 4 PIN
- Dimensions: 62 x 26 x 10mm

# Electronic Components and Circuitry used in experimental version<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_Light_ESP32S3_T_Display_Wiring.png"  alt="Circuitry version 2.0">
I have chosen for the following 4 compact active components that are slightly different from the earlier SIMCLINE project:<br>
<b>Adafruit DRV8871 DC Motor Driver</b><br>
A small one channel motor driver for 12 V (6.5 - 48 V) and 3,6 Amperes max. This board enables the processor to set the Actuator motor in up or down movement. It transforms logical digital levels (Go Up, Go Down and Stop) from the Feather nRF52/ESP32 to switching of 12 Volt at 3,6 Amperes max., the levels at which the Actuator works. Notice that default the board comes limited to 2,6 Amperes and you need to add a resistor to set for max current level. Install Vertical Through Hole Male PCB Header Pins on the board; this will allow correct mounting of the board inside the components box!<br>
<b>LYLIGO ESP32S3 T-Display</b><br>
The programmed ESP32S3 is communicating with (a) the trainer to collect power output  information and (b) with the training App for resistance settings (like grade) or (c) with the Companion App on your mobile phone. The programmed ESP32S3 is in full control of the Simcline operation.<br>
<b>Pololu Time-of-Flight-Distance sensor VL6180X</b><br> The sensor board (12.7 * 17.8 mm) contains a very tiny laser source, and a matching sensor. The VL6180X can detect the "time of flight", or how long the laser light has taken to bounce back to the sensor. Since it uses a very narrow light source, it is perfect for determining distance of only the surface directly in front of it. The sensor registers quite accurately the (change in) position of the wheel axle during operation, by measuring the distance between the top of the inner frame and the reflection plate that is mounted on the carriage. The distance feedback of the sensor is crucial for determining how to set the position of the carriage and axle in accordance with the grade information that for example Zwift is using to set the resistance of the trainer. NOTICE: a) VL6180X boards are also offered by different suppliers and have different formfactors; b) Install Straight Angle Through Hole Male PCB Header Pins on the board; this will allow later flat mounting of the sensor board in the components box!<br>
<b>Pololu D24V5F5</b><br>
This is a small 5V, 500mA Step-Down Voltage Regulator that is responsible for voltage conversion from 12V to 5V, the power supply for all components boards. NOTICE: Install Straight Angle Through Hole Male PCB Header Pins on the board; this will allow later easy mounting of the sensor board in the components box!<br>

All components are documented very well and are low cost. There are lots of examples for use in an Arduino environment. They have turned out to be very reliable. The exact wiring of the components can be followed in the figure above.<br>

## Experimental Setup
<img src="https://github.com/Berg0162/simcline/blob/master/images/ESP32S3_Simcline_01.jpg" align="left" width=300 height=400>
<img src="https://github.com/Berg0162/simcline/blob/master/images/ESP32S3_Simcline_02.jpg" align="left" width=300 height=400>
<br clear="left">

## Display during a ride
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_ESP32S3_T_Display.png" align="left" width=283 height=628><br>
- Icons bar on the top shows active connections
- Actual road grade is shown in digits in the center of a gauge
- Gauge shows clockwise, positive up-hill-grades and counter clockwise, negative down-hill-grades
- Road profile is refreshed with every event (i.e. change of road grade) and shows the last 24 events
- Position in vertical color palette legend is dynamically shown
- Color palette for grade percentage is taken from [www.CylingCols.com](https://www.cyclingcols.com/col/Angliru).
<img src="https://github.com/Berg0162/simcline/blob/master/images/GalibierSE.gif" align="left" width=539 height=280>
<br clear="left">

## Video of display during fictional riding
Notice that the road inclination data were randomly generated (between -10 and +20) in a pace that one will never meet in the real world, just to show the interface during a long ride!
[See video on Youtube](https://www.youtube.com/watch?v=asnAkheFVb0&t=11s)
