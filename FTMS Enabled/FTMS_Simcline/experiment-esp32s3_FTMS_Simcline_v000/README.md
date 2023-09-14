# <img src="https://github.com/Berg0162/simcline/blob/master/images/SC_logo.png" width="64" height="64" alt="SIMCLINE Icon"> &nbsp; SIMCLINE for FTMS enabled Trainers based on the LILYGO ESP32S3 T-Display board (work in progress!)

<img src="https://github.com/Berg0162/simcline/blob/master/images/T-Display-S3-pinout.png">

# Electronic Components and Circuitry used in version 2.0<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_Light_ESP32S3_T_Display_Wiring.png"  alt="Circuitry version 2.0">
I have chosen for the following 5 compact active components that are slightly different from the earlier SIMCLINE project and that can finally all be mounted inside the components box:<br>
<b>Adafruit DRV8871 DC Motor Driver</b><br>
A small one channel motor driver for 12 V (6.5 - 48 V) and 3,6 Amperes max. This board enables the processor to set the Actuator motor in up or down movement. It transforms logical digital levels (Go Up, Go Down and Stop) from the Feather nRF52/ESP32 to switching of 12 Volt at 3,6 Amperes max., the levels at which the Actuator works. Notice that default the board comes limited to 2,6 Amperes and you need to add a resistor to set for max current level. Install Vertical Through Hole Male PCB Header Pins on the board; this will allow correct mounting of the board inside the components box!<br>
<b>LYLIGO ESP32S3 T-Display</b><br>

The programmed ESP32S3 is communicating with (a) the trainer to collect power output  information and (b) with the training App for resistance settings (like grade) or (c) with the Companion App on your mobile phone. The programmed ESP32S3 is in full control of the Simcline operation.<br>
<b>Pololu Time-of-Flight-Distance sensor VL6180X</b><br> The sensor board (12.7 * 17.8 mm) contains a very tiny laser source, and a matching sensor. The VL6180X can detect the "time of flight", or how long the laser light has taken to bounce back to the sensor. Since it uses a very narrow light source, it is perfect for determining distance of only the surface directly in front of it. The sensor registers quite accurately the (change in) position of the wheel axle during operation, by measuring the distance between the top of the inner frame and the reflection plate that is mounted on the carriage. The distance feedback of the sensor is crucial for determining how to set the position of the carriage and axle in accordance with the grade information that for example Zwift is using to set the resistance of the trainer. NOTICE: a) VL6180X boards are also offered by different suppliers and have different formfactors; b) Install Straight Angle Through Hole Male PCB Header Pins on the board; this will allow later flat mounting of the sensor board in the components box!<br>
<b>Pololu D24V5F5</b><br>
This is a small 5V, 500mA Step-Down Voltage Regulator that is responsible for voltage conversion from 12V to 5V, the power supply for all components boards. NOTICE: Install Straight Angle Through Hole Male PCB Header Pins on the board; this will allow later easy mounting of the sensor board in the components box!<br>

All components are documented very well and are low cost. There are lots of examples for use in an Arduino environment. They have turned out to be very reliable. The exact wiring of the components can be followed in the figure above.<br>
