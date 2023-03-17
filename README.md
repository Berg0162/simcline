# <img src="https://github.com/Berg0162/simcline/blob/master/images/SC_logo.png" width="64" height="64" alt="SIMCLINE Icon"> &nbsp; SIMCLINE for Smart TACX, Wahoo KICKR and FTMS enabled trainers
# Simulation of Changing Road Inclination for Indoor Cycling<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_And_Wheel.jpg" width="300" height="300" ALIGN="left" alt="SIMCLINE 1">
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_2_0.jpg" width="300" height="300" ALIGN="left" alt="SIMCLINE 2">
The SIMCLINE physically adjusts the bike position to mimic hilly roads, climbing and descending. This allows the rider to naturally change position on the bike, engage climbing muscles, and improve pedaling technique to become a more efficient and powerful climber.<br>
Without user intervention the SIMCLINE will replicate inclines and declines depicted in (online & offline) training programs (like Zwift, Rouvy, VeloReality and many others) that adjust accordingly the resistance of the indoor trainer.<br>
The SIMCLINE auto connects at power up with a smart Wahoo KICKR trainer or a smart TACX trainer and let's relive the ascents and descents from favorite rides or routes while training indoors.<br>
The physical reach is: 20% maximum incline and -10% maximum decline. However, the reach that the rider is comfortable with can be adjusted!<br>
The SIMCLINE pairs directly to the smart trainer and with your PC/Laptop with (Zwift) training App for a connection that notifies the SIMCLINE to simulate autonomous the (change in) physical grade of the road during an indoor ride.<br>
During operation an OLED display shows the road grade in digits and in graphics.<br>
The SIMCLINE Companion App (for Android smartphones) can be paired, only when the training App is disconnected, for adjusting operational settings, like Ascent Grade Limit (between 0-20%), Descent Grade Limit (between 0-10%), Road Grade Change Factor (between 0-100%) and manual Up and Down control. Notice that the Companion App has a slightly different functionality depending of what brand of trainer (TACX or Wahoo) is connected, due to specific connectivity differences. <br clear="left"> 
<br>
There are <b>Instructables</b> available with all the nitty gritty of how to create, construct and install the various parts and components of the different SIMCLINE versions.<br>

# [SIMCLINE version 1.0](https://www.instructables.com/id/SIMCLINE-Simulation-of-Changing-Road-Incline-for-I/)<br>
# [SIMCLINE version 2.0](https://www.instructables.com/SIMCLINE-20-Easy-Simulation-of-Road-Incline/)<br>

The applied electronic components in the 2 projects are somewhat different but NOT in an essential way, as documented hereafter.<br>
<b>NOTICE that the SIMCLINE control code can be run with both (mechanically different) SIMCLINE versions!</b> However, 3 different code versions are specially designed for operating with a TACX <b>or</b> Wahoo Kickr <b>or</b> FTMS enabled trainers!</b> You will find in the arduino folder all the test programs and the common libraries. These will help you to test the electronic components separately and built together of both SIMCLINE versions.<br> 

Finally you have to select the SIMCLINE code that is appropriate for the trainer of your liking!<br>

# [nRF52/ESP32 Code support for Smart TACX trainers](https://github.com/Berg0162/simcline/tree/master/Tacx%20Smart)<br>

# [nRF52/ESP32 Code support for Wahoo KICKR trainers](https://github.com/Berg0162/simcline/tree/master/Wahoo%20Kickr/)<br>

# [nRF52/ESP32 Code support for FTMS enabled trainers](https://github.com/Berg0162/simcline/tree/master/FTMS%20Enabled/)<br>

# SIMCLINE 1.0<br>
See: <img src="https://www.instructables.com/assets/img/instructables-logo-v2.png" width="32" height="48" align="left" alt="Instructables"> [SIMCLINE 1.0 Instructables](https://www.instructables.com/id/SIMCLINE-Simulation-of-Changing-Road-Incline-for-I/)
<br clear="left">

# Electronic Components and Circuitry used in version 1.0<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_circuitry.jpg"  alt="Circuitry version 1.0">
The original project has been elaborated with different electronic parts than the later version. I have then chosen for the following 4 active components: <br>
<b>Cytron Motor Driver MDD3A.</b> Two channel motor driver for 12 V and 3 Amperes with buttons to test manually the working of the attached DC motor. This board enables the processor to set the Actuator motor in up or down movement. It transforms logical digital levels (Go Up, Go Down and Stop) from the Feather nRF52/ESP32 to switching of 12 Volt at 3 Amperes max., the levels at which the Actuator works.<br>
<b>Adafruit Feather nRF52840 Express</b><br>
Is another easy-to-use all-in-one Bluetooth Low Energy board with a native-Bluetooth chip, the nRF52840! Notice that the Feather nRF52840 Express is to be prefered and has better value for money! It's Adafruits's take on an 'all-in-one' Arduino-compatible + Bluetooth Low Energy with built in USB and battery charging. It is a low power, handsome and fast processor board with lots of memory and I/O pins. Can easily be programmed over the USB connection. <br>
<b>Adafruit HUZZAH32 ESP32 Feather V2</b><br>
Recently we have shared Simcline code that is targeted for another of the Adafruit star Feathers: the <b>Adafruit HUZZAH32 ESP32 Feather V2</b> - with the fabulous ESP32 WROOM module. The ESP32 has both WiFi and Bluetooth Classic/LE support. The new HUZZAH32 V2 is Adafruit's redesigned ESP32-based Feather V2. Compared to the original Feather with only 4 MB Flash and no PSRAM, the V2 has 8 MB Flash and 2 MB PSRAM. Packed with everything people love about Feathers: built in USB-to-Serial converter, automatic bootloader reset, Lithium Ion/Polymer charger, and just about all of the GPIOs brought out.<br>
The programmed Feather nRF52/ESP32 is communicating with (a) the trainer to collect power output  information and (b) with the training App for resistance settings (like grade) or (c) with the Companion App on your mobile phone. The programmed Feather nRF52/ESP32 is in full control of the Simcline operation.<br>

<b>OLED display blue 128x64 pixels</b><br>
Small display with screen of: 25 mm x 19 mm. Shows cycling data and diagnostic info that is gathered during operation by the programmed Feather nRF52/ESP32 to inform the Simcline user about relevant information. NOTICE: Install Straight Pin Through Hole Male PCB Header on the board; these will allow later flat mounting of the board on top of the frame!<br>
<b>Time-of-Flight-Distance sensor VL6180X</b><br>
The sensor contains a very tiny laser source, and a matching sensor. The VL6180X can detect the "time of flight", or how long the laser light has taken to bounce back to the sensor. Since it uses a very narrow light source, it is perfect for determining distance of only the surface directly in front of it. The sensor registers quite accurately the (change in) position of the wheel axle during operation, by measuring the distance between the top of the inner frame and the reflection plate that is mounted on the carriage. The distance feedback of the sensor is crucial for determining how to set the position of the carriage and axle in accordance with the grade information that for example Zwift is using to set the resistance of the trainer. NOTICE: Install Right Angle Through Hole Male PCB Header Pins on the board.<br>
In retrospect I do not regret the component choices made. All components are documented very well. There are lots of examples for use in an Arduino enviroment. They have turned out to be very reliable.<br clear="left">

# SIMCLINE 2.0<br>
See: <img src="https://www.instructables.com/assets/img/instructables-logo-v2.png" width="32" height="48" align="left" alt="Instructables"> [SIMCLINE 2.0 Instructables](https://www.instructables.com/SIMCLINE-20-Easy-Simulation-of-Road-Incline/)
<br clear="left">

# Electronic Components and Circuitry used in version 2.0<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_Light_Components_Wiring.png"  alt="Circuitry version 2.0">
I have chosen for the following 5 compact active components that are slightly different from the earlier SIMCLINE project and that can finally all be mounted inside the components box:<br>
<b>Adafruit DRV8871 DC Motor Driver</b><br>
A small one channel motor driver for 12 V (6.5 - 48 V) and 3,6 Amperes max. This board enables the processor to set the Actuator motor in up or down movement. It transforms logical digital levels (Go Up, Go Down and Stop) from the Feather nRF52/ESP32 to switching of 12 Volt at 3,6 Amperes max., the levels at which the Actuator works. Notice that default the board comes limited to 2,6 Amperes and you need to add a resistor to set for max current level. Install Vertical Through Hole Male PCB Header Pins on the board; this will allow correct mounting of the board inside the components box!<br>
<b>Adafruit Feather nRF52840 Express</b><br>
Is another easy-to-use all-in-one Bluetooth Low Energy board with a native-Bluetooth chip, the nRF52840! Notice that the Feather nRF52840 Express is to be preferred and has better value for money! It's Adafruit's take on an 'all-in-one' Arduino-compatible + Bluetooth Low Energy with built in USB and battery charging. It is a low power, handsome and fast processor board with lots of memory and I/O pins. Can easily be programmed over the USB connection.<br>
<b>Adafruit HUZZAH32 ESP32 Feather V2</b><br>
Recently we have shared Simcline code that is targeted for another of the Adafruit star Feathers: the <b>Adafruit HUZZAH32 ESP32 Feather V2</b> - with the fabulous ESP32 WROOM module. The ESP32 has both WiFi and Bluetooth Classic/LE support. The new HUZZAH32 V2 is Adafruit's redesigned ESP32-based Feather V2. Compared to the original Feather with only 4 MB Flash and no PSRAM, the V2 has 8 MB Flash and 2 MB PSRAM. Packed with everything people love about Feathers: built in USB-to-Serial converter, automatic bootloader reset, Lithium Ion/Polymer charger, and just about all of the GPIOs brought out.<br>
The programmed Feather nRF52/ESP32 is communicating with (a) the trainer to collect power output  information and (b) with the training App for resistance settings (like grade) or (c) with the Companion App on your mobile phone. The programmed Feather nRF52/ESP32 is in full control of the Simcline operation.<br>

<b>OLED display blue/white 128x64 pixels (0,96 Inch, I2C)</b><br>
Small display board has a critical overall board size of 25 mm * 27 mm (!); (See for example: [Webshop](https://www.pcboard.ca/oled-128x64)). Display area itself is: 25 mm x 14 mm. Shows cycling data and diagnostic info that is gathered during operation by the Feather nRF52/ESP32 to inform the SIMCLINE user about relevant information. NOTICE: a) Many different formfactors are offered at webshops; b) Install Vertical Through Hole Male PCB Header Pins on the board; this will allow correct mounting of the board inside the components box!<br>
<b>Pololu Time-of-Flight-Distance sensor VL6180X</b><br> The sensor board (12.7 * 17.8 mm) contains a very tiny laser source, and a matching sensor. The VL6180X can detect the "time of flight", or how long the laser light has taken to bounce back to the sensor. Since it uses a very narrow light source, it is perfect for determining distance of only the surface directly in front of it. The sensor registers quite accurately the (change in) position of the wheel axle during operation, by measuring the distance between the top of the inner frame and the reflection plate that is mounted on the carriage. The distance feedback of the sensor is crucial for determining how to set the position of the carriage and axle in accordance with the grade information that for example Zwift is using to set the resistance of the trainer. NOTICE: a) VL6180X boards are also offered by different suppliers and have different formfactors; b) Install Straight Angle Through Hole Male PCB Header Pins on the board; this will allow later flat mounting of the sensor board in the components box!<br>
<b>Pololu D24V5F5</b><br>
This is a small 5V, 500mA Step-Down Voltage Regulator that is responsible for voltage conversion from 12V to 5V, the power supply for all components boards. NOTICE: Install Straight Angle Through Hole Male PCB Header Pins on the board; this will allow later easy mounting of the sensor board in the components box!<br>

All components are documented very well and are low cost. There are lots of examples for use in an Arduino environment. They have turned out to be very reliable. The exact wiring of the components can be followed in the figure above.<br>

