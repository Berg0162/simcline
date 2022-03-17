# <img src="https://github.com/Berg0162/simcline/blob/master/images/SC_logo.png" width="64" height="64" alt="SIMCLINE Icon"> &nbsp; SIMCLINE for Smart TACX and Wahoo KICKR trainers
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

# SIMCLINE 1.0<br>
There is an elaborated <b>Instructable</b> available with all the nitty gritty of how to buy or create, construct and install the various parts and components of the SIMCLINE.<br> 
See: <img src="https://www.instructables.com/assets/img/instructables-logo-v2.png" width="32" height="48" align="left" alt="Instructables"> [SIMCLINE 1.0 Instructables](https://www.instructables.com/id/SIMCLINE-Simulation-of-Changing-Road-Incline-for-I/)
<br clear="left">

# SIMCLINE 2.0<br>
There is an elaborated <b>Instructable</b> available with all the nitty gritty of how to buy or create, construct and install the various parts and components of the SIMCLINE.<br> 
See: <img src="https://www.instructables.com/assets/img/instructables-logo-v2.png" width="32" height="48" align="left" alt="Instructables"> [SIMCLINE 2.0 Instructables](https://www.instructables.com/SIMCLINE-20-Easy-Simulation-of-Road-Incline/)
<br clear="left">

# Electronic Components and Circuitry used in version 2.0<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_circuitry.jpg"  alt="Circuitry">
This project could have been elaborated with many different electronic parts that would lead to more or less the same succesfull end product. I have choosen for the following 4 active components: <br>
<b>Cytron Motor Driver MDD3A.</b> Two channel motor driver for 12 V and 3 Amperes with buttons to test manually the working of the attached DC motor. This board enables the processor to set the Actuator motor in up or down movement. It transforms logical digital levels (Go Up, Go Down and Stop) from the Feather nRF52 to switching of 12 Volt at 3 Amperes max., the levels at which the Actuator works.<br>
<b>Adafruit Feather nRF52840 Express</b><br>
Is another easy-to-use all-in-one Bluetooth Low Energy board with a native-Bluetooth chip, the nRF52840! Notice that the Feather nRF52840 Express is to be prefered and has better value for money!

> This chip has twice the flash, and four times the SRAM of its earlier sibling, the nRF52832 - 1 MB of FLASH and 256KB of SRAM. Compared to the nRF51, this board has 4-8 times more of everything.

It's Adafruits's take on an 'all-in-one' Arduino-compatible + Bluetooth Low Energy with built in USB and battery charging. It is a low power, handsome and fast processor board with lots of memory and I/O pins. Can easily be programmed over the USB connection. The programmed Feather nRF52840 is communicating with (a) the trainer to collect power output  information and (b) with the training App for resistance settings (like grade) or (c) with the Companion App on your mobile phone. The programmed Feather nRF52 is in full control of the Simcline operation.<br>
<b>OLED display blue 128x64 pixels</b><br>
Small display with screen of: 25 mm x 19 mm. Shows cycling data and diagnostic info that is gathered during operation by the programmed Feather nRF52 to inform the Simcline user about relevant information. NOTICE: Install Straight Pin Through Hole Male PCB Header on the board; these will allow later flat mounting of the board on top of the frame!<br>
<b>Time-of-Flight-Distance sensor VL6180X</b><br>
The sensor contains a very tiny laser source, and a matching sensor. The VL6180X can detect the "time of flight", or how long the laser light has taken to bounce back to the sensor. Since it uses a very narrow light source, it is perfect for determining distance of only the surface directly in front of it. The sensor registers quite accurately the (change in) position of the wheel axle during operation, by measuring the distance between the top of the inner frame and the reflection plate that is mounted on the carriage. The distance feedback of the sensor is crucial for determining how to set the position of the carriage and axle in accordance with the grade information that for example Zwift is using to set the resistance of the trainer. NOTICE: Install Right Angle Through Hole Male PCB Header Pins on the board.<br>
In retrospect I do not regret the component choices made. All components are documented very well. There are lots of examples for use in an Arduino enviroment. They have turned out to be very reliable.<br clear="left">

