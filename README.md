# SIMCLINE 
<img src="https://github.com/Berg0162/simcline/blob/master/images/SC_logo.png" width="64" height="64" alt="SIMCLINE Icon"> <b>Simulation of Changing Road Inclination for Indoor Cycling</b></br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_And_Wheel.jpg" width="300" height="300" alt="SIMCLINE"></br>
The SIMCLINE physically adjusts the bike position to mimic hilly roads, climbing and descending. This allows the rider to naturally change position on the bike, engage climbing muscles, and improve pedaling technique to become a more efficient and powerful climber.</br>
Without user intervention the SIMCLINE will replicate inclines and declines depicted in (online & offline) training programs (like Zwift, Rouvy, VeloReality and many others) that adjust accordingly the resistance of the indoor trainer.</br>
The SIMCLINE auto connects at power up with a smart TACX trainer and let's relive the ascents and descents from favorite rides or routes while training indoors.</br>
The physical reach is: 20% maximum incline and -10% maximum decline. However, the reach that the rider is comfortable with can be adjusted even during a ride!</br>
The SIMCLINE pairs directly to the TACX smart trainer for a connection that notifies the SIMCLINE to simulate autonomous the (change in) physical grade of the road during an indoor ride.</br>
During operation an OLED display shows the road grade in digits and in graphics or cycling data from the trainer (speed, power, cadence, elapsed time and distance).</br>
The SIMCLINE Companion App (for Android smartphones) can be paired simultaneously for adjusting operational settings, like Ascent Grade Limit (between 0-20%) and Descent Grade Limit (between 0-10%), Road Grade Change Factor (between 0-100%) and the type of OLED display format.</br>
# Electronic Components and Circuitry</br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_circuitry.jpg" width="400" height="200" alt="Circuitry">
This project could have been elaborated with many different electronic parts that would lead to more or less the same succesfull end product. I have choosen for the following 4 active components: </br>
<b>Cytron Motor Driver MDD3A.</b> Two channel motor driver for 12 V and 3 Amperes with buttons to test manually the working of the attached DC motor. This board enables the processor to set the Actuator motor in up or down movement. It transforms logical digital levels (Go Up, Go Down and Stop) from the Feather nRF52 to switching of 12 Volt at 3 Amperes max., the levels at which the Actuator works.</br>
<b>Adafruit Feather nRF52 Bluefruit</b></br>
Is another easy-to-use all-in-one Bluetooth Low Energy board, with a native-Bluetooth chip, the nRF52832! It's Adafruits's take on an 'all-in-one' Arduino-compatible + Bluetooth Low Energy with built in USB and battery charging. It is a low power, handsome and fast processor board with lots of memory and I/O pins. Can easily be programmed over the USB connection. The Feather nRF52 is communicating with (a) the trainer to collect resistance information (like grade) and (b) with your mobile phone to transfer information that is shown in the Companion App. The Feather nRF52 is in full control of the Simcline operation.</br>
<b>OLED display blue 128x64 pixels</b></br>
Small display with screen of: 26.6 mm x 19 mm. Shows cycling data and diagnostic info that is gathered during operation by the Feather nRF52 to inform the Simcline user about relevant information. NOTICE: Install Right Angle Through Hole Male PCB Header Pins on the board; these will allow later flat mounting of the board on top of the frame! The photos show straight headers that had to be replaced later!</br>
<b>Time-of-Flight-Distance sensor VL6180X</b></br>
The sensor contains a very tiny laser source, and a matching sensor. The VL6180X can detect the "time of flight", or how long the laser light has taken to bounce back to the sensor. Since it uses a very narrow light source, it is perfect for determining distance of only the surface directly in front of it. The sensor registers quite accurately the (change in) position of the wheel axle during operation, by measuring the distance between the top of the inner frame and the reflection plate that is mounted on the carriage. The distance feedback of the sensor is crucial for determining how to set the position of the carriage and axle in accordance with the grade information that for example Zwift is using to set the resistance of the trainer.</br>
In retrospect I do not regret the choices made. All components are documented very well. There are lots of examples for use in an Arduino enviroment. They have turned out to be very reliable.</br>

# How to start?</br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_circuitry_02.jpg" width="250" height="250" alt="Cardboard">
When I started the project I did not have any practical experience with any of the components. So I had to setup the circuitry step by step adding components and did a lot of time consuming but instructive testing first.
My advice is to setup the electronic components first in a similar way as shown on the photo with the cardboard base. Use double sided adhesive tape but only attach it on sections that have no pcb-wiring or soldering, to avoid possible electrical interference. Install the Arduino IDE and all the libraries on a PC/Mac. You will find in this Github repository all the code that controls the Simcline and the Arduino test programs (modified for this project) that focus on components seperately and in conjunction. Download all the code from Github and install.</br>
# Physical Construction of SIMCLINE </br>

some text....</br>

# Construction of SIMCLINE </br>
<img src="https://www.instructables.com/assets/img/instructables-logo-v2.png" width="32" height="48" alt="Instructables"> [See Instructables](https://www.instructables.com)
