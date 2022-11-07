# <img src="https://github.com/Berg0162/simcline/blob/master/images/SC_logo.png" width="64" height="64" alt="SIMCLINE Icon"> &nbsp; SIMCLINE for FTMS enabled Trainers
# Simulation of Changing Road Inclination for Indoor Cycling<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_And_Wheel.jpg" width="300" height="300" ALIGN="left" alt="SIMCLINE">
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_2_0.jpg" width="300" height="300" ALIGN="left" alt="SIMCLINE 2">
The SIMCLINE physically adjusts the bike position to mimic hilly roads, climbing and descending. This allows the rider to naturally change position on the bike, engage climbing muscles, and improve pedaling technique to become a more efficient and powerful climber.<br>
Without user intervention the SIMCLINE will replicate inclines and declines depicted in (online & offline) training programs (like Zwift, Rouvy, VeloReality and many others) that adjust accordingly the resistance of the indoor trainer.<br>
The SIMCLINE auto connects at power up with a FTMS enabled smart trainer and let's relive the ascents and descents from favorite rides or routes while training indoors.<br>
The physical reach is: 20% maximum incline and -10% maximum decline. However, the reach that the rider is comfortable with can be adjusted!<br>
The SIMCLINE pairs directly to the FTMS enabled trainer and with your PC/Laptop/Tablet with (Zwift) training App for a connection that notifies the SIMCLINE to simulate autonomous the (change in) physical grade of the road during an indoor ride.<br>
During operation an OLED display shows the road grade in digits and in graphics.<br>
The SIMCLINE Companion App (for Android smartphones) can be paired, only when the training App is disconnected, for adjusting operational settings, like Ascent Grade Limit (between 0-20%), Descent Grade Limit (between 0-10%), Road Grade Change Factor (between 0-100%) and manual Up and Down control. Notice that the Companion App has a slightly different functionality depending of what brand of trainer (TACX, Wahoo or FTMS enabled) is connected, due to specific connectivity differences. <br clear="left">

# FiTness Machine Service a Bluetooth Service Specification
From 2015 to 2017 the Sports and Fitness Working Group (SIG) designed a Bluetooth Service specification. This service exposes training-related data in the sports and fitness environment, which allows a Server (e.g., a fitness machine) to send training-related data to a Client. In Februari 2017 the service specification reached a stable version: 1.0 [see: FiTness Machine Service](https://www.bluetooth.com/specifications/specs/fitness-machine-service-1-0/).
<b>FTMS</b> is an open (nonproprietary) protocol that is not owned by any particular company and not limited to a particular company's product. It can be compared in that respect with FE-C over ANT+, however <b>FTMS</b> is targeted to control fitness equipment over <b>Bluetooth</b>!<br>
According to the smart trainer recommendations guide winter 2020-2021 of [DCRainmaker](https://www.dcrainmaker.com/2020/11/smart-cycle-trainer-recommendations-guide-winter.html) the situation evolved:
>Meanwhile, for Bluetooth Smart, there’s FTMS, which is basically the same thing as FE-C when it comes to trainers. It’s not quite as widely adopted yet by trainer companies, but is by app companies. On the trainer company side only Elite, Saris, and Kinetic support it across the board. With Tacx having it on some but not all units, and Wahoo having it on no units (but all Wahoo and Tacx trainers support private Bluetooth Smart with all major apps anyway). Each year Wahoo and Tacx say they’ll get around to adding it to their higher-end units, and each year they don’t (practically speaking though, it won’t impact your app usage, since all major apps support Tacx/Wahoo variants anyway).<br>

|Trainer  |Supported protocols|
|-----------|-------------------------------------------------------------------------------------| 
|Elite |ANT+ FE-C and Bluetooth Smart FTMS on all 2020 smart trainers.|
|Gravat |ANT+ FE-C and Bluetooth Smart FTMS on all 2020 smart trainers|
|JetBlack |ANT+ FE-C and Bluetooth Smart FTMS on all 2020 smart trainers.|
|Kinetic |ANT+ FE-C and Bluetooth Smart FTMS on all 2020 smart trainers.|
|Minoura |ANT+ FE-C and Bluetooth Smart FTMS on all 2020 smart trainers.|
|Saris |ANT+ FE-C and Bluetooth Smart FTMS on all 2020 smart trainers.|
|STAC |ANT+ FE-C and Bluetooth Smart FTMS on all 2020 smart trainers.|
|Tacx |ANT+ FE-C on all ‘Smart’ branded trainers (except Satori). FTMS on all non-NEO models.|
|Wahoo |ANT+ FE-C on all smart trainers. Gives developers access to private Wahoo Bluetooth Smart control, no proper FTMS support yet.|
| | cf: [DCRainmaker](https://www.dcrainmaker.com/2020/11/smart-cycle-trainer-recommendations-guide-winter.html)|

# Sofar the good news!<br>
When a training app (like Zwift) has connected to your trainer using the FTMS protocol: is it possible to connect multiple devices via FTMS?
As FTMS enables control of a physical device there can only be one “master” to avoid safety issues. This means that you will not be able to connect multiple devices directly to the indoor bike trainer or treadmill using FTMS. If the trainer does not appear in an app’s (e.g. Zwift's) device list (on the Zwift pairing screen) it generally means the trainer is (still) connected to another app or device. It is virtually impossible to connect the trainer to Zwift, have a nice indoor ride and at the same time to connect for example the Simcline to the trainer or Zwift for simulating road incline..... 

# Man-In-The-Middle (MITM) software pattern<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/FTMS_Man_in_the_Middle.jpg" align="left" width="1000" height="500" alt="Man in the Middle"><br>
<b>Man-In-The-Middle</b> is a powerful software engineering pattern that is applied in many software designs. Unfortunately it is also known for a negative application in communication traffic: MITM is a common type of cybersecurity attack that allows attackers to eavesdrop on the communication between two targets.
We have applied the very principle: the Simcline is strategicly positioned in between the BLE communication of the FTMS enabled Trainer and the training App (like Zwift) running on the PC/Laptop, all communication traffic can be inspected in that MITM position, when it is passed on from one to the other, in both directions. When Zwift sends resistance information (like the road inclination) to the FTMS enabled trainer, this information can be intercepted and applied to determine the up/down positioning of the Simcline. <br>

# How to start?<br>
+ Install the [Arduino IDE](https://www.arduino.cc/en/Main/Software) and all the libraries on a PC/Mac.
+ Download all the code from [Github](https://github.com/Berg0162/simcline) and install.
<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/Simcline_circuitry_02.jpg" align="left" width="200" height="200" alt="Cardboard">
When I started the project in 2020 I did not have any practical experience with any of the components. So I had to setup the circuitry step by step adding components and did a lot of time consuming but instructive testing first. My advice is to setup (<b>some of</b>) the electronic components first in a similar way as shown on the photo with the cardboard base. Use double sided adhesive tape but only attach it on sections that have no pcb-wiring or soldering, to avoid possible electrical interference. You will find in this Github repository all the code that controls the Simcline and the Arduino test programs (modified for this project) that focus on components separately and in conjunction. <br clear="left">

# To see is to believe!<br>
I can understand and respect that you have some reserve: Is this really working in my situation? Better test if it is working, before buying all components and start building.
In the Github repository (see above) you will find the appropriate test code named: <b>Test_FTMS_Client_v03</b>, <b>Test_FTMS_Server_v03</b> and <b>Test_FTMS_Zwift_Bridge_v031</b>. It is coded with the only intention to check if the MITM solution is delivering in your specific situation.<br>

<b>What it does in short:</b><br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/FTMS_Feather_Zwift_BLE.jpg" align="middle" width="950" height="700" alt="Simcline in the Middle"><br>
The <b>Test_FTMS_Zwift_Bridge</b> code links a bike trainer (BLE Server FTMS) and a PC/Laptop (BLE Client running Zwift) with the Feather nRF52840/832, like a bridge in between. The MITM bridge can pass on, control, filter and alter the interchanged trafic data! This test code is fully ignorant of the mechanical components that drive the Simcline. It simply estabishes a virtual BLE bridge and allows you to ride the bike on the FTMS enabled trainer and feel the resistance that comes with it, thanks to Zwift. The experience should not differ from a normal direct one-to-one connection, Zwift - FTMS enabled trainer!<br>
+ The client-side scans for and connects with <b>FTMS, CPS and CSC</b> and collects cyling power, speed and cadence data like Zwift would do! The code with the name: <b>Test_FTMS_Client_v03</b> is doing just that one side of the "bridge"!
+ The Server-side advertises and enables connection with Cycling apps like Zwift and collects relevant resistance data, it simulates as if an active FTMS enable trainer is connected to Zwift or alike! The code with the name: <b>Test_FTMS_Server_v03</b> is doing just the other side of the "bridge"!
+ The <b>Test_FTMS_Zwift_Bridge_v031</b> code is connecting both sides at the same time: the full-blown bridge!<br clear="left">

<b>How to make it work?</b><br>
The requirements are simple: 
+ running Zwift app or alike, 
+ working Feather nRF52840/52832 board and 
+ a FTMS enabled Trainer.<br>

<b>Use the test code for reconnaissance!</b><br>
Please follow the instructions at the first part of the program code!
+ Start your reconnaissance with running <b>Test_FTMS_Client_v03</b> and experience how the Feather is controlling the resistance of your FTMS enabled trainer. 
+ Next step is running <b>Test_FTMS_Server_v03</b>, pairing with Zwift and then notice how your avatar is moving effortless in the Zwift world controlled by the nRF52 Feather.<br>

<i>The 3 <b>test</b> programs (Client, Server and Bridge) are only using Serial Monitor (screen output) to show what is happening!</i><br>
Please write down the MAC or Device Addresses of a) your FTMS enabled trainer and b) your Desktop/Laptop with Zwift. These are presented in the Serial Monitor log file when running the Client and Server test code.<br>

<b>Now it is time to test the bridge!</b><br>
The <b>Test_FTMS_Zwift_Bridge_v031</b> code needs these "hardware" addresses to unmistakingly establish a BLE connection with the right device. I know it can be implemented differently but this is to avoid unwanted BLE connection(s) with an additional power meter, another fitness device or a second computer/laptop, etcetera.
