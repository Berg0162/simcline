# <img src="https://github.com/Berg0162/simcline/blob/master/images/SC_logo.png" width="64" height="64" alt="SIMCLINE Icon"> &nbsp; SIMCLINE for FTMS enabled Trainers (work in progress!)
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
From 2015 to 2017 the Sports and Fitness Working Group (SIG) designed a Bluetooth Service specification. This service exposes training-related data in the sports and fitness environment, which allows a Server (e.g., a fitness machine) to send training-related data to a Client. In Februari 2017 the service specification reached a stable version: 1.0 [see: FiTness Machine Service](https://www.bluetooth.com/specifications/specs/fitness-machine-service-1-0/) and it was adopted by the Bluetooth SIG Board of Directors. Have a look at the document to appreciate the effort of all the contributors and the companies they represented!<br>
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
+ When a training app (like Zwift) has connected to your trainer using the FTMS protocol: is it possible to connect multiple devices via FTMS? As FTMS enables control of a physical device there can only be one “master” to avoid safety issues. This means that you will not be able to connect multiple devices directly to the indoor bike trainer or treadmill using FTMS. If the trainer does not appear in an app’s (e.g. Zwift's) device list (on the Zwift pairing screen) it generally means the trainer is (still) connected to another app or device. It is virtually impossible to connect the trainer to Zwift, have a nice indoor ride and at the same time to connect for example the Simcline to the trainer or Zwift for simulating road incline..... A working solution is a Man-In-The-Middle (MITM)!
+ Notice that a fully working Simcline (acting as a MITM), supporting all FTMS enabled Trainers, is still in the future. At this moment only test code is available and you are invited to test this code with your FTMS enabled Trainer (if you own one).

# Man-In-The-Middle (MITM) software pattern<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/FTMS_Man_in_the_Middle.jpg" align="left" width="1000" height="500" alt="Man in the Middle"><br>
<b>Man-In-The-Middle</b> is a powerful software engineering pattern that is applied in many software designs. Unfortunately it is also known for a negative application in communication traffic: MITM is a common type of cybersecurity attack that allows attackers to eavesdrop on the communication between two targets.
We have applied the very principle: the Simcline is strategicly positioned in between the BLE communication of the FTMS enabled Trainer and the training App (like Zwift) running on the PC/Laptop, all communication traffic can be inspected in that MITM position, when it is passed on from one to the other, in both directions. When Zwift sends resistance information (like the road inclination) to the FTMS enabled trainer, this information can be intercepted and applied to determine the up/down positioning of the Simcline. <br>

# How to start?<br>
+ Install the [Arduino IDE](https://www.arduino.cc/en/Main/Software) and all the libraries on a PC/Mac.
+ Download all the code from [Github](https://github.com/Berg0162/simcline/tree/master/FTMS%20Enabled) and install. <br>

# How to make it work?<br>
The requirements in this test phase are simple: 
+ running Zwift app or alike, 
+ working Feather nRF52840/52832 board and 
+ a FTMS enabled Trainer.<br>

# Testing is Knowing!<br>
I can understand and respect that you have some reserve: Is this really working in my situation? Better test if it is working, before buying all components and start building.
In the Github repository (see above) you will find the appropriate files with test code: <b>FTMS_Client_v01</b> and <b>FTMS_Server_v01</b>. It is coded with the only intention to check if the MITM solution is delivering in your specific situation.<br>

<b>What it does in short:</b><br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/FTMS_Feather_Zwift_BLE.jpg" align="middle" width="950" height="700" alt="Simcline in the Middle"><br>
A working <b>MITM</b> implementation links a bike trainer (BLE Server FTMS) and a PC/Laptop (BLE Client running Zwift) with the Feather nRF52840/832, like a <b>bridge</b> in between. The MITM bridge can pass on, control, filter and alter the interchanged trafic data! The <b>MITM</b> test code is fully ignorant of mechanical or electronic components that drive the Simcline construction. It simply estabishes a virtual BLE bridge and allows you to ride the bike on the FTMS enabled Trainer and feel the resistance that comes with it, thanks to Zwift. The experience should not differ from a normal direct one-to-one connection, Zwift - FTMS enabled Trainer!<br>
All FTMS enabled indoor trainers expose your efforts on the bike in 2 additional BLE services: Cyling Power (CPS) and Speed & Cadence (CSC). These services are detected and applied by many training app's and are therefore an integral part of the present design of the MITM bridge. Training app's simply expect, when they connect to the FTMS enabled trainer, that the CPS and CSC services are available in one go! The Zwift pairing screen is a good example: it expects Power (CPS), Cadence (CSC) and a "Controllable" (with FTMS) to be connected...
+ The client-side (Feather nRF52) scans for (a trainer) and connects with <b>FTMS, CPS and CSC</b> and collects cyling power, speed and cadence data like Zwift would do! The code with the name: <b>FTMS_Client_v01</b> is doing just that one side of the "bridge"!
+ The Server-side (Feather nRF52) advertises and enables connection with training/cycling/game apps like Zwift and collects relevant resistance data, it simulates as if an active <b>FTMS</b> enabled trainer is connected to Zwift or alike! Notice that the Server-side also exposes active <b>CPS</b> and <b>CSC</b> services. The code with the name: <b>FTMS_Server_v01</b> is doing just the other side of the "bridge"!
+ The <b>MITM</b> code is connecting both sides at the same time: a full-blown bridge! (This code is still work in progress)<br clear="left">

<b>Use the test code for reconnaissance!</b><br>
Please follow the instructions at the first part of the program code!
+ Start your reconnaissance with running <b>FTMS_Client_plus_v01</b> and experience how the Feather is controlling the resistance of your FTMS enabled trainer. 
+ Next step is running <b>FTMS_Server_v01</b>, pairing with Zwift and then notice how your avatar is moving effortless in the Zwift world controlled by the nRF52 Feather.<br>

<i>The 2 presently available <b>test</b> programs (FTMS Client and FTMS Server) are only using Serial Monitor (screen output) to show what is happening!</i><br>
Please supply me with the Serial Monitor output (Copy-Paste) when pairing and/or connection processes are not successful or when error messages appear... If that is the case: Open an <b>Issue</b> and paste the relevant screen output in your <b>Issues</b> post to detail what went wrong! The community will be very gratefull with your help and feedback!

