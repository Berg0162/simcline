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
From 2015 to 2017 the Sports and Fitness Working Group (SIG) designed a Bluetooth Service specification. This service exposes training-related data in the sports and fitness environment, which allows a Server (e.g., a fitness machine) to send training-related data to a Client. In Februari 2017 the service specification reached a stable version: [Fitness Machine Service 1.0](https://www.bluetooth.com/specifications/specs/fitness-machine-service-1-0/) when it was adopted by the Bluetooth SIG Board of Directors. Have a look at the document to appreciate the effort of all the contributors and the companies they represented!<br>
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
|Zwift|ANT+ FE-C and Bluetooth Smart FTMS on Zwift Hub smart trainer.|
| | cf: [DCRainmaker](https://www.dcrainmaker.com/2020/11/smart-cycle-trainer-recommendations-guide-winter.html)|

# Sofar the good news!<br>
+ When a training app (like Zwift) has connected to your trainer using the FTMS protocol: is it possible to connect multiple devices via FTMS? As FTMS enables control of a physical device there can only be one <b>“controller”</b> to avoid safety issues. This means that you will not be able to connect multiple devices directly to the indoor bike trainer or treadmill using FTMS. If the trainer does not appear in an app’s (e.g. Zwift's) device list (on the Zwift pairing screen) it generally means the trainer is (still) connected to another controlling app or device. It is virtually impossible to connect the trainer to Zwift using FTMS, have a nice indoor ride and at the same time to connect for example the Simcline to the trainer or Zwift for simulating road incline..... A working solution is a Man-In-The-Middle (MITM)!
+ Notice that a fully working Simcline (acting as a MITM), supporting FTMS enabled Trainers, is up for grabs. Several trainers of very different brands have been tested successfuly by now, and you are invited to test the available code with your FTMS enabled Trainer.
# What about ANT+ (FE-C) and FTMS at the same time<br>
+ When a training app (like Zwift) has connected to your trainer using the ANT+ protocol: is it possible to connect other devices via FTMS?<br>
Since this ANT+ connection enables control of the physical device (trainer) there can NOT be connected another <b>“controller”</b> at the same time over FTMS to avoid safety issues. Only one (1) controlling app is allowed to connect and drive the Trainer at any time. You know, 2 captains on one ship is a recipe for disaster!
If this case, unfortunately and undesirebly, happens with your equipment setup the controlling Client-side code will not connect or disconnect with an error message! So keep these worlds separated! If you intend to use devices with BLE and FTMS: mechanically disconnect the ANT+ dongle to avoid your controller App (like Zwift) to (auto)connect over ANT+.
# Man-In-The-Middle (MITM) software pattern<br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/FTMS_Man_in_the_Middle_02.jpg" align="left" width="1000" height="500" alt="Man in the Middle"><br>
<b>Man-In-The-Middle</b> is a powerful software engineering pattern that is applied in many software designs. Unfortunately it is also known for a negative application in communication traffic: MITM is a common type of cybersecurity attack that allows attackers to eavesdrop on the communication between two targets.
We have applied the very principle: the Simcline is strategicly positioned in between the BLE communication of the FTMS enabled Trainer and the training App (like Zwift) running on the PC/Laptop, all communication traffic can be inspected in that MITM position, when it is passed on from one to the other, in both directions. When Zwift sends resistance information (like the road inclination) to the FTMS enabled trainer, this information can be intercepted and applied to determine the up/down positioning of the Simcline. <br>

# Choose a Development board: nRF52840 or ESP32?<br>
Until early 2023 the Simcline project (a.o.) solely worked with the <b>Feather nRF52840 Express</b> developement board and the Bluefruit/Adafruit libraries. This is a very stable platform and gave reliable results during development and what's more during many indoor seasons of three times (5 hours in total) a week operation! However, the MITM application and actuator control (a.k.a. Simcline) is at the capacity limits of the nRF52840 processor. 
See for a reliable and proven solution: [Adafruit Feather nRF52840 Express](https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather) <br>

So the question was raised why not use the <b>ESP32</b>, a series of low-cost and low-power System on a Chip (SoC) microcontrollers developed by Espressif that include Wi-Fi and Bluetooth wireless capabilities and dual-core processor? See for an introduction: [Random Nerds Tutorials](https://randomnerdtutorials.com/getting-started-with-esp32/). Particularly the multiprocessing capabilities of the dual-core processor make the ESP32 a very tempting option in this project!
To benefit of the same formfactor (-> no change in design of the Simcline 2.0 component box!), I decided to (re)develop the Simcline code with the [Adafruit Feather ESP32 V2](https://learn.adafruit.com/adafruit-esp32-feather-v2) for a second platform.

# How to start?<br>
+ Install the [Arduino IDE](https://www.arduino.cc/en/Main/Software) and all the libraries on a PC/Mac.
+ Download all the code from [Github](https://github.com/Berg0162/simcline/tree/master/FTMS%20Enabled) and install. <br>

# How to make it work?<br>
The requirements in this phase are simple: 
+ running Zwift app or alike, 
+ working Feather nRF52840/ESP32-V2 development board and 
+ a FTMS enabled Trainer.<br>

# Testing is Knowing!<br>
I can understand and respect that you have some reserve: Is this really working in my situation? Better test if it is working, before buying all components and start building.
In the Github repository (see above) you will find the appropriate files with code: <b>FTMS_Client</b> and <b>FTMS_Server</b>. It is coded with the only intention to check if the MITM solution is delivering in your specific situation.<br>

<b>What it does in short:</b><br>
<img src="https://github.com/Berg0162/simcline/blob/master/images/FTMS_Feather_Zwift_BLE_02.jpg" align="middle" width="1000" height="500" alt="Simcline in the Middle"><br>
A working <b>MITM</b> implementation links a bike trainer (BLE Server FTMS) and a PC/Laptop (BLE Client running Zwift) with the Feather nRF52/ESP32, like a <b>bridge</b> in between. The MITM bridge can pass on, control, filter and alter the interchanged trafic data! The <b>MITM</b> code is fully ignorant of mechanical or electronic components that drive the Simcline construction.<br>
```
It simply estabishes a virtual BLE bridge and allows you to ride the bike on the FTMS enabled Trainer and 
feel the resistance that comes with the route you have choosen, thanks to Zwift.
The experience should not differ from a normal direct one-to-one connection, Zwift - FTMS enabled Trainer!
```
All FTMS enabled indoor trainers expose your efforts on the bike in 2 additional BLE services: Cyling Power (CPS) and Speed & Cadence (CSC). These services are detected and applied by many training app's and are therefore an integral part of the present design of the MITM bridge. Training app's simply expect, when they connect to the FTMS enabled trainer, that the CPS and CSC services are available in one go! The Zwift pairing screen is a good example: it expects Power (CPS), Cadence (CSC) and a "Controllable" (with FTMS) to be connected...
+ The client-side (Feather nRF52/ESP32) scans for (a trainer) and connects with <b>FTMS, CPS and CSC</b> and collects cyling power, speed and cadence data like Zwift would do! The code with the name: <b>FTMS_Client</b> is doing just that at the left side of the "bridge"!
+ The Server-side (Feather nRF52/ESP32) advertises and enables connection with training/cycling/game apps like Zwift and collects relevant resistance data, it simulates as if an active <b>FTMS</b> enabled trainer is connected to Zwift or alike! Notice that the Server-side also exposes active <b>CPS</b> and <b>CSC</b> services. The code with the name: <b>FTMS_Server</b> is doing just at the right side of the "bridge"!
+ The <b>MITM</b> code is connecting both sides at the same time: a full-blown working bridge, <b>FTMS_Bridge</b><br clear="left">

<i>The test programs (FTMS Client, FTMS Server and FTMS-Zwift-Bridge) are only using Serial Monitor (screen output) to show what is happening!</i><br>
```
Please write down the MAC/Device Addresses of a) your FTMS enabled trainer and b) your Desktop/Laptop with Zwift. 
These are presented in the Serial Monitor log file when running the Client and Server test code.
```
<b>Use the code for reconnaissance and testing!</b><br>
Please follow <b>ALWAYS</b> the different usage instructions at the first part of the respective program codes!
+ Start your reconnaissance with running <b>FTMS_Client</b> and experience how your development board is controlling the resistance of your FTMS enabled trainer. Notice that this piece of code is highly dependent on the type and brand of FTMS enabled Trainer and therefore most critical!
+ Be aware of undesirebly <b>autoconnect</b> with your standard equipment setup using ANT+ or FTMS. The FTMS Client (or FTMS-Zwift-Bridge) will reach an error state that does not help you getting representative results during the reconnaisance! Once again: 2 captains on one ship is a recipe for disaster!
```
/* 
 *  This Feather nRF52840/ESP32 tested code scans for the CPS, CSC and FTMS
 *  that the trainer is advertising, it tries to connect and then 
 *  enables .....
 *  
 *  Requirements: FTMS trainer and Feather nRF52/ESP32 board
 *  1) Upload and Run this code on the Feather nRF5/ESP322
 *  2) Start the Serial Monitor to catch verbose debugging and data info
 *  3) Power ON and Wake UP trainer -> do NOT connect with other devices
 *  4) Trainer and Feather should successfully pair or disconnect...
 *  5) Keep the Serial Monitor visible on top of all windows 
 *  6) Move the trainer pedals and notice/feel changes in resistance...
 *     The Client sends Resistance Parameters to the Trainer that coincide 
 *     with the first 5 minutes of the Zwift Volcano Circuit!
 *  7) Inspect the info presented by Serial Monitor.....
 *  
 */

```
+ Next step is running <b>FTMS_Server</b>, pairing with Zwift and then notice how your avatar is moving effortless in the Zwift world controlled by the Feather nRF52/ESP32. Notice that this particular piece of code is tested intensively by the author with the Zwift app.<br>
```
/* 
 *  This Feather nRF52/ESP32 tested code advertises and enables the relevant 
 *  Cycling Trainer Services: CPS, CSC and FTMS.
 *  It allows to connect to Cycling apps like Zwift (aka Client or Central)!
 *  It simulates a connected Cycling Trainer and in BLE terms it is a Server or 
 *  or in BlueFruit BLE library terms it is a Peripheral
 *  Requirements: Zwift app or alike and Feather nRF52/ESP32 board
 *  1) Upload and Run this code on the Feather nRF52/ESP32
 *  2) Start the Serial Monitor to catch debugging info
 *  3) Start Zwift and wait for the Devices Pairing Screen
 *  4) Unpair all previously paired devices
 *  5) Search on Zwift pairing screens for the Feather nRF52/ESP32: "Sim nRF52", resp. "Sim ESP32"
 *  6) Pair all four "simulated" devices: Power, Cadence, Heart Rate and Controllable
 *  7) Start a default Zwift ride or any ride you wish
 *     No need for you to do work on the trainer!
 *  8) Make Serial Monitor visible on top of the Zwift window 
 *  9) Inspect the info presented by Serial Monitor
 * 10) Notice how your avatar is riding the route all by itself...
 *  
 *
```
+ After two smoothly runs of the FTMS Client and Server, it is time to test the FTMS bridge!<br>
The <b>FTMS_Bridge</b> code needs the "hardware" addresses to unmistakingly establish a BLE connection with the targeted devices. I know it can be implemented differently but this is to avoid unwanted BLE connection(s) with an additional power meter, another fitness device or a second computer/laptop, etcetera.<br>
```
/* -----------------------------------------------------------------------------------------------------
 *             This code should work with all indoor cycling trainers that fully support,
 *        Fitness Machine Service, Cycling Power Service and Cycling Speed & Cadence Service
 * ------------------------------------------------------------------------------------------------------
 *
 *  The code links a BLE Server (a peripheral to Zwift) and a BLE Client (a central to the Trainer) with a bridge 
 *  in between, the Feather nRF52/ESP32 being man-in-the-middle (MITM). 
 *  The nRF52-bridge can control, filter and alter the bi-directional interchanged data!
 *  The client-side (central) scans and connects with the trainer relevant services: FTMS, CPS and CSC. It collects 
 *  all cyling data of the services and passes these on to the server-side....  
 *  The client-side supplies the indoor trainer with target and resistance control data.
 *  The server-side (peripheral) advertises and enables connection with cycling apps like Zwift and collects the app's  
 *  control commands, target and resistance data. It passes these on to the client-side....  
 *  The server-side supplies the app with the generated cycling data in return. 
 *  
 *  The client plus server (MITM) is transparent to the indoor trainer as well as to the training app Zwift or alike!
 *  
 *  Requirements: Zwift app or alike, Feather nRF52/ESP32 board and a FTMS/CPS/CSC supporting indoor trainer
 *  1) Upload and Run this code on the Feather nRF52/ESP32
 *  2) Start the Serial Monitor to catch debugging info
 *  3) Start/Power On the indoor trainer  
 *  4) Feather nRF52/ESP32 and trainer (with <name>) will pair as reported in the output
 *  5) Start Zwift on your computer or tablet and wait....
 *  6) Search on the Zwift pairing screens for the Feather nRF52/ESP32 a.k.a. "Sim <name>"
 *  7) Pair: Power, Cadence and Controllable one after another with "Sim <name>"
 *  8) Optionally one can pair as well devices for heartrate and/or steering (Sterzo)
 *  9) Start the default Zwift ride or any ride you wish
 * 10) Make Serial Monitor output window visible on top of the Zwift window 
 * 11) Hop on the bike: do the work and feel resistance change with the road
 * 12) Inspect the info presented by Serial Monitor.....
 *  
 *  Your trainer's device <name> is modified by the bridge to "Sim <name>", to allow for a clear distinction 
 *  between the bridge (simulating your trainer) and your original trainer, when advertising the trainer's services!
 *  You will notice this only when connecting to Zwift on the pairing screens!
 *  
 */
```

Look in the Bridge code for the following snippet and fill in the required addresses:
```C++
.
// -----------------------------------------------------------------
// Your hardware MAC/DEVICE ADDRESSES
// Laptop/Desktop Device Address that runs Zwift, printed as: [00:01:02:03:04:05]
// Little Endian: in reversed order !!!!
#define LAPTOPADDRESS {0x05,0x04,0x03,0x02,0x01,0x00}
// FTMS enabled Trainer Device Address, printed as: [00:01:02:03:04:05]
// Little Endian: in reversed order !!!!
#define TRAINERADDRESS {0x05,0x04,0x03,0x02,0x01,0x00}
// -----------------------------------------------------------------
.
```
The two precise device addresses are critical to assure a reliable test run! You have to insert the values in the program code before uploading the code to the Feather nRF52/ESP32!<br><br>

# Become a testing partner<br>

|Trainer brand/type  |Testing parter|FTMS_Client|FTMS_Server|FTMS_Bridge|Simcline_FTMS|Dev.Board|
|--------------------|--------------|-----------|-----------|-----------|-------------|---------|
|Elite Direto XR |[cherryphilip74](https://github.com/cherryphilip74)|Working|Working|Working|Working|nRF52840|
|Zwift Hub |[le-joebar](https://github.com/le-joebar)|Working|Working|Working|Working|nRF52840|

When you are ready for testing a trainer brand/type that is <b>NOT</b> yet shown in the above list: Please supply me with the Serial Monitor output (Copy-Paste) when pairing and/or connection processes are not successful or when error messages appear... 
Please supply me with detailed info about the trainer and your setup, preferably with screen shots of the [nRF Connect by Nordic](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp&hl=en&gl=US&pli=1) showing which Services and Characteristics your trainer is exposing. If you have collected all trainer and setup detailing information: Open [Issues](https://github.com/Berg0162/simcline/issues), click the green button: <b>New issue</b> and paste the relevant info in your <b>Issues</b> post to detail what went wrong! The community will be very gratefull with your help and feedback!<br>
# Zwift Hub users<br>
There is an excellent review available [DCRainmaker Zwift Hub review](https://www.dcrainmaker.com/2022/10/zwift-hub-smart-trainer-in-depth-review-the-best-bang-for-your-buck.html), that describes o.a. a special goodie that comes with the Zwift Hub:
>**– Protocol Compatibility:** ANT+ FE-C, ANT+ Power, Bluetooth Smart Trainer Control, Bluetooth Smart Power (everything you need)<br>
**– Unique Party Trick: Can rebroadcast your heart rate sensor within a single channel, ideal for Apple TV Zwift users (who are Bluetooth channel limited)**<br>
**– App Compatibility:** Every app out there basically (Zwift, TrainerRoad, Rouvy, RGT, The Sufferfest, Kinomap, etc…)<br>

The latest code versions for FTMS enabled trainers **fully** support this proprietary Zwift function for your heart rate sensor connection. The Simcline code supports the same features as you would have had with only Zwift App connected to Zwift Hub trainer! Use your heart rate band the way you are used too before the Simcline came in between your Zwift Hub trainer and the Zwift App! It should be fully transparent with respect to this feature! Please test yourself!

# Cleanup Zwift devices from the past<br>
Zwift can sometimes hang onto the wrong info, such as trainers or sensors that were paired to the game in the past. Zwift uses Mac Addresses from previous connections to identify devices. So when device names change Zwift hangs on to the unique Mac Addresses rather than the names that you see in the pairing screens! This can be rather confusing and lead to misunderstandings when you connect devices having only their original names shown and not the actual names....<br>
Check out the steps below.<br>
For <b>PC/Mac</b> to reset all the Zwift stored devices on a PC or Mac, complete these steps:
+ Close Zwift
+ On your desktop, open Documents
+ Double-click Zwift
+ Delete knowndevices.xml<br>

Next time you go for a Zwift ride:
+ Launch Zwift
+ Pair your devices

