# DAWA-6.1

French official website project : http://dawa.panik-po.com/

DAWA is an Arduino datalogger (and laptimer) shield for your motorbike.
It records 10 times per seconds in a CSV file lots of information.

# What's new in v6
* __New PCB__ : DAWA v6 is now a standalone PCB (Atmel SAMD21 chip like Arduino M0). v5 was a shield which needs an Arduino M0
* __New IMU__ : MPU-9250 is now used. This IMU is on a secondary PCB with his own µproc (Atmel SAMD21)
* __New OLED screen__ : A little bit bigger (1.3" vs 0.96"). The screen connector (SPI) is compatible with 2.4" 240x320 color LCD
* __New enclosure__ : A brand new 3D printed enclosure
* __Sensors autodetection__ : Infrared temperature sensors are now autodetected
* __Navigation__ : Add a navigation menu system and 4 buttons to allow configuration and use without smartphone and Bluetooth
* __Lights__ : Add 4 LEDS for specific events (best time on lap for example)
* __Connectors__ : All connectors are now on the same side (bottom), GPS connector is now SMA
* __GPS battery__ : Rechargeable LIR 2025 Battery

# What information is logged ?
* __Raw data acquisition__ : On Triumph bikes and many others, ECU values can be directly read (I personnaly use : RPM, SPEED, GEAR POSITION, THROTTLE and BRAKE state)
* __Environement values__ : A 9-axis sensor (MPU-9250) is used to store G-forces and I hope soon, roll and pitch
* __Position values__ : A UBLOX 10Hz GPS chip gets realtime coordinates
* __Infrared temperatures__ : You can plug up to 6 infrared temperature sensors (tyres or ground t° for example)
* __Additional inputs__ : You can measure 2 analog inputs and 2 digital inputs (suspension sensors for exemple)

# Where is it logged ?
Everything is stored on a micro SD card.  
10 times per seconds, a new line is created in a CSV file. This line contains every data values separated by a semicolon.  
Current values are displayed in realtime on the OLED screen attached.

# Could it be used as a laptimer ?
YES ! Since v4 and the integration of a 10Hz GPS chip, laptimer functions are available.

# How does it work ?
I couldn't make it easier !  
Press the button start recording, press again stop recording :)  
One CSV file is created on each new record.

# What about laptimer functions
A little bit more difficult, you have to put a file named "TRACKS.CSV" on the sdcard.  
This file will contain track name and finishline coordinates, one line per track :  
`<trackname>;<finishline lat. A>;<finishline lon. A>;<finishline lat. B>;<finishline lon. B>`  
`CAROLE;489799930;25224350;489800230;25226330`  
To keep precision, latitude and longitude should be converted to integers (multiply by 10 000 000).  
When start recording the closer track is automatically chosen.

# You said Bluetooth ?
Bluetooth connection is usefull in these 2 cases :
- You just finish your track session and want to know lap times / best lap
- Before using DAWA, some parameters could be adjusted, use the bluetooth console to setup them !

I'm using "Serial Bluetooth Terminal" on Androïd. Connect and type "help" to view all available commands

# What's next  
* __Laptimer split time__ : Add split time management

# Known bugs
* __Lean angle__ : Seems easy but in fact very complicated to obtain good values without drift

# Repository Contents
* /3D Case - Fusion 360 parts and STL files for 3D printing
* /Arduino - The .ino file you need to put in the Arduino M0
* /Documentation - Some brief explanations about this shield (french - not translated)
* /Eagle - All Eagle files (schematics, PCB/Gerber, libraries, parts list)

