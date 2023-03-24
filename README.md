# robawt2023

For robocupz 2023
*I RECOMMEND WHOEVER IS LOOKING AT THIS TO DOWNLOAD COLORFUL COMMENTS EXTENSION FOR EASE OF BROWSING THIS REPO (at least on the pi side)*

GUIDE TO DIRECTORIES

/pi/
Except for the images in .jpg and hsvcalib2.py, everything else is an updated copy of the pi's /home/pi/bot/ directory

/main/
For pico side code. 
Copied libraries include Pololu's L0X and L1X, Brett Beauregard Arduino's PID Library and elapsedMillis from Arduino
Created libraries include the drivebase for the robot (Vroom)

/main/main.ino
To be uploaded to Raspberry Pi Pico using Earl E. Philhower's Arduino-Pico Core (refer: https://arduino-pico.readthedocs.io/en/latest/index.html)
It can be uploaded to the pico using Arduino IDE with the setup mentioned in the documentation

/teensy/
For teensy side code.

/teensy/teensy.ino
To be uploaded to Teensy 2.0 using Teensyduino (refer: https://www.pjrc.com/teensy/teensyduino.html)
It can be uploaded to the teensy using Arduino IDE with the setup mentioned in the documentation

/prev_files_for_ref/
USED IN ROBOCUP 2022
app.ino --> Arduino Mega 2560 code
lt.py --> Linetrack on pi
roomba.py --> Evac on pi
roomba_lt.py --> Full code on pi
USED IN RCAP 2022
irworksiswear.py --> Full code on pi

