v.st vector board
====
This is a modified version of the v.st vector board that targets installation in arcade cabinets with an Electrohome GO5-802 / 805 monitor.

The original project which targets Vectrex monitors can be found here: https://github.com/osresearch/vst

## Step 1: Building the hardware
The circuit board design in this repo is not up to date.  
https://github.com/osresearch/vst/issues/25#issuecomment-702268536

## Step 2: Flashing the Firmware
The firmware in this repo is not up to date.
https://github.com/osresearch/vst/issues/25#issuecomment-700355945

Make sure you have [Arduino](https://www.arduino.cc/en/Main/Software) and [Teensyduino](https://www.pjrc.com/teensy/td_download.html) installed.

Open *teensyv/teensyv.ino* in the Arduino IDE. Next select *Teensy 3.1 / 3.2* from the *Tools -> Board* menu in the Arduino IDE. Hit verify and follow the on screen instructions to flash your Teensy with the firmware.

## Step 3: Running the Demos
The demos are not up to date for the current version of Processing and use on Windows.

Make sure you have [Processing](https://processing.org/) installed and your Teensy is flashed with the firmware.

Open *processingDemo/processingDemo.pde* in Processing. Plug your v.st board into your computer and hit Run in Processing.

## Step 4: Compiling MAME
https://github.com/osresearch/mame4all-pi

The current version of mame4all-pi and compilation instructions only works out of the box on Raspbian Jessie, meaning it cannot work on a Pi4.

You may want to try using a Pi0 for testing.  An old Raspbian Jessie image can be found here:
http://downloads.raspberrypi.org/raspbian/images/raspbian-2017-07-05/

I am currently investigating more up to date options.
