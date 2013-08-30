#Marlin firmware for Solidoodle 2/3
Previous official Solidoodle firmware was based on early version of Marlin and could not be altered to accomodate a Panelolu LCD screen and encoder without many errors. This version has configuration.h and configuration_adv.h altered to reflect all changes found in official Solidoodle firmware.

##Step 1: Download Arduino

*Arduino0022 is no longer needed.* A preprepared version of Arduino 1.0.5 has been setup by ozadr1an. It features the required changes needed for the Sanguinololu electronics to work with versions of Arduino beyond 0022.
Official Arduino download is available, but you will need to move the files from the Add-ons folder of this repository for the firmware to successfully compile and upload:

-http://arduino.cc/hu/Main/Software

###Preprepared PC version:

-http://www.mediafire.com/download/leatx20mqiicipz/Solidoodle-arduino-1.0.5-windows.zip
-https://docs.google.com/file/d/0B7IeeziM0bp9S3FMQUM2Rk02ajQ

Simply unzip into a folder of your choice.

###Preprepared Mac version:

-http://www.mediafire.com/download/9z357ec4rrwpfu7/Solidoodle-arduino-1.0.5-macosx.zip
-https://docs.google.com/file/d/0B7IeeziM0bp9Yjd4SFZfb0QwZ0U

##Step 2: Choose Solidoodle version

If you have a Solidoodle 2, then the firmware is already setup for you by default.
If you have a Solidodle 3, simply change line 15 in configuration.h from:
```C
#define SOLIDOODLE_VERSION 2
```
to:
```C
#define SOLIDOODLE_VERSION 3 
```

##Step 3: Select Microcontroller

For the standard Solidoodle 2/3 model with the 644P microcontroller, upload the firmware as is and select the 'Sanguino W/ ATmega644P' option.
If you have upgraded to a 1284P microcontroller, select the 'Sanguino W/ ATmega1284p 16mhz' option.

##Step 4: Enabling Accessories (Optional)

If you're adding an SDSL SDCARD reader, or Panelolu LCD display and rotary encoder with SDSL, you will need to select the 'Sanguino W/ ATmega1284P' board. Please purchase a 1284P with a bootloader already in place.

For a Panelolu/SD card reader combo, uncomment line 316 in configuration.h from:
```C
//#define ULTIPANEL  //the ultipanel as on thingiverse
```
to:
```C
#define ULTIPANEL  //the ultipanel as on thingiverse
```
Now compile and upload.

-http://www.soliforum.com/topic/127/panelolu-complete-guide/
-http://www.emakershop.com/browse/listing?l=280
-http://www.emakershop.com/browse/listing?l=307
-http://blog.think3dprint3d.com/2012/06/panelolu-in-depth.html

If you're only adding SDSL, uncomment line 313 in configuration.h from:

```C
//#define SDSUPPORT // Enable SD Card Support in Hardware Console
```
to:
```C
#define SDSUPPORT // Enable SD Card Support in Hardware Console
```

Now compile and upload.

-http://reprap.org/wiki/SDSL
-http://www.emakershop.com/browse/listing?l=182
