# SP_VFO_Controller_Keyer
A PLL VFO/controller for a multi-band SSB/CW transceiver, targeting Arduino Nano and si5351. The script is SP_VFO_Controller.ino. An initialiser script must be executed once to initialise EEPROM. 

# Bare bones Arduino/si5351 VFO alternative
Peter VK3TPM stripped down my 2k lines of code to 200, for a bare bones si5251 VFO.  It thats all you want, then maybe try this. 
https://bitbucket.org/peter_marks/dc-vfo/src/master/  

When your project needs the additional control features of a more conventional receiver/tranceiver, come back here:

This script has been used for a number of homebrew transceiver, receiver and transmitter projects at VH3HN, each using the Arduino Nano/si5351 combination.  All these projects share this common script.  By using a common script, a bug fixed or parameter changed is done once, in one place only, and gets deployed the next time a particuar transceiver's Arduino is flashed.  

'SP_' stands for Summit Prowler.  Summit Prowlers are scratch-built QRP tranceivers I have been building for SOTA activations since 2017.  The fourth, fifth and sixth rigs in this series are all multiband SSB/CW single conversion (high-side VFO) superhets.  SP_VFO_Controller.ino is the VFO/controller script for all of these rigs, and other projects as well.    

Code blocks and unique code paths are selected for each project using the C preprocessor directives (#define's). At the top of the script, each project is given a #define'd label, which is used to control other label and variable declarations, code paths and optional code blocks.  Only one project label should be #define'd at any time, all others should be commented.       

For discussion of this project and some information to help you decide if this is the script for you, see this repository's Wiki in this repository.

Please feel free to raise an issue, defect or feature request on Github.  Please let me know if you use this code, with or without change. 

 -- Paul Taylor, VK3HN.  Created 25 July 2019.

Feature summary:
* Uses si5351 clock 0 for VFO 
* optionally uses si5351 clock 2 for the BFO; high-side IF is parameterised; 
* automatically selects USB or LSB BFO based on current receiver frequency
* supports any number of VFOs (defaults to 10)
* extensive (and hopefully descriptive) use of #DEFINEs at the top of the file for parameter setting 
* assumes an HD7044 style 16*2 LCD, also 20x4, 8x2; also supports OLED (128x64)
* writes VFO, selected VFO, and other current parameters to EEPROM for power-on restoration 
* includes s-meter code 
* includes code to use I2C to a PCF8574 for band switching (separate control of band pass and low pass) filters
* includes interrupt damping (to slow down fast optical encoders)
* PTT line sensing, receiver muting and T/R relay driver, with T/R sequencing  
* includes code for the ND6T power/SWR monitoring and metering  
* memory CW keyer, optional adjustable speed, semi-break-in, any number of (hard coded) pre-canned CW messages
* generates CW carrier via si5351 CLK#1, with 700Hz CW offset above or below receiver frequency, depending on the band
* generates 700Hz CW sidetone on a digital output, easily filtered for injection into the receiver's audio stage
* includes a Tune function (behind a front panel long-push push button) that drops a continuous carrier on the current frequency (no offset) for a parameterised 3 seconds, for testing, tune-up etc.   

Notes:
* Uses the following libraries: etherkit/si5351Arduino, Wire (Arduino), EEPROM (Arduino), PCF8574 (Arduino), Rotary (Arduino) 
*  - for LCD: LiquidCrystal (Arduino)
*  - for OLED: greiman/SSD1306Ascii
*  - for Real Time Clock: Arduino RTClib.h
* VK3HN's 'Summit Prowler' homebrew rigs using this script: 
*  https://vk3hn.wordpress.com/2018/02/16/summit-prowler-iv-a-six-band-scratch-built-80m-to-17m-5w-ssb-cw-transceiver-for-portable-and-sota-use/ 
*  https://vk3hn.wordpress.com/2018/10/25/a-scratchbuilt-g6lbq-bitx-walkie-talkie-for-40-and-20m-ssb-cw/ 
*  https://vk3hn.wordpress.com/2019/05/01/summit-prowler-6-a-pocket-sized-ssb-cw-transceiver-for-80-40-30-and-20m/ 
* Custom S-meter font: https://github.com/prt459/Arduino_si5351_VFO_Controller_Keyer/blob/master/s_meter_font.h

Libraries:
The script uses commonly available libraries; here are the ones I used. Substitutes should work, but code changes may be necessary:
* Rotary - Ben Buxton's rotary library  http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html
           This is an old library, others should work
* PCF8574 - Rob Tillaart, 02-feb-2013, V0.1.02 - latest at https://github.com/RobTillaart/Arduino/tree/master/libraries/PCF8574
* SSD1306Ascii - William Greiman (2015) https://github.com/greiman/SSD1306Ascii 
* si5351 - the etherkit library from Jason Millstrom NT7S https://github.com/etherkit/Si5351Arduino  
* Wire - comes with Arduino IDE
* EEPROM - comes with Arduino IDE
* LiquidCrystal - comes with Arduino IDE
* RTClib - used in one of my transceivers, library for a Real Time Clock breakout to get an on-screen clock

# Problems and solutions 
Here a a few problems that have been reported byto me by experimenters who have tried this code: 
* Compile error at **si5351.set_correction()** in **init()** at around line 2630:
   The signature to the **set_correction()** method in Jason's si5351 library changed between releases, the latest version includes a second parameter.
   
   **#ifdef SP_V**
   
      si5351.set_correction(19100);    
   **#endif** 
   
   
   If this call throws a compiler error, add in the second argument, as follows: 
   
   **si5351.set_correction(19100, SI5351_PLL_INPUT_XO);**
       
* Having more than one source file open in the Arduino IDE at once -- some people open all the files in the repo (or perhaps the IDE does this for you
   when you open a repository).  This will fail as the repo contains multiple compilable scripts.  You should only ever compile two of the files,
   the SP_Initialiser script or the main SP_VFO_Controller script.  **Only have one of these two files open in the Arduino IDE at a time.**  
 
