# vk3hn_VFO_controller
A PLL VFO/controller for a multi-band SSB/CW transceiver, targeting Arduino Nano and si5351. The VFO main script is SP_VFO_Controller.ino. 

This script has been used for a number of  homebrew transceiver, receiver and transmitter projects at VH3HN, each using the Arduino Nano/si5351 combination.  All these projects share this common script.  By using a common script, a bug fixed or parameter changed is done once, in one place only, and gets deployed the next time a particuar transceiver's Arduino is flashed.  

'SP_' stands for Summit Prowler.  Summit Prowlers are scratch-built QRP tranceivers I have been building for SOTA activations since 2017.  The fourth, fifth and sixth rigs in this series are all multiband SSB/CW single conversion (high-side VFO) superhets.  SP_VFO_Controller.ino is the VFO/controller script for all of these rigs, and other projects as well.    

Code blocks and unique code paths are selected for each project using the C preprocessor directives (#define's). At the top of the script, each project is given a #define'd label, which is used to control other label and variable declarations, code paths and optional code blocks.  Only one project label should be #define'd at any time, all others should be commented.       

For discussion of this project and some information to help you decide if this is the script for you, see this repository's Wiki in this repository.

Please feel free to raise an issue, defect or feature request on Github.  Please let me know if you use this code, with or without change. 

 -- Paul Taylor, VK3HN.  1 Apr 2019.

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
* VK3HN's 'Summit Prowler' homebrew rigs using this script: 
*  https://vk3hn.wordpress.com/2018/02/16/summit-prowler-iv-a-six-band-scratch-built-80m-to-17m-5w-ssb-cw-transceiver-for-portable-and-sota-use/ 
*  https://vk3hn.wordpress.com/2018/10/25/a-scratchbuilt-g6lbq-bitx-walkie-talkie-for-40-and-20m-ssb-cw/ 
*  https://vk3hn.wordpress.com/2019/05/01/summit-prowler-6-a-pocket-sized-ssb-cw-transceiver-for-80-40-30-and-20m/ 
* Custom S-meter font: https://github.com/prt459/Arduino_si5351_VFO_Controller_Keyer/blob/master/s_meter_font.h
