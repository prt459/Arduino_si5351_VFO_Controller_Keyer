# vk3hn_VFO_controller
Two VFO/controllers for a multi-band transceiver, targeting Arduino Nano and si5351. The VFOs are Progressive_VFO.ino and SP_IV_VFO_Controller.ino.    

PROGRESSIVE VFO:
Progressive_VFO.ino was written for my build of the Progressive Receiver (Wes W7ZOI and John K5IRK), a classic design that I reproduced, with changes, in early 2017.  This VFO supports explicit band switching across all HF amateur bands, 3 VFOs on each band, mode and frequency persistence over power-cycle.  Because Progressive is a dual conversion superhet, the script uses all 3 si5351 VFOs (VFO, heterodyne oscillator, BFO).  A standalone script (Progressive_VFO_initialiser.ino) must be run once before Progressive_VFO.ino to initialise EEPROM values.  

Notes:
* Reference to project: "A Progressive Communications Receiver" November '81 QST by W7ZOI, Wes Hayward and K5IRK, John Lawson. Subsequently also published for several years as a design in the ARRL Handbook circa '85 - '90 as "A High-Performance Communications Receiver".
* Written for a 20*2 LCD
* Targets Nano
* Traces variable values and other state on Serial at 9600 
* Uses a bank of 5 pushbuttons/switches on one of the analog inputs for control functions 
* rotary encoder on interrupts 
* uses Jason NT7S si5351 library
* Here is a Youtube video of the Progressive VFO in operation: https://www.youtube.com/watch?v=G19XXfZzdbQ 

Status: This script works but I make no promises as to code quality or unimplemented features!

SP_IV VFO:
SP_IV stands for Summit Prowler 4.  Summit Prowlers are my series of scratch-built QRP tranceivers that I build and operate on SOTA activations.  The fourth rig is a multiband SSB/CW single conversion (high-side VFO) superhet.  SP_IV_VFO_Controller.ino is the VFO/controller script for this rig.  It is under active development as of October 2017.  This script abolishes manual band selection, instead, it provides 10 VFOs which may be tuned anywhere in the HF spectrum; the script detects the right filters to switch in based on the current frequency.  

Notes:
* VK3HN's Summit Prowler homebrew rigs: 
https://vk3hn.wordpress.com/2016/10/25/summit-prowler-one-a-homebrew-7mhz-ssb-qrp-transceiver-for-sota/ 
https://vk3hn.wordpress.com/2016/12/27/summit-prowler-two-a-scratch-built-30m-cw-transceiver-wilderness-sst-for-sota/
https://vk3hn.wordpress.com/2017/03/27/homebrew-160-meter-amcw-transmitter-receiver/

* Uses si5351 clocks 0 and 2 for VFO and BFO; VFO is 12MHz (IF) above received signal on each band 
* assumes a 16*2 LCD
* does not use EEPROM yet
* includes s-meter code (untested)
* includes code to use I2C to a PCF8574 for band switching filters
* includes interrupt damping (only acts on every nth interrupt, added to slow down the 360ppt optical encoder I used)
* to be added: PTT Line sensing, T/R relay driver, ND6T power/SWR reading, CW keyer

Good luch experimenting with these scripts, please let me know if you use them!  

