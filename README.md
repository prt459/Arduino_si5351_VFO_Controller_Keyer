# vk3hn_VFO_controller
A VFO/controller for a multi-band SSB/CW transceiver, targeting Arduino Nano and si5351. The VFO main script is SP_VFO_Controller.ino. 

SP_IV VFO:
SP_IV stands for Summit Prowler 4.  Summit Prowlers are my series of scratch-built QRP tranceivers that I have been building for shack and SOTA activations since 2017.  The fourth rig in this series is a multiband SSB/CW single conversion (high-side VFO) superhet.  SP_VFO_Controller.ino is the VFO/controller script for this rig.  This script supports automatic band (bandpass and low pass filter) selection

Feature summary:
* Uses si5351 clock 0 for VFO 
* optionally uses si5351 clock 2 for the BFO; high-side IF is parameterised; 
* automatically selects USB or LSB BFO based on current receiver frequency
* supports any number of VFOs (defaults to 10)
* uses Jason NT7S's si5351 library 
* extensive (and hopefully descriptive) use of #DEFINEs at the top of the file for parameter setting 
* assumes an HD7044 style 16*2 LCD, also 20x4, 8x2; also supports OLED (128x64)
* writes VFO, selected VFO, and other current parameters to EEPROM for power-on restoration 
* includes s-meter code 
* includes code to use I2C to a PCF8574 for band switching (band pass and low pass) filters
* includes interrupt damping (to slow down fast optical encoders)
* PTT line sensing, receiver muting and T/R relay driver, with T/R sequencing  
* ND6T power/SWR monitoring and metering code 
* Memory CW keyer, optional adjustable speed, semi-break-in, any number of (hard coded) pre-canned CW messages
* generates CW carrier via si5351 CLK#1, with 700Hz CW offset above or below receiver frequency
* includes a Tune function (behind a front panel long-push push button) that drops a continuous carrier on the current frequency (no offset) for a parameterised 3 seconds, for testing, tune-up etc.   

Notes:
* VK3HN's Summit Prowler homebrew rigs: 
https://vk3hn.wordpress.com/2016/10/25/summit-prowler-one-a-homebrew-7mhz-ssb-qrp-transceiver-for-sota/ 
https://vk3hn.wordpress.com/2016/12/27/summit-prowler-two-a-scratch-built-30m-cw-transceiver-wilderness-sst-for-sota/
https://vk3hn.wordpress.com/2017/03/27/homebrew-160-meter-amcw-transmitter-receiver/

For discussion of this project and some information to help you decide if this is the script for you, see this repository's Wiki.

Raise an issue, defect or feature request.  

Please let me know if you use this code, with or without change. 

Paul Taylor, VK3HN.  11 Dec 2018.
