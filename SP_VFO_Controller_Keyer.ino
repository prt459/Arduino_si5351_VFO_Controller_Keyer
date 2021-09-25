/*  
Arduino Nano script for homebrew multiband SSB/CW transceivers. 
Written by Paul Taylor, VK3HN (https://vk3hn.wordpress.com/) standing on the shoulders of:
  - Przemek Sadowski, SQ9NJE (basic controller script)
  - Jason Mildrum NT7S (si5351 library)
  - too many others to mention (ideas, code snippets). 
  
Targets Ashar Farhan VU2ESE's Arduino Nano/si5351 module (Raduino). 

V1.0, 20 Jun 2017 - first version for homebrew Arduino/si5351 VFO
V1.4, 27 Nov 2018 - revised version with CW keyer for homebrew SP_IV transceiver
V1.5, 3 Dec 2018  - Refactored #define's and code blocks for project labels
V1.6  6 Jun 2019  - Added MCP9808 temp sensor (Adafruit)
V1.7  2 Mar 2020  - Added code for AM transmitters that use one clock only as transmitter VFO at signal frequency 
V1.8  2 Mar 2020  - Code for VFO / BFO reversing.
V1.9  25 Sep 2021 - general update.

Labels that need to be #define'd for your target radio/rig/project:
  Rotary encoder         {ENCODER_OPTICAL_360, ENCODER_MECHANICAL} 
  Display technology     {DISPLAY_LCD, DISPLAY_OLED}
  Display type           {LCD_20X4, LCD_16X2, LCD_8X2, OLED_128X64}
  Project name           {SP_IV, SP_11, SP_V, SS_EI9GQ, SP_6, SP_7, etc }
  BFO enable             {BFO_ENABLED}
  VSWR meter             {VSWR_METER}
  CW keyer               {CW_KEYER} 
  Tune mS                {TUNE_MS}            Typical value: 3000mS
  Held button mS         {BUTTON_HELD_MS}     Typical value: 700mS
  Diagnostics on display {DIAGNOSTIC_DISPLAY} 
  VFO/BFO swap between transmit and receive {VFO_BFO_SWAP}
*/

// common libraries
#include <Rotary.h>
#include <si5351.h>     // Etherkit si3531  V2.0.1   https://github.com/etherkit/Si5351Arduino 
#include <pcf8574.h>    // pcf8574 library by Rob Tillaart 02-febr-201; many others should work 
#include <Wire.h>
#include <EEPROM.h>

// ------------------------------------------------------------------------------------------------------------------
// #define one (and only one) label to pull in the right code for the specific project

// [ Summit Prowlers ]
//#define SP_IV         // for code specific to Summit Prowler IV (6 band MST) (16x2 LCD, optical encoder)
// #define SP_V         // for code specific to Summit Prowler V rig (2 band G6LBQ BiTx) (8x2 LCD, mechanical encoder)
// #define SP_6         // for code specific to DK7IH Compact Multibander (128x64 OLED with SSD1308, mechanical encoder)
// #define SP_7         // for code specific to VK3WAC SSDRA four-bander (LCD, optical encoder)
// #define SP_8         // 28MHz SSB transceiver 
// #define SP_9         // 5-band compact SSB/CW transceiver (5/2020) 
// #define SP_X         // 4-band hand held channelised CW 'appliance' transceiver (1/2021) 
// #define SP_11          // 3-band QRO SOTA CW rig (4/2021)
#define UNIVERSAL_VFO_CONTROLLER   // Universal VFO Controller (22/9/2021)

// [ Shack projects ]
// #define SS_EI9GQ     // for code specific to Shack Sloth base station transceiver by Eamon EI9GQ) (20x4 LCD, mechanical encoder)

// [ AM transmitters ]
// #define SS_FAT5_160AM_TX // for code specific to the GW8LLJ (FAT5) 160 AM/CW transmitter/receiver (16x2 LCD, optical encoder)
// #define SS_VK3SJ_160AM_TX // for code specific to the Laurie VK3SJ's PWM AM transmitter (16x2 LCD, optical encoder)
// #define SS_VK3SJ_160AM_PORTABLE_TX // for code specific to the Laurie VK3SJ's portable PWM AM transmitter (16x2 LCD, optical encoder)
// #define SS_AM_TX_TEST_VFO // for code specific to a bench VFO for testing AM transmitters (11/2019)
// #define SS_VK3SJ_40AM_TX // for code specific to the VK3SJ 40m AM transmitter (16x2 LCD, optical encoder)
// ------------------------------------------------------------------------------------------------------------------



// common #define's that precede other declarations
#define LCD_RS    8  // Register Select is LCD pin 4 
#define LCD_E     9  // Enable/clock LCD pin 6
#define LCD_D4   10  // LCD D4 
#define LCD_D5   11  // LCD D5 
#define LCD_D6   12  // LCD D6 
#define LCD_D7   13  // LCD D7  

#define BUTTON_HELD_MS 700  // button down period in mS required to activate 2nd pushbutton function
#define TUNE_MS       5000  // tune (key down) period (mS)
//#define TUNE_MS       30000  // tune (key down) period (mS) 30 for testing
#define LPF_DROPOUT_DELAY 8000  // period of inactivity before the LPF drops out to save battery (mS)
#define FAN_DROPOUT_DELAY  5000  // period of inactivity before the fan drops out to save battery (mS)
#define VFO_DRIVE_THRESHOLD 18000000  // threshold freq above which a higher si5351 clock output level on the VFO clock is used
#define CO_DRIVE_THRESHOLD  18000000  // threshold freq above which a higher si5351 clock output level on the carrier oscillator (CW) clock is used
//#define CO_DRIVE_THRESHOLD  18000  // threshold freq above which a higher si5351 clock output level on the carrier oscillator (CW) clock is used

#define RX_MUTE_DELAY_MS   10 // mS delay after muting receiver and before unmuting receiver 

#define BREAK_IN_DELAY   800  // break-in hang time (mS) (may be overridden in the project sections) 

// ------------------------------------------------------------------------------------------------------------------
// Arduino Nano digital pin assignments (aligns with Raduino)

//                         0    Serial
//                         1    Serial
#define ENCODER_B          2  // Encoder pin B
#define ENCODER_A          3  // Encoder pin A
#define PTT_SENSE          4  // sense the PTT button being pressed (low == transmit)
#define RX_MUTE_LINE       5  // receiver mute 
#define   RX_MUTE_OFF_VALUE  0   // default value for an un-muted receiver (low), can be inverted in a project block
#define   RX_MUTE_ON_VALUE   1   // default value for a muted receiver (high), can be inverted in a project block
//                         6    keyer sidetone, declared below 
#define TRANSMIT_LINE      7  // controls the T/R relay (high == transmit)

// Arduino Nano analogue pins
#define SWITCH_BANK       A0 // front panel push buttons
//#ifdef  CW_KEYER
#define PIN_PADDLE        A1 // paddle on analog pin 1
#define PIN_PUSHBTTN_REAR A2 // keyer memory pushbuttons on analog pin 2
#define PIN_KEYER_SPEED   A3 // speed potentiometer wiper
//#endif
#define PIN_S_METER       A3 // s-meter TBA (alternately, use A7)
//                        A4    SDA
//                        A5    SCL
#define PIN_PWR_METER     A6 // analogue pin for relative RF sensing circuit
int pwr_val = 0;             // stores the last reading of the RF power sensing input (on pin PIN_PWR_METER)


#ifdef VSWR_METER
#define PIN_SWR_FWD       A6 // analogue pin for SWR bridge forward
#define PIN_SWR_REV       A7 // analogue pin for SWR bridge reverse
#endif




// specific declarations and #define's for each project -------------------------------------------------

// 'Summit Prowlers'  - - -

#ifdef SP_IV                       // 6-band OzQRP-based MST SSB/CW transceiver
#define ENCODER_OPTICAL_360 
#define BFO_ENABLED
#define BFO_TUNE_LO   11990000ULL  // lowest BFO frequency
#define BFO_TUNE_HI   12005000ULL  // highest BFO frequency
volatile uint32_t USB = 11998500ULL;  // the reference BFO freq for LSB, may be tuned, put in EEPROM once
volatile uint32_t LSB = 11995500ULL;  // the reference BFO freq for USB, may be tuned, put in EEPROM once
#define DISPLAY_LCD
#define LCD_16X2
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 
#define NBR_VFOS    10 // number of selectable VFOs 
// #define VSWR_METER // uncomment to compile VSWR meter code
#define CW_KEYER      // include the CW keyer code
#endif


#ifdef SP_V                        // Two-band G6LBQ 'walkie talkie' SSB/CW transceiver
#define ENCODER_MECHANICAL 
#define BFO_ENABLED
#define BFO_TUNE_LO    8990000ULL  // lowest BFO frequency
#define BFO_TUNE_HI    9010000ULL  // highest BFO frequency
volatile uint32_t USB =  9002200ULL; // USB on 20m
// was volatile uint32_t LSB =  8999800ULL; // LSB on 40m (too much bass)
// volatile uint32_t LSB =  8999450ULL; // LSB on 40m (upper limit for treble)
volatile uint32_t LSB =  8999300ULL; // LSB on 40m (reset June 2020)
#define DISPLAY_LCD
#define LCD_8X2
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 
#define NBR_VFOS    4 // number of selectable VFOs 
#define CW_KEYER      // include the CW keyer code
#endif


#ifdef SP_6                        // Compact mono-band DK7IH SSB/CW transceiver
#define ENCODER_MECHANICAL 
#define BFO_ENABLED
#define BFO_TUNE_LO    11990000ULL  // lowest BFO frequency
#define BFO_TUNE_HI    12005000ULL  // highest BFO frequency
volatile uint32_t USB = 11998500ULL;  // the reference BFO freq for LSB, may be tuned, put in EEPROM once
volatile uint32_t LSB = 11996000ULL;  // the reference BFO freq for USB, may be tuned, put in EEPROM once
// original --  11,995,500ULL; 
#define DISPLAY_OLED
#define OLED_128X64
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_OLED_ADDRESS 0x3C // correct!
//#define I2C_OLED_ADDRESS 0x3D  // test for dead display
#define RST_PIN -1     // Define proper RST_PIN if required
  SSD1306AsciiAvrI2c oled;
#define NBR_VFOS    10 // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
// #define DIAGNOSTIC_DISPLAY  // allow use of the OLED for tracing simple values (eg button values)
#define TX_SSB_MUTE_LINE 9    // D9 is used to mute the mic amp to silence a T/R squeal in SP_6 
                              // normally high, this line goes low TX_SSB_MUTE_DELAY mS after PTT when transmitting SSB
#define TX_SSB_MUTE_DELAY 350 // delay (mS) before the mic amp is unmuted after SSB PTT 
#define CO_DC_SUPPLY  8          // this pin controls a high side DC switch to enable the carrier oscillator when keyed
#endif


#ifdef SP_7                        // VK3WAC SSDRA Mono-band SSB/CW transceiver
#define ENCODER_OPTICAL_360 
#define BFO_ENABLED
#define BFO_TUNE_LO     5100000ULL  // lowest BFO frequency
#define BFO_TUNE_HI     5130000ULL  // highest BFO frequency
volatile uint32_t USB =  5120000ULL;  // the reference BFO freq for USB, may be tuned, put in EEPROM once
//volatile uint32_t LSB =  5121500ULL;  // the reference BFO freq for LSB, may be tuned, put in EEPROM once 
volatile uint32_t LSB =  5116000ULL;  // the reference BFO freq for LSB, may be tuned, put in EEPROM once 
#define DISPLAY_LCD
#define LCD_16X2
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 
#define NBR_VFOS    4  // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
#define CO_DC_SUPPLY A7   // this pin controls a high side DC switch to enable the carrier oscillator when keyed
#endif


#ifdef SP_8                      // Compact 4-band CW-only transceiver
#define ENCODER_MECHANICAL
#define BFO_ENABLED
#define BFO_TUNE_LO     8998000ULL  // lowest BFO frequency
#define BFO_TUNE_HI     9002000ULL  // highest BFO frequency
volatile uint32_t USB = 8989000ULL;  // selected sideband at 28MHz 
volatile uint32_t LSB = 8986100ULL;

#define DISPLAY_OLED
#define OLED_128X64
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_OLED_ADDRESS 0x3C // correct!

#define RST_PIN -1     // Define proper RST_PIN if required
  SSD1306AsciiAvrI2c oled;
#define NBR_VFOS    5  // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
#define CO_DC_SUPPLY   11 // this pin controls a high side DC switch to enable the carrier oscillator when keyed
#define ENCODER_BUTTON 12 // the digital input used to read the mech encoder's pushbutton 
#define BAND_SENSE_2M  10 // this pin senses when the 2m transverter is selected and powered on 
bool  sp8_band_2m;    // true if band selected is 2m 

#define SP_8_DIAL_OFFSET -1700 // offset to be applied to the displayed frequency to correct for transverter crystal frequency error 
#endif


#ifdef SP_9                      // Compact 5-band SSB/CW transceiver
#define ENCODER_MECHANICAL
#define ENCODER_BUTTON  12          // the digital input used to read the mech encoder's pushbutton 
#define BFO_ENABLED
#define BFO_TUNE_LO     8998500ULL  // lowest BFO frequency
#define BFO_TUNE_HI     9001500ULL  // highest BFO frequency
volatile uint32_t USB = 9001500ULL;  // selected sideband  
volatile uint32_t LSB = 8998500ULL;

#define DISPLAY_OLED
#define OLED_128X64
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_OLED_ADDRESS 0x3C // correct!

#define RST_PIN -1     // Define proper RST_PIN if required
  SSD1306AsciiAvrI2c oled;
#define NBR_VFOS    5  // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
#define CO_DC_SUPPLY 11   // this  pin controls a high side DC switch to enable the carrier oscillator when keyed
// #define DIAGNOSTIC_DISPLAY // FOR TESTING THOSE PESKY PUSHBUTTONS!!
#define RX_MUTE_DELAY_MS   20 // override the mS delay after muting receiver and before unmuting receiver 
#endif



#ifdef SP_X                      // Compact 4-band channelised CW transceiver
#define ENCODER_MECHANICAL       // not used
#define ENCODER_BUTTON  12       // digital input used to read the mech encoder's pushbutton (not used)
#define BFO_ENABLED
#define BFO_TUNE_LO     3998500ULL  // lowest BFO frequency
#define BFO_TUNE_HI     4001200ULL  // highest BFO frequency
volatile uint32_t USB = 4000700ULL;  
volatile uint32_t LSB = 3998900ULL;

// default channel frequencies
#define SP_X_CH0_FREQ_DEFAULT 7025700   //  7,025  CW calling channel in VK
#define SP_X_CH1_FREQ_DEFAULT 7032700   //  7,032  CW SOTA channel - primary
#define SP_X_CH2_FREQ_DEFAULT 10110930  // 10,110  CW SOTA channel - primary TBC
#define SP_X_CH3_FREQ_DEFAULT 14059300  // 14,060  CW SOTA channel - secondary
#define SP_X_CH4_FREQ_DEFAULT 14061300  // 14,062  CW SOTA channel - primary
#define SP_X_CH5_FREQ_DEFAULT 18094300  // 18,095  CW SOTA channel - primary TBC

// display declarations are not used, but left in to avoid compiler errors (!)
#define DISPLAY_OLED
#define OLED_128X64
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_OLED_ADDRESS 0x3C // correct!

#define RST_PIN -1     // Define proper RST_PIN if required
  SSD1306AsciiAvrI2c oled;

#define NBR_VFOS    6  // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
// #define CO_DC_SUPPLY 11   // this  pin controls a high side DC switch to enable the carrier oscillator when keyed (not used)
// #define DIAGNOSTIC_DISPLAY // FOR TESTING THOSE PESKY PUSHBUTTONS!!

#define   RX_MUTE_OFF_VALUE  1   // value for an un-muted receiver (high)
#define   RX_MUTE_ON_VALUE   0   // value for a muted receiver (low)

#define RX_MUTE_DELAY_MS   10 // override the mS delay after muting receiver and before unmuting receiver 
#define BREAK_IN_DELAY  500  // override the default break-in hang time (mS)
#define SP_X_MUTE_SPKR  2  // D2 is used as a digital line to control a mute relay in series with the speaker
bool SP_X_init = true; 
unsigned int SP_X_counter = 0; 
#endif


#ifdef SP_11                 // 3-band QRO SOTA CW rig 
#define ENCODER_OPTICAL_360 
#define BFO_TUNE_LO   11990000ULL  // lowest BFO frequency
#define BFO_TUNE_HI   12005000ULL  // highest BFO frequency
volatile uint32_t USB = 3999700ULL;  
volatile uint32_t LSB = 3999700ULL;  
#define DISPLAY_LCD
#define LCD_16X2
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 
#define NBR_VFOS    3  // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
#endif


#ifdef UNIVERSAL_VFO_CONTROLLER   // Universal VFO Controller  23/9/2021 
#define ENCODER_MECHANICAL 
#define BFO_ENABLED
#define BFO_TUNE_LO      5100000ULL  // lowest BFO frequency
#define BFO_TUNE_HI      5130000ULL  // highest BFO frequency

volatile uint32_t USB =  5120000ULL;  // the reference BFO freq for USB, may be tuned, put in EEPROM once
volatile uint32_t LSB =  5116000ULL;  // the reference BFO freq for LSB, may be tuned, put in EEPROM once 
#define DISPLAY_LCD
#define LCD_16X2
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 
#define NBR_VFOS    4  // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
// #define CO_DC_SUPPLY A7   // this pin controls a high side DC switch to enable the carrier oscillator when keyed
#endif
// ------------------------------------------------------------------------------------------------------------------


#ifdef SS_EI9GQ                    // 8-band EI9GQ base SSB/CW transceiver
#define ENCODER_OPTICAL_360 
#define BFO_TUNE_LO    8990000ULL  // lowest BFO frequency
#define BFO_TUNE_HI    9010000ULL  // highest BFO frequency
volatile uint32_t USB =  9002200ULL; 
volatile uint32_t LSB =  8998500ULL; 
#define DISPLAY_LCD
#define LCD_20X4
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 
#include "RTClib.h"
  RTC_PCF8523 rtc;    // declare a RTC (on I2C address 0x68)
#define NBR_VFOS   10 // number of selectable VFOs 
// #define VSWR_METER // uncomment to compile VSWR meter code
#define CW_KEYER      // include the CW keyer code
#endif

// ------------------------------------------------------------------------------------------------------------------


#ifdef SS_FAT5_160AM_TX          // GW8LJJ FAT5 160m 100W Class E AM transmitter
#define ENCODER_OPTICAL_360 
#define BFO_TUNE_LO   11990000ULL  // lowest BFO frequency (not currently used)
#define BFO_TUNE_HI   12005000ULL  // highest BFO frequency (not currently used)
volatile uint32_t USB =  0ULL;     // BFO not used in this Tx
volatile uint32_t LSB =  0ULL;
#define DISPLAY_LCD
#define LCD_16X2
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 
#define NBR_VFOS    4 // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
#define PWM_ENABLE_LINE 6 // normally low, take high to enable PWM (used in T/R sequencing) 
#endif


#ifdef SS_VK3SJ_160AM_TX          // VK3SJ Laurie's 160m Class D AM PWM transmitter
#define ENCODER_OPTICAL_360 
#define BFO_TUNE_LO   11990000ULL  // lowest BFO frequency (not used)
#define BFO_TUNE_HI   12005000ULL  // highest BFO frequency (not used)
volatile uint32_t USB =  0ULL;     // BFO not used 
volatile uint32_t LSB =  0ULL;
#define DISPLAY_LCD
#define LCD_16X2
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 
#define NBR_VFOS    3 // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
#define PWM_ENABLE_LINE 6 // normally low, take high to enable PWM (used in T/R sequencing) 
#include <Adafruit_MCP9808.h>
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();  // I2C address defaults to ...
byte loop_cntr=0;
#endif



#ifdef SS_VK3SJ_160AM_PORTABLE_TX       // VK3SJ 160m Class D AM PWM PORTABLE transmitter / receiver 
#define ENCODER_MECHANICAL 
#define BFO_TUNE_LO     455000ULL  // lowest BFO frequency (not currently used)
#define BFO_TUNE_HI     455000ULL  // highest BFO frequency (not currently used)
volatile uint32_t USB = 456000ULL;  // the reference BFO freq for LSB, may be tuned
volatile uint32_t LSB = 454000ULL;  // the reference BFO freq for USB, may be tuned

#define DISPLAY_LCD
#define LCD_16X2
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 
#define NBR_VFOS    2  // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
#define PWM_ENABLE_LINE 6 // normally low, take high to enable PWM (used in T/R sequencing) 

#define PIN_VOLTMETER     A6 // analogue pin for HT voltage sensor (0..5v)
unsigned int HT_v = 0; 
#define PIN_AMMETER       A7 // analogue pin for HT current sensor (via INA169), amps
unsigned int HT_a = 0; 
float HT_a_f = 0.0; 
#endif


#ifdef SS_VK3SJ_40AM_TX          // VK3SJ Laurie's 40m Class D AM PWM transmitter
#define ENCODER_OPTICAL_360 
#define BFO_TUNE_LO   11990000ULL  // lowest BFO frequency (not used)
#define BFO_TUNE_HI   12005000ULL  // highest BFO frequency (not used)
volatile uint32_t USB =  0ULL;     // BFO not used 
volatile uint32_t LSB =  0ULL;
#define DISPLAY_LCD
#define LCD_16X2
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 
#define NBR_VFOS    2 // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
#define PWM_ENABLE_LINE 6 // normally low, take high to enable PWM (used in T/R sequencing) 
byte loop_cntr=0;
#endif

#ifdef SS_AM_TX_TEST_VFO          // Test/bench VFO for Class D/E AM transmitters
#define ENCODER_MECHANICAL 
#define BFO_TUNE_LO     455000ULL  // lowest BFO frequency (not currently used)
#define BFO_TUNE_HI     455000ULL  // highest BFO frequency (not currently used)
volatile uint32_t USB = 455000ULL;  // the reference BFO freq for LSB, may be tuned
volatile uint32_t LSB = 455000ULL;  // the reference BFO freq for USB, may be tuned
#define DISPLAY_LCD
#define LCD_16X2
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 
#define NBR_VFOS    8  // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
#endif




#ifdef DIAGNOSTIC_DISPLAY
// flags and variables to trace values to the display, for temporary diagnostics
bool diagnostic_flag = false;  // trace on the display as a diagnostic
int  diagnostic_int = 0;       // trace an integer value
// String diagnostic_string("");  // trace a String
#endif






// frequency ranges for automatic band pass and low pass filter switching

#define FILTER_630_LB  200000ULL  // Filter set lower bound (MHz)
#define FILTER_630_UB  500000ULL  // Filter set upper bound (MHz)

#define FILTER_MW_LB   500100ULL  // Filter set lower bound (MHz)
#define FILTER_MW_UB  1750000ULL  // Filter set upper bound (MHz)

#define FILTER_160_LB 1750100ULL  // Filter set lower bound (MHz)
#define FILTER_160_UB 2500000ULL  // Filter set upper bound (MHz)

#define FILTER_80_LB  2501000ULL  // Filter set lower bound (MHz)
#define FILTER_80_UB  4000000ULL  // Filter set upper bound (MHz)

#define FILTER_60_LB  4001000ULL  // Filter set lower bound (MHz)
#define FILTER_60_UB  6500000ULL  // Filter set upper bound (MHz)

#define FILTER_40_LB  6501000ULL  // Filter set lower bound (MHz)
#define FILTER_40_UB  8000000ULL  // Filter set upper bound (MHz)

#define FILTER_30_LB  8001000ULL  // Filter set lower bound (MHz)
#define FILTER_30_UB 12000000ULL  // Filter set upper bound (MHz)

#define FILTER_20_LB 12001000ULL  // Filter set lower bound (MHz)
#define FILTER_20_UB 16000000ULL  // Filter set upper bound (MHz)

#define FILTER_17_LB 16001000ULL  // Filter set lower bound (MHz)
#define FILTER_17_UB 18500000ULL  // Filter set upper bound (MHz)

#define FILTER_15_LB 19000000ULL  // Filter set lower bound (MHz)
#define FILTER_15_UB 21500000ULL  // Filter set upper bound (MHz)

// ------------------------------------------------------------------------------------------------------------------

// i2c devices and addresses:
// si5351  x60
// BPF selector PCF8574 x20 .. x38
// LPF selector PCF8574 x40

#define I2C_BPF_DEMUX   0x20         // default I2C address of the BPF PCF8574 
#define I2C_LPF_DEMUX   0x24         // default I2C address of the LPF PCF8574 

#ifdef SS_EI9GQ
#define I2C_BPF_DEMUX   0x38         // I2C address of the BPF PCF8574 in this rig
#endif

#ifdef SP_6
#define I2C_BPF_DEMUX   0x20         // I2C address of the BPF PCF8574 in this rig
#define I2C_LPF_DEMUX   0x3E         // I2C address of the LPF PCF8574 in this rig
#endif

#ifdef SP_7
#define I2C_BPF_DEMUX   0x20         // I2C address of the BPF PCF8574 in this rig
#define I2C_LPF_DEMUX   0x38         // I2C address of the LPF PCF8574 in this rig
#endif

#ifdef SP_8
#define I2C_BPF_DEMUX   0x20         // I2C address of the BPF PCF8574 in this rig
#define I2C_LPF_DEMUX   0x38         // (there is no LPF PCF8574 in this rig)
#endif

#ifdef SP_9
#define I2C_BPF_DEMUX   0x20         // I2C address of the BPF PCF8574 in this rig
#endif

#ifdef SP_11
#define I2C_BPF_DEMUX   0x20         // I2C address of the BPF PCF8574 in this rig
#endif

PCF8574 PCF_BPF(I2C_BPF_DEMUX);       
PCF8574 PCF_LPF(I2C_LPF_DEMUX);  

// ------------------------------------------------------------------------------------------------------------------

bool message_playing = false;      // true when the keyer is playing a CW message 
#define CW_TONE_HZ       700  // CW tone frequency (Hz)
unsigned int dot_dash_counter = 0;  // total nbr CW chars sent since power-up
unsigned int dot_dash_sent = 0;     // nbr CW chars sent this transmit period, used for timing a refresh to the (interleaved) RF relative power meter
#define PIN_TONE_OUT       6  // digital pin with keyed audio tone on it
bool key_down = false; 
unsigned long char_sent_ms, curr_ms;
bool space_inserted;
#define KEYER_MSG_SPEED 50



#ifdef CW_KEYER
// start of CW Keyer block -------------------------------------------------------------

#define PADDLE_R           1      // value representing analog value for paddle left (dot)
#define PADDLE_L           2      // value representing analog value for paddle right (dash)

#define CW_DASH_LEN        5  // length of dash (in dots)
#define SERIAL_LINE_WIDTH 80  // number of morse chars on Serial after which we newline 

// set the CW keyer speed and canned messages for each project
// for the keyer speed, lower is faster, 60 is 10 w.p.m.

#ifdef SS_EI9GQ
String morse_msg[] = {"CQ CQ VK3HN VK3HN K", "DE VK3HN ", "VK3HN ", "73 TU . ." };
#endif

#ifdef SP_IV
byte  dot_length_ms = 55; 
#define KEYER_MSG_SPEED 42 // make the keyer play out canned messages faster 
String morse_msg[] = {"CQ SOTA VK3HN/P K", "VK3HN ", "73 TU . ."};
#endif

#ifdef SP_V
byte  dot_length_ms = 60;  // make the CW slower, for pushbutton CW
#define KEYER_MSG_SPEED 45 // make the keyer play out canned messages faster 
String morse_msg[] = {"CQ SOTA VK3HN/P K", "CQ VK3HN/P K", "VK3HN/P K" };
//String morse_msg[] = {"CQ SOTA VK3HN/P K", "VK3HN ", " RST 599 5NN BK" };
#endif

#ifdef SP_6
byte  dot_length_ms = 56;  // 
#define KEYER_MSG_SPEED 45 // make the keyer play out canned messages faster 
String morse_msg[] = {"CQ SOTA VK3HN/P K", "DE VK3HN" };
//String morse_msg[] = {"1", "2" };  // for testing
#endif

#ifdef SP_7
byte  dot_length_ms = 56;  // 
#define KEYER_MSG_SPEED 45 // make the keyer play out canned messages faster 
String morse_msg[] = {"CQ SOTA VK3HN/P K", "CQ CQ VK3HN K" };
//String morse_msg[] = {"1", "2" };  // for testing
#endif

#ifdef SP_8
byte  dot_length_ms = 60;  
#define KEYER_MSG_SPEED 45 
String morse_msg[] = {"NOT USED"};
#endif

#ifdef SP_9
byte  dot_length_ms = 65;  // 
#define KEYER_MSG_SPEED 45 // make the keyer play out canned messages faster 
String morse_msg[] = {"CQ SOTA VK3HN/P K"};
//String morse_msg[] = {"1" };  // for testing
#endif

#ifdef SP_X
byte  dot_length_ms = 57;  // 
#define KEYER_MSG_SPEED 50 // make the keyer play out canned messages faster 
// String morse_msg[] = {"V V V DE VK3HN ."};
String morse_msg[] = {"CQ SOTA VK3HN/P K"};
#define CW_DASH_LEN        4  // re-define the length of dash (in dots), this is personal preference!
#endif

#ifdef SP_11
byte  dot_length_ms = 60;  // 
#define KEYER_MSG_SPEED 45 // make the keyer play out canned messages faster 
//String morse_msg[] = {"CQ SOTA VK3HN/P K", "CQ CQ VK3HN K", "CQ TEST VK3HN K" };
String morse_msg[] = {"1", "2", "3" };  // for testing
#endif


#ifdef SS_EI9GQ
byte  dot_length_ms = 56;  
#define KEYER_MSG_SPEED 45 // make the keyer play out canned messages faster 
#endif

#if defined(SS_FAT5_160AM_TX) || defined(SS_VK3SJ_160AM_TX)  || defined(SS_AM_TX_TEST_VFO) || defined(SS_VK3SJ_40AM_TX) || defined(SS_VK3SJ_160AM_PORTABLE_TX)
byte  dot_length_ms = 56;  
String morse_msg[] = {"CQ CQ DE VK3HN VK3HN K", "DE VK3HN" };
#endif

#ifdef UNIVERSAL_VFO_CONTROLLER
byte  dot_length_ms = 56;  // 
#define KEYER_MSG_SPEED 45 // make the keyer play out canned messages faster 
String morse_msg[] = {"MSG 1", "MSG 2", "MSG 3" };
#endif


                         
// morse reference table
struct morse_char_t {
  char ch[7]; 
};

morse_char_t MorseCode[] = {
  {'A', '.', '-',  0,   0,   0,   0},
  {'B', '-', '.', '.', '.',  0,   0},
  {'C', '-', '.', '-', '.',  0,   0},
  {'D', '-', '.', '.',  0,   0,   0},
  {'E', '.',  0,   0,   0,   0,   0},
  {'F', '.', '.', '-', '.',  0,   0},
  {'G', '-', '-', '.',  0,   0,   0},
  {'H', '.', '.', '.', '.',  0,   0},
  {'I', '.', '.',  0,   0,   0,   0},
  {'J', '.', '-', '-', '-',  0,   0},
  {'K', '-', '.', '-',  0,   0,   0},
  {'L', '.', '-', '.', '.',  0,   0},
  {'M', '-', '-',  0,   0,   0,   0},
  {'N', '-', '.',  0,   0,   0,   0},
  {'O', '-', '-', '-',  0,   0,   0},
  {'P', '.', '-', '-', '.',  0,   0},
  {'Q', '-', '-', '.', '-',  0,   0},
  {'R', '.', '-', '.',  0,   0,   0},
  {'S', '.', '.', '.',  0,   0,   0},
  {'T', '-',  0,   0,   0,   0,   0},
  {'U', '.', '.', '-',  0,   0,   0},
  {'V', '.', '.', '.', '-',  0,   0},
  {'W', '.', '-', '-',  0,   0,   0},
  {'X', '-', '.', '.', '-',  0,   0},
  {'Y', '-', '.', '-', '-',  0,   0},
  {'Z', '-', '-', '.', '.',  0,   0},
  {'0', '-', '-', '-', '-', '-',  0},
  {'1', '.', '-', '-', '-', '-',  0},
  {'2', '.', '.', '-', '-', '-',  0},
  {'3', '.', '.', '.', '-', '-',  0},
  {'4', '.', '.', '.', '.', '-',  0},
  {'5', '.', '.', '.', '.', '.',  0},
  {'6', '-', '.', '.', '.', '.',  0},
  {'7', '-', '-', '.', '.', '.',  0},
  {'8', '-', '-', '-', '.', '.',  0},
  {'9', '-', '-', '-', '-', '.',  0},
  {'/', '-', '.', '.', '-', '.',  0},
  {'?', '.', '.', '-', '-', '.', '.'},
  {'.', '.', '-', '.', '-', '.', '-'},
  {',', '-', '-', '.', '.', '-', '-'}
};

byte   curr_msg_nbr;   // index into morse_msg[] array
byte   cw_msg_index;   // index into morse_msg[cw_msg_index] array
#endif
// end CW Keyer block -------------------------------------------------------------


byte curr_line = 0;    // the currently selected filter control line

// struct for 'VFO parameter set' records -- the parameters that will change with each VFO
typedef struct {
  boolean  active;
  uint32_t vfo;
  uint32_t radix;
} VFOset_type;

VFOset_type VFOSet[NBR_VFOS]; // array of band parameter sets
byte v;                       // index into VFOSet array (representing the current VFO)
byte v_prev;


Si5351 si5351;                // I2C address defaults to x60 in the NT7S lib
Rotary r = Rotary(ENCODER_A, ENCODER_B);

// USB/LSB initialisation
#define SIDEBAND_THRESHOLD  10000000ULL  // threshold VFO freq for auto sideband selection: above use USB, below use LSB
volatile uint32_t bfo = LSB; // the actual BFO freq for si5351 CLK2, arbitrary set to LSB, reset in main loop  

// ------------------------------------------------------------------------------------------------------------------

// variables for transmit-receive control 
bool mode_tx = false; 
bool mode_cw = false; 
bool mode_tune = false; 

// LPF control variables
byte LPF_line=0;           // current LPF line (0..5), set in set_filters()
bool LPF_engaged = false;  // to allow the LPF to be engaged just in time
unsigned long  last_T_R_ms = 0;   // time of last Transmit to receive change

// button variables used in main loop for sensing the multiplexed buttons
byte button_nbr; 
byte old_button_nbr = 0; 

// S meter reading
String S_meter; 
int s_meter_reading=100; 
int s_meter_update_cnt = 0;
int last_s_meter_val = 0;

// ------------------------------------------------------------------------------------------------------------------
//-- VSWR meter code begins ---// Note: left without #defines purposely
  int fwd_max=0, rev_max=0; 
//-- VSWR meter code ends ---------------------------------------------

bool func_button_pressed = false; // if true, the next button pressed is interpreted as a Function 
bool BFO_tune_flg = false;       // BFO Tune feature
byte dial_tick=0;

#ifdef SP_7
byte dial_speed=6;  // slightly different value for this particular rig and encoder 
#else 
byte dial_speed=8;  // control rotary encoder /dial speed
#endif

// ------------------------------------------------------------------------------------------------------------------
// variables for controlling EEPROM writes
unsigned long last_freq_change_ms;
bool eeprom_written_since_last_freq_change; 
bool changed_f = false;





/**************************************/
/* Interrupt service routine for encoder frequency change */
/**************************************/
ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result == DIR_CW)
    set_frequency(1);
  else if (result == DIR_CCW)
    set_frequency(-1);
}

/**************************************/
/* Change frequency; dir=1 Increment; dir=-1 Decrement
/**************************************/
void set_frequency(short dir)
{
#ifdef ENCODER_OPTICAL_360
  if(++dial_tick%dial_speed != 0) return;  // damp the (very fast) 360PPM optical encoder
#endif

  if(mode_tx) return;  // dial locks in transmit

  if (dir == 1)
  {
      if(!BFO_tune_flg)
         VFOSet[v].vfo += VFOSet[v].radix;
      else
         if(bfo < BFO_TUNE_HI) bfo += 100;
  }
  else 
  {
     if (dir == -1)
       if(!BFO_tune_flg)
          VFOSet[v].vfo -= VFOSet[v].radix; 
       else
          if(bfo > BFO_TUNE_LO) bfo -= 100; 
  };

  if(BFO_tune_flg)
  {
    if(VFOSet[v].vfo >= SIDEBAND_THRESHOLD) 
    {
      USB = bfo;
    }
    else 
    {
      LSB = bfo;  
    }
  }  
  changed_f = 1;
};


int read_analogue_pin(byte p)
{
// Take an averaged reading of analogue pin 'p'  
  int i, val=0, nbr_reads=2; 
  for (i=0; i<nbr_reads; i++)
  {
    val += analogRead(p);
    delay(1); 
  }
  return val/nbr_reads; 
};


#ifdef CW_KEYER
int read_keyer_speed()
{ 
  int n = read_analogue_pin((byte)PIN_KEYER_SPEED);
  //Serial.print("Speed returned=");
  //Serial.println(n);
  dot_length_ms = 60 + (n-183)/5;   // scale to wpm (10 wpm == 60mS dot length)
                                     // '511' should be mid point of returned range
                                     // change '5' to widen/narrow speed range...
                                     // smaller number -> greater range  
  return n;
};
#endif



byte get_front_panel_button()
// Take a reading of the front panel buttons and map it to a button number (0..4)
// Take multiple readings and average them
{
  byte b=0; 
  int z;
  z = read_analogue_pin((byte)SWITCH_BANK);
//  Serial.print("Frnt bttn="); Serial.println(z);

#ifdef SP_V
  if(z > 450 && z < 600)   b = 4;      // 524
  else if(z > 650 && z < 900) b = 6; // 717
#endif

#ifdef SS_EI9GQ
  if     (z > 1021)               b = 0;  // 1023
  else if(z >= 1000 && z <= 1021) b = 3;  // 1015-1021
  else if(z > 940   && z < 1000)  b = 1;  // 966-973
  else if(z >= 895   && z < 940)  b = 6;  // 910
  else if(z > 800   && z < 895)   b = 2;  // 880-886
  else if(z > 700   && z < 800)   b = 5;  // 737 
  else if(z > 400   && z < 480)   b = 4;  // 444 
#endif

//----------------------------------------------------------------------------------
#ifdef SP_6
                                      // open (USB power: 850) (LiFePO+7812: 1023)  
  if(z > 300 && z < 800) b = 4;       // B1   (USB power: 436) (LiFePO+7812: 719)  
  else if(z > 50 && z <= 300) b = 1;  // B2   (USB power:  91) (LiFePO+7812: 142)  
                                      // both:(USB power:  85) (LiFePO+7812: 132)  

  if(!digitalRead(10)){         // on this rig, the mechanical encoder's pushbutton is used to change radix
    while(!digitalRead(10)) ;   // spin here until encoder button is released
    b = 6;   
  }
  
#ifdef DIAGNOSTIC_DISPLAY
  diagnostic_flag = true;  // trace on the display as a diagnostic
//  diagnostic_int = z;      // trace 'z' 
  refresh_display();
#endif                                       
#endif
//----------------------------------------------------------------------------------

#ifdef SP_7
                                      // open (USB power: xxx) (LiFePO+7812: xxx)  
  if(z > 300 && z < 800) b = 4;       // B1   (USB power: xxx) (LiFePO+7812: xxx)  
  else if(z > 50 && z <= 300) b = 1;  // B2   (USB power:  xxx) (LiFePO+7812: xxx)  
                                      // both:(USB power:  xxx) (LiFePO+7812: xxx)
#endif                                       

#ifdef SP_8
  if(z > 5 && z < 100) b = 4;        // B4   increment the band (with wrap-around)

  if(!digitalRead(ENCODER_BUTTON)){         // on this rig, the mechanical encoder's pushbutton is used to change radix
    while(!digitalRead(ENCODER_BUTTON)) ;   // spin here until encoder button is released
    b = 6;                           // B6  increment the radix (with wrap-around)
  }
#endif     

#ifdef SP_9
  if(z > 2 && z < 100) b = 4;       // 15   B4   increment the band (with wrap-around)
  if(z >= 100 && z < 750) b = 6;    // 560   B4   increment the band (with wrap-around)
  if(z >= 750 && z < 1000) b = 1;   // 842   B4   increment the band (with wrap-around)

  if(!digitalRead(ENCODER_BUTTON)){         // on this rig, the mechanical encoder's pushbutton is used to change radix
    while(!digitalRead(ENCODER_BUTTON)) ;   // spin here until encoder button is released
    b = 6;                           // B6  increment the radix (with wrap-around)
  }

#ifdef DIAGNOSTIC_DISPLAY
  diagnostic_flag = true;  // trace on the display as a diagnostic
  diagnostic_int = z;      // trace 'z' 
  refresh_display();
#endif            
#endif    

#ifdef SP_X
  if(z > 500 && z < 800) b = 8;        // 580   freq step up 
  if(z > 180 && z < 400) b = 4;        // 236   B4   channel up (with wrap-around)
  if(z > 40  && z < 150) b = 9;        // 74    freq step down 
#endif     

#ifdef SP_11
  if(z > 700) b = 0;                  // 1011
  else if(z >= 60)  b = 6;            // 78 
  else if(z > 5   && z < 60)  b = 4;  // 42 
#endif  

#ifdef SS_FAT5_160AM_TX
  if(z > 900) b = 0;                  // 1008-1017
  else if(z > 77 && z < 899)   b = 1;    // 90
  else if(z > 55 && z < 76)   b = 4;    // 66 
  else if(z > 30  && z < 55)  b = 5;  // 41 
  else if(z > 5   && z < 30)  b = 6;  // 14 
#endif  

#ifdef SS_VK3SJ_160AM_TX
  if (z < 250) b = 1;       // 120 
  else if(z > 900) b = 2;  // 1023
#endif  

#ifdef SS_VK3SJ_40AM_TX
  if ((z> 50) && (z < 100)) b = 1;       // 120 
  else if((z > 25) && (z < 49)) b = 4;  // 1023
  else if((z > 5) && (z < 24)) b = 6;  // 1023
#endif  

#ifdef SS_AM_TX_TEST_VFO
  if(z > 800) b = 0;                   // 850
  else if(z > 50 && z < 200)   b = 4;  // 105
  else if(z > 250 && z < 800)  b = 1;  // 438 
#endif  

#ifdef SS_VK3SJ_160AM_PORTABLE_TX
  if(z > 800) b = 0;                  // 1022
  else if(z > 400 && z < 800) b = 1;  // top (black) button (525)
  else if(z > 20  && z < 400) b = 5;  // bottom (red) button (123) 
#endif  

#ifdef UNIVERSAL_VFO_CONTROLLER
  if(z > 0 && z < 200) b = 4;          //  14  band up
  else if(z > 400 && z <= 645) b = 6;  // 522  radix
  else if(z > 645 && z <= 900) b = 1;  // 768  band down
#endif

  if(b>0){ Serial.print("Front button="); Serial.print(b); Serial.print("   z="); Serial.println(z);}
  return b;  
} // get_front_panel_button()



void read_meter()
// in receive mode, reads an analog port and constructs an S-meter
// in transmit mode, reads 2 analog ports and constructs a power/SWR meter
// analogue port returns 0..1023;  meter string is 8 chars. 
{
  int sum, y, z;
#ifdef SP_V
  byte width = 6;
#else
  byte width = 8;
#endif

  int s_point_val[width] = {0, 128, 256, 384, 512, 640,  768,  896};
  //                        s1 s3   s5   s7   s9   s9+20 s9+40 s9+60
  
  if(mode_tx)
  {
    // calculate SWR or relative power using fwd_max, rev_max 
    z = fwd_max*10;  // arbitrary scaling for a relative output power indication    
  }
  else
  {
//    if(((++s_meter_update_cnt) % 2)!=0) return; // dampen a jittery s-meter 
    // take an S-meter reading

#define NBR_SAMPLES 2
#if defined(SP_7)
    // take an averaged reading from the analogue pin with the s-meter on it
    for (byte i=0; i<NBR_SAMPLES; i++) sum += analogRead(PIN_S_METER);    // read PIN_S_METER analog pin
    y = sum/NBR_SAMPLES;   
    // specific mappings for each particular receiver's AGC characteristics 
    
#ifdef SP_7
    switch (curr_line){
    case 1: z = map(y, 80, 140, 900, 80); break;  // 80m
    case 2: z = map(y, 80, 180, 900, 80); break;  // 40m
    case 3: z = map(y, 80, 140, 900, 80); break;  // 30m
    case 4: z = map(y, 80, 180, 900, 80); break;  // 20m
    }
#endif

#else
    z = s_meter_reading + rand()/100; // assign arbitrary value to try out a moving s-meter
#endif 
  }

//  Serial.print(curr_line ); Serial.print(" s-meter y="); Serial.print(y); Serial.print(" s-meter z="); Serial.println(z);

  S_meter = "1 :"; 
  S_meter += (char)165;    // for s1 - s9 print a small 'centered dot'

  if(!mode_tx)
  {
    if((z < s_point_val[1]) && (random(0,10) > 5)) 
    { 
      // we have a low reading, so simulate a bit of band noise (make the s-meter flicker between s1 and s2) 
      S_meter += (char)165;  // small 'centered dot'
      S_meter[0] = char(50); // '2'
      //    delay(100);  // careful, this will introduce a delay into tuning
    }
  }
  
  int s = 1;

  // now construct the meter string in either case
  for (byte j=0; j<width; j++)
  {
    if(z > s_point_val[j])
    {
      s=s+1; 
      // write in a visible char to the s-meter bar
      if(j<4)
      {
        S_meter += (char)165;    // for s1 - s9 append a small 'centered dot' 
        S_meter[0] = char(49+s); // numeric '1'..'9' 
        S_meter[1] = char(32);   // " ";
      }
      else if(j==4) // S9
      {
        S_meter += (char)165;    // for s1 - s9 append a small 'centered dot' 
        S_meter[0] = char(57);   // numeric '1'..'9' 
        S_meter[1] = char(32);   // " ";
      }
      else // j is 5..7
      {
        S_meter += (char)219;    // for s9+ append a large 'centered dot'     (char)219
        S_meter[0] = char(57);   // '9' 
        S_meter[1] = char(43);   // '+'
      }
    }
    else
      S_meter += ' ';     // append a blank
  }    

//  if(mode_tx)
//  {
//   Serial.print(sizeof(S_meter)); Serial.print(" s <"); Serial.print(S_meter); Serial.println(">");
//  }
  return;
}


// ------------------------------------------------------------------------------------------------------------------
// display routines

#ifdef DISPLAY_LCD
void LCD_diagnostic(char c)
{
  lcd.setCursor(10, 1);
  lcd.print(c);
}
#endif



void refresh_display()
{
// Update the display (may be Liquid Crystal Display, OLED) 
// call thru to the function for the installed display type

#ifdef DISPLAY_LCD
  refresh_LCD();
#endif
#ifdef DISPLAY_OLED
  refresh_OLED();
#endif
}


void refresh_OLED(){
#ifdef DISPLAY_OLED
// Update the OLED display 

#ifdef DIAGNOSTIC_DISPLAY
  if(diagnostic_flag)
  {
    oled.clear();
    oled.set1X();
    oled.setCursor(0, 0);
    oled.print(diagnostic_int, DEC);
    return;
  };
#endif  // DIAGNOSTIC_DISPLAY
  
  
  oled.set2X();
  oled.setCursor(0, 0);

  unsigned long f_Hz = VFOSet[v].vfo;
  
#ifdef SP_8
  sp8_band_2m = !digitalRead(BAND_SENSE_2M);
  // Serial.print("SP_8 D10=");   Serial.println(r);
  if(sp8_band_2m) f_Hz += SP_8_DIAL_OFFSET; 
#endif
  
  uint16_t f = f_Hz/1000;   // frequency in kHz

  int mhz = f/1000;                       // MHz (0..99)
  int khz = f - mhz*1000;                 // kHz (0..999)
//  int hz = VFOSet[v].vfo - (mhz*1000000 + khz*1000);  // Hz (0..999)   // original line 
  int hz = f_Hz - (mhz*1000000 + khz*1000);  // Hz (0..999)
  
//  Serial.print(" hz="); Serial.println(hz);

#ifdef SP_8
  if(sp8_band_2m ) mhz=mhz+16;  
#endif

  if(mhz<10) oled.print(" ");
  oled.print(mhz, DEC);
//  if(mhz<100) oled.print(",");
  oled.print(",");
  if(khz<10) oled.print("0");
  if(khz<100) oled.print("0");
  oled.println(khz, DEC);

  oled.set1X();
  oled.setCursor(0, 4);

  if(mode_tx)
  {
    // we are transmitting
    if(mode_cw)  
      oled.print(  "CW    ");
    else 
    {
      if(mode_tune)
        oled.print("TUNE  ");
      else 
        oled.print("Tx    ");
    }       
  }
  else
  {
    // we are receiving
//    Serial.print("radix="); Serial.println(VFOSet[v].radix); 

    if(VFOSet[v].radix==10000) oled.print("10kHz ");
    if(VFOSet[v].radix==1000)  oled.print("1kHz  ");
    if(VFOSet[v].radix==100)   oled.print("100Hz ");
    if(VFOSet[v].radix==10)    oled.print("10Hz  ");
  }
  
  oled.setCursor(84, 4);  // was 96, 4
  oled.print(".");
  if(hz<10) oled.print("0");
  if(hz<100) oled.print("0");
  oled.println(hz, DEC);
  
  oled.setCursor(0, 6);
  oled.print("V");
  oled.print(v+1);
  oled.print(" ");
  oled.setCursor(0, 7);

  int val = 0, s=0;
  if(!mode_tx) 
  {
    oled.setCursor(30, 6);
    oled.print("S");
    oled.setFont(s_meter10x15);

    // read and format the S-meter 
//    byte s = (rand()%2)+2;
    val = analogRead(PIN_S_METER);

#ifdef SP_6
    // do band-specific sensitivity scaling
    if((VFOSet[v].vfo >= FILTER_80_LB) && (VFOSet[v].vfo <= FILTER_80_UB)) val = val*3;   // '3' is the magic number for 80m
    s = map(val, 0, 550, 8, 1);  // map s-meter analogue reading to s-scale
    // now add some random movement (make the meter flicker up by 1 S-point to animate it!)
    s = s + 1 + rand()%2;
#endif

#ifdef SP_8 
    s = map(val, 180, 220, 8, 1);  // map s-meter analogue reading to s-scale
    //  add some decay...
    if(s > last_s_meter_val) {}
    else if(s < last_s_meter_val) s = last_s_meter_val - 1;
    last_s_meter_val = s; 

#endif

#ifdef SP_9  
    s = map(val, 20, 1023, 8, 1);  // map s-meter analogue reading to s-scale
    //  add some decay...
    if(s > last_s_meter_val) {}
    else if(s < last_s_meter_val) s = last_s_meter_val - 1;
    last_s_meter_val = s; 
#endif
   
//    Serial.print(" val=");  Serial.print(val); Serial.print(" s=");  Serial.println(s);     
    byte c=0;
    for(byte j=1; j<=s; j++){
      oled.print((char)j);
      c++;
    };
    oled.setFont(fixed_bold10x15);
    for(byte k=8; k>c; k--) oled.print(" "); // right pad the s-meter display space
  }
  else
  {
    // in transmit mode, so display a relative RF power meter
    if(!mode_cw) {
      pwr_val = read_analogue_pin(PIN_PWR_METER);
//      Serial.print(" refresh_OLED():: pwr_val=");  Serial.println(pwr_val);       
    }
    refresh_pwr_meter();
  };
#endif // #ifdef DISPLAY_OLED
}


void refresh_pwr_meter()
{
#ifdef DISPLAY_OLED
  // write to the power meter line on the OLED 
  oled.setCursor(30, 6);
  oled.print("P");
  oled.setFont(s_meter10x15);
        
  // do band-specific sensitivity scaling
//    if((VFOSet[v].vfo >= FILTER_80_LB) && (VFOSet[v].vfo <= FILTER_80_UB)) val = val*3;   // '2' is the magic number for 80m

  // NOTE: the RF power sensing analog pin is read elsewhere, and the value stored in global pwr_val 
  byte s = map(pwr_val, 0, 550, 1, 8);
    
//  Serial.print(" refresh_pwr_meter()::pwr_val=");  Serial.print(pwr_val);  Serial.print(" s=");  Serial.println(s);  
  byte c=0;
  for(byte j=1; j<=s; j++){
    oled.print((char)j);
    c++;
  };
  oled.setFont(fixed_bold10x15);
  for(byte k=8; k>c; k--) oled.print(" "); // right pad the pwr-meter display space
#endif // #ifdef DISPLAY_OLED
};



void refresh_LCD() {
#ifdef DISPLAY_LCD

// Update the LCD  
  uint16_t f, g;
  uint32_t vfo_l; 

  lcd.setCursor(0, 0);
  if(!BFO_tune_flg)
  {
    vfo_l = VFOSet[v].vfo; 
    if(mode_tx && !mode_cw) lcd.print("Tx");
    if(mode_tx && mode_cw) lcd.print("CW");
    if(mode_tx && mode_tune) lcd.print("TUNE ");
    if(!mode_tx) lcd.print("Rx");
    lcd.print(" ");
    lcd.print(v+1);
    lcd.print(" ");
  }
  else
  {
    lcd.print("BFO>");
    lcd.setCursor(13, 0);
    vfo_l = bfo;      
  }

#ifdef LCD_8X2 
  lcd.setCursor(0, 0);
#else 
  lcd.setCursor(6, 0);
#endif

  f = vfo_l / 1000000;   
  
#ifndef LCD_8X2
  if (f < 10) lcd.print(' ');
#endif
  lcd.print(f);

#ifdef LCD_8X2
  if (f < 10) lcd.print(',');
#else
  lcd.print('.');
#endif

  f = (vfo_l % 1000000) / 1000;
  if (f < 100)
    lcd.print('0');
  if (f < 10)
    lcd.print('0');
  lcd.print(f);
  lcd.print('.');
  f = vfo_l % 1000;
  if (f < 100)
    lcd.print('0');
  if (f < 10)
    lcd.print('0');
  lcd.print(f);
//  lcd.print("Hz              ");
  lcd.print(" ");

  lcd.setCursor(15, 0);
  if(func_button_pressed) lcd.print("F");    
  else lcd.print(" ");

#ifdef LCD_20X4
  lcd.setCursor(19, 0);
  lcd.print("*");  
#endif

// line 2
  lcd.setCursor(0, 1);  // start of line 2
  if(message_playing)
  {
#ifdef CW_KEYER
    // write the canned CW message across line 2...
    byte first=0, last=cw_msg_index; 
    if(last>=16) first= last%16 + 1; // implement scrolling
    lcd.print( (morse_msg[curr_msg_nbr-1]).substring(first,last+1) );
#endif 
  }
  else
  {
    // display s-meter in receiver or pwr meter in transmit 
    if(!mode_tx) {
      read_meter();  // fix this at the nbr of chars supported by the display
#ifdef SP_V
      lcd.print(S_meter.substring(3));  // limited size on this rig's display
#else
      lcd.print("s");
      lcd.print(S_meter);
      lcd.print("     "); // clear the rest of line 2 (replace this if right half of line 2 gets used for something) 
#endif
    }
    else
    {
      // transmit line 2, in modes other than keyer message playing
      // placeholder for the relative power meter
    }
    
#ifdef LCD_8X2
    lcd.setCursor(6, 2);
    if(mode_tx && !mode_cw) lcd.print("Tx");
    if(mode_tx && mode_cw) lcd.print("CW");
    if(!mode_tx) {
      lcd.setCursor(6, 2);
      lcd.print(" ");
      lcd.print(v+1);
    }
#endif
  }

#ifdef LCD_20X4
  lcd.setCursor(0, 3);
  lcd.print("16C ");  

  // get the time and display
  lcd.setCursor(12, 3);
  DateTime nw = rtc.now();
  
  Serial.print(nw.hour(), DEC); 
  Serial.print(':');
  Serial.print(nw.minute(), DEC);
  Serial.print(':');
  Serial.println(nw.second(), DEC);
  
  if(nw.hour() < 10) lcd.print(" ");
  lcd.print(nw.hour(), DEC);
  lcd.print(":");

  if(nw.minute() < 10) lcd.print("0");
  lcd.print(nw.minute(), DEC);
  lcd.print(":");

  if(nw.second() < 10) lcd.print("0");
  lcd.print(nw.second(), DEC);
//  lcd.print("23:00:01");
#endif

#ifdef LCD_20X4
  // show something useful on line 3
  lcd.setCursor(0, 2);
//  lcd.print("____        ________");  
    lcd.print("13.8v 0.4A      1.3S");  
#endif

  byte cursor_pos;  
#ifdef LCD_8X2 
  cursor_pos = 7;
#endif
#ifndef LCD_8X2
  cursor_pos = 14;
#endif

  switch (VFOSet[v].radix)
  {
    case 10:
      lcd.setCursor(cursor_pos, 0);
    break;
      
    case 100:
      lcd.setCursor(cursor_pos-1, 0);
    break;
      
    case 1000:
      lcd.setCursor(cursor_pos-3, 0);
    break;
      
    case 10000:
      lcd.setCursor(cursor_pos-4, 0);
    break;
  };


#ifdef SS_VK3SJ_160AM_TX
  // read temp on the heatsink
  float c = tempsensor.readTempC();
  String s(c);
  String C_str = s.substring(0, s.indexOf('.')+2) + "C";
//  Serial.print("C_str: "); Serial.println(C_str);
  lcd.setCursor(11, 1);
  lcd.print(C_str);
#endif

#ifdef SS_VK3SJ_160AM_PORTABLE_TX
  // display volt and ammeter readings on LCD line 2
  if(mode_tx){
    lcd.setCursor(0, 1);
    lcd.print(HT_v);
    lcd.print("V    ");
    
    lcd.setCursor(11, 1);
    lcd.print(HT_a_f);
    lcd.print("A");
  }
#endif

#endif // #ifdef DISPLAY_LCD
}


// ------------------------------------------------------------------------------------------------------------------
// filter control

void set_filters(uint32_t f)
{
    // select the appropriate filter set for the frequency passed in
    // Note that LPF_line and curr_line are global
    byte BPF_line=0;  
    bool band_changed = false;

#ifdef SS_AM_TX_TEST_VFO
// set the BPF line for this 8 band receiver

    if ((f >= FILTER_630_LB) && (f <= FILTER_630_UB)) BPF_line = 1;
    if ((f >= FILTER_MW_LB) && (f <= FILTER_MW_UB)  ) BPF_line = 2;
    if ((f >= FILTER_160_LB) && (f <= FILTER_160_UB)) BPF_line = 3;
    if ((f >= FILTER_80_LB) && (f <= FILTER_80_UB)  ) BPF_line = 4;
    if ((f >= FILTER_60_LB) && (f <= FILTER_60_UB)  ) BPF_line = 5;
    if ((f >= FILTER_40_LB) && (f <= FILTER_40_UB)  ) BPF_line = 6;
    if ((f >= FILTER_30_LB) && (f <= FILTER_30_UB)  ) BPF_line = 7;
    if ((f >= FILTER_20_LB) && (f <= FILTER_20_UB)  ) BPF_line = 8;
// end SS_AM_TX_TEST_VFO
#else
    if ((f >= FILTER_160_LB) && (f <= FILTER_160_UB)) 
    {
      BPF_line = 1;
      LPF_line = 1;
    }
    else if((f >= FILTER_80_LB) && (f <= FILTER_80_UB))   
    {
      BPF_line = 2;
      LPF_line = 2;
    }
    else if((f >= FILTER_60_LB) && (f <= FILTER_60_UB))   
    {
      BPF_line = 7;
      LPF_line = 7;
    }
    else if((f >= FILTER_40_LB) && (f <= FILTER_40_UB))   
    {
      BPF_line = 3;
      LPF_line = 3;
    }
    else if((f >= FILTER_30_LB) && (f <= FILTER_30_UB))   
    {
      BPF_line = 4;
      LPF_line = 4;
    }
    else if((f >= FILTER_20_LB) && (f <= FILTER_20_UB))   
    {
      BPF_line = 5;
      LPF_line = 5;
#ifdef SP_6
      BPF_line = 1;  // need to do a specific mapping here to deal with a hardware quirk in SP_6!
#endif
    }
    else if((f >= FILTER_17_LB) && (f <= FILTER_17_UB))   
    {
      BPF_line = 6;
      LPF_line = 5;
    }
    else if((f >= FILTER_15_LB) && (f <= FILTER_15_UB))   
    {
      BPF_line = 7;
      LPF_line = 6;
    };
#endif

#ifdef SP_7
    BPF_line--;
    LPF_line--; 
#endif    

#ifdef SP_9
    // BPF and LPFs share the same decoder 
    // 80m BPF_line = 2->1   
    // 60m BPF_line = 7->2   
    // 40m BPF_line = 3->3   
    // 30m BPF_line = 4->4   
    // 20m BPF_line = 5->5   
  
    if(BPF_line == 2) BPF_line = 1;
    if(BPF_line == 7) BPF_line = 2;
    if(BPF_line == 3) BPF_line = 3;
    if(BPF_line == 4) BPF_line = 4;
    if(BPF_line == 5) BPF_line = 5;
    LPF_line = BPF_line;               // (LPF_line not used) 
#endif    

#ifdef SP_X
    BPF_line -= 3;
    LPF_line -= 3; 
#endif  

#ifdef SP_11
  // map BPF_line for this rig
  if(BPF_line == 2) BPF_line = 1;
  if(BPF_line == 3) BPF_line = 2;
  if(BPF_line == 5) BPF_line = 3;
#endif    

#ifdef UNIVERSAL_VFO_CONTROLLER
    // do the necessary mappings here 
#endif    

//-----------------------------
//   Serial.print("BPF_line, LPF_line="); Serial.print(BPF_line);Serial.print(","); Serial.println(LPF_line); 

  // handle the BPFs, if the band has changed
    byte prev_BPF_line = curr_line; 
    if(BPF_line != curr_line)
    {
        // *** raise the appropriate control line 
        // (where 0 means no line which will implement 'out of band' protection
        for (int i=0; i<8; i++) PCF_BPF.write(i, 0); //turn all pins of the I/O expander off 
        delay(50);
        if(BPF_line > 0){

           // may need to do some BPF_line mapping here...

           PCF_BPF.write(BPF_line - 1, 1);  //turn the band-specific pins of the I/O expander on 
           Serial.print("New band:");
           Serial.println(BPF_line);
           delay(50); 
           band_changed = true;
        }
        curr_line = BPF_line;  
    }

    // handle the LPFs, if the band has changed
    if(band_changed)
    {
        // the band changed, so reset the LPF
        for (byte i=0; i<8; i++) PCF_LPF.write(i, 0); //turn all pins of the I/O expander off 
                                                     // all relays will drop out
        if(LPF_line > 0){
           byte pcf_pin = LPF_line - 1;
           // handle LPF_line to PCF8574 pin mapping for specifric rigs here
#ifdef SP_6
           pcf_pin--;  // need to do a specific mapping here to deal with SP_6 use of LPF PCF8574 pins
#endif      
           PCF_LPF.write(pcf_pin, 1);  //turn the band-specific pin of the I/O expander on 
//           Serial.print("1:");   Serial.println(PCF_LPF.lastError());

           Serial.print("New LPF:");
           Serial.println(LPF_line);
           LPF_engaged = true; 
//           delay(20); 
        }
    };

#ifdef SP_V
    // convenient place to implement band switching (40/20m) in SP-V
    if(prev_BPF_line != curr_line)
    {
      // the band changed since last time we were in here
      if(curr_line == 5)  // VFO is currently on 20m so pull in the band relays  
        {
          Serial.println("engaging 20m");
          digitalWrite(A3, HIGH);
        }  
        else 
        if(curr_line == 3) // 40m
         {
          Serial.println("dis-engaging 20m");
          digitalWrite(A3, LOW);
        }
        delay(100); 
    }
#endif
}


#ifdef SP_X
void reset_channel_freq()
{
  // reset the VFO frequency back to a default
  if(v == 0) VFOSet[v].vfo = SP_X_CH0_FREQ_DEFAULT;
  if(v == 1) VFOSet[v].vfo = SP_X_CH1_FREQ_DEFAULT;
  if(v == 2) VFOSet[v].vfo = SP_X_CH2_FREQ_DEFAULT;
  if(v == 3) VFOSet[v].vfo = SP_X_CH3_FREQ_DEFAULT;
  if(v == 4) VFOSet[v].vfo = SP_X_CH4_FREQ_DEFAULT;
  if(v == 5) VFOSet[v].vfo = SP_X_CH5_FREQ_DEFAULT;
}
#endif


// ------------------------------------------------------------------------------------------------------------------
// EEPROM

void update_eeprom()
{
  if(abs( millis() - last_freq_change_ms ) > 10000)
  {
    if(eeprom_written_since_last_freq_change == false)
    {
      // do the eeprom write
      // Serial.println("*** eeprom write");
      EEPROM.write(0, v);   // write the band index (v) to the first byte
        
      int element_len = sizeof(VFOset_type);
      for(int i=0; i<NBR_VFOS ; i++)    // write each element of the VFOSet array
      {
        EEPROM.put(1 + (i * element_len), VFOSet[i]);
      }
      eeprom_written_since_last_freq_change = true;
    }
  }
};


// ------------------------------------------------------------------------------------------------------------------
// TR switching and sequencing 

void receive_to_TRANSMIT()
{
  mode_tx = true;
  Serial.println("receive_to_TRANSMIT()");
  digitalWrite(RX_MUTE_LINE, RX_MUTE_ON_VALUE);     // mute the receiver
  delay(RX_MUTE_DELAY_MS);

  // pull in the current LPF (if it is not engaged already)
  if(!LPF_engaged)
  {
    byte pcf_pin = LPF_line - 1;
#ifdef SP_6
    pcf_pin--;  // need to do a specific mapping here to deal with SP_6 use of LPF PCF8574 pins
#endif
    PCF_LPF.write(pcf_pin, 1);  //turn the band-specific pin of the I/O expander on 
//     Serial.print("2:");   Serial.println(PCF_LPF.lastError());

    delay(5);
    Serial.print("JIT LPF:"); Serial.println(LPF_line);
    LPF_engaged = true; 
    last_T_R_ms = millis(); 

    // start the fan, if there is one...
 
  };
  
  digitalWrite(TRANSMIT_LINE, 1); // pull in the T/R relay
  delay(10); 

  if(mode_cw)
  {
    // Serial.println("  mode_cw==true");  
    si5351.output_enable(SI5351_CLK0, 0);  // turn the VFO off
    // Serial.println("VFO disabled");

#ifdef BFO_ENABLED
    si5351.output_enable(SI5351_CLK2, 0);  // turn the BFO off
#endif
    
    // prime CLK1 to the current frequency (+- CW offset), ready to transmit

    volatile uint32_t f; 
    f = VFOSet[v].vfo; 
    // choose whether to add or subtract the CW offset
    if(f >= SIDEBAND_THRESHOLD) 
      f += CW_TONE_HZ; 
    else 
      f -= CW_TONE_HZ;
    
    Serial.print("CO:"); Serial.println(f);
    si5351.set_freq(f * SI5351_FREQ_MULT, SI5351_CLK1);
    if(f < CO_DRIVE_THRESHOLD)
    {
      si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_4MA);       
//      Serial.println("CO Lo");
    }
    else
    {
      si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_6MA); 
//      Serial.println("CO Hi");
    }
    
    si5351.output_enable(SI5351_CLK1, 0); // turn the CW clock off until keyed
    dot_dash_sent = 0;  // count nbr chars sent this transmit period

#ifdef SP_6
    digitalWrite(CO_DC_SUPPLY, 1);  // powers up the carrier oscillator buffer 
#endif   
#ifdef SP_7
    digitalWrite(CO_DC_SUPPLY, 1);  // powers up the carrier oscillator buffer 
#endif  
#ifdef SP_8
    digitalWrite(CO_DC_SUPPLY, 1);  // powers up the carrier oscillator buffer (NOT USED!) 
#endif  
#ifdef SP_9 
    digitalWrite(CO_DC_SUPPLY, 1);  // powers up the carrier oscillator buffer 
#endif
 
  }
  else
  {
    // mode is NOT CW ...

#if defined(SS_FAT5_160AM_TX) || defined(SS_VK3SJ_160AM_TX) || defined(SS_AM_TX_TEST_VFO) || defined(SS_VK3SJ_40AM_TX) || defined(SS_VK3SJ_160AM_PORTABLE_TX)
    si5351.set_freq(VFOSet[v].vfo * SI5351_FREQ_MULT, SI5351_CLK0); // turn on the drive (VFO on signal frequency, CLK0)
    si5351.output_enable(SI5351_CLK0, 1); 
    si5351.output_enable(SI5351_CLK2, 0);   // turn receiver VFO off, if it is being used 
    delay(20);
    digitalWrite(PWM_ENABLE_LINE, 1);     // enable the Pulse Width Modulator    
#endif
    
#ifdef SP_6
    // mode is SSB, so un-mute the mic amp
    delay(TX_SSB_MUTE_DELAY);
    digitalWrite(TX_SSB_MUTE_LINE, 1); // un-mute the mic amp (to silence a T/R squeal in SP_6) 
#endif
    
#ifdef VFO_BFO_SWAP
    // swap the VFO and the BFO on CLK0 and CLK2 for the transmit period
    si5351.output_enable(SI5351_CLK0, 0);  // turn VFO off 
    si5351.output_enable(SI5351_CLK2, 0);  // turn BFO off 
    delay(10);
    si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK0); // set CLK0 to the BFO frequency    
    si5351.set_freq((VFOSet[v].vfo + bfo) * SI5351_FREQ_MULT, SI5351_CLK2); // set CLK2 to  VFO+IF freq for current band 
    si5351.output_enable(SI5351_CLK0, 1);  // turn reversed BFO back on 
    si5351.output_enable(SI5351_CLK2, 1);  // turn reversed VFO back on 
#endif
  }
};


void TRANSMIT_to_receive()
{
  mode_tx = false;
  Serial.println("TRANSMIT_to_receive()");

#if defined(SS_FAT5_160AM_TX) || defined(SS_VK3SJ_160AM_TX) || defined(SS_AM_TX_TEST_VFO) || defined (SS_VK3SJ_40AM_TX) || defined(SS_VK3SJ_160AM_PORTABLE_TX)
    digitalWrite(PWM_ENABLE_LINE, 0);     // disable the Pulse Width Modulator
    delay(20); 
    si5351.output_enable(SI5351_CLK0, 0); // disable the clock (transmit VFO)
    si5351.output_enable(SI5351_CLK2, 1); // enable the AM receiver VFO, if one is being used
#endif
  
  digitalWrite(TRANSMIT_LINE, 0); // drop out the T/R relay
  delay(10);

  if(mode_cw)
  {
    // Serial.println("  mode_cw==true");
#if defined(SS_FAT5_160AM_TX) || defined(SS_VK3SJ_160AM_TX) || defined (SS_VK3SJ_40AM_TX)
    // nothing
#else
    si5351.output_enable(SI5351_CLK0, 1);  //  turn the VFO back on again 
    if(VFOSet[v].vfo < VFO_DRIVE_THRESHOLD)
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); 
    else
    {
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_4MA);
    } 
#endif
      
#ifdef BFO_ENABLED
    si5351.output_enable(SI5351_CLK2, 1);  //  turn the BFO back on again 
//    Serial.println("VFO enabled");
#endif 

#ifdef SP_6
    digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 
#endif
#ifdef SP_7
    digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 
#endif
#ifdef SP_8
    digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 
#endif
#ifdef SP_9 
    digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 
#endif
//#ifdef SP_X 
//    digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 
//#endif
  }
  else
  {
    // mode is SSB (or AM)  
#ifdef SP_6
    // mode is SSB, so mute the mic amp
    digitalWrite(TX_SSB_MUTE_LINE, 0); // mute the mic amp (to silence a T/R squeal in SP_6) 
#endif
    
#ifdef VFO_BFO_SWAP
    // swap the VFO and the BFO on CLK0 and CLK2 back to their normal clocks 
    si5351.output_enable(SI5351_CLK0, 0);  // turn BFO off 
    si5351.output_enable(SI5351_CLK2, 0);  // turn VFO off 
    delay(10);
    si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK2); // set CLK2 to the BFO frequency    
    si5351.set_freq((VFOSet[v].vfo + bfo) * SI5351_FREQ_MULT, SI5351_CLK0); // set CLK0 to  VFO+IF freq for current band 
    si5351.output_enable(SI5351_CLK0, 1);  // turn reversed VFO back on 
    si5351.output_enable(SI5351_CLK2, 1);  // turn reversed BFO back on 
#endif    
  };

#ifdef SP_X
  digitalWrite(SP_X_MUTE_SPKR, LOW);  // mute the speaker line
#endif
  delay(3 * RX_MUTE_DELAY_MS);
  digitalWrite(RX_MUTE_LINE, RX_MUTE_OFF_VALUE);     // un-mute the receiver
#ifdef SP_X
  delay(50);
  digitalWrite(SP_X_MUTE_SPKR, HIGH);  // un-mute the speaker line
#endif

  mode_cw = false;  // mode will be set next time the paddle or keyer is touched
  last_T_R_ms = millis();  // start the LPF drop out timer from now
  pwr_val = 0;  // flush out residual RF power reading

#ifdef SP_X
  if(v==5) digitalWrite(13, 1); // turn the green CW LED back on (it doubles as a channel indicator)
#endif
};

// ------------------------------------------------------------------------------------------------------------------


void tune()
// puts the carrier on, for TUNE_MS milliseconds
// NOTE - this function relies on some functions inside the CW_KEYER block -- factor these out!
{
  if(!mode_tx)
  {
    // prime CLK1 to the current frequency 
     si5351.set_freq(VFOSet[v].vfo * SI5351_FREQ_MULT, SI5351_CLK1);
     si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); 
     si5351.output_enable(SI5351_CLK1, 0); // turn the CW clock off until keyed
     receive_to_TRANSMIT(); 
     
#if defined(SP_6) || defined(SP_7) || defined(SP_8) || defined(SP_9)
    digitalWrite(CO_DC_SUPPLY, 1);  // power up the carrier oscillator buffer 
#endif

     tone(PIN_TONE_OUT, CW_TONE_HZ);
     Serial.println("Tune -- Carrier on"); 
     set_key_state2('D');  
     refresh_display();

     delay(TUNE_MS);  // key down for this number of mS
     Serial.println("Tune -- Carrier off"); 
     set_key_state2('U');  

     noTone(PIN_TONE_OUT);
     
#if defined(SP_6) || defined(SP_7) || defined(SP_8) || defined(SP_9)
    digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 
#endif

     TRANSMIT_to_receive(); 
  }
  mode_tune = false; 
}


void set_key_state2(char k)
{
// push the morse key down, or let it spring back up
// changes global 'key_state' (bool)

  if(!key_down and k=='D')
  {
    // do whatever you need to key the transmitter
    digitalWrite(13, 1);  // turn the Nano's LED on
    si5351.output_enable(SI5351_CLK1, 1); // turn the (CW clock) on
    key_down = true; 
  };

  if(key_down and k=='U')
  {
    // do whatever you need to un-key the transmitter 
    digitalWrite(13, 0);  // turn the Nano's LED off
    si5351.output_enable(SI5351_CLK1, 0); // turn of (CW freq) clock off

    char_sent_ms = millis();
    space_inserted = false;
    key_down = false; 
  };
}  



#ifdef CW_KEYER
// start of CW Keyer block ------------------------------------------------------------------------------------------

int morse_lookup(char c)
// returns the index of parameter 'c' in MorseCode array, or -1 if not found
{
  for(int i=0; i<sizeof(MorseCode); i++)
  {
    if(c == MorseCode[i].ch[0])
      return i;
  }
  return -1; 
};


byte check_keyer_pushbutton()
{
// Reads the keyer pushbuttons and returns the button number as a byte; 0 if not pressed  
  byte b=0; 
  int  z;
  z = read_analogue_pin(PIN_PUSHBTTN_REAR);    // read the analog pin

// Serial.print("Kyr pshbtn z="); Serial.println(z);

  // reading is from the rear keyer pushbuttons  
#ifdef SP_IV
  if(z >= 65 && z < 120) b = 1;        // 88
  else if(z > 130 && z < 200) b = 2;   // 164
  else if(z >= 200 && z < 250) b = 3;  // 228
#endif

#ifdef SP_V
  if(z >= 140 && z < 200) b = 1;       // 171
  else if(z > 280 && z < 350) b = 2;   // 317
  else if(z >= 550 && z < 650) b = 3;  // 601
#endif

#ifdef SP_6
                                      // open (USB power: 840) (LiFePO+7812: 1023)  
  if(z > 300 && z < 990) b = 1;       // L    (USB power: 418) (LiFePO+7812: 712)  
  else if(z > 10 && z <= 300) b = 2;  // R    (USB power:  74) (LiFePO+7812: 122)  
                                      // both:(USB power:  66) (LiFePO+7812: 112)  
#endif

#ifdef SP_7
  if(z > 0 && z <= 60) b = 2;       //  75
  else if(z > 60 && z < 500) b = 1; //  42 
#endif

#ifdef SP_8
  if(z <= 100) b = 1;       //  15 (NOT USED!)
#endif

#ifdef SP_9
  if(z <= 100) b = 1;       //  15
#endif

#ifdef SP_X
  if(z <= 100) b = 1;       //  15
#endif

#ifdef SP_11
  if(z >= 5 && z <= 30) b = 1;       // 17
  else if(z > 30 && z < 100) b = 2;   // 41
  else if(z >= 100 && z < 300) b = 3;  // 135
#endif

#ifdef UNIVERSAL_VFO_CONTROLLER
  if(z > 0 && z <= 80) b = 3;       //  14
  else if(z > 300 && z < 650) b = 2; //  526 
  else if(z > 650 && z < 900) b = 1; //  771 
#endif

  if(b>0){
      Serial.print("Keyer pushbutton="); Serial.print(b); Serial.print(", z="); Serial.println(z);
    }
  return b;
}


byte check_paddle()
{
// Reads the paddle, returns the paddle number as a byte; 0 if not pressed  
  byte b=0; 
  int  z=0;

  z = read_analogue_pin(PIN_PADDLE);    // read the analog pin
  
#ifdef DIAGNOSTIC_DISPLAY
  diagnostic_flag = true;  // trace on the display as a diagnostic
  diagnostic_int = z;      // trace 'z' 
//  delay(200);
#endif 

// Serial.print("Kyr pdl, z="); Serial.println(z);

#ifdef SP_IV
  if(z > 130 && z < 199) b = 2;      // L 165
  else if(z > 70 && z < 99) b = 1;   // R 89 
#endif

#ifdef SP_V
  if(z > 650 && z < 750) b = 2;      // L 681
  else if(z > 450 && z < 550) b = 1; // R 510 
                                     // both: 407 for future reference
#endif

#ifdef SP_6
// SP-IV uses D11 and D12 for the paddle
  if(!digitalRead(11)) b=2;
  if(!digitalRead(12)) b=1;
#endif

#ifdef SP_7
  if(z > 0 && z < 50) b = 2;        // L 14
  else if(z > 80 && z < 150) b = 1; // R 110 
                                     // both: xxx for future reference
#endif

#ifdef SP_8
  if(z < 30) b = 2;                 // L 14
  else if(z > 30 && z < 100) b = 1; // R 110 
#endif

#ifdef SP_9
  if(z < 30) b = 2;                 // L 14
  else if(z > 30 && z < 100) b = 1; // R 110 
#endif

#ifdef SP_X
  if(z < 30) b = 2;                 // L 14
  else if(z > 30 && z < 100) b = 1; // R 110 
#endif

#ifdef SP_11
  if(z < 90) b = 2;                 // L 43
  else if(z > 90 && z < 400) b = 1; // R 133 
#endif

#ifdef UNIVERSAL_VFO_CONTROLLER
  if(z > 0 && z < 150) b = 1;        // L 39-45
  else if(z > 500 && z < 900) b = 2; // R 770 
#endif

  if(b>0){
//     Serial.print("Kyr pdl, b="); Serial.print(b); 
//     Serial.print(", z="); Serial.println(z); 
  }
  return b;
};


void activate_state2(char c)
{
// if necessary, activates the receiver or the transmitter 
// 'c' may be either 'T' or 'R' 

  if(!mode_tx && c=='T') receive_to_TRANSMIT();
  else 
    if(mode_tx && c=='R') TRANSMIT_to_receive();
  // in  all other cases, do nothing!
}


void send_dot()
{
  delay(dot_length_ms);  // wait for one dot period (space)

  // send a dot and the following space
  tone(PIN_TONE_OUT, CW_TONE_HZ);
  set_key_state2('D');  
  if(dot_dash_counter % SERIAL_LINE_WIDTH == 0) Serial.println(); 
  Serial.print(".");
  delay(dot_length_ms);  // key down for one dot period
  noTone(PIN_TONE_OUT);
  
  // read the RF power sensing circuit while the key is down and store the result for later display
  pwr_val = read_analogue_pin(PIN_PWR_METER);

  set_key_state2('U');  
  dot_dash_counter++;
  dot_dash_sent++;
}

void send_dash()
{
  delay(dot_length_ms);  // wait for one dot period (space)
  // send a dash and the following space
  tone(PIN_TONE_OUT, CW_TONE_HZ);
  set_key_state2('D');  
  if(dot_dash_counter % SERIAL_LINE_WIDTH == 0) Serial.println(); 
  Serial.print("-");
  delay(dot_length_ms * CW_DASH_LEN);  // key down for CW_DASH_LEN dot periods
  noTone(PIN_TONE_OUT);

  // read the RF power sensing circuit while the key is down and store the result for later display
  pwr_val = read_analogue_pin(PIN_PWR_METER);

  set_key_state2('U');  
  dot_dash_counter++;
  dot_dash_sent++;
}

void send_letter_space()
{
  delay(dot_length_ms * 4);  // wait for 3 dot periods
  Serial.print(" ");
}

void send_word_space()
{
  delay(dot_length_ms * 7);  // wait for 6 dot periods
  Serial.print("  ");
}

void send_morse_char(char c)
{
  // 'c' is a '.' or '-' char, so send it 
  if(c == '.') send_dot();
  else if (c == '-') send_dash();
  // ignore anything else, including 0s
}

void play_message(String m, int s)
{
// sends the message in string 'm' as CW, with inter letter and word spacing
// s is the speed to play at; if s == 0, use the current speed  
  byte j, n, old_s; 
  char buff[30];   // magic number, should guard this!

  message_playing = true; 
  Serial.println(m);
//  Serial.println(m.length());

  // use ch = m.charAt(index);
  m.toCharArray(buff, m.length()+1);

  if(s > 0)  // caller has passed in a speed to send message at 
  {
    old_s = dot_length_ms; // preserve the current keyer speed
    dot_length_ms = s;
  }
//  Serial.print("play_message()::dot_length_ms:");   Serial.println(dot_length_ms);

  //activate_state2('T'); 

  for (cw_msg_index=0; cw_msg_index<m.length(); cw_msg_index++)
  {
    if(buff[cw_msg_index] == ' ') 
    {
       send_word_space(); 
    }
    else
    {
      if( (n = morse_lookup(buff[cw_msg_index])) == -1 )
      {
        // char not found, ignore it (but report it on Serial)
//        Serial.print("!! not found <");
//        Serial.print(buff[cw_msg_index]);
//        Serial.println(">");
      }
      else
      {
        // char found, so send it as dots and dashes
        // Serial.println(n);
        refresh_display();         // first, refresh the LCD so the current CW char gets displayed

        for(j=1; j<7; j++)
          send_morse_char(MorseCode[n].ch[j]);
        send_letter_space();  // send an inter-letter space
        // if(s==0)read_keyer_speed();  // see if speed has changed mid message ***
      } // else
    } // else 
  } // for  
  Serial.println();
  if(s > 0)  // reset speed back to what it was  
    dot_length_ms = old_s;

  //activate_state2('R'); 
  message_playing = false; 
  curr_msg_nbr = 0;
  cw_msg_index = 0;
} // play_message
// end CW Keyer block -------------------------------------------------------------
#endif


// ------------------------------------------------------------------------------------------------------------------
// setup()


void setup()
{
  Serial.begin(9600);  
  Wire.begin();
  
#ifdef SP_IV
  Serial.println("SP_IV MST");
  lcd.begin(16, 2);   
  lcd.cursor();
  lcd.noBlink();   
  lcd.print("SP Gnrc Bld 1.40");
  delay(1000); 
  lcd.setCursor(0,1);
  lcd.print("VK3HN 28/01/2018");
  delay(1000); 
  lcd.clear();
#endif

#ifdef SP_V
  Serial.println("SP_V ...");
  lcd.begin(8, 2);                      
  lcd.cursor();
  lcd.noBlink();
  lcd.setCursor(0,1);
//  lcd.print(".VK3HN. ");
  pinMode(A3, OUTPUT);   // in SP-V, use this analogue pin to control band relays
  digitalWrite(A3, LOW); // start with band relays dis-engaged (40m) 
#endif

#ifdef SS_EI9GQ
  Serial.println("SS_EI9GQ ...");
  lcd.begin(20, 4);                      
  lcd.print("SP Gnrc Bld 1.40");
  delay(1000); 
  lcd.setCursor(0,1);
  lcd.print("VK3HN 28/01/2018");
  delay(1000); 
  lcd.clear();
#endif

#ifdef SP_6
  Serial.println("SP_6 ...");
#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_OLED_ADDRESS, RST_PIN);
#else // RST_PIN < 0
  oled.begin(&Adafruit128x64, I2C_OLED_ADDRESS);
#endif // RST_PIN >= 0
  oled.setFont(fixed_bold10x15);
  oled.clear();
  oled.println("   Summit     ");
  oled.println(" Prowler_VI");
  oled.println(" < VK3HN > ");
  oled.println("R1.4 11/2018");
//  oled.println();
  delay(1000);
  oled.clear();

  pinMode(10, INPUT_PULLUP); // this rig uses D10 to read the encoder pushbutton for 'step' (radix) control
  pinMode(11, INPUT_PULLUP); // this rig uses D11 for paddle LEFT
  pinMode(12, INPUT_PULLUP); // this rig uses D12 for paddle RIGHT

  pinMode(CO_DC_SUPPLY, OUTPUT);  // this rig uses this digital pin to control DC power to the carrier osc buffer
  digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 

  pinMode(TX_SSB_MUTE_LINE, OUTPUT); // this rig uses D9 to mute the mic amp, goes low after SSB PTT
  digitalWrite(TX_SSB_MUTE_LINE, 0);  // mute the mic until ready to transmit SSB 
#endif

#ifdef SP_7
  Serial.println("SP_7 SSD");
  lcd.begin(16, 2);   
  lcd.cursor();
  lcd.noBlink();   
  lcd.print("SP Gnrc Bld 1.90");
  delay(1000); 
  lcd.setCursor(0,1);
  lcd.print("VK3HN 21/4/2020");
  delay(1000); 
  lcd.clear();

  pinMode(CO_DC_SUPPLY, OUTPUT);  // this rig uses this analog pin to control DC power to the carrier osc buffer
  digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 
  
#ifdef  CW_KEYER
  pinMode(PIN_PADDLE, INPUT_PULLUP);
  pinMode(PIN_PUSHBTTN_REAR, INPUT_PULLUP);
#endif  
#endif


#ifdef SP_8
  Serial.println("SP_8 ...");
#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_OLED_ADDRESS, RST_PIN);
#else // RST_PIN < 0
  oled.begin(&Adafruit128x64, I2C_OLED_ADDRESS);
#endif // RST_PIN >= 0
  oled.setFont(fixed_bold10x15);
  oled.clear();
  oled.println("   Summit     ");
  oled.println(" Prowler_VIII");
  oled.println(" < VK3HN > ");
  oled.println("R1.9 10/2020");
  delay(1000);
  oled.clear();

  pinMode(ENCODER_BUTTON, INPUT_PULLUP); // this rig uses ENCODER_BUTTON to read the encoder pushbutton for 'step' (radix) control
  pinMode(CO_DC_SUPPLY, OUTPUT);  // this rig uses this pin to control DC power to the carrier osc buffer (not sure this is correct?)
  digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 

  // this rig reads analog input BAND_SENSE_2M, if high, the internal 2m transverter is selected (28MHz otherwise)
  // (no initialisation to do here)
  
#ifdef  CW_KEYER
  pinMode(PIN_PADDLE, INPUT_PULLUP);
  pinMode(PIN_PUSHBTTN_REAR, INPUT_PULLUP);
#endif
#endif


#ifdef SP_9
  Serial.println("SP_9 ...");
#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_OLED_ADDRESS, RST_PIN);
#else // RST_PIN < 0
  oled.begin(&Adafruit128x64, I2C_OLED_ADDRESS);
#endif // RST_PIN >= 0
  oled.setFont(fixed_bold10x15);
  oled.clear();
  oled.println("   Summit     ");
  oled.println(" Prowler_9");
  oled.println(" < VK3HN > ");
  oled.println("R1.9 06/2020");
  delay(1000);
  oled.clear();

  pinMode(ENCODER_BUTTON, INPUT_PULLUP); // this rig uses ENCODER_BUTTON to read the encoder pushbutton for 'step' (radix) control
  pinMode(CO_DC_SUPPLY, OUTPUT);  // this rig uses this pin to control DC power to the carrier osc buffer
  digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 
  
#ifdef  CW_KEYER
  pinMode(PIN_PADDLE, INPUT_PULLUP);
  pinMode(PIN_PUSHBTTN_REAR, INPUT_PULLUP);
#endif
#endif

#ifdef SP_X
  Serial.println("SP_X");
#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_OLED_ADDRESS, RST_PIN);
#else // RST_PIN < 0
  oled.begin(&Adafruit128x64, I2C_OLED_ADDRESS);
#endif // RST_PIN >= 0
  
// #ifdef  CW_KEYER
  pinMode(PIN_PADDLE, INPUT_PULLUP);
  pinMode(PIN_PUSHBTTN_REAR, INPUT_PULLUP);
// #endif

// receiver muting during power-up
  pinMode(RX_MUTE_LINE, OUTPUT);  
  digitalWrite(RX_MUTE_LINE, RX_MUTE_ON_VALUE);     // mute the receiver
  
  pinMode(SP_X_MUTE_SPKR, OUTPUT);     // used as a second receiver mute relay controller 
  digitalWrite(SP_X_MUTE_SPKR, LOW);  // this line is normally high; low mutes the speaker line
  
// initialise the channel lines
  pinMode(8, OUTPUT);  
  pinMode(9, OUTPUT);  
  pinMode(10, OUTPUT);  
  pinMode(11, OUTPUT);  
  pinMode(12, OUTPUT);  
  pinMode(13, OUTPUT);  

// cycle thru the 6 channel LEDs on the front panel 
  for(int b=8; b<14; b++)
  {
    digitalWrite(b, 1); 
    delay(200);
    digitalWrite(b, 0);
  };
#endif

#ifdef SP_11
  Serial.println("SP_11");
  lcd.begin(16, 2);    
  lcd.print("SP Gnrc Bld 1.40");
  delay(1000); 
  lcd.setCursor(0,1);
  lcd.print("VK3HN 29/04/2021");
  delay(1000);   
  lcd.clear();   

  pinMode(PIN_PADDLE, INPUT_PULLUP);
  pinMode(PIN_PUSHBTTN_REAR, INPUT_PULLUP);
#endif

#ifdef SS_FAT5_160AM_TX
  Serial.println("SS_FAT5_160AM_TX");
  lcd.begin(16, 2);      
  lcd.cursor();
  lcd.noBlink();   
  lcd.print("SP Gnrc Bld 1.40");
  lcd.setCursor(0,1);
  lcd.print("SS_FAT5_160AM_TX");
  delay(1500); 
  lcd.setCursor(0,1);
  lcd.print("VK3HN 11/04/2019");
  delay(1000);  
  lcd.clear();

  pinMode(PWM_ENABLE_LINE, OUTPUT);
  digitalWrite(PWM_ENABLE_LINE, 0);     // disable the PWM (active low)   
#endif

#ifdef SS_VK3SJ_160AM_TX
  Serial.println("SS_VK3SJ_160AM_TX");
  lcd.begin(16, 2);      
  lcd.cursor();
  lcd.noBlink();   
  lcd.print("SP Gnrc Bld 1.41");
  lcd.setCursor(0,1);
  lcd.print("SS_VK3SJ_160AM_TX");
  delay(1500); 
  lcd.setCursor(0,1);
  lcd.print("VK3HN 01/01/2020");
  delay(1000);  
  lcd.clear();    

  byte c=0;
  while ((!tempsensor.begin()) && c<10) {
    Serial.println("MCP9808 err");
    lcd.print("MCP9808 err");
    delay(100);
    c++;
  };

  pinMode(PWM_ENABLE_LINE, OUTPUT);
  digitalWrite(PWM_ENABLE_LINE, 0);     // disable the PWM (active low)   
#endif


#ifdef SS_VK3SJ_40AM_TX
  Serial.println("SS_VK3SJ_40AM_TX");
  lcd.begin(16, 2);      
  lcd.cursor();
  lcd.noBlink();   
  lcd.print("SP Gnrc Bld 1.41");
  lcd.setCursor(0,1);
  lcd.print("SS_VK3SJ_40AM_TX");
  delay(1500); 
  lcd.setCursor(0,1);
  lcd.print("VK3HN 26/02/2020");
  delay(1000);  
  lcd.clear();
  pinMode(PWM_ENABLE_LINE, OUTPUT);
  digitalWrite(PWM_ENABLE_LINE, 0);     // disable the PWM (active low)   
#endif


#ifdef SS_AM_TX_TEST_VFO
  Serial.println("SS_AM_TX_TEST_VFO");
  lcd.begin(16, 2);      
  lcd.cursor();
  lcd.noBlink();   
  lcd.print("SP Gnrc Bld 1.40");
  lcd.setCursor(0,1);
  lcd.print("SS_AM_TX_TEST_VFO");
  delay(1500); 
  lcd.setCursor(0,1);
  lcd.print("VK3HN 06/03/2020");
  delay(1000);  
  lcd.clear();

  pinMode(PWM_ENABLE_LINE, OUTPUT);
  digitalWrite(PWM_ENABLE_LINE, 0);     // disable the PWM (active low)   
#endif

#ifdef SS_VK3SJ_160AM_PORTABLE_TX 
  Serial.println("SS_VK3SJ_160AM_PORTABLE_TX");
  lcd.begin(16, 2);      
  lcd.cursor();
  lcd.noBlink();   
  lcd.print("SP Gnrc Bld 1.40");
  lcd.setCursor(0,1);
  lcd.print("SS_VK3SJ_160AM_PORTABLE_TX");
  delay(1500); 
  lcd.setCursor(0,1);
  lcd.print("VK3HN 06/03/2020");
  delay(1000);  
  lcd.clear();

  pinMode(PWM_ENABLE_LINE, OUTPUT);
  digitalWrite(PWM_ENABLE_LINE, 0);     // disable the PWM (active low)   
#endif

#ifdef UNIVERSAL_VFO_CONTROLLER
  Serial.println("Universal ");
  lcd.begin(16, 2);   
  lcd.cursor();
  lcd.noBlink();   
  lcd.print("SP Gnrc Bld 1.90");
  delay(1000); 
  lcd.setCursor(0,1);
  lcd.print("VK3HN 21/9/2021");
  delay(1000); 
  lcd.clear();
    
#ifdef  CW_KEYER
  pinMode(PIN_PADDLE, INPUT_PULLUP);
  pinMode(PIN_PUSHBTTN_REAR, INPUT_PULLUP);
#endif  
#endif




// reset the Band/Low Pass Filter controllers
  for (byte i=0; i<8 ; i++) 
  {
    PCF_BPF.write(i, 0); //turn all the BPF relays off 
    PCF_LPF.write(i, 0); //turn all the LPF relays off 
//    Serial.print(" 0: ");   Serial.println(PCF_LPF.lastError());
  }

// set digital and analogue pins
  pinMode(SWITCH_BANK, INPUT_PULLUP); // switch bank is Pull-up
  pinMode(PTT_SENSE, INPUT_PULLUP);  // senses the PTT switch in the microphone, normally open, grounded when PTT pressed
  
  pinMode(RX_MUTE_LINE, OUTPUT); 
  digitalWrite(RX_MUTE_LINE, RX_MUTE_OFF_VALUE);     // initialise mute line to 'off' value (un-mute)  
  
  pinMode(TRANSMIT_LINE, OUTPUT); 
  digitalWrite(TRANSMIT_LINE, 0); // put the transmit line low (relay not energised)

// start with transmit line low (in receive) 
  mode_tx = false;
  
  PCICR |= (1 << PCIE2);           // Enable pin change interrupt for the encoder
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
  
  // load up VFOSet array from EEPROM

  v = EEPROM.read(0);
  Serial.print("setup() eeprom: v=");
  Serial.println(v);
  if(v >= NBR_VFOS) v = 1;  // in case NBR_VFOS has been reduced since the last run (EEPROM update)
  v_prev = v;
  
  int element_len = sizeof(VFOset_type);
  for(int i=0; i < NBR_VFOS; i++)
  {
    EEPROM.get(1 + (i * element_len), VFOSet[i]);
  };

/* // initialise VFOSet array
  for(int n=0; n<NBR_VFOS; n++) VFOSet[n].active = 0;   // make sure all are inactive to start with 
  VFOSet[0] = (VFOset_type){1,  1825000ULL, 1000};
  VFOSet[1] = (VFOset_type){1,  3525000ULL, 1000};
  VFOSet[2] = (VFOset_type){1,  3625000ULL, 1000};
  VFOSet[3] = (VFOset_type){1,  7025000ULL,  100};
  VFOSet[4] = (VFOset_type){1,  7090000ULL, 1000};
  VFOSet[5] = (VFOset_type){1,  7125000ULL, 1000};
  VFOSet[6] = (VFOset_type){1, 10105000ULL,  100};
  VFOSet[7] = (VFOset_type){1, 14060000ULL, 1000};
  VFOSet[8] = (VFOset_type){1, 18068000ULL, 1000};
  VFOSet[9] = (VFOset_type){1, 18068000ULL, 1000};
*/
// dump out the VFOset array for diagnostics
  for(int n=0; n < NBR_VFOS ; n++)
  {
    Serial.print((int)VFOSet[n].active);
    Serial.print(" ");
    Serial.print(VFOSet[n].vfo);
    Serial.print(" ");
    Serial.print((long)VFOSet[n].radix);
    Serial.println();
  }
  
// initialise and start the si5351 clocks

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0); // If using 27Mhz xtal, put 27000000 instead of 0 (0 is the default xtal freq of 25Mhz)
  
#ifdef SP_V
  si5351.set_correction(15500);    // Library update 26/4/2020: requires destination register address  ... si5351.set_correction(19100, SI5351_PLL_INPUT_XO);
#endif 
 
#ifdef SP_6
  si5351.set_correction(19100);    // to determine the correction using the serial monitor
#endif  

#ifdef SP_7
  si5351.set_correction(19100);    // to determine the correction using the serial monitor
#endif  

#ifdef SP_8
  si5351.set_correction(34000);    // calibrated 18/06/2020
#endif 

#ifdef SP_9
  si5351.set_correction(33000);    // calibrated 23/11/2020
#endif 

#ifdef SP_X
  si5351.set_correction(28000);    // calibrated 25/01/2021
#endif 

#ifdef SS_FAT5_160AM_TX
 si5351.set_correction(160000);    // for FAT5  
#endif  

#ifdef SS_VK3SJ_160AM_TX 
  si5351.set_correction(5000);     // tune this
#endif 

#ifdef SS_VK3SJ_40AM_TX 
  si5351.set_correction(5000);     // tune this
#endif 

#ifdef SS_AM_TX_TEST_VFO 
  si5351.set_correction(18800);    // calibrated 3/11/2019 
#endif 

#ifdef SS_VK3SJ_160AM_PORTABLE_TX  
  si5351.set_correction(18800);    // calibrated ?? 
#endif 

#ifdef UNIVERSAL_VFO_CONTROLLER
  si5351.set_correction(19100);    // 
#endif


                                        
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  
  // choose a high or low BFO frequency
  if(VFOSet[v].vfo >= SIDEBAND_THRESHOLD) 
    bfo = USB;
  else 
    bfo = LSB;

// VFO
  Serial.print("VFO=");    Serial.println(VFOSet[v].vfo + bfo);  
  
#if defined(SS_FAT5_160AM_TX) || defined(SS_VK3SJ_160AM_TX) || defined (SS_VK3SJ_40AM_TX) || defined(SS_VK3SJ_160AM_PORTABLE_TX)
  // nothing
#else
  {
    si5351.set_freq((VFOSet[v].vfo + bfo) * SI5351_FREQ_MULT, SI5351_CLK0); // set CLK0 to VFO freq for current band 
    if(VFOSet[v].vfo < VFO_DRIVE_THRESHOLD)
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); 
    else
    {
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_4MA); 
    }

    si5351.output_enable(SI5351_CLK0, 1);  // turn VFO on 
  }
#endif

#if defined(SS_FAT5_160AM_TX) || defined(SS_VK3SJ_160AM_TX) || defined(SS_VK3SJ_40AM_TX)
  si5351.output_enable(SI5351_CLK0, 0);  // turn VFO off 
  si5351.output_enable(SI5351_CLK1, 0);  // turn CO off 
  si5351.output_enable(SI5351_CLK2, 0);  // turn BFO off 
#endif

#if defined(SS_AM_TX_TEST_VFO)
  si5351.output_enable(SI5351_CLK1, 0);  // turn VFO off 
  si5351.output_enable(SI5351_CLK2, 0);  // turn BFO off 
#endif

#if defined(SS_VK3SJ_160AM_PORTABLE_TX)
    si5351.output_enable(SI5351_CLK0, 0);  // transmit clock
    
    si5351.set_freq((VFOSet[v].vfo + bfo) * SI5351_FREQ_MULT, SI5351_CLK2); // set CLK2 to VFO for the inbuilt AM receiver  
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); 
    si5351.output_enable(SI5351_CLK2, 1);  

    si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK1);  // init the BFO
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); 
    si5351.output_enable(SI5351_CLK1, 0);  // start the receiver in AM mode (no BFO) 
#endif


// BFO
#ifdef BFO_ENABLED  
  si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK2); // set CLK2 to the BFO frequency for the current band
  si5351.drive_strength(SI5351_CLK2,SI5351_DRIVE_2MA);  // can change this for a stronger BFO
#endif

  changed_f = true; 
  last_freq_change_ms = millis(); 
  eeprom_written_since_last_freq_change = false; 
  LPF_engaged = false;

#ifdef SS_EI9GQ
// initialise the Real Time Clock
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
  }
   // may or may not be needed for the RTC to work! 
   if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
#endif

#ifdef SP_X
  // turn on the LED on the current channel
  digitalWrite(v+8, 1); 
  delay(200);
#endif

#ifdef CW_KEYER
// start of CW Keyer block -----------------------------------------------------------
  key_down = false;  
  char_sent_ms = millis();
  space_inserted = false;
  message_playing = false;
  curr_msg_nbr = 0;
  cw_msg_index = 0;
  dot_dash_counter = 0;

  // dump out the MorseCode table for diagnostic
/*  for(int i=0; i<40; i++)
  {
    Serial.print(MorseCode[i].ch[0]);
    Serial.print(' ');
    for(int j=1; j<7; j++)
      Serial.print(MorseCode[i].ch[j]);
    Serial.println();
  }

  // play the two messages as a diagnostic
  //  play_message(morse_msg[0], 0);
  //  delay(2000);
  //  play_message(morse_msg[1], 0);
  //  delay(2000);
  */
#endif
// end of CW Keyer block -------------------------------------------------------------

}


// ------------------------------------------------------------------------------------------------------------------
// loop()

void loop()
{
    update_eeprom(); 
#ifdef DISPLAY_LCD
    refresh_display();  
#endif
#ifdef DISPLAY_OLED
    if(!mode_cw) refresh_display();  // to get around slow OLED refresh slowing down CW sending!
#endif

   
#ifdef VSWR_METER
//-- VSWR meter code begins -------------------------------------------
  fwd_max=0; // resets peak counters each time through loop (this timing may need to change)
  rev_max=0; 
//-- VSWR meter code ends ---------------------------------------------
#endif

  // Update the display if the frequency has been changed (and we are not transmitting)
  if (!mode_tx and changed_f)
  {
    volatile uint32_t f; 
    bool bfo_change = false; 
    
    f = VFOSet[v].vfo; 
#if defined(SS_FAT5_160AM_TX) || defined(SS_VK3SJ_160AM_TX) || defined (SS_VK3SJ_40AM_TX) || defined(SS_VK3SJ_160AM_PORTABLE_TX)
    // these transmitters have no receiver so do nothing
#else
    si5351.set_freq((bfo + f) * SI5351_FREQ_MULT, SI5351_CLK0);  
#endif

#ifdef SS_VK3SJ_160AM_PORTABLE_TX
    si5351.set_freq((bfo + f) * SI5351_FREQ_MULT, SI5351_CLK2);  // this project has an AM receiver so set the VFO on CLK2
#endif 

    // BFO logic...
    if (f >= SIDEBAND_THRESHOLD )  // f >= 10MHz so use USB 
    {
      if(bfo == LSB) // need to change BFO
      {
        bfo = USB;
        bfo_change = true; 
      }
    }
    else  // f < 10MHz 
    {
      if(bfo == USB) // need to change BFO
      {
        bfo = LSB;
        bfo_change = true; 
      }
    };
    if(bfo_change || BFO_tune_flg) 
    {
      // for rigs with an external crystal BFO, need to send a pin high to flip the crystal BFO
      // ...
#ifdef BFO_ENABLED
      si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK2); 
#endif
    }

    // make sure the right BPF/LPF filters are selected
    set_filters(f);
    
    //refresh_display();
    changed_f = false;
    last_freq_change_ms = millis(); 
    eeprom_written_since_last_freq_change = false;
  } // endif changed_f

#ifdef SP_X
  SP_X_counter++;
  if(SP_X_counter > 20){
    SP_X_counter=0;
//  if(SP_X_init){
    Serial.println("SP_X_init");
//      digitalWrite(RX_MUTE_LINE, RX_MUTE_OFF_VALUE);     // un-mute the receiver
      // let the series speaker relay drop out (this silences a switch-on thump)
//      delay(100);
//      digitalWrite(SP_X_MUTE_SPKR, LOW);  // mute the speaker line
//            delay(200);
//      digitalWrite(SP_X_MUTE_SPKR, HIGH);  // un-mute the speaker line


//      digitalWrite(RX_MUTE_LINE, RX_MUTE_ON_VALUE);     // mute the receiver
//      delay(400);
//      digitalWrite(RX_MUTE_LINE, RX_MUTE_OFF_VALUE);     // un-mute the receiver


      SP_X_init = false;
  }
#endif

  // sense the PTT switch, paddle and key lines
  if(!mode_cw)  // CW manages T/R for itself
  {
    if(!digitalRead(PTT_SENSE) && !mode_tx) 
    {
      receive_to_TRANSMIT(); // PTT button pressed
      //refresh_display();
    };
//    delay(10);
    if(digitalRead(PTT_SENSE)  &&  mode_tx) 
    {
      TRANSMIT_to_receive(); // PTT button released
      //refresh_display();
    }
  }

  // see if the rig has been idle long enough to drop out the LPF relay...
  if( (!mode_tx) && (millis() - last_T_R_ms) > LPF_DROPOUT_DELAY )
  {
//    Serial.print("-drop LPF");     Serial.println(millis()); 
    for (byte i=0; i<8; i++) PCF_LPF.write(i, 0); // easy way (just turn them all off) 
    LPF_engaged = false;
    last_T_R_ms = millis(); 

    // stop the fan...
    
  }

  
  //------------------------------------------------------------------------
  // if any of the buttons have been pressed...
  old_button_nbr = 0;  // clear the last command 
  bool button_held = false; 
  unsigned long button_pressed_ms = millis();
  
  button_nbr = get_front_panel_button();
  byte first_button_nbr = 0;
  while (button_nbr > 0)
  {
  // one of the multiplexed switches is being held down
    delay(5);  // was 20
    if(first_button_nbr == 0) first_button_nbr = button_nbr;
    old_button_nbr = button_nbr;
    button_nbr = get_front_panel_button();
  }

  button_nbr = first_button_nbr; // experiment to accept the first reading
  
  if((millis() - button_pressed_ms) >= BUTTON_HELD_MS) button_held = true;
    
 // if one of the buttons was pressed (and is now released) act on it...
 
  if (button_nbr == 1)
  {
      if(!func_button_pressed)
      {
          Serial.println("<B1>VFO down");
          if(v == 0) v = (NBR_VFOS-1);
          else v--;
      }
      else
      {
          Serial.println("<F><B1>N/A");
          func_button_pressed = false;
      };
    changed_f = true;
  };

  if (button_nbr == 2) 
  {
    // Button 2:  
#ifdef SS_EI9GQ
    // implement 'prev VFO' feature...
    byte v_tmp = v;
    v = v_prev;
    v_prev = v_tmp;
#endif
  }; 

  if (button_nbr == 3) {};  

  if (button_nbr == 4)
  {
      if(!func_button_pressed)
      {
             if(button_held) 
             {
               Serial.println("<B4>held-Tune"); 
               button_held = false;
#ifdef SP_X
               // reset channel freq
               reset_channel_freq(); 
               changed_f = true;
#else
               // default behaviour us tune
               mode_tune = true;
               tune();
#endif
               
             }
             else
             {
               Serial.println("<B4>VFO up"); 
//               Serial.print("B4 v="); Serial.print(v); Serial.print(" VFO="); Serial.println(VFOSet[v].vfo);
               int v_prev = v; 
               if(v == (NBR_VFOS-1)) v = 0; else v++;

//               Serial.print("Aft v="); Serial.print(v); Serial.print(" VFO="); Serial.println(VFOSet[v].vfo);


#ifdef SP_X
               // advance the channel LED
               digitalWrite(v_prev+8, 0); 
               digitalWrite(v+8, 1); 
#endif

               
/*               // SP_8
               // in SP_8 there are two banks of VFOs, one for 28MHz, one for 144MHz
               // VFOs in positions 0..(NBR_VFOS-1) are for 28MHz
               // VFOs in positions NBR_VFOS..((2*NBR_VFOS)-1) are for 144MHz
               if(sp8_band_2m)
               {
                 // SP_8 is in 2m mode so use the high range
                 byte v2m = 2*NBR_VFOS - 1; 
                 if(v == v2m) v = NBR_VFOS; else v++;
               } 
               else
               {
                 // SP_8 is in 10m mode so use the normal (low) range
                 if(v == (NBR_VFOS-1)) v = 0; else v++;
               }
#endelse
*/
             }
      }
      else
      {
         // BFO Tune toggle
         Serial.println("<F><B4>BFOTune");
         BFO_tune_flg = !BFO_tune_flg;
#ifdef DISPLAY_LCD
         lcd.clear(); 
#endif
         func_button_pressed = false;
       }
    changed_f = true;
  }

  if (button_nbr == 5) 
  {
    Serial.println("<B5>Fn tgl");
    func_button_pressed = !func_button_pressed;   
    if(func_button_pressed) Serial.println("Function...");
    changed_f = true;

#ifdef SS_VK3SJ_160AM_PORTABLE_TX
    if(func_button_pressed) 
      si5351.output_enable(SI5351_CLK1, 1);  // enable the BFO on CLK1
    else
      si5351.output_enable(SI5351_CLK1, 0);  // disable the BFO on CLK1
#endif  
  }

  if (button_nbr == 6)
  // Button 6: change frequency step up
  // Fn: tbd
  {
    if(!func_button_pressed)
       Serial.println("<B6>f step r");
    else
       Serial.println("<F><B6>f step l");

    if(button_held)
    {
      Serial.println("<B6>held -- toggle IF filters"); 
    }
    else
    {
#if defined(SP_V) or defined (SP_6) or defined (SP_8) or defined (SP_9) or defined (SP_11)
      switch (VFOSet[v].radix)
      {
        case 10:
        {
           VFOSet[v].radix = 100;
#ifdef SP_11
           // clear residual 10Hz frequency component from the active VFO         
           uint16_t f = VFOSet[v].vfo % 100;
           VFOSet[v].vfo -= f;
#endif // SP_11
        }
        break;

#ifdef SP_11
        case 100:
        {
           VFOSet[v].radix = 10;
        }
        break;
#else 
        case 100:
        {
           VFOSet[v].radix = 1000;
           // clear residual < 1kHz frequency component from the active VFO         
           uint16_t f = VFOSet[v].vfo % 1000;
#ifdef SP_8
           if(sp8_band_2m) f = (VFOSet[v].vfo + SP_8_DIAL_OFFSET) % 1000;
#endif
           VFOSet[v].vfo -= f;
        }
        break;
#endif


        case 1000:
        {
           VFOSet[v].radix = 100;
        }
        break;
      
        case 10000:
        {
           VFOSet[v].radix = 1000;
        }
        break;
      } 
#else
      // default radfix increment/decrement behaviour...
      switch (VFOSet[v].radix)
      {
        case 10:
        {
            if(!func_button_pressed)
            {
                // change radix up
                VFOSet[v].radix = 10000;
                // clear residual < 1kHz frequency component from the active VFO         
                uint16_t f = VFOSet[v].vfo % 1000;
                VFOSet[v].vfo -= f;
            }
            else
            {
               func_button_pressed = false;
               // change radix down
               VFOSet[v].radix = 100;
               // clear residual < 100Hz frequency component from the active VFO         
               uint16_t f = VFOSet[v].vfo % 100;
               VFOSet[v].vfo -= f;
            }
        }
        break;
  
        case 100:
        {
            if(!func_button_pressed)
            {
                VFOSet[v].radix = 10;
            }
            else
            {
                func_button_pressed = false;
                VFOSet[v].radix = 1000;
                // clear residual < 1kHz frequency component from the active VFO         
                uint16_t f = VFOSet[v].vfo % 1000;
                VFOSet[v].vfo -= f;
            }
        }
        break;
        
        case 1000:
        {
            if(!func_button_pressed)
            {
                VFOSet[v].radix = 100;
            }
            else
            {
               func_button_pressed = false;
               VFOSet[v].radix = 10000;            
            }
            break;
        }
      
        case 10000:
        {
            if(!func_button_pressed)
            {
                VFOSet[v].radix = 1000;
            }
            else
            {
                func_button_pressed = false;
                VFOSet[v].radix = 10;
            }
            break;
        }
      }
#endif
    } // else
    changed_f = true;
  }

#ifdef SP_X
  if (button_nbr == 8)
  {
    // change channel up
    VFOSet[v].vfo += VFOSet[v].radix;
    Serial.print("<B8>up "); Serial.println(VFOSet[v].vfo);
    changed_f = true;    
  };
  if (button_nbr == 9)
  {
    // change channel down
    VFOSet[v].vfo -= VFOSet[v].radix;    
    Serial.print("<B9>dn "); Serial.println(VFOSet[v].vfo);
    changed_f = true;    
  }
#endif

#ifdef CW_KEYER
// start of CW Keyer block -----------------------------------------------------------
//  read_keyer_speed(); // read the speed control 
// see if a memory button has been pushed
  if((curr_msg_nbr = check_keyer_pushbutton()) > 0)
  {
    mode_cw = true; 
    activate_state2('T'); 
    refresh_display();  // to show 'CW' on the display  
    byte msg_speed=0;
#if defined(SP_IV) || defined(SP_V) || defined(SP_6) || defined(SP_7)  || defined(SP_8) || defined(SP_9) || defined(SP_X) || defined(UNIVERSAL_VFO_CONTROLLER)
    msg_speed = KEYER_MSG_SPEED;   // this plays the canned keyer messages faster than the paddle, to save time on cold summits
#endif
    play_message(morse_msg[curr_msg_nbr-1], msg_speed);  
    delay(5); 
    activate_state2('R');  
  };

  // see if the paddle is being pressed
  byte j = check_paddle();
  if((j==PADDLE_L) or (j==PADDLE_R)) 
  {
    mode_cw = true;
    activate_state2('T'); 
#ifdef DISPLAY_OLED 
#ifndef SP_X
    if(dot_dash_sent == 3) refresh_pwr_meter();  // only want to do this first time, as I2C OLED refresh is *slow*
#endif 
#endif 
    if(j==PADDLE_L) send_dot();
    if(j==PADDLE_R) send_dash();
 //   Serial.print("loop()::dot_length_ms:");   Serial.println(dot_length_ms);
  };

  curr_ms = millis();
  // if paddle has been idle for BREAK_IN_DELAY drop out of transmit 
  if(mode_cw and mode_tx and ((curr_ms - char_sent_ms) > BREAK_IN_DELAY))
  {
//    Serial.print("curr_ms="); Serial.print(curr_ms); 
//    Serial.print("  char_sent_ms="); Serial.print(char_sent_ms); 
//    Serial.print("  curr_ms - char_sent_ms="); Serial.println(curr_ms - char_sent_ms); 
    // drop back to receive to implement break-in
    activate_state2('R');  
  }
// end of CW Keyer block -------------------------------------------------------------
#endif

#ifdef VSWR_METER
  //-- VSWR meter code begins -------------------------------------------
  int n; 
//  n = read_analogue_pin(PIN_SWR_FWD);

  n = analogRead(PIN_SWR_FWD);
//  Serial.print("      fwd=");   Serial.println(n);
  if(n > fwd_max) 
  {
    fwd_max = n;     
//    Serial.print("     F=");   Serial.println(fwd_max);
  }
  n = analogRead(PIN_SWR_REV);
//  Serial.print("      rev=");   Serial.println(n);
  if(n > rev_max) 
  {
    rev_max = n;
//    Serial.print("     R=");   Serial.println(rev_max);
  }
  //-- VSWR meter code ends ---------------------------------------------
#endif

#ifdef SS_VK3SJ_160AM_PORTABLE_TX
  // this transceiver does HT voltage and current sensing and displays on the LCD
  // read the HT voltage
  int val1 = analogRead(PIN_VOLTMETER);
  // map(value, fromLow, fromHigh, toLow, toHigh)
  HT_v = map((int)(float(val1)/1.0), 0, 1023, 0, 80);  // map s-meter analogue reading to 0..80V, more or less 

  // read the HT current (going to need to average these readings) 
  int val2 = analogRead(PIN_AMMETER);
  // map(value, fromLow, fromHigh, toLow, toHigh)
  HT_a = map((int)(float(val2)/1.0), 0, 1023, 0, 50);  // map ammeter analogue reading to 0..50 (x10A)  
  HT_a_f = float(HT_a)/10.0;

//  Serial.print("A6 val1=");   Serial.print(val1);  Serial.print(" HT_v=");  Serial.print(HT_v);
//  Serial.print("      A7 val2=");   Serial.print(val2);  Serial.print(" HT_a=");  Serial.println(HT_a_f);
#endif
}
