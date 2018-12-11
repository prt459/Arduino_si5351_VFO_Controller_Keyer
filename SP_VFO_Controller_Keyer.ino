/*
Arduino Nano script for homebrew multiband SSB/CW transceiver. 
Based on source from Przemek Sadowski, SQ9NJE, with si5351 lib from Jason Mildrum NT7S
Targets VU2ESE's Nano/si5351 module (Raduino). 

VK3HN, 20 June 2017.

Test:
- code for i2c demux for 6 band lines
- automatic sideband selection algorithm and USB/LSB BFO tuning 
Todo:
- detection of external controls (key down, PTT-sense)
- software keyer
- CW on key-down
- (optional) IF filter selection code
- (optional) ND6T power/SWR meter code
- (optional) s-meter code 
*/

#include <Rotary.h>
#include <si5351.h>
#include <pcf8574.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>

#define F_MAX        5000000000UL  // Upper frequency limit (50MHz)
#define F_MIN         100000000UL  // Lower frequency limit  (1MHz)
#define BFO_TUNE_LO   11990000ULL  // lowest BFO frequency
#define BFO_TUNE_HI   12005000ULL  // highest BFO frequency

// Arduino Nano digital pin assignments (aligns with Raduino)
//                0     Serial
//                1     Serial
#define ENCODER_B 2  // Encoder pin B
#define ENCODER_A 3  // Encoder pin A
#define PTT_SENSE 4  // sense the PTT button being pressed
#define KEY       5  // key sense
#define KEYER_L   6  // keyer paddle left
#define KEYER_R   7  // keyer paddle right
#define LCD_RS	  8  // Register Select is LCD pin 4 
#define LCD_E	    9  // Enable/clock LCD pin 6
#define LCD_D4	 10  // LCD D4 is LCD pin 11
#define LCD_D5	 11  // LCD D5 is LCD pin 12
#define LCD_D6	 12  // LCD D6 is LCD pin 13
#define LCD_D7	 13  // LCD D7 is LCD pin 14 

// Arduino Nano analogue pins
#define SWITCH_BANK A0 // string of multiplexed push buttons
#define BAND_1      A1 // band 1
#define BAND_2      A2 // band 2
#define BAND_3      A3 // band 3
//                  A4    SDA
//                  A5    SCL
#define SUPPLY_V    A6 // battery/supply voltage (via resistive divider)
//                  A7    not assigned

// i2c devices and addresses:
// si5351  x60
// INA219 voltage and current sampler x40
// band selector PCF8574 x20
// Temperature TBD

// VFO indexes and strings
#define NBR_VFOS    10 // number of selectable VFOs 

// frequency ranges for automatic band pass and low pass filter switching
#define FILTER_160_LB  1800000ULL  // Filter set lower bound (MHz)
#define FILTER_160_UB  2000000ULL  // Filter set upper bound (MHz)

#define FILTER_80_LB  3500000ULL  // Filter set lower bound (MHz)
#define FILTER_80_UB  3900000ULL  // Filter set upper bound (MHz)

#define FILTER_60_LB  5300000ULL  // Filter set lower bound (MHz)
#define FILTER_60_UB  5600000ULL  // Filter set upper bound (MHz)

#define FILTER_40_LB  7000000ULL  // Filter set lower bound (MHz)
#define FILTER_40_UB  7300000ULL  // Filter set upper bound (MHz)

#define FILTER_30_LB 10100000ULL  // Filter set lower bound (MHz)
#define FILTER_30_UB 10150000ULL  // Filter set upper bound (MHz)

#define FILTER_20_LB 14000000ULL  // Filter set lower bound (MHz)
#define FILTER_20_UB 14350000ULL  // Filter set upper bound (MHz)

#define FILTER_17_LB 18068000ULL  // Filter set lower bound (MHz)
#define FILTER_17_UB 18168000ULL  // Filter set upper bound (MHz)

#define I2C_DEMUX   0x20         // I2C address of the PCF8574 (0x20)
PCF8574 PCF_20(I2C_DEMUX);  


byte curr_line = 0;    // the currently selected filter control line

// struct for 'VFO parameter set' records -- the parameters that will change with each VFO
typedef struct {
  boolean  active;
  uint32_t vfo;
  uint32_t radix;
} VFOset_type;

VFOset_type VFOSet[NBR_VFOS]; // array of band parameter sets
byte v;                       // index into VFOSet array (representing the current VFO)

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); 

Si5351 si5351;
Rotary r = Rotary(ENCODER_A, ENCODER_B);

#define SIDEBAND_THRESHOLD  10000000ULL  // threshold VFO freq for auto sideband selection: above use USB, below use LSB
volatile uint32_t USB = 11998500ULL;  // the reference BFO freq for LSB, may be tuned, put in EEPROM once
volatile uint32_t LSB = 11995500ULL;  // the reference BFO freq for USB, may be tuned, put in EEPROM once
volatile uint32_t bfo = LSB; // the actual BFO freq for si5351 CLK2, arbitrary set to LSB, reset in main loop  

String curr_rx_tx = "RX";

// button variables used in main loop for sensing the multiplexed buttons
byte button_nbr; 
byte old_button_nbr = 0; 

// S meter reading
int s_meter_reading=256; // set to about s3

// Function button pressed, if true, the next button pressed is interpreted as a Function
bool func_button_pressed = false; 
bool BFO_tune_flg = false; // BFO Tune feature
byte dial_tick=0, dial_speed=8;  // dial speed

boolean changed_f = 0;

/**************************************/
/* Interrupt service routine for      */
/* encoder frequency change           */
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
  if(++dial_tick%dial_speed != 0) return; 

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
}


int get_pressed_button()
// Take a single reading of the buttons and map it from the A0 value to a button number (0..6)
// (if the switches produce any spurious readings, you can take 10 readings and average them)
{
  int b, i, sum, z;
  sum = 0;
  for (i=0; i<4; i++)
  {
    sum += analogRead(SWITCH_BANK);    // read A0 
  }
  z = sum/4;
/*
  if(z<1000)
  {
    Serial.print("Button value=");
    Serial.println(z);
  }
*/  
  if(z > 900) b = 0;                  // 1008-1017
  else if(z > 80 && z < 99)   b = 1;    // 90
  else if(z > 100 && z < 120) b = 2;  // 112 
  else if(z > 120 && z < 150) b = 3;  // 134
  else if(z > 55 && z < 76)   b = 4;    // 66 
  else if(z > 30  && z < 55)  b = 5;  // 41 
  else if(z > 5   && z < 30)  b = 6;  // 14 
  else b = 0;

  if(b>0){
    Serial.print("Button=");
    Serial.println(b);
  }
  
  return b;  
}


String read_meter()
// analogue port returns 0..1023
// s-meter string is 8 chars... 
{
  String s = "";
  int i, j, sum=0, z;
  int s_point_val[8] = {0, 128, 256, 384, 512, 640,  768,  896};
  //                    s1 s3   s5   s7   s9   s9+20 s9+40 s9+60
  
  // take an averaged reading from the analogue pin with the s-meter on it
  for (i=0; i<4; i++)
  {
    // sum += analogRead(S_METER);    // read A?
  }
  z = sum/4;

  z = s_meter_reading; // temp to try out a moving s-meter

  for (j=0; j<8; j++)
  {
    if(z > s_point_val[j])
    {
      // write in a visible char to the s-meter bar
      if(j<=4)
        s += (char)9;    // for s1 - s9
      else
        s += (char)10;   // for s9+  
    }
    else
      s += ' ';     // write in a blank
  }
//  Serial.println(sizeof(s));
  return s;
}


void display_frequency()
// Display the frequency
{
  uint16_t f, g;
  uint32_t vfo_l; 

  lcd.setCursor(0, 0);
  if(!BFO_tune_flg)
  {
    lcd.print(curr_rx_tx);
    lcd.print(" ");
    lcd.print(v+1);
    lcd.print(" ");
    vfo_l = VFOSet[v].vfo; 
  }
  else
  {
    lcd.print("BFO>");
    lcd.setCursor(13, 0);
    vfo_l = bfo;      
  }

  lcd.setCursor(6, 0);
  f = vfo_l / 1000000; 	
  if (f < 10)
    lcd.print(' ');
  lcd.print(f);
  lcd.print('.');
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
  
  String s_meter = read_meter();  // fix this at 8 chars
  lcd.setCursor(10, 1);
  lcd.print(s_meter);

  switch (VFOSet[v].radix)
  {
    case 10:
      lcd.setCursor(14, 0);
    break;
      
    case 100:
      lcd.setCursor(13, 0);
    break;
      
    case 1000:
      lcd.setCursor(11, 0);
    break;
      
    case 10000:
      lcd.setCursor(10, 0);
    break;
  }
}

void set_filters(uint32_t f)
{
    // select the appropriate filter set for the frequency passed in
    byte line=0;
    if     ((f >= FILTER_160_LB) && (f <= FILTER_160_UB)) line = 1;
    else if((f >= FILTER_80_LB) && (f <= FILTER_80_UB))   line = 2;
    else if((f >= FILTER_60_LB) && (f <= FILTER_60_UB))   line = 0;
    else if((f >= FILTER_40_LB) && (f <= FILTER_40_UB))   line = 3;
    else if((f >= FILTER_30_LB) && (f <= FILTER_30_UB))   line = 4;
    else if((f >= FILTER_20_LB) && (f <= FILTER_20_UB))   line = 5;
    else if((f >= FILTER_17_LB) && (f <= FILTER_17_UB))   line = 6;

    if(line != curr_line)
    {
        // *** raise the appropriate control line 
        // (where 0 means no line which will implement 'out of band' protection
        // *** write to i2c demux

        for (int i=0; i<7; i++) PCF_20.write(i, 0); //turn all pins of the I/O expander off 
        delay(100);
        if(line > 0){
           PCF_20.write(line-1, 1);  //turn the band-specific pins of the I/O expander on 
           Serial.print("Changed band to: ");
           Serial.println(line);
           delay(100); 
        }
        curr_line = line;  
    }
}


void setup()
{
  Serial.begin(9600);  
  lcd.begin(16, 2);  // Initialize and clear the LCD
  lcd.print("SummitProwler IV");
  lcd.setCursor(0,0);
  lcd.print("VK3HN 1.0 8/8/17");
  //                            
  lcd.cursor();
  lcd.noBlink();
  lcd.setCursor(1,1);  // (col, line)
  delay(2000); 
  lcd.clear();
  Wire.begin();

//  pinMode(ENCODER_BTN, INPUT_PULLUP);
  pinMode(SWITCH_BANK, INPUT_PULLUP);  // switch bank is Pull-up
  
  PCICR |= (1 << PCIE2);           // Enable pin change interrupt for the encoder
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
  
  // load up VFOSet array from EEPROM
/*
  v = EEPROM.read(0);
  Serial.print("setup() from eeprom: v=");
  Serial.println(v);
  
  int element_len = sizeof(VFOset_type);
  for(int i=0; i < NBR_VFOS; i++)
  {
    EEPROM.get(1 + (i * element_len), VFOSet[i]);
  };
*/
 // initialise VFOSet array
  for(int n=0; n<NBR_VFOS; n++) VFOSet[n].active = 0;   // make sure all are inactive to start with 
  VFOSet[0] = (VFOset_type){1,  1825000ULL, 1000};
  VFOSet[1] = (VFOset_type){1,  3525000ULL, 1000};
  VFOSet[2] = (VFOset_type){1,  3625000ULL, 1000};
  VFOSet[3] = (VFOset_type){1,  5350000ULL,  100};
  VFOSet[4] = (VFOset_type){1,  7020000ULL,  100};
  VFOSet[5] = (VFOset_type){1,  7090000ULL,  100};
  VFOSet[6] = (VFOset_type){1, 10105000ULL,  100};
  VFOSet[7] = (VFOset_type){1, 14060000ULL, 1000};
  VFOSet[8] = (VFOset_type){1, 18068000ULL, 1000};
  VFOSet[9] = (VFOset_type){1, 18068000ULL, 1000};

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
  
  v = 5;  // set the selected VFO (this will be read back from EEPROM later...
  
  // initialise and start the si5351 clocks
  // si5351.set_correction(140); // use File/Examples/si5351Arduino-Jason
  si5351.set_correction(1);      // to determine the correction using the serial monitor

  //initialize the Si5351
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0); //If using 27Mhz xtal, put 27000000 instead of 0
                                              // 0 is the default xtal freq of 25Mhz
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);

  volatile uint32_t f; 
  f = VFOSet[v].vfo; 
  
  // choose a high or low BFO frequency
  if(f >= SIDEBAND_THRESHOLD) bfo = USB;
  else bfo = LSB;

  // set CLK0 to the VFO frequency for the current band (fsig + fbfo)
  si5351.set_freq((bfo + f) * SI5351_FREQ_MULT, SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_2MA); 

  // set CLK2 to the BFO frequency for the current band
  si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK2);
  si5351.drive_strength(SI5351_CLK2,SI5351_DRIVE_2MA); 

//  display_frequency();  // Update the display
  changed_f = true; 
}


void loop()
{
  // Update the display if the frequency has been changed
  if (changed_f)
  {
    volatile uint32_t f; 
    bool bfo_change = false; 
    
    f = VFOSet[v].vfo; 
    si5351.set_freq((bfo + f) * SI5351_FREQ_MULT, SI5351_CLK0);  

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
    if(bfo_change || BFO_tune_flg) si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK2); 

    // make sure the right BPF/LPF filters are selected
    set_filters(f);
    
    display_frequency();
    changed_f = 0;
  } // endif changed_f

  //------------------------------------------------------------------------
  // if any of the buttons have been pressed...
  old_button_nbr = 0;  // clear the last command 
  button_nbr = get_pressed_button();
  while (button_nbr > 0)
  {
  // one of the multiplexed switches is being held down
    delay(100);  // was 20
    old_button_nbr = button_nbr;
    button_nbr = get_pressed_button();
  }

  button_nbr = old_button_nbr;
 // if one of the buttons was pressed (and is now released) act on it...
  if (button_nbr == 1)
  {
    if(!func_button_pressed)
    {
      // tbd
    }
    else
    {
      // <Fn><1> == BFO Tune toggle
      BFO_tune_flg = !BFO_tune_flg;
      lcd.clear(); 
      func_button_pressed = false;
    }
    changed_f = 1;
  }

  if (button_nbr == 2)
  // Button 2: change frequency step up
  // Fn: not assigned 
  {
    Serial.println("Change freq step");
    switch (VFOSet[v].radix)
    {
      case 10:
      {
          if(!func_button_pressed)
          {
             VFOSet[v].radix = 100;
             // clear residual < 100Hz frequency component from the active VFO         
             uint16_t f = VFOSet[v].vfo % 100;
             VFOSet[v].vfo -= f;
          }
          else
          {
            // tbd
            func_button_pressed = false;
          }
      }
      break;
  
      case 100:
      {
          if(!func_button_pressed)
          {
              VFOSet[v].radix = 1000;
              // clear residual < 1kHz frequency component from the active VFO         
              uint16_t f = VFOSet[v].vfo % 1000;
              VFOSet[v].vfo -= f;
          }
          else
          {
              // tbd
              func_button_pressed = false;
          }
      }
      break;
        
      case 1000:
      {
          if(!func_button_pressed)
          {
              VFOSet[v].radix = 10000;
          }
          else
          {
              // tbd
              func_button_pressed = false;
          }
          break;

      }
      
      case 10000:
      {
          if(!func_button_pressed)
          {
              VFOSet[v].radix = 10;
          }
          else
          {
              // tbd 
              func_button_pressed = false;
          }
      }
      break; 
    }
    changed_f = 1;
  }

  if (button_nbr == 3)
  // Button 3: change VFO up
  // Fn: toggle IF filter (SSB/CW)
  {
      if(!func_button_pressed)
      {
          Serial.println("Change VFO up");
          if(v == (NBR_VFOS-1)) v = 0;
          else v++;
      }
      else
      {
          // toggle IF filter ***
          func_button_pressed = false;
      }
      changed_f = 1;
  }

  if (button_nbr == 4)
  // Button 4: Function button 
  {
    func_button_pressed = !func_button_pressed;   
    if(func_button_pressed) Serial.println("Function...");
    changed_f = 1;
  }

  if (button_nbr == 5)
  // Button 5: change frequency step up
  // Fn: tbd
  {
    Serial.println("Change freq step");
    switch (VFOSet[v].radix)
    {
      case 10:
      {
          if(!func_button_pressed)
          {
              VFOSet[v].radix = 10000;
              // clear residual < 1kHz frequency component from the active VFO         
              uint16_t f = VFOSet[v].vfo % 1000;
              VFOSet[v].vfo -= f;
          }
          else
          {
            // tbd
            func_button_pressed = false;
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
              // tbd
              func_button_pressed = false;
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
              // tbd
              func_button_pressed = false;
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
              // tbd
              func_button_pressed = false;
          }
      }
      break; 
    }
    changed_f = 1;
  }

  if (button_nbr == 6)
  // Button 6: change VFO down
  // Fn: not assigned
  {
      if(!func_button_pressed)
      {
          Serial.println("Change VFO down");
          if(v == 0) v = (NBR_VFOS-1);
          else v--;
      }
      else
      {
          // TBD
          func_button_pressed = false;
      }
    changed_f = 1;
  }
  
}

