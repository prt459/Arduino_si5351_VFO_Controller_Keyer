/*
This entire program is taken from Jason Mildrum, NT7S and Przemek Sadowski, SQ9NJE.
There is not enough original code written by me to make it worth mentioning.
http://nt7s.com/
http://sq9nje.pl/
http://ak2b.blogspot.com/

Modified by VK3HN, 11 Jan to 19 Feb 2017.
*/

#include <Rotary.h>
#include <si5351.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>

#define F_MIN   40000000UL    // Lower frequency limit (400 kHz)
#define F_MAX  5000000000UL   // Upper frequency limit (50MHz)

#define ENCODER_A    3   // Encoder pin A
#define ENCODER_B    2   // Encoder pin B
// #define ENCODER_BTN  11  // Encoder step button is on Nano pin 11
#define SWITCH_BANK  A0  // analog pin with the string of multiplexed push buttons

#define LCD_RS	5    // Register Select is connected to Nano pin 5
#define LCD_E		6    // assume LCD_E means the clock, which is connected to Nano pin 6
#define LCD_D4	7    // LCD D4 is on Nano pin 7
#define LCD_D5	8    // LCD D5 is on Nano pin 8
#define LCD_D6	9    // LCD D6 is on Nano pin 9
#define LCD_D7	10   // LCD D7 is on Nano pin 10

// mode chars
#define MODE_AM   0
#define MODE_CW   1
#define MODE_USB  2
#define MODE_LSB  3
#define MODE_SSB  4

// array of printable mode strings
String mode_strings[] = {"AM ", "CW ", "USB", "LSB", "SSB" }; 

// VFO chars
#define VFO_CHR_A  'A'
#define VFO_CHR_B  'B'
#define VFO_CHR_C  'C'

// band indexes and strings

#define NBR_BANDS     8  // number of selectable bands in the receiver 
#define BAND_INDEX_BC  0  
#define BAND_INDEX_160 1  
#define BAND_INDEX_80  2  
#define BAND_INDEX_60  3  
#define BAND_INDEX_40  4  
#define BAND_INDEX_30  5  
#define BAND_INDEX_20  6  
#define BAND_INDEX_17  7  
/*
#define BAND_INDEX_15  9  
#define BAND_INDEX_12  10 
#define BAND_INDEX_10  11 
#define BAND_INDEX_6   12 
*/

// array of printable band strings
String band_strings[NBR_BANDS] = {"BC ", "160", "80 ", "60 ", "40 ", "30 ", "20 ", "17 " }; 


// struct for 'band parameter set' records -- all of the parameters that will change with each band
typedef struct {
  boolean  active;
  byte     bs_index; 
  uint32_t vfo_a;
  uint32_t vfo_b;
  uint32_t vfo_c;
  char     curr_vfo;
  byte     curr_mode;
  byte     curr_sideband;
  uint32_t radix;
  uint32_t f_heterodyne;
  char     tuning_direction;  
} bandset_type;

bandset_type BandSet[NBR_BANDS];  // array of band parameter sets
byte bs_index;                    // index into BandSet array

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);    // LCD - pin assignement

Si5351 si5351;
Rotary r = Rotary(ENCODER_A, ENCODER_B);
volatile uint32_t LSB = 899850000ULL;
volatile uint32_t USB = 900150000ULL;
volatile uint32_t bfo = 900150000ULL; //start in USB
String curr_rx_tx = "RX";

// button variables used in main loop for sensing the multiplexed buttons
int button_nbr; 
int old_button_nbr = 0; 

// S meter reading, remove when we have a live analogue port connected circuit
int s_meter_reading=256; // set to about s3

// variables for controlling EEPROM writes
int  dial_moved_ms;
bool eeprom_written_since_last_dial_move; 

boolean changed_f = 0;
int loop_counter = 0;    // main loop counter as part of limiting EEPROM updates

//------------------------------- Set Optional Features here --------------------------------------
#define IF_Offset //Output is the display plus or minus the bfo frequency
//--------------------------------------------------------------------------------------------------

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
/* Change the frequency               */
/* dir = 1    Increment               */
/* dir = -1   Decrement               */
/**************************************/
void set_frequency(short dir)
{
  if (dir == 1)
    {
       if (BandSet[bs_index].curr_vfo == VFO_CHR_A) BandSet[bs_index].vfo_a += BandSet[bs_index].radix; 
       if (BandSet[bs_index].curr_vfo == VFO_CHR_B) BandSet[bs_index].vfo_b += BandSet[bs_index].radix; 
       if (BandSet[bs_index].curr_vfo == VFO_CHR_C) BandSet[bs_index].vfo_c += BandSet[bs_index].radix; 
    }
  else
     if (dir == -1)
       {
          if (BandSet[bs_index].curr_vfo == VFO_CHR_A) BandSet[bs_index].vfo_a -= BandSet[bs_index].radix; 
          if (BandSet[bs_index].curr_vfo == VFO_CHR_B) BandSet[bs_index].vfo_b -= BandSet[bs_index].radix; 
          if (BandSet[bs_index].curr_vfo == VFO_CHR_C) BandSet[bs_index].vfo_c -= BandSet[bs_index].radix; 
       };
 
  changed_f = 1;
}

/**************************************/
/* Read the buttons with debouncing    */
/**************************************/
/*
boolean get_encoder_button()
{
  if (!digitalRead(ENCODER_BTN))   {
    delay(20);
    if (!digitalRead(ENCODER_BTN))
    {
      while (!digitalRead(ENCODER_BTN));
      return 1;
    }
  }
  return 0;
}
*/

// Take a single reading of the buttons and map it from the A0 value to a button number (0..6)
// (if the switches produce any spurious readings, you can take 10 readings and average them)

int get_pressed_button()
{
  int b, i, sum, z;
  sum = 0;
  for (i=0; i<4; i++)
  {
    sum += analogRead(SWITCH_BANK);    // read A0 
  }
  z = sum/4;
  
  if(z > 900) b = 0;                  // 1008-1017
  else if(z > 230 && z < 280) b = 1;  // 258 
  else if(z > 200 && z < 230) b = 2;  // 220 
  else if(z > 160 && z < 190) b = 3;  // 177-178 
  else if(z > 120 && z < 140) b = 4;  // 130 
  else if(z > 50  && z < 100) b = 5;  // 76 
  else if(z > 5   && z < 40)  b = 6;  // 14-15 
  else b = 0;
  
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


/**************************************/
/* Display the frequency             */
/**************************************/
void display_frequency()
{
  uint16_t f, g;
  uint32_t vfo_l;

  if (BandSet[bs_index].curr_vfo == VFO_CHR_A) vfo_l = BandSet[bs_index].vfo_a; 
  if (BandSet[bs_index].curr_vfo == VFO_CHR_B) vfo_l = BandSet[bs_index].vfo_b; 
  if (BandSet[bs_index].curr_vfo == VFO_CHR_C) vfo_l = BandSet[bs_index].vfo_c; 

  lcd.setCursor(0, 0);
  lcd.print(curr_rx_tx);
  lcd.print(" ");

  lcd.print(BandSet[bs_index].curr_vfo);
  lcd.print(" ");

  lcd.setCursor(3, 0);
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

  //lcd.setCursor(0, 1);
  if(BandSet[bs_index].curr_mode == MODE_SSB)
    lcd.print(mode_strings[BandSet[bs_index].curr_sideband]);
  else
    lcd.print(mode_strings[BandSet[bs_index].curr_mode]);
 
  lcd.print("xxxxxxxxxxxx");  // these char spots are not displayed for some strange reason

  String s_meter = read_meter();  // fix this at 8 chars
  lcd.print(s_meter);

  switch (BandSet[bs_index].radix)
  {
    case 10:
      lcd.print("      ");
      lcd.print((char)24);   // ASCII up-arrow char
    break;
      
    case 100:
      lcd.print("     ");
      lcd.print((char)24);   // ASCII up-arrow char
      lcd.print(" ");
    break;
      
    case 1000:
      lcd.print("   ");
      lcd.print((char)24);   // ASCII up-arrow char
      lcd.print("   ");
    break;
      
    case 10000:
      lcd.print("  ");
      lcd.print((char)24);   // ASCII up-arrow char
      lcd.print("    ");
    break;
  }
  // Serial.println(vfo_l + bfo);
  // Serial.println(curr_sideband);
}


void setup()
{
  dial_moved_ms = millis(); 
  eeprom_written_since_last_dial_move = false; 
  
  Serial.begin(19200);  
  lcd.begin(20, 4);  // Initialize and clear the LCD
  lcd.clear();
  lcd.print("DDS VFO        v1.1 0123456789abVK3HN     19/2/2017");
  //                             xxxxxxxxxxxx                              
  delay(2000); 
  lcd.clear();
  Wire.begin();

  // load up BandSet array from EEPROM

  bs_index = EEPROM.read(0);
  Serial.print("setup() from eeprom: bs_index=");
  Serial.println(bs_index);
  
  int element_len = sizeof(bandset_type);
  for(int i=0; i < NBR_BANDS; i++)
  {
    EEPROM.get(1 + (i * element_len), BandSet[i]);
  };

  for(int n=0; n < NBR_BANDS ; n++)
  {
    Serial.print((int)BandSet[n].active);
    Serial.print(" ");
    Serial.print(BandSet[n].bs_index);
    Serial.print(" ");
    Serial.print((long)BandSet[n].vfo_a);
    Serial.print(" ");
    Serial.print((long)BandSet[n].vfo_b);
    Serial.print(" ");
    Serial.print((long)BandSet[n].vfo_c);
    Serial.print(" ");
    Serial.print(BandSet[n].curr_vfo);
    Serial.print(" ");
    Serial.print(BandSet[n].curr_mode);
    Serial.print(" ");
    Serial.print(BandSet[n].curr_sideband);
    Serial.print(" ");
    Serial.print((long)BandSet[n].radix);
    Serial.print(" ");
    Serial.print((long)BandSet[n].f_heterodyne);
    Serial.print(" ");
    Serial.println(BandSet[n].tuning_direction);
    delay(50);
  }
  
  // init and start the si5351 clocks
  // si5351.set_correction(140); // use File/Examples/si5351Arduino-Jason
  si5351.set_correction(1);     // to determine the correction using the serial monitor

  //initialize the Si5351
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0); //If using 27Mhz xtal, put 27000000 instead of 0
                                              // 0 is the default xtal freq of 25Mhz
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);

#ifdef IF_Offset
  volatile uint32_t vfo_l, hetero_l, f; 
  if (BandSet[bs_index].curr_vfo == VFO_CHR_A) vfo_l = BandSet[bs_index].vfo_a; 
  if (BandSet[bs_index].curr_vfo == VFO_CHR_B) vfo_l = BandSet[bs_index].vfo_b;  
  if (BandSet[bs_index].curr_vfo == VFO_CHR_C) vfo_l = BandSet[bs_index].vfo_c;  
  hetero_l = BandSet[bs_index].f_heterodyne;   

  if(BandSet[bs_index].tuning_direction == 'D')
  {
    f = vfo_l - hetero_l;
    bfo = LSB;       // Down tuning needs LSB BFO
  }
  else
  {
    f = hetero_l - vfo_l;
    bfo = LSB;       // Up tuning needs USB BFO
  }
     
  // set CLK0 to the VFO frequency for the current band
  si5351.set_freq(bfo - (f * SI5351_FREQ_MULT), SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_8MA); 
  
  // volatile uint32_t vfoT = bfo - ((vfo_l - hetero_l) * SI5351_FREQ_MULT);
  // BandSet[bs_index].curr_sideband = MODE_USB;

  // if needed, set CLK2 to the heterodyne frequency for the current band
  if(hetero_l != 0)
  {
    si5351.set_freq(hetero_l  * SI5351_FREQ_MULT, SI5351_CLK2);
    Serial.print("starting heterodyne osc on ");
    Serial.println(hetero_l);
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA); 
  }

  // Set CLK2 to output bfo frequency
  //si5351.set_freq(bfo, SI5351_CLK2);
  //si5351.drive_strength(SI5351_CLK2,SI5351_DRIVE_8MA); 
#endif

//  pinMode(ENCODER_BTN, INPUT_PULLUP);
  pinMode(SWITCH_BANK, INPUT_PULLUP);  // switch band is Pull-up
  
  PCICR |= (1 << PCIE2);           // Enable pin change interrupt for the encoder
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();

/*
  // initialise BandSet array
  for(int i=0; i<NBR_BANDS ; i++)
  {
    BandSet[i].active = 0;  // make all band sets empty by default
  };
  BandSet[BAND_INDEX_BC] = (bandset_type){1,  0,   472000ULL,   621000ULL,   774000ULL, 'A', MODE_AM,  MODE_LSB, 1000, 1000000ULL, 1};
  BandSet[BAND_INDEX_160]= (bandset_type){1,  1,  1825000ULL,  1843000ULL,  1850000ULL, 'A', MODE_AM,  MODE_LSB, 1000, 1000000ULL, 1};
  BandSet[BAND_INDEX_80] = (bandset_type){1,  2,  3510000ULL,  3570000ULL,  3650000ULL, 'B', MODE_SSB, MODE_LSB, 1000,       0ULL, 0};
  BandSet[BAND_INDEX_60] = (bandset_type){1,  3,  5350000ULL,  5360000ULL,  5365000ULL, 'A', MODE_SSB, MODE_LSB,  100, 1000000ULL, 1};
  BandSet[BAND_INDEX_40] = (bandset_type){1,  4,  7025000ULL,  7090000ULL,  7125000ULL, 'B', MODE_SSB, MODE_LSB,  100, 3300000ULL, 1};
  BandSet[BAND_INDEX_30] = (bandset_type){1,  5, 10105000ULL, 10118000ULL, 10140000ULL, 'B', MODE_SSB, MODE_USB,  100, 6500000ULL, 1};
  BandSet[BAND_INDEX_20] = (bandset_type){1,  6, 14060000ULL, 14190000ULL, 14320000ULL, 'B', MODE_SSB, MODE_USB,  100, 1000000ULL, 1};
  BandSet[BAND_INDEX_17] = (bandset_type){1,  7, 18068000ULL, 18088000ULL, 18228000ULL, 'B', MODE_SSB, MODE_USB,  100, 1000000ULL, 1};
  /*
  BandSet[BAND_INDEX_15] = (bandset_type){1,  8, 21040000ULL, 21125000ULL, 21200000ULL, 'B', MODE_SSB, MODE_USB,  100, 1000000ULL, 1};
  BandSet[BAND_INDEX_12] = (bandset_type){1,  9, 24890000ULL, 24920000ULL, 24990000ULL, 'B', MODE_SSB, MODE_USB,  100, 1000000ULL, 1};
  BandSet[BAND_INDEX_10] = (bandset_type){1, 10, 28480000ULL, 28500000ULL, 28600000ULL, 'B', MODE_SSB, MODE_USB,  100, 1000000ULL, 1};
  BandSet[BAND_INDEX_6]  = (bandset_type){1, 11, 52100000ULL, 52250000ULL, 52250000ULL, 'B', MODE_SSB, MODE_USB,  100, 1000000ULL, 1};
*/

  display_frequency();  // Update the display
}


void loop()
{
  // TEMP!! get a random s-meter reading -- delete this when s-meter is live!
  if(loop_counter == 0)
  {
    if(abs( millis() - dial_moved_ms ) > 10000)
    {
      if(eeprom_written_since_last_dial_move == false)
      {
        // do the eeprom write
        Serial.println("*** eeprom write");
        EEPROM.write(0, bs_index);   // write the band index (bs_index) to the first byte
        
        int element_len = sizeof(bandset_type);
        for(int i=0; i<NBR_BANDS ; i++)    // write each element of the entire BandSet array
        {
          EEPROM.put(1 + (i * element_len), BandSet[i]);
        }
        eeprom_written_since_last_dial_move = true;
      }
    }
    /*
    k = random(0,10);
    n = random(127, 256);  // get 0 or 1 s-point
    if (k>4)
      s_meter_reading = (s_meter_reading + n) % 1024;  // up an s-point
    else
    {
      s_meter_reading = (s_meter_reading - n) % 1024;  // down an s-point
      if(s_meter_reading < 128) s_meter_reading = 128; // keep the simulated noise at s1 or above
    }
    changed_f = 1; 
    */
  }
  loop_counter = (loop_counter + 1) % 15000;  // controls how often the s-meter and eeprom algorithm updates  

 
  // Update the display if the frequency has been changed
  if (changed_f)
  {
    display_frequency();

#ifdef IF_Offset
    volatile uint32_t vfo_l, hetero_l, f; 
    if (BandSet[bs_index].curr_vfo == VFO_CHR_A) vfo_l = BandSet[bs_index].vfo_a; 
    if (BandSet[bs_index].curr_vfo == VFO_CHR_B) vfo_l = BandSet[bs_index].vfo_b; 
    if (BandSet[bs_index].curr_vfo == VFO_CHR_C) vfo_l = BandSet[bs_index].vfo_c; 
    hetero_l = BandSet[bs_index].f_heterodyne;  

    if(BandSet[bs_index].tuning_direction == 'D')
    {
      f = vfo_l - hetero_l;
      bfo = LSB;       // Down tuning needs LSB BFO
    }
    else
    {
      f = hetero_l - vfo_l;
      bfo = LSB;       // Up tuning needs USB BFO
    }
      
    si5351.set_freq(bfo - (f * SI5351_FREQ_MULT), SI5351_CLK0);  

    if (vfo_l >= 10000000ULL & BandSet[bs_index].curr_sideband != MODE_USB)
    {
      bfo = USB;
      BandSet[bs_index].curr_sideband = MODE_USB;
      //si5351.set_freq(bfo, SI5351_CLK2);
      Serial.println("We've switched from LSB to USB");
    }
    else if (vfo_l < 10000000ULL & BandSet[bs_index].curr_sideband != MODE_LSB)
    {
      bfo = LSB;
      BandSet[bs_index].curr_sideband = MODE_LSB;
      //si5351.set_freq(bfo, SI5351_CLK2);
      Serial.println("We've switched from USB to LSB");
    }
#endif

    changed_f = 0;
    dial_moved_ms = millis(); 
    eeprom_written_since_last_dial_move = false;
  } // endif changed_f

  //------------------------------------------------------------------------
  // if any of the buttons have been pressed...
  old_button_nbr = 0;  // clear the last command 
  button_nbr = get_pressed_button();
  while (button_nbr > 0)
  {
  // one of the multiplexed switches is being held down
    delay(20);
    old_button_nbr = button_nbr;
    button_nbr = get_pressed_button();
  }

  button_nbr = old_button_nbr;
  // Serial.print(", button_nbr=");
  // Serial.print(button_nbr);     

  // Serial.print(", old_button_nbr=");
  // Serial.println(old_button_nbr);  
 
// if one of the buttons was pressed (and is now released) act on it...
  
  if (button_nbr == 1)
  // Button 1 changes the frequency step 
  {
    Serial.println("Change freq step");
    switch (BandSet[bs_index].radix)
    {
      case 10:
      {
         BandSet[bs_index].radix = 100;
         // clear residual < 100Hz frequency component from the active VFO         
         uint32_t vfo_l;
         uint16_t f;

         if (BandSet[bs_index].curr_vfo == VFO_CHR_A) {
            vfo_l = BandSet[bs_index].vfo_a;
            f = vfo_l % 100;
            BandSet[bs_index].vfo_a -= f;
         }  

         if (BandSet[bs_index].curr_vfo == VFO_CHR_B) {
            vfo_l = BandSet[bs_index].vfo_b;
            f = vfo_l % 100;
            BandSet[bs_index].vfo_b -= f;
         }  

         if (BandSet[bs_index].curr_vfo == VFO_CHR_C) {
            vfo_l = BandSet[bs_index].vfo_c;
            f = vfo_l % 100;
            BandSet[bs_index].vfo_c -= f;
         }
      }
      break;
  
      case 100:
      {
         BandSet[bs_index].radix = 1000;
         // clear residual < 1kHz frequency component from the active VFO         
         uint32_t vfo_l;
         uint16_t f;

         if (BandSet[bs_index].curr_vfo == VFO_CHR_A) {
            vfo_l = BandSet[bs_index].vfo_a; 
            f = vfo_l % 1000;
            BandSet[bs_index].vfo_a -= f;
         }  

         if (BandSet[bs_index].curr_vfo == VFO_CHR_B) {
            vfo_l = BandSet[bs_index].vfo_b;
            f = vfo_l % 1000;
            BandSet[bs_index].vfo_b -= f;
         }  

         if (BandSet[bs_index].curr_vfo == VFO_CHR_C) {
            vfo_l = BandSet[bs_index].vfo_c;
            f = vfo_l % 1000;
            BandSet[bs_index].vfo_c -= f;
         }
      }
      break;
        
      case 1000:
        BandSet[bs_index].radix = 10;
        break;
    }
 //   display_radix();
    display_frequency();
  }

  if (button_nbr == 2)
  // Button 2 changes the VFO 
  {
    Serial.println("Change VFO");
    switch (BandSet[bs_index].curr_vfo)
    {
      case VFO_CHR_A:
        BandSet[bs_index].curr_vfo = VFO_CHR_B;
        break;      
      case VFO_CHR_B:
        BandSet[bs_index].curr_vfo = VFO_CHR_C;
        break;      
      case VFO_CHR_C:
        BandSet[bs_index].curr_vfo = VFO_CHR_A;
        break;
    }
    changed_f = 1;
    display_frequency();
  }

  if (button_nbr == 3)
  // Button press changes the band up in sequence
  {
    Serial.println("Change band up");
    bs_index = (bs_index + 1) % NBR_BANDS;

    if(BandSet[bs_index].f_heterodyne == 0)
    {
      // kill the heterodyne oscillator
      si5351.output_enable(SI5351_CLK2, 0);  // disable 
    }
    else
    {
      // change the freq of the heterodyne oscillator  
      si5351.set_freq(BandSet[bs_index].f_heterodyne * SI5351_FREQ_MULT, SI5351_CLK2);
      si5351.drive_strength(SI5351_CLK2,SI5351_DRIVE_8MA); 
      Serial.print("New heterodyne osc freq = "); 
      Serial.println(long(BandSet[bs_index].f_heterodyne));  
    }

    changed_f = 1;
    display_frequency();
  }

  if (button_nbr == 4)
  // Button press changes the band down in sequence
  {
    Serial.println("Change band down");
    if(bs_index == 0)
      bs_index = NBR_BANDS - 1;
    else
      bs_index--; 

    if(BandSet[bs_index].f_heterodyne == 0)
    {
      // kill the heterodyne oscillator
      si5351.output_enable(SI5351_CLK2, 0);  // disable 
    }
    else
    {
      // change the freq of the heterodyne oscillator  
      si5351.set_freq(BandSet[bs_index].f_heterodyne * SI5351_FREQ_MULT, SI5351_CLK2);
      si5351.drive_strength(SI5351_CLK2,SI5351_DRIVE_8MA); 
      Serial.print("New heterodyne osc freq = "); 
      Serial.println(long(BandSet[bs_index].f_heterodyne));  
    }

    changed_f = 1;
    display_frequency();
  }

  if (button_nbr == 5)
  // Button 5 changes the mode 
  {
    Serial.println("Change mode");

    byte b = BandSet[bs_index].curr_mode;
    if (b == MODE_SSB)
      BandSet[bs_index].curr_mode = MODE_CW;
    else if (b == MODE_CW)       
      BandSet[bs_index].curr_mode = MODE_AM;
    else if (b == MODE_AM)      
      BandSet[bs_index].curr_mode = MODE_SSB;
 
    display_frequency();
  }

  if (button_nbr == 6)
  // Button 6 changes the receiver from RX to TX (MUTE) 
  {
    Serial.println("Change rx/tx");

    if (curr_rx_tx == "RX")
      curr_rx_tx = "TX";
    else if (curr_rx_tx == "TX")       
      curr_rx_tx = "RX";
    else 
      curr_rx_tx = "RX";
 
    display_frequency();
  }
 
}
