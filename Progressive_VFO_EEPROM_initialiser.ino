// script to write initial BandSet array into the EEPROM 
#include <EEPROM.h>

// mode chars
#define MODE_AM   0
#define MODE_CW   1
#define MODE_USB  2
#define MODE_LSB  3
#define MODE_SSB  4

// array of printable mode strings
String band_strings[] = {"AM ", "CW ", "USB", "LSB", "SSB" }; 
 
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


void setup()
{
  Serial.begin(19200);  
  Serial.println("setup()");

  // print EEPROM size
  int ee_len = EEPROM.length();
  Serial.print("EEPROM length =");
  Serial.println(ee_len);
  
  Serial.print("BandSet[] length =");
  Serial.println(sizeof(bandset_type) * NBR_BANDS);

  // clear the EEPROM contents
  for(int j=0; j<ee_len; j++)
  {
    EEPROM.write(j, 0);
  }

 // initialise BandSet array
  for(int i=0; i<NBR_BANDS ; i++)
  {
    BandSet[i].active = 0;  // make all band sets empty by default
  };
  BandSet[BAND_INDEX_BC] = (bandset_type){1,  0,   472000ULL,   621000ULL,   774000ULL, 'A', MODE_AM,  MODE_LSB, 100,  3900000ULL, 'U'};
  BandSet[BAND_INDEX_160]= (bandset_type){1,  1,  1825000ULL,  1843000ULL,  1850000ULL, 'A', MODE_AM,  MODE_LSB, 100,  5500000ULL, 'U'};
  BandSet[BAND_INDEX_80] = (bandset_type){1,  2,  3510000ULL,  3570000ULL,  3650000ULL, 'B', MODE_SSB, MODE_LSB, 100,        0ULL, 'D'};
  BandSet[BAND_INDEX_60] = (bandset_type){1,  3,  5350000ULL,  5360000ULL,  5365000ULL, 'A', MODE_SSB, MODE_LSB, 100,  8800000ULL, 'U'};
  BandSet[BAND_INDEX_40] = (bandset_type){1,  4,  7025000ULL,  7090000ULL,  7125000ULL, 'B', MODE_SSB, MODE_LSB, 100,  3300000ULL, 'D'};
  BandSet[BAND_INDEX_30] = (bandset_type){1,  5, 10105000ULL, 10118000ULL, 10140000ULL, 'B', MODE_SSB, MODE_USB, 100,  6500000ULL, 'D'};
  BandSet[BAND_INDEX_20] = (bandset_type){1,  6, 14060000ULL, 14190000ULL, 14320000ULL, 'B', MODE_SSB, MODE_USB, 100, 10500000ULL, 'D'};
  BandSet[BAND_INDEX_17] = (bandset_type){1,  7, 18068000ULL, 18088000ULL, 18228000ULL, 'B', MODE_SSB, MODE_USB, 100, 14500000ULL, 'D'};
  /*
  BandSet[BAND_INDEX_15] = (bandset_type){1,  8, 21040000ULL, 21125000ULL, 21200000ULL, 'B', MODE_SSB, MODE_USB, 100, 1000000ULL, 'D'};
  BandSet[BAND_INDEX_12] = (bandset_type){1,  9, 24890000ULL, 24920000ULL, 24990000ULL, 'B', MODE_SSB, MODE_USB, 100, 1000000ULL, 'D'};
  BandSet[BAND_INDEX_10] = (bandset_type){1, 10, 28480000ULL, 28500000ULL, 28600000ULL, 'B', MODE_SSB, MODE_USB, 100, 1000000ULL, 'D'};
  BandSet[BAND_INDEX_6]  = (bandset_type){1, 11, 52100000ULL, 52250000ULL, 52250000ULL, 'B', MODE_SSB, MODE_USB, 100, 1000000ULL, 'D'};
*/


  // write the current band index (into BandSet[]) into EEPROM byte 0
  byte b = BAND_INDEX_40;
  EEPROM.write(0, b);

  // put the entire BandSet array into EEPROM
  int element_len = sizeof(bandset_type);
  for(int i=0; i<NBR_BANDS ; i++)
  {
    EEPROM.put(1 + (i * element_len), BandSet[i]);
  };

  // dump out the EEPROM contents
  Serial.println("EEPROM contents:");
  b = EEPROM.read(0);
  Serial.print("bs_index="); 
  Serial.println(b); 
  
  for(int bs_index=0; bs_index < NBR_BANDS ; bs_index++)
  {
    Serial.print((int)BandSet[bs_index].active);
    Serial.print(" ");
    Serial.print(BandSet[bs_index].bs_index);
    Serial.print(" ");
    Serial.print((long)BandSet[bs_index].vfo_a);
    Serial.print(" ");
    Serial.print((long)BandSet[bs_index].vfo_b);
    Serial.print(" ");
    Serial.print((long)BandSet[bs_index].vfo_c);
    Serial.print(" ");
    Serial.print(BandSet[bs_index].curr_vfo);
    Serial.print(" ");
    Serial.print(BandSet[bs_index].curr_mode);
    Serial.print(" ");
    Serial.print(BandSet[bs_index].curr_sideband);
    Serial.print(" ");
    Serial.print((long)BandSet[bs_index].radix);
    Serial.print(" ");
    Serial.print((long)BandSet[bs_index].f_heterodyne);
    Serial.print(" ");
    Serial.println(BandSet[bs_index].tuning_direction);
    delay(1000);
  }

}

void loop()
{
}
