/*
Arduino Nano script for homebrew multiband SSB/CW transceiver. 
EEPROM initialiser script
*/

#include <Wire.h>
#include <EEPROM.h>

#define NBR_VFOS    10 // number of selectable VFOs 

// struct for 'VFO parameter set' records -- the parameters that will change with each VFO
typedef struct {
  boolean  active;
  uint32_t vfo;
  uint32_t radix;
} VFOset_type;

VFOset_type VFOSet[NBR_VFOS]; // array of band parameter sets
byte v;                       // index into VFOSet array (representing the current VFO)

int t=0, second_counter=0;


void setup(){
  Serial.begin(9600);  
  Serial.println("setup()");

  // print EEPROM size
  int ee_len = EEPROM.length();
  Serial.print("EEPROM length =");
  Serial.println(ee_len);
  
  Serial.print("VFOSet[] length =");
  Serial.println(sizeof(VFOset_type) * NBR_VFOS);

  // clear the EEPROM contents
  for(int j=0; j<ee_len; j++)
  {
    EEPROM.write(j, 0);
  }

 // initialise VFOSet array
  for(int n=0; n<NBR_VFOS; n++) VFOSet[n].active = 0;   // make sure all are inactive to start with 
  VFOSet[0] = (VFOset_type){1,  1825000ULL, 1000};
  VFOSet[1] = (VFOset_type){1,  3525000ULL,  100};
  VFOSet[2] = (VFOset_type){1,  3625000ULL,  100};
  VFOSet[3] = (VFOset_type){1,  7025000ULL,  100};
  VFOSet[4] = (VFOset_type){1,  7090000ULL, 1000};
  VFOSet[5] = (VFOset_type){1, 10105000ULL,  100};
  VFOSet[6] = (VFOset_type){1, 14060000ULL,  100};
  VFOSet[7] = (VFOset_type){1, 18068000ULL, 1000};
  VFOSet[8] = (VFOset_type){1, 28480000ULL, 1000};
  VFOSet[9] = (VFOset_type){1, 52100000ULL, 1000};

  // write the current band index (into VFOSet[]) into EEPROM byte 0
  byte b = 3;
  EEPROM.write(0, b);

  // put the entire BandSet array into EEPROM
  int element_len = sizeof(VFOset_type);
  for(int i=0; i<NBR_VFOS; i++)
  {
    EEPROM.put(1 + (i * element_len), VFOSet[i]);
  };

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

    t = millis();
}


void loop()
{
  /*
  if((millis() - t) > 1000){
    t=millis();
    second_counter++;
    Serial.println(second_counter);
  }
      */
}
