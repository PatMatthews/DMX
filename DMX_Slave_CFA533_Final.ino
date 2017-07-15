
//============================================================================
#include <Conceptinetics.h>
#include <EEPROM.h>
#include "DHT.h"
#include <Wire.h>
#include <stdarg.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN 2     // what digital pin we're connected to
#define DHTHOT 145    // hot temperature range limit
#define DMX_SLAVE_CHANNELS   1 //DMX channels
//#define P(str) (strcpy_P(p_buffer, PSTR(str)), p_buffer)
#define CONV 1/2.55
#define KP_UP     0x01
#define KP_ENTER  0x02
#define KP_CANCEL 0x04
#define KP_LEFT   0x08
#define KP_RIGHT  0x10
#define KP_DOWN   0x20

//char p_buffer[17];

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

int bklDelay    = 100000;    // ms for the backlight to idle before turning off
unsigned long bklTime = 0;  // counter since backlight turned on
// create the menu counter
int menuCount   = 1;
int menuSelect = 0;
//create the plus and minus navigation delay counter with its initial maximum of 250.
//byte btnMaxDelay = 200;
//byte btnMinDelay = 25;
//byte btnMaxIteration = 5;
//byte btnCurrIteration;
boolean Show = true;
boolean ManualMode = false;
// establish temperature humidity pin on A0
//create manual override variables
boolean SetAddress = false;
boolean SetIntensity = false;
int DMX_Slave_Address = 1;
int DMX_SLAVE_ADDRESS = 1;
float M;
int R;
int G;
int B;
int W;
int WHITE;
int MW;

float m = 256;
int r = 1;

int w = 1;
typedef struct {
int versionCheck; // Just a number to check to ensure we've read in valid settings!  
int address; // minute to start this channel.
} addressVals_t;

addressVals_t ADDRESS[1];
int EEPROM_ADD[1]={1};
int EEPROM_address = 0;
byte overmenu = 0;
int overpercent = 0;
// button read variables for lcd shield
uint8_t i=0;
uint8_t buttons =0;
float h = 0; // humidity for dht22
float t = 0; // temperature for dht22
float f = 0; // farhenheit temperature
// Configure a DMX slave controller
DMX_Slave dmx_slave ( DMX_SLAVE_CHANNELS );

//---------------------------------------PIN configuration
const int StatusLed = 13;
const int PWM_R = 11;
const int PWM_G = 10;
const int PWM_B = 9;
const int PWM_W = 5;
const int RELAY = 4;

unsigned long       lastFrameReceivedTime;
const unsigned long dmxTimeoutMillis = 10000UL;
//============================================================================
typedef struct
  {
  uint8_t   command;
  uint8_t   length;
  uint8_t   data[24];
  uint16_t  crc;
  } CFPacket_t;
//----------------------------------------------------------------------------
class CrystalfontzI2CPacketLCD
  {
  public:
    //vars
    //functions
    CrystalfontzI2CPacketLCD(uint8_t address);
    uint8_t sendPacket_getReply(
              CFPacket_t *packet_sent,
              CFPacket_t *packet_received,
              uint8_t print_errors);
    uint8_t Search_I2C_Adresses(void);
    void Set_I2C_Adress(uint8_t address);
    void writeText(uint8_t x, uint8_t y, char *text, uint8_t length);
    void clearScreen(void);
    uint8_t getKeys(uint8_t *down,uint8_t *presses,uint8_t *releases);
    void setUpBar(uint8_t spec_char,uint8_t vert_mask);
    void drawBar(uint8_t col,uint8_t row,uint8_t chars_wide,
                 uint8_t px_length,uint8_t spec_char_solid,
                 uint8_t spec_char_variable,uint8_t vert_mask);
  private:
    //vars
    uint8_t  i2c_address;
    //functions
    uint16_t CRC(uint8_t *ptr, uint16_t len);
  };
//----------------------------------------------------------------------------
CrystalfontzI2CPacketLCD::CrystalfontzI2CPacketLCD(uint8_t address)
  {
  i2c_address=address;
  }
//----------------------------------------------------------------------------
uint16_t CrystalfontzI2CPacketLCD::CRC(uint8_t *data, uint16_t length)
  {
  //calculate the CRC for the packet data
  uint16_t crc = 0xFFFF;
  while(length--)
    crc = _crc_ccitt_update(crc, *data++);
  return ~crc;
  }
//----------------------------------------------------------------------------
uint8_t CrystalfontzI2CPacketLCD::sendPacket_getReply(
        CFPacket_t *packet_to_send,
        CFPacket_t *packet_received,
        uint8_t print_errors)
  {
  uint8_t
    bytes_received;
  uint8_t
    i;
  //Valid commands are from 0-35.
  //The maximum received length is known for each packet from the data sheet,
  //so this table will allow us to optimize read performance/minimize I2C
  //traffic by reading only the number of bytes that are significant.
  //0xFF is a magic value for invalid commands
  //Storing data in flash in the Arduino is really obtuse.
  static const uint8_t receive_packet_length[36] PROGMEM= {
    1+1+16+2,  //  0 = Ping Command (variable, 16 is max)
    1+1+16+2,  //  1 = Get Hardware & Firmware Version
    1+1+ 0+2,  //  2 = Write User Flash Area
    1+1+16+2,  //  3 = Read User Flash Area
    1+1+ 0+2,  //  4 = Store Current State As Boot State
    1+1+ 0+2,  //  5 = Reboot CFA-533, Reset Host, or Power Off Host
    1+1+ 0+2,  //  6 = Clear LCD Screen
    1+1+ 0+2,  //  7 = Set LCD Contents, Line 1
    1+1+ 0+2,  //  8 = Set LCD Contents, Line 2
    1+1+ 0+2,  //  9 = Set LCD Special Character Data
    1+1+ 9+2,  // 10 = Read 8 Bytes of LCD Memory
    1+1+ 0+2,  // 11 = Set LCD Cursor Position
    1+1+ 0+2,  // 12 = Set LCD Cursor Style
    1+1+ 0+2,  // 13 = Set LCD Contrast
    1+1+ 0+2,  // 14 = Set LCD & Keypad Backlight
    1+1+ 4+2,  // 15 = Read Temperature
    0xFF,      // 16 = (reserved)
    0xFF,      // 17 = (reserved)
    1+1+ 9+2,  // 18 = Read DOW Device Information
    0xFF,      // 19 = (reserved)
    1+1+16+2,  // 20 = Arbitrary DOW Transaction (variable, 16 is max)
    1+1+ 7+2,  // 21 = Setup Live Temperature Display (2 or max of 7)
    1+1+ 0+2,  // 22 = Send Command Directly to the LCD Controller
    0xFF,      // 23 = (reserved)
    1+1+ 3+2,  // 24 = Read Keypad, Polled Mode
    0xFF,      // 25 = (reserved)
    0xFF,      // 26 = (reserved)
    0xFF,      // 27 = (reserved)
    1+1+ 0+2,  // 28 = Set ATX Switch Functionality
    1+1+ 0+2,  // 29 = Enable/Feed Host Watchdog Reset
    1+1+15+2,  // 30 = Read Reporting/ATX/Watchdog (debug)
    1+1+ 0+2,  // 31 = Send data to LCD
    1+1+ 1+2,  // 33 = Set I2C slave address
    1+1+ 0+2,  // 34 = Set/Configure GPIO
    1+1+ 4+2}; // 35 = Read GPIO & Configuration

  //Table of times to delay in order to assure that the reply is valid.
  //These cheat/optimize a bit from the data sheet, since I have access
  //to the module firmware ;)
  static const uint16_t command_execution_delay[36] PROGMEM = {
       3,  //  0 = Ping Command (variable, 16 is max)
       2,  //  1 = Get Hardware & Firmware Version
      20,  //  2 = Write User Flash Area
       2,  //  3 = Read User Flash Area
      30,  //  4 = Store Current State As Boot State
    1500,  //  5 = Reboot CFA-533, Reset Host, or Power Off Host
       2,  //  6 = Clear LCD Screen
       3,  //  7 = Set LCD Contents, Line 1
       3,  //  8 = Set LCD Contents, Line 2
       2,  //  9 = Set LCD Special Character Data
       2,  // 10 = Read 8 Bytes of LCD Memory
       1,  // 11 = Set LCD Cursor Position
       1,  // 12 = Set LCD Cursor Style
       1,  // 13 = Set LCD Contrast
      50,  // 14 = Set LCD & Keypad Backlight
       2,  // 15 = Read Temperature
       0,  // 16 = (reserved)
       0,  // 17 = (reserved)
       2,  // 18 = Read DOW Device Information
       0,  // 19 = (reserved)
      50,  // 20 = Arbitrary DOW Transaction (variable, 16 is max)
       3,  // 21 = Setup Live Temperature Display (2 or max of 7)
       2,  // 22 = Send Command Directly to the LCD Controller
       0,  // 23 = (reserved)
       2,  // 24 = Read Keypad, Polled Mode
       0,  // 25 = (reserved)
       0,  // 26 = (reserved)
       0,  // 27 = (reserved)
       2,  // 28 = Set ATX Switch Functionality
       2,  // 29 = Enable/Feed Host Watchdog Reset
       3,  // 30 = Read Reporting/ATX/Watchdog (debug)
       4,  // 31 = Send data to LCD
       2,  // 33 = Set I2C slave address
       2,  // 34 = Set/Configure GPIO
       3}; // 35 = Read GPIO & Configuration
   
  //Validate the command
  if(35 < packet_to_send->command)
    {
    return(1);
    }
  if(0xFF == receive_packet_length[packet_to_send->command])
    {

    return(2);
    }
  //Validate the data length
  if(18 < packet_to_send->length)
    {

    return(3);
    }
  //Start the I2C transaction    
  Wire.beginTransmission(i2c_address);
  //Send the command byte
  Wire.write(packet_to_send->command);
  //Send the significant data length (will match the I2C since we are in
  //control of it here)
  Wire.write(packet_to_send->length);
  //Send the data[]
  for(i=0;i<packet_to_send->length;i++)
    {
    Wire.write(packet_to_send->data[i]);
    }
  //Calculate the crc
  packet_to_send->crc = CRC((uint8_t*)packet_to_send, packet_to_send->length+2);
  //Low byte of CRC
  Wire.write(*((uint8_t*)(&(((uint8_t*)&(packet_to_send->crc))[0]))));
  //High byte of CRC
  Wire.write(*((uint8_t*)(&(((uint8_t*)&(packet_to_send->crc))[1]))));
  //Stop the I2C transaction
  Wire.endTransmission();
  
  //Now we need to wait for the command to complete, based on the
  //delay table.
  //Even with all the crazy cariable type macros above, this still does not work:
  //delay(command_execution_delay[packet_to_send->command]);
#define EXECUTION_DELAY (pgm_read_word_near(&command_execution_delay[packet_to_send->command]))
  delay(EXECUTION_DELAY);

#define EXPECTED_BYTES (pgm_read_byte_near(&receive_packet_length[packet_to_send->command]))
  //Now it is safe to read the response packet back from the CFA533.
  bytes_received=Wire.requestFrom(i2c_address,EXPECTED_BYTES);

  if(1<=bytes_received)
    {
    //Get the command byte of the respose
    packet_received->command=Wire.read();
    }
  else
    {
    return(4);
    }

  if(2<=bytes_received)
    {
    //Find out how may bytes of the transfer the CFA533 thinks are significant.
    packet_received->length=Wire.read();
    
    //Range check the length. There should not be more than 18 (max data) + 2 (crc), and
    //we should have at least response->length still available.
    if(((18 + 2) < packet_received->length)||
       (Wire.available() < packet_received->length))
      {
      //Attempt to gracefully continue: Override the length
      packet_received->length = Wire.available();
      }
    }
  else
    {
    return(5);
    }
    
  //Transfer over the data
  for(i=0;i<packet_received->length;i++)
    {
    packet_received->data[i]=Wire.read();
    }
    
  //Check the CRC of the incoming packet.
  uint16_t
    calculated_crc;
  calculated_crc = CRC((uint8_t*)packet_received, packet_received->length+2);
    
  //Low byte of CRC
#define LOW_CRC  (*((uint8_t*)(&(((uint8_t*)&(calculated_crc))[0]))))
  //High byte of CRC
#define HIGH_CRC (*((uint8_t*)(&(((uint8_t*)&(calculated_crc))[1]))))

  uint8_t
    crc_low;  
  uint8_t
    crc_high;  
  crc_low=Wire.read();
  crc_high=Wire.read();
    
  if((crc_low != LOW_CRC) || (crc_high != HIGH_CRC))
    {
    return(6);
    }
  //All good.
  return(0);
  }
//----------------------------------------------------------------------------
  const char cmd_Str_00[] PROGMEM = " 0 = Ping Command";
  const char cmd_Str_01[] PROGMEM = " 1 = Get Hardware & Firmware Version";
  const char cmd_Str_02[] PROGMEM = " 2 = Write User Flash Area";
  const char cmd_Str_03[] PROGMEM = " 3 = Read User Flash Area";
  const char cmd_Str_04[] PROGMEM = " 4 = Store Current State As Boot State";
  const char cmd_Str_05[] PROGMEM = " 5 = Reboot CFA-533, Reset Host, or Power Off Host";
  const char cmd_Str_06[] PROGMEM = " 6 = Clear LCD Screen";
  const char cmd_Str_07[] PROGMEM = " 7 = Set LCD Contents, Line 1";
  const char cmd_Str_08[] PROGMEM = " 8 = Set LCD Contents, Line 2";
  const char cmd_Str_09[] PROGMEM = " 9 = Set LCD Special Character Data";
  const char cmd_Str_10[] PROGMEM = "10 = Read 8 Bytes of LCD Memory";
  const char cmd_Str_11[] PROGMEM = "11 = Set LCD Cursor Position";
  const char cmd_Str_12[] PROGMEM = "12 = Set LCD Cursor Style";
  const char cmd_Str_13[] PROGMEM = "13 = Set LCD Contrast";
  const char cmd_Str_14[] PROGMEM = "14 = Set LCD & Keypad Backlight";
  const char cmd_Str_15[] PROGMEM = "15 = Read Temperature";
  const char cmd_Str_16[] PROGMEM = "16 = (reserved)";
  const char cmd_Str_17[] PROGMEM = "17 = (reserved)";
  const char cmd_Str_18[] PROGMEM = "18 = Read DOW Device Information";
  const char cmd_Str_19[] PROGMEM = "19 = (reserved)";
  const char cmd_Str_20[] PROGMEM = "20 = Arbitrary DOW Transaction (variable, 16 is max)";
  const char cmd_Str_21[] PROGMEM = "21 = Setup Live Temperature Display (2 or max of 7)";
  const char cmd_Str_22[] PROGMEM = "22 = Send Command Directly to the LCD Controller";
  const char cmd_Str_23[] PROGMEM = "23 = (reserved)";
  const char cmd_Str_24[] PROGMEM = "24 = Read Keypad, Polled Mode";
  const char cmd_Str_25[] PROGMEM = "25 = (reserved)";
  const char cmd_Str_26[] PROGMEM = "26 = (reserved)";
  const char cmd_Str_27[] PROGMEM = "27 = (reserved)";
  const char cmd_Str_28[] PROGMEM = "28 = Set ATX Switch Functionality";
  const char cmd_Str_29[] PROGMEM = "29 = Enable/Feed Host Watchdog Reset";
  const char cmd_Str_30[] PROGMEM = "30 = Read Reporting/ATX/Watchdog (debug)";
  const char cmd_Str_31[] PROGMEM = "31 = Send data to LCD";
  const char cmd_Str_32[] PROGMEM = "32 = (reserved)";
  const char cmd_Str_33[] PROGMEM = "33 = Set I2C slave address";
  const char cmd_Str_34[] PROGMEM = "34 = Set/Configure GPIO";
  const char cmd_Str_35[] PROGMEM = "35 = Read GPIO & Configuration";  

  //const PROGMEM char * const PROGMEM Command_Strings[36] = {
  const char *  Command_Strings[36] = {
    cmd_Str_00,cmd_Str_01,cmd_Str_02,cmd_Str_03,cmd_Str_04,cmd_Str_05,cmd_Str_06,
    cmd_Str_07,cmd_Str_08,cmd_Str_09,cmd_Str_10,cmd_Str_11,cmd_Str_12,cmd_Str_13,
    cmd_Str_14,cmd_Str_15,cmd_Str_16,cmd_Str_17,cmd_Str_18,cmd_Str_19,cmd_Str_20,
    cmd_Str_21,cmd_Str_22,cmd_Str_23,cmd_Str_24,cmd_Str_25,cmd_Str_26,cmd_Str_27,
    cmd_Str_28,cmd_Str_29,cmd_Str_30,cmd_Str_31,cmd_Str_32,cmd_Str_33,cmd_Str_34,
    cmd_Str_35};
//----------------------------------------------------------------------------
uint8_t CrystalfontzI2CPacketLCD::Search_I2C_Adresses(void)
  {
  CFPacket_t
    address_command;
  CFPacket_t
    address_response;
  uint8_t
    original_address;
  uint8_t
    device_found_at_address;

  original_address=i2c_address;
  device_found_at_address=0xFF;
    
  //Set up the packet
  address_command.command = 1;
  address_command.length = 0;
  
  for(i2c_address=0;i2c_address<=127;i2c_address++)
    {
    address_response.command = 0xFF;
    address_response.length = 0;        
    uint8_t
       response;
    response=sendPacket_getReply(&address_command,&address_response,0);
   
    if(0x00 == response)
      {
      if(0xFF == device_found_at_address)
        {
        device_found_at_address=i2c_address;
        }
      }
    }
    
  i2c_address=original_address;
  return(device_found_at_address);
  }
  
//----------------------------------------------------------------------------
void CrystalfontzI2CPacketLCD::Set_I2C_Adress(uint8_t address)
  {
  i2c_address=address;
  }
//----------------------------------------------------------------------------
void CrystalfontzI2CPacketLCD::writeText(uint8_t x, uint8_t y, char *text, uint8_t length)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;
  
  //Set up the packet
  command.command = 31;
  command.length = length + 2;
  command.data[0] = x;
  command.data[1] = y;
  memcpy(command.data + 2, text, length);
  
  //send the packet
  sendPacket_getReply(&command,&response,0);
  }
//----------------------------------------------------------------------------
void CrystalfontzI2CPacketLCD::clearScreen(void)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;
  
  //Set up the packet
  command.command = 6;
  command.length = 0;

  //send the packet
  sendPacket_getReply(&command,&response,0);
  }
//----------------------------------------------------------------------------

uint8_t CrystalfontzI2CPacketLCD::getKeys(uint8_t *down,uint8_t *presses,uint8_t *releases)
  {
  CFPacket_t
    command;
  CFPacket_t
    response;

  //Set up the packet
  command.command = 24; //24 (0x18): Read Keypad, Polled Mode
  command.length = 0;
  //Send the packet, get the response
  sendPacket_getReply(&command,&response,0);
  // type: 0x40 | 0x18 = 0x58 = 8810
  // data_length: 3
  // data[0] = bit mask showing the keys currently pressed
  // data[1] = bit mask showing the keys that have been pressed since the last poll
  // data[2] = bit mask showing the keys that have been released since the last poll  
  //Pull the goodies out of the response, into the users varaibles.
  *down=response.data[0];
  *presses=response.data[1];
  *releases=response.data[2];
  
  if(*down || *presses || *releases)
    {
    return(1);
    }
  return(0);
  }
//============================================================================
CrystalfontzI2CPacketLCD   *cfPacket;
//============================================================================
//--------------------------------------
void readDHT(){
//  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
//  t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  f = dht.readTemperature(true);
}
//------------------------------------------
// read EEPROM settings
void readSettings() {
EEPROM.get(EEPROM_address, ADDRESS);
DMX_SLAVE_ADDRESS = ADDRESS[0].address;
if (ADDRESS[0].versionCheck != 9898) {
  // Set default values for the settings and save them to EEPROM
  ADDRESS[0].versionCheck = 9898;
  ADDRESS[0].address = 1;  // default setting
  EEPROM.put(EEPROM_address, ADDRESS[0]);
  } // end if
}
// -----------------------------------------end read settings
// Shutdonw PWM service for overheat
void stopPWM(){

         analogWrite(PWM_W,0);
         analogWrite(RELAY,0);
         digitalWrite(StatusLed,LOW);
}
//-----------------------------------------
//============================================
// splash screen
void SplashScreen(void){
  
  cfPacket->writeText(0,0,"     SKT LED    ", 16);
  cfPacket->writeText(0,1, "Controller by GG", 16);
  delay(5000);
  cfPacket->writeText(0,0, "push SELECT  to ", 16);
  cfPacket->writeText(0,1, "SET ADDRESS     ", 16);
  delay(1000);
  cfPacket->writeText(0,1, "SET INTENSITY   ", 16);
  delay(1000);
  ClearScreen();
  }
//============================================================================
  void ClearScreen(void){
  cfPacket->writeText(0,0,"                 ", 16);
  cfPacket->writeText(0,1,"                 ", 16);
  }
//============================================================================
// check KeyPress
void CheckKeyPress()
{
  uint8_t keys_down;
  uint8_t key_presses;
  uint8_t key_releases;
//    while(1)
//    { 
      cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
      if(key_presses & KP_ENTER)
        {
         DoMainmenu();
         Show = true;
         return;
        }
      if(key_releases & KP_ENTER)
        {
        return;
        }
      if(key_presses & KP_CANCEL)
        return;        
//    } // end while 1   
}
// DoAddress
void DoAddress(void){
  ClearScreen();
  cfPacket->writeText(0,0, "Set Address     ", 16);
  char buffer[16];
  uint8_t keys_down;
  uint8_t key_presses;
  uint8_t key_releases;
    //Set Slave Address
    // showe current slave address
    dtostrf(DMX_SLAVE_ADDRESS, 3, 0, buffer); // Leave room for too large numbers!
    cfPacket->writeText(0,1, buffer, 3);  
   while(1)
    {   
      cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
      while(0x00 == cfPacket->getKeys(&keys_down,&key_presses,&key_releases));
//      if(key_releases & KP_ENTER)

      if(key_presses & KP_CANCEL)
        {
        SetAddress = false;
        return;
        }
      if(key_presses & KP_RIGHT)
       if(DMX_SLAVE_ADDRESS < 513)
          {
          DMX_SLAVE_ADDRESS++;
          dtostrf(DMX_SLAVE_ADDRESS, 3, 0, buffer); // Leave room for too large numbers!
          cfPacket->writeText(0,1, buffer, 3); 
          } 
      if(key_presses & KP_UP)
        if(DMX_SLAVE_ADDRESS < 503)
          {
          DMX_SLAVE_ADDRESS+=10;
          dtostrf(DMX_SLAVE_ADDRESS, 3, 0, buffer); // Leave room for too large numbers!
          cfPacket->writeText(0,1, buffer, 3);
          }        
      if(key_presses & KP_LEFT)
        if(DMX_SLAVE_ADDRESS > 0)
          {
          DMX_SLAVE_ADDRESS--;
          dtostrf(DMX_SLAVE_ADDRESS, 3, 0, buffer); // Leave room for too large numbers!
          cfPacket->writeText(0,1, buffer, 3);  
          }
      if(key_presses & KP_DOWN)
      if(DMX_SLAVE_ADDRESS > 11)
        {
        DMX_SLAVE_ADDRESS-=10;
        dtostrf(DMX_SLAVE_ADDRESS, 3, 0, buffer); // Leave room for too large numbers!
        cfPacket->writeText(0,1, buffer, 3);
        }
       if(key_presses & KP_ENTER)
        {
        // update address to new current selected
        ADDRESS[0].address = DMX_SLAVE_ADDRESS;
        EEPROM.put(EEPROM_address, ADDRESS[0]);
        dmx_slave.setStartAddress (DMX_SLAVE_ADDRESS);
        SetAddress = true;
        ClearScreen();
        return;
        }
  } // end while 1
//  ClearScreen();
} // end Do Address
// DoBrightness
void DoBrightness (void){
    ClearScreen();
    cfPacket->writeText(0,0, "Set  Intensity  ", 16);    
    char buffer[16];
    int mval = m*CONV;
    uint8_t keys_down;
    uint8_t key_presses;
    uint8_t key_releases;
  while(1)
    {     
    dtostrf(mval, 3, 0, buffer); // Leave room for too large numbers!
    cfPacket->writeText(0,1, buffer, 3);  
    cfPacket->writeText(3,1, "%", 1);
    cfPacket->getKeys(&keys_down,&key_presses,&key_releases);
    while(0x00 == cfPacket->getKeys(&keys_down,&key_presses,&key_releases));
//    if(key_releases & KP_ENTER)

    if(key_releases & KP_CANCEL)
        {
        ManualMode = false;
        return;
        }
     if(key_presses & KP_RIGHT)
        if(m < 257)
          {
           m++;
           mval=m*CONV;
           dtostrf(mval, 3, 0, buffer); // Leave room for too large numbers!
           cfPacket->writeText(0,1, buffer, 3);  
           cfPacket->writeText(3,1, "%", 1);
           analogWrite(PWM_W,m); 
           digitalWrite(RELAY,HIGH);
          }
      if(key_presses & KP_UP)
        if(m < 232)
          {
           m+=25;
           mval=m*CONV;
           dtostrf(mval, 3, 0, buffer); // Leave room for too large numbers!
           cfPacket->writeText(0,1, buffer, 3);  
           cfPacket->writeText(3,1, "%", 1);
           analogWrite(PWM_W,m); 
           digitalWrite(RELAY,HIGH);
          }        
      if(key_presses & KP_LEFT)
        if(m >  0)
          {
          m--;
         mval=m*CONV;
         dtostrf(mval, 3, 2, buffer); // Leave room for too large numbers!
         cfPacket->writeText(1,1, buffer, 5);  
         cfPacket->writeText(7,1, "%", 1);
         analogWrite(PWM_W,m);
         digitalWrite(RELAY,HIGH);
          }   
      if(key_presses & KP_DOWN)
        if(m > 26)
          {
           m-=25;
           mval=m*CONV;
           dtostrf(mval, 3, 0, buffer); // Leave room for too large numbers!
           cfPacket->writeText(1,1, buffer, 3);  
           cfPacket->writeText(4,1, "%", 1);
           analogWrite(PWM_W,m); 
           digitalWrite(RELAY,HIGH);
          } 
        if(m < 2)
          {
         m = 0;
         mval=m*CONV;
         dtostrf(mval, 3, 0, buffer); // Leave room for too large numbers!
         cfPacket->writeText(1,1, buffer, 3);  
         cfPacket->writeText(4,1, "%", 1);
         analogWrite(PWM_W,0);
         digitalWrite(RELAY,LOW);                    
          }       
      if(key_presses & KP_ENTER)
        {
         ManualMode = true;
         SetIntensity = true;
         MW=m;
         ClearScreen();
         return;
        }
      if(key_releases & KP_ENTER)
        return;
    if(key_releases & KP_CANCEL)
        {
        //ManualMode = false;
        //SetIntensity = true;
        //SetAddress = true;
        //ClearScreen();
        return;
        }
    } // end while 1
} // end Do Brightness
// MainMenu
void DoMainmenu(void){
 Show = false;
  // set Address
  while (!SetAddress)
   {
    cfPacket->writeText(0,0, "Set Address     ", 16);
    DoAddress();
    cfPacket->writeText(0,1, "                ", 16);
    }
    // set intensity
  while (!SetIntensity)
  {
  cfPacket->writeText(0,0, "Set  Intensity  ", 16);
  DoBrightness();
  cfPacket->writeText(0,1, "                ", 16);
  }
  // return to dmxmode
  Show = true;
  SetAddress = false;
  SetIntensity = false;
}// end main menu

// DoDmxMode
void DoDmxMode(void){
char buffer[16];
char buffer1[16];
int w;
int wh;
  while(Show)
  {

  if(!ManualMode)
    { 
    //f = dht.readTemperature(true);
    // dmx mode
    M = dmx_slave.getChannelValue (1);
    W = M;
    w = W * CONV;
    if(W < 2){
        w = 0;
        analogWrite(PWM_W,W);
        digitalWrite(RELAY,LOW);
        cfPacket->writeText(0,0, "ADD:", 4);
        dtostrf(DMX_SLAVE_ADDRESS, 3, 0, buffer); // Leave room for too large numbers!
        cfPacket->writeText(4,0, buffer, 3);
        cfPacket->writeText(8,0, "TMP:", 4);
        //dtostrf(f, 3, 0, buffer); // Leave room for too large numbers!
        //cfPacket->writeText(12,0, buffer,3);
        dtostrf(w, 3, 0, buffer1); // Leave room for too large numbers! 
        cfPacket->writeText(0,1, buffer1, 3);
        cfPacket->writeText(3,1, "%", 1);
        }
        else{
        analogWrite(PWM_W,W);
        digitalWrite(RELAY,HIGH);
        cfPacket->writeText(0,0, "ADD:", 4);
        dtostrf(DMX_SLAVE_ADDRESS, 3, 0, buffer); // Leave room for too large numbers!
        cfPacket->writeText(4,0, buffer, 3);
        cfPacket->writeText(8,0, "TMP:", 4);
        //dtostrf(f, 3, 0, buffer); // Leave room for too large numbers!
        //cfPacket->writeText(12,0, buffer,3);
        dtostrf(w, 3, 0, buffer1); // Leave room for too large numbers! 
        cfPacket->writeText(0,1, buffer1, 3);
        cfPacket->writeText(3,1, "%", 1);        
        }
    } 
    if(ManualMode)
    {
    f = dht.readTemperature(true);
    wh = MW * CONV;
    // manual mode
    if(MW < 2){
      MW = 0;
      wh = MW * CONV;
      analogWrite(PWM_W,MW);
      digitalWrite(RELAY,LOW);
      cfPacket->writeText(0,0, "ADD:", 4);
      dtostrf(DMX_SLAVE_ADDRESS, 3, 0, buffer); // Leave room for too large numbers!
      cfPacket->writeText(4,0, buffer, 3);
      cfPacket->writeText(8,0, "TMP:", 4);
      dtostrf(f, 3, 0, buffer); // Leave room for too large numbers!
      cfPacket->writeText(12,0, buffer,3);
      dtostrf(wh, 3, 0, buffer1); // Leave room for too large numbers! 
      cfPacket->writeText(0,1, buffer1, 3);
      cfPacket->writeText(3,1, "%", 1);}      
    else{
      wh = MW * CONV;
      analogWrite(PWM_W,MW);
      digitalWrite(RELAY,HIGH);
      cfPacket->writeText(0,0, "ADD:", 4);
      dtostrf(DMX_SLAVE_ADDRESS, 3, 0, buffer); // Leave room for too large numbers!
      cfPacket->writeText(4,0, buffer, 3);
      cfPacket->writeText(8,0, "TMP:", 4);
      dtostrf(f, 3, 0, buffer); // Leave room for too large numbers!
      cfPacket->writeText(12,0, buffer,3);    
      dtostrf(wh, 3, 0, buffer1); // Leave room for too large numbers!
      cfPacket->writeText(0,1, buffer1, 3);
      cfPacket->writeText(3,1, "%", 1);              
      }
    } // end if manualMode4
  // check for loss of dmx signal and not in manual mode
  while(millis() - lastFrameReceivedTime > 3000 && !ManualMode)
    {
    digitalWrite(StatusLed,LOW);
    analogWrite(PWM_W,0);
    digitalWrite(RELAY,LOW);
    w=0;
    dtostrf(w, 3, 0, buffer1); // Leave room for too large numbers! 
    cfPacket->writeText(0,1, buffer1, 3);
    cfPacket->writeText(3,1, "%", 1);
    }
    CheckKeyPress();
    //if(f > DHTHOT){
    //  stopPWM();}
  } // end while Show
} // end do dmxmode
//============================================================================
bool sta = false;
void OnFrameReceiveComplete (unsigned short channelsReceived)
{
  if ( channelsReceived == DMX_SLAVE_CHANNELS)
  {
    // All slave channels have been received
    digitalWrite(StatusLed,LOW);
    ManualMode = false;    
  }
  else
  {
    // We have received a frame but not all channels we where 
    // waiting for, master might have transmitted less
    // channels
  }
  
  digitalWrite(StatusLed,sta);
  sta = !sta;
  // Update receive time to determine signal timeout
  lastFrameReceivedTime = millis ();
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
void setup()
  {
  // read defult / EEPROM settings and update to current
  readSettings();
  // DHT start
  dht.begin();
  // Enable DMX slave interface and start recording
  // DMX data
  dmx_slave.enable ();  
  // Set start address to 1, this is also the default setting
  // You can change this address at any time during the program
  dmx_slave.setStartAddress (DMX_SLAVE_ADDRESS);
  // Register on frame complete event to determine signal timeout
  dmx_slave.onReceiveComplete ( OnFrameReceiveComplete );
  
  // Set led pin as output pin
  pinMode ( PWM_R, OUTPUT );
  pinMode ( PWM_G, OUTPUT );
  pinMode ( PWM_B, OUTPUT );
  pinMode ( PWM_W, OUTPUT ); 
  pinMode ( RELAY, OUTPUT );
  pinMode ( StatusLed, OUTPUT );
 // initiate lcd interface
  Wire.begin(); // join i2c bus (address optional for master)
  //If you know the address:
  //  cfPacket = new CrystalfontzI2CPacketLCD(42);
  //If you do not know the address, this will search for the module.
  cfPacket = new CrystalfontzI2CPacketLCD(0xFF);
  cfPacket->Set_I2C_Adress(cfPacket->Search_I2C_Adresses());
  SplashScreen();
  }
//============================================================================

void loop()
  {

// if temp outside of range turn off output
   if (!Show)
  {
   DoMainmenu();
  }
  if(Show){
  // dmx mode
   DoDmxMode();

//   CheckKeyPress();
   } // end while show
  }
//============================================================================
