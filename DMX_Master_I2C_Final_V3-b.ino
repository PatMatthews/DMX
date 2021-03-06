// GrowGreen DMX512 Controller with capacity to control four meanwell powered CLU 100v CLU COB's with dimming 
// relay control temperature control and manual control with automatic override of manual
// mode.  Use CrystalFontz Serial i2c LCD with buttons.
//============================================================================
#include <Conceptinetics.h>
#include <EEPROM.h>
#include <stdarg.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

#define MAX_CHANNEL 8
#define MAX_CHANNELS 512
#define DMX_MASTER_CHANNELS MAX_CHANNELS
// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

int bklDelay    = 100000;    // ms for the backlight to idle before turning off
unsigned long bklTime = 0;  // counter since backlight turned on
// create the menu counter
int menuCount   = 1;
int menuSelect = 0;
//create the plus and minus navigation delay counter with its initial maximum of 250.
byte btnMaxDelay = 200;
byte btnMinDelay = 25;
byte btnMaxIteration = 5;
byte btnCurrIteration;
//boolean Show = true;
//boolean ManualMode = false;
// lcd screen variable
int LCD_R=2;  // lcd rows
int LCD_C=16;  // lcd columns

// establish temperature humidity pin on A0
//create manual override variables
boolean SetAddress = false;
boolean SetIntensity = false;
int DMX_MASTER_ADDRESS;
int DMX_ADDRESS = 1;
int SelectVal = 0;


typedef struct {
int versionCheck;   // Just a number to check to ensure we've read in valid settings!  
int channel;        // channel address.
} addressVals_t;

addressVals_t ADDRESS[MAX_CHANNEL];

int EEPROM_ADD[MAX_CHANNEL] = {0,0,0,0,0,0,0,0};
// button pin assignments digital pins
int button_address[MAX_CHANNEL] = {5,6,7,8,9,10,11,12};   // digital pins
 // slider pin assignments analog pins
int slider_address[MAX_CHANNEL] = {1,2,3,6,7,8,9,10};   // analopg pins
// button state variable for all buttons HIGH for OFF LOW for ON
int button_state[MAX_CHANNEL] = {LOW,LOW,LOW,LOW,LOW,LOW,LOW,LOW};
int old_button_state[MAX_CHANNEL] = {HIGH,HIGH,HIGH,HIGH,HIGH,HIGH,HIGH,HIGH};

int button_State1 = HIGH;
int button_State2 = HIGH;
int button_State3 = HIGH;
int button_State4 = HIGH;
int button_State5 = HIGH;
int button_State6 = HIGH;
int button_State7 = HIGH;
int button_State8 = HIGH;

byte overmenu = 0;
// button read variables for lcd shield
uint8_t i=0;
uint8_t buttons =0;

// configure a DMX Master Controller
DMX_Master dmx_master ( MAX_CHANNELS , 2 );
// The first parameter is the number of channels that you are controlling. 
// Note that no matter the number of channels given always 512 channels 
// are transmitted by the master... you can only control the first 100 
// of them which saves you memory if you don't need to control all 512 channels.
// The second parameter is the pin assigned to control read and write operations 
// on the shield (Data enable Read enable or any name assigned to it according 
// to the shield you are using)

//---------------------------------------PIN configuration
const int StatusLed = 13;
const int BUTTON1 = 5;    // digital D5
const int BUTTON2 = 6;
const int BUTTON3 = 7;
const int BUTTON4 = 8;
const int BUTTON5 = 9;
const int BUTTON6 = 10;
const int BUTTON7 = 11;
const int BUTTON8 = 12;

const int SLIDER1 = 0;    // analog A1
const int SLIDER2 = 1;
const int SLIDER3 = 2;
const int SLIDER4 = 3;
const int SLIDER5 = A6;
const int SLIDER6 = A7;
const int SLIDER7 = A8;
const int SLIDER8 = A9;

const int LED1 = 23;      // digital D23
const int LED2 = 24;
const int LED3 = 25;    // 
const int LED4 = 26;    // 
const int LED5 = 27;    // 
const int LED6 = 28;    // 
const int LED7 = 29;
const int LED8 = 30;

// A4 and A5 reserved for SCL and SDA i2c bus
const int break_usec = 200;
unsigned long       lastFrameReceivedTime;
const unsigned long dmxTimeoutMillis = 10000UL;

// Adafruit urtilities for lcd and buttons
//button hold function
int btnCurrDelay(byte curr)
{
  if(curr==btnMaxIteration)
  {
    btnCurrIteration = btnMaxIteration;
    return btnMaxDelay;
  }
  else if(btnCurrIteration ==0)
  {
    return btnMinDelay;
  }
  else
  {
    btnCurrIteration--;
    return btnMaxDelay;
  }
}
// readButtons from lcd shield
void ReadButtons()
   {
    uint8_t buttons = lcd.readButtons();

  if (buttons) {

    if (buttons & BUTTON_UP) {
     
    }
    if (buttons & BUTTON_DOWN) {
    
    }
    if (buttons & BUTTON_LEFT) {
     
    }
    if (buttons & BUTTON_RIGHT) {
     
    }
    if (buttons & BUTTON_SELECT) {
      
    }
  }
}
// end ReadButtons
void PrintBegin()
{
  lcd.begin(LCD_C, LCD_R);   // start the library  
  //lcd.setBacklight(ON);
   
}

// clean lcd screen
void cleanScreen()
{
  lcd.setCursor(0,0);
  lcd.print("                "); 
  lcd.setCursor(0,1);
  lcd.print("                ");
}
// -----------------------------------------
// read settingsfrom eeprom
void readSettings() 
{
int EEPROM_address = 0;
for (byte i = 0; i < MAX_CHANNEL; i++) 
  {
    EEPROM.get(EEPROM_address, ADDRESS);
    EEPROM_ADD[i] = EEPROM_address;
    if (ADDRESS[i].versionCheck != 1234) 
      {
      // Set default values for the settings and save them to EEPROM
      ADDRESS[i].versionCheck = 1234;
      switch (i) 
        {
        case 0:
          ADDRESS[0].channel = 3;  // default setting
        break;
        case 1:
          ADDRESS[1].channel = 13;  // default setting
        break;
        case 2:
          ADDRESS[2].channel = 23;  // default setting
        break;
        case 3:
          ADDRESS[3].channel = 33;  // default setting
        break;
        case 4:
          ADDRESS[4].channel = 43;  // default setting
        break;
        case 5:
          ADDRESS[5].channel = 53;  // default setting
        break;
        case 6:
          ADDRESS[6].channel = 63;  // default setting
        break;
        case 7:
          ADDRESS[7].channel = 73;  // default setting
        break;
        break;
        } // end switch
        EEPROM.put(EEPROM_ADD[i], ADDRESS[i]);
      } // end if
  } // end for

}  // end read settings
//------------------------------------------
// splash screen
void SplashScreen(void){
  lcd.setCursor(0,0);
  lcd.print("PMA DMX Tester  ");
  lcd.setCursor(0,1);
  lcd.print("Controller      ");
  delay(3000);
  lcd.setCursor(0,1);
  lcd.print("SELCT CHANNEL   ");
  delay(1000);
  lcd.clear(); 
  btnCurrIteration = btnMaxIteration;
} // end Splash screen
// DoIntensity
void DoIntensity(int j)
{
  char buffer[16];
  lcd.setCursor(0,1);
  lcd.print("                ");
            switch(j) {
            case 0:
            {
              // send value to channel
              while (button_state[j] == LOW) 
                {
                digitalWrite(LED1,HIGH);
                lcd.setCursor(0,0);
                lcd.print("CHANNEL:");
                lcd.setCursor(0,1);
                dtostrf(ADDRESS[j].channel, 3, 0, buffer); // Leave room for too large numbers!
                lcd.setCursor(9,0);
                lcd.print( buffer);  
                int sensor_Value = analogRead(SLIDER1);
                int sensorValue = sensor_Value/4;
                dtostrf(sensorValue, 3, 0, buffer); // Leave room for too large numbers!              
                lcd.setCursor(0,1);
                lcd.print( buffer);
                digitalWrite(StatusLed,HIGH);  
                dmx_master.setChannelValue ( ADDRESS[j].channel, sensorValue );
                uint8_t buttons = lcd.readButtons();
                if(buttons & BUTTON_RIGHT) { button_state[j] = HIGH;digitalWrite(LED1,LOW);DoAddress();}
                if(buttons & BUTTON_LEFT) { button_state[j] = HIGH;digitalWrite(LED1,LOW);DoAddress ();}
                }
                digitalWrite(LED1,LOW);
            break;
            }
            case 1:
            {
              // send value to channel
              while (button_state[j] == LOW) 
                {
                digitalWrite(LED2,HIGH);
                lcd.setCursor(0,0);
                lcd.print("CHANNEL:");
                dtostrf(ADDRESS[j].channel, 3, 0, buffer); // Leave room for too large numbers!
                lcd.setCursor(9,0);
                lcd.print( buffer);  
                int sensor_Value = analogRead(SLIDER2);
                int sensorValue = sensor_Value/4;
                dtostrf(sensorValue, 3, 0, buffer); // Leave room for too large numbers!              
                lcd.setCursor(0,1);
                lcd.print( buffer);
                digitalWrite(StatusLed,HIGH);  
                dmx_master.setChannelValue ( ADDRESS[j].channel, sensorValue );
                uint8_t buttons = lcd.readButtons();
                if(buttons & BUTTON_RIGHT) { button_state[j] = HIGH;DoAddress();}
                if(buttons & BUTTON_LEFT) { button_state[j] = HIGH;DoAddress ();}
                }
                digitalWrite(LED2,LOW);    
          break;
            }
          case 2:
          {
              // send value to channel
              while (button_state[j] == LOW) 
                {
                digitalWrite(LED3,HIGH);
                lcd.setCursor(0,0);
                lcd.print("CHANNEL:");
                dtostrf(ADDRESS[j].channel, 3, 0, buffer); // Leave room for too large numbers!
                lcd.setCursor(9,0);
                lcd.print( buffer);  
                int sensor_Value = analogRead(SLIDER3);
                int sensorValue = sensor_Value/4;
                dtostrf(sensorValue, 3, 0, buffer); // Leave room for too large numbers!              
                lcd.setCursor(0,1);
                lcd.print( buffer);
                digitalWrite(StatusLed,HIGH);  
                dmx_master.setChannelValue ( ADDRESS[j].channel, sensorValue );
                uint8_t buttons = lcd.readButtons();
                if(buttons & BUTTON_RIGHT) { button_state[j] = HIGH;DoAddress();}
                if(buttons & BUTTON_LEFT) { button_state[j] = HIGH;DoAddress ();}
                }
                digitalWrite(LED3,LOW);
          break;
          }
          case 3:
          {
              // send value to channel
              while (button_state[j] == LOW) 
                {
                digitalWrite(LED4,HIGH);
                lcd.setCursor(0,0);
                lcd.print("CHANNEL:");
                dtostrf(ADDRESS[j].channel, 3, 0, buffer); // Leave room for too large numbers!
                lcd.setCursor(9,0);
                lcd.print( buffer);  
                int sensor_Value = analogRead(SLIDER4);
                int sensorValue = sensor_Value/4;
                dtostrf(sensorValue, 3, 0, buffer); // Leave room for too large numbers!              
                lcd.setCursor(0,1);
                lcd.print( buffer);
                digitalWrite(StatusLed,HIGH);  
                dmx_master.setChannelValue ( ADDRESS[j].channel, sensorValue );
                uint8_t buttons = lcd.readButtons();
                if(buttons & BUTTON_RIGHT) { button_state[j] = HIGH;DoAddress();}
                if(buttons & BUTTON_LEFT) { button_state[j] = HIGH;DoAddress ();}
                }
                digitalWrite(LED4,LOW);
          break;
          }
          case 4:
          {
              // send value to channel
              while (button_state[j] == LOW) 
                {
                digitalWrite(LED5,HIGH);
                lcd.setCursor(0,0);
                lcd.print("CHANNEL:");
                dtostrf(ADDRESS[j].channel, 3, 0, buffer); // Leave room for too large numbers!
                lcd.setCursor(9,0);
                lcd.print( buffer);  
                int sensor_Value = analogRead(SLIDER5);
                int sensorValue = sensor_Value/4;
                dtostrf(sensorValue, 3, 0, buffer); // Leave room for too large numbers!              
                lcd.setCursor(0,1);
                lcd.print( buffer);
                digitalWrite(StatusLed,HIGH);  
                dmx_master.setChannelValue ( ADDRESS[j].channel, sensorValue );
                uint8_t buttons = lcd.readButtons();
                if(buttons & BUTTON_RIGHT) { button_state[j] = HIGH;DoAddress();}
                if(buttons & BUTTON_LEFT) { button_state[j] = HIGH;DoAddress ();}
                }
                digitalWrite(LED5,LOW);
          break;
          }
          case 5:
          {
              // send value to channel
              while (button_state[j] == LOW) 
                {
                digitalWrite(LED6,HIGH);
                lcd.setCursor(0,0);
                lcd.print("CHANNEL:");
                dtostrf(ADDRESS[j].channel, 3, 0, buffer); // Leave room for too large numbers!
                lcd.setCursor(9,0);
                lcd.print( buffer);  
                int sensor_Value = analogRead(SLIDER6);
                int sensorValue = sensor_Value/4;
                dtostrf(sensorValue, 3, 0, buffer); // Leave room for too large numbers!              
                lcd.setCursor(0,1);
                lcd.print( buffer);
                digitalWrite(StatusLed,HIGH);  
                dmx_master.setChannelValue ( ADDRESS[j].channel, sensorValue );
                uint8_t buttons = lcd.readButtons();
                if(buttons & BUTTON_RIGHT) { button_state[j] = HIGH;DoAddress();}
                if(buttons & BUTTON_LEFT) { button_state[j] = HIGH;DoAddress ();}
                }
                digitalWrite(LED6,LOW);
          break;
          }
        case 6:
          {
              // send value to channel
              while (button_state[j] == LOW) 
                {
                digitalWrite(LED7,HIGH);
                lcd.setCursor(0,0);
                lcd.print("CHANNEL:");
                dtostrf(ADDRESS[j].channel, 3, 0, buffer); // Leave room for too large numbers!
                lcd.setCursor(9,0);
                lcd.print( buffer);  
                int sensor_Value = analogRead(SLIDER7);
                int sensorValue = sensor_Value/4;
                dtostrf(sensorValue, 3, 0, buffer); // Leave room for too large numbers!              
                lcd.setCursor(0,1);
                lcd.print( buffer);
                digitalWrite(StatusLed,HIGH);  
                dmx_master.setChannelValue ( ADDRESS[j].channel, sensorValue );
                uint8_t buttons = lcd.readButtons();
                if(buttons & BUTTON_RIGHT) { button_state[j] = HIGH;DoAddress();}
                if(buttons & BUTTON_LEFT) { button_state[j] = HIGH;DoAddress ();}
                }
                digitalWrite(LED7,LOW);
          break;
          }
       case 7:
          {
              // send value to channel
              while (button_state[j] == LOW) 
                {
                digitalWrite(LED8,HIGH);
                lcd.setCursor(0,0);
                lcd.print("CHANNEL:");
                dtostrf(ADDRESS[j].channel, 3, 0, buffer); // Leave room for too large numbers!
                lcd.setCursor(9,0);
                lcd.print( buffer);  
                int sensor_Value = analogRead(SLIDER8);
                int sensorValue = sensor_Value/4;
                dtostrf(sensorValue, 3, 0, buffer); // Leave room for too large numbers!              
                lcd.setCursor(0,1);
                lcd.print( buffer);
                digitalWrite(StatusLed,HIGH);  
                dmx_master.setChannelValue ( ADDRESS[j].channel, sensorValue );
                uint8_t buttons = lcd.readButtons();
                if(buttons & BUTTON_RIGHT) { button_state[j] = HIGH;DoAddress();}
                if(buttons & BUTTON_LEFT) { button_state[j] = HIGH;DoAddress ();}
                }
                digitalWrite(LED8,LOW);
          break;
          }
        
          break;
          }// end switch
   } // end DoIntensity
// DoAddress
void DoAddress(void)
  {
  char buffer[16];
  uint8_t buttons = lcd.readButtons();
    lcd.setCursor(0,0);
    lcd.print("CHANNEL:");
    lcd.setCursor(0,1);
    lcd.print("SELECT SLIDER");
    dtostrf(DMX_MASTER_ADDRESS, 3, 0, buffer); // Leave room for too large numbers!
    lcd.setCursor(9,0);
    lcd.print( buffer); 
    old_button_state[0] = button_State1;
    button_State1 = digitalRead(BUTTON1);
  if (button_State1 == LOW)
     {
     SelectVal = 0;
     int EEPROM_address = (SelectVal * sizeof(addressVals_t));
     EEPROM.put(EEPROM_address, ADDRESS[SelectVal]);
     button_state[SelectVal] = button_State1;
     if(button_State1 != old_button_state[SelectVal])
     {
       button_state[SelectVal] = button_State1;
     } else {
        button_state[SelectVal] = old_button_state[SelectVal];
     }
     ADDRESS[SelectVal].channel = DMX_MASTER_ADDRESS;
     cleanScreen();
     DoIntensity(SelectVal);              
     }// end button_state1
     old_button_state[1] = button_State2;
     button_State2 = digitalRead(BUTTON2);
   if (button_State2 == LOW)
     {
     SelectVal = 1;
     int EEPROM_address = (SelectVal * sizeof(addressVals_t));
     EEPROM.put(EEPROM_address, ADDRESS[SelectVal]);
     button_state[SelectVal] = button_State2;
     if(button_State2 != old_button_state[SelectVal])
     {
       button_state[SelectVal] = old_button_state[SelectVal];
     } else {
        button_state[SelectVal] = old_button_state[SelectVal];
     }
     ADDRESS[SelectVal].channel = DMX_MASTER_ADDRESS;
     cleanScreen();
     DoIntensity(SelectVal);              
     }// end button_state2
    old_button_state[2] = button_State3;
    button_State3 = digitalRead(BUTTON3);
  if (button_State3 == LOW)
     {
     SelectVal = 2;
     int EEPROM_address = (SelectVal * sizeof(addressVals_t));
     EEPROM.put(EEPROM_address, ADDRESS[SelectVal]);
     button_state[SelectVal] = button_State3;
  if(button_State3 != old_button_state[SelectVal])
     {
       button_state[SelectVal] = button_State3;
     } else {
        button_state[SelectVal] = old_button_state[SelectVal];
     }
     ADDRESS[SelectVal].channel = DMX_MASTER_ADDRESS;
     cleanScreen();
     DoIntensity(SelectVal);              
     }// end button_state3
   old_button_state[3] = button_State4;
   button_State4 = digitalRead(BUTTON4);
  if (button_State4 == LOW)
     {
      SelectVal = 3;
     int EEPROM_address = (SelectVal * sizeof(addressVals_t));
     EEPROM.put(EEPROM_address, ADDRESS[SelectVal]);
     button_state[SelectVal] = button_State4;
     button_State4 = digitalRead(BUTTON4);
  if(button_State4 != old_button_state[SelectVal])
     {
       button_state[SelectVal] = button_State4;
     } else {
        button_state[SelectVal] = old_button_state[SelectVal];
     }
     ADDRESS[SelectVal].channel = DMX_MASTER_ADDRESS;
     cleanScreen();
     DoIntensity(SelectVal);              
     }// end button_state4
     old_button_state[4] = button_State5;
     button_State5 = digitalRead(BUTTON5);
  if (button_State5 == LOW)
     {
      SelectVal = 4;
     int EEPROM_address = (SelectVal * sizeof(addressVals_t));
     EEPROM.put(EEPROM_address, ADDRESS[SelectVal]);
     button_state[SelectVal] = button_State5;
     button_State5 = digitalRead(BUTTON5);
  if(button_State5 != old_button_state[SelectVal])
     {
       button_state[SelectVal] = button_State5;
     } else {
        button_state[SelectVal] = old_button_state[SelectVal];
     }
     ADDRESS[SelectVal].channel = DMX_MASTER_ADDRESS;
     cleanScreen();
     DoIntensity(SelectVal);              
     }// end button_state5
     old_button_state[5] = button_State6;
     button_State6 = digitalRead(BUTTON6);
  if (button_State6 == LOW)
     {
     SelectVal = 5;
     int EEPROM_address = (SelectVal * sizeof(addressVals_t));
     EEPROM.put(EEPROM_address, ADDRESS[SelectVal]);
     button_state[SelectVal] = button_State6;
     if(button_State6 != old_button_state[SelectVal])
     {
       button_state[SelectVal] = button_State6;
     } else {
        button_state[SelectVal] = old_button_state[SelectVal];
     }
     ADDRESS[SelectVal].channel = DMX_MASTER_ADDRESS;
     cleanScreen();
     DoIntensity(SelectVal);              
     }// end button_state6
     old_button_state[6] = button_State7;
     button_State7 = digitalRead(BUTTON7);
  if (button_State7 == LOW)
     {
     SelectVal = 6;
     int EEPROM_address = (SelectVal * sizeof(addressVals_t));
     EEPROM.put(EEPROM_address, ADDRESS[SelectVal]);
     button_state[SelectVal] = button_State7;
     if(button_State7 != old_button_state[SelectVal])
     {
       button_state[SelectVal] = button_State7;
     } else {
        button_state[SelectVal] = old_button_state[SelectVal];
     }
     ADDRESS[SelectVal].channel = DMX_MASTER_ADDRESS;
     cleanScreen();
     DoIntensity(SelectVal);              
     }// end button_state7  
     old_button_state[7] = button_State8;
     button_State8 = digitalRead(BUTTON8);
  if (button_State8 == LOW)
     {
     SelectVal = 7;
     int EEPROM_address = (SelectVal * sizeof(addressVals_t));
     EEPROM.put(EEPROM_address, ADDRESS[SelectVal]);
     button_state[SelectVal] = button_State8;
     if(button_State8 != old_button_state[SelectVal])
     {
       button_state[SelectVal] = button_State8;
     } else {
        button_state[SelectVal] = old_button_state[SelectVal];
     }
     ADDRESS[SelectVal].channel = DMX_MASTER_ADDRESS;
     cleanScreen();
     DoIntensity(SelectVal);              
     }// end button_state8
       
     if(buttons & BUTTON_RIGHT)
       {
       if(DMX_MASTER_ADDRESS < 513)
          {
          DMX_MASTER_ADDRESS++;
          DMX_ADDRESS = DMX_MASTER_ADDRESS;

//        button_State1 = digitalRead(BUTTON1);
          } // end if < 513
      }// end if BUTTON_RIGHT
     if(buttons & BUTTON_LEFT)
       {
       if(DMX_MASTER_ADDRESS > 0)
         {
          DMX_MASTER_ADDRESS--;
          DMX_ADDRESS = DMX_MASTER_ADDRESS;
          } // end > 0
        }// end if BUTTON_LEFT
 } // end Do Address


//----------------------------------------------------------------------------
void setup()
  {
  PrintBegin();
  // read defult / EEPROM settings and update to current
  readSettings();
  DMX_MASTER_ADDRESS = ADDRESS[0].channel;
  // enable DMX master interface
  dmx_master.enable ();
  // Set channel 1 - 512 @ 100%
  dmx_master.setChannelRange ( 1, 512, 255 );
  // Set  pin as output pin
  pinMode ( StatusLed, OUTPUT );
  pinMode ( BUTTON1, INPUT );
  digitalWrite ( BUTTON1, HIGH );
  pinMode ( BUTTON2, INPUT_PULLUP );
  digitalWrite ( BUTTON2, HIGH );
  pinMode ( BUTTON3, INPUT_PULLUP );
  digitalWrite ( BUTTON3, HIGH );
  pinMode ( BUTTON4, INPUT_PULLUP );
  digitalWrite ( BUTTON4, HIGH );
  pinMode ( BUTTON5, INPUT_PULLUP );
  digitalWrite ( BUTTON5, HIGH );
  pinMode ( BUTTON6, INPUT_PULLUP );
  digitalWrite ( BUTTON6, HIGH );
  pinMode ( BUTTON7, INPUT_PULLUP );
  digitalWrite ( BUTTON7, HIGH );
  pinMode ( BUTTON8, INPUT_PULLUP );
  digitalWrite ( BUTTON8, HIGH );

  pinMode ( LED1, OUTPUT );
  pinMode ( LED2, OUTPUT );
  pinMode ( LED3, OUTPUT );
  pinMode ( LED4, OUTPUT );
  pinMode ( LED5, OUTPUT );
  pinMode ( LED6, OUTPUT );
  pinMode ( LED7, OUTPUT );
  pinMode ( LED8, OUTPUT );

  //pinMode (SLIDER5, INPUT );
  //pinMode (SLIDER6, INPUT );
  //pinMode (SLIDER7, INPUT );
  //pinMode (SLIDER8, INPUT );

  SplashScreen();
  }
//============================================================================

void loop()
  {
// Check if the DMX master is waiting for a break
    // to happen
    if ( dmx_master.waitingBreak () )
    {      
        // 50 ms interframe delay
        delay ( 50 );

        // Generate break and continue transmitting the next frame
        dmx_master.breakAndContinue ( break_usec );
    }

   DoAddress();

  }
//==========================================
// during runtime you can use the enable() and diable() functions to stop
// and start transmitting.  this gives you the flexibility
// to stop transmissions temporarily when you are in a time 
// critical job for example
//
// from application you can use the following two functions to set channel values
// to be buffered and transmitted.  the channel numbering always starts at 1
// and the last channel possible is 512 or any lower
// configured number of channels
// dmx_master.setChannelRange ( begin_channel, end_channel, byte_value );
// dmx_master.setChannelValue ( channel, byte_value );
//
//============================================================================
