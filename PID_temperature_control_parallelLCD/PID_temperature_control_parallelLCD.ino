/**
*
*Inspired by http://www.electronoobs.com/eng_arduino_tut24_code3.php
*Author :Mariwan Jalal
*PID Controller with Parallel LCD, rotary encoder and EEProm for saving setting.
*
 */
#include <SPI.h>
#include <Rotary.h>
#include <Eeprom24C128_256.h>

//LCD config
#include <Wire.h> 
// include the library code:
#include <LiquidCrystal.h>

//LM35ND
#include <PreciseLM35.h>
const float StepsToMaxPWMOutput= 0.147058824;   // 37,5/255

const int RS = 10;
const int EN = 2;
const int D4 = 4;
const int D5 = 5;
const int D6 = 6;
const int D7 = 7;
const int pinLM35 = A0;

LiquidCrystal lcd(RS,EN,D4,D5,D6,D7);
PreciseLM35 lm35(pinLM35, DEFAULT);

/**************************************************************************//**
 * \def EEPROM_ADDRESS
 * \brief Address of EEPROM memory on TWI bus.
 ******************************************************************************/
#define EEPROM_ADDRESS  0x50

/******************************************************************************
 * Private variable definitions.
 ******************************************************************************/

static Eeprom24C128_256 eeprom(EEPROM_ADDRESS);



//I/O
int PWM_pin = 3;  //Pin for PWM signal to the MOSFET driver (the BJT npn with pullup)

//Rotary encoder
int clk = 8;      //Pin 1 from rotary encoder
int Ddata = 9;     //Pin 2 from rotary encoder

int dirCW=0;
int dirCCW=0;

int Sswitch=11;   // pin switch rotary encoder

//Variables
float set_temperature = 0;            //Default temperature setpoint. Leave it 0 and control it with rotary encoder
float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
int button_pressed = 0;
int menu_activated = 0;
float last_set_temperature = 0;

//Variables for rotary encoder state detection
int clk_State;
int Last_State;
bool dt_State;

//PID constants
//////////////////////////////////////////////////////////
byte kp = 90;   byte ki = 30;   byte kd = 80;
//////////////////////////////////////////////////////////

int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int last_ksave=1;
byte menuChanged=0;  // This will be active whenever menu changes to display the change.
bool ksave=0;
int menu5buttonPressed=0;
int PID_values_fixed = 0;

Rotary rotary = Rotary( Ddata,clk);
  


// rotate is called anytime the rotary inputs change state.
void inline rotate() {
  unsigned char result = rotary.process();
  if (result == DIR_CW) {
    dirCW=1;
    dirCCW=0;
     Serial.println("rotate cw");
    }
   else if (result == DIR_CCW){
    dirCW=0;
    dirCCW=1;
     Serial.println("rotate ccw");
   }
}

//Check if the push button is pressed
void inline SwitchPressed(void){
    //Push button was pressed!
  if (digitalRead(Sswitch)==1) 
  {
    button_pressed = 0;
  }else{
    delay(10);//wait 10 msec
    if (digitalRead(Sswitch)==0){
      button_pressed = 1;
      
      Serial.println("Switch");
     }
   }
     dirCW=0;
     dirCCW=0;
}
//Do all necessary calculation and action regarding chaning the rotary encoder
void inline MenyAction(void) {
  if (menu_activated == 1)
  {
  if(dirCW){
        set_temperature = set_temperature + 0.5;
      }
  else if(dirCCW){
        set_temperature = set_temperature - 0.5;
      }
      dirCCW=0;
      dirCW=0;  
    }
 if (menu_activated == 2)
  {
   if(dirCW){
        kp = kp + 1;
        
      }
      else if(dirCCW) {
        kp = kp - 1;
      }
      dirCCW=0;
      dirCW=0;  
    }
 if (menu_activated == 3)
  {
      if(dirCW){
        ki = ki + 1;
      }
      else if(dirCCW) {
        ki = ki - 1;
      }
      dirCCW=0;
      dirCW=0; 
    }

 if (menu_activated == 4)
  {
     if(dirCW) {
        kd = kd + 1;
      }
      else if(dirCCW) {
        kd = kd - 1;
      }
      dirCCW=0;
      dirCW=0; 
    }
  if (menu_activated == 5)
  {
     if(dirCCW==1 || dirCW==1) {
        ksave=!ksave;
      }
      dirCCW=0;
      dirCW=0; 
  }
//Menu change and accepting value.
 if (button_pressed == 1)
  {
    if(menu_activated==5){
      menu5buttonPressed++;
      if(menu5buttonPressed==2){
      menu_activated =0;
      button_pressed = 0;
      menu5buttonPressed=0;
      Serial.println("Out Menu5");
      PID_values_fixed=1;
      if(ksave==1){
           ksave=0;
           writeSetting(0,set_temperature,kp,ki,kd);
        } 
       }       
      delay(500);
      }
    if (menu_activated == 4)
    {
      menu_activated =menu_activated + 1;
      PID_values_fixed = 1;
      menuChanged=1;
      button_pressed = 0;
      delay(500);
      menu5buttonPressed++;// First Menu 5 button is pressed here.
    }

  if (menu_activated == 3)
    {
      menu_activated = menu_activated + 1;
      button_pressed = 0;
      menuChanged=1;
      delay(500);
    }

   if (menu_activated == 2)
    {
      menu_activated = menu_activated + 1;
      button_pressed = 0;
      menuChanged=1;
      delay(500);
    }

    if (menu_activated == 1)
    {
      menu_activated = menu_activated + 1;
      button_pressed = 0;
      menuChanged=1;
      delay(500);
    }

   if (menu_activated == 0 && PID_values_fixed != 1)
    {
      menu_activated = menu_activated + 1;
      button_pressed = 0;
      menuChanged=1;
      delay(500);
    }
    PID_values_fixed = 0;
  }
}


// EEPROM AT 24C128n read PID and Temp set.
inline byte readEEprom(int address){
  return (eeprom.readByte(address));
  }
//EEPROM AT 24C128n to save data 
inline void writeEEprom(byte address, byte data){
    eeprom.writeByte(address, data);
      delay(10);
  }

// Writing setting the to default address   
inline void  writeSetting(byte address, float &TempSet, byte &p, byte &i, byte &d){
  // Temp will be written to the first two bytes.
  word temperature= (word)(TempSet*100);
  byte temp[2];
  temp[0] = (byte)(temperature);
  temp[1] = (byte)(temperature>>8);

  writeEEprom(address+0,temp[0]);
  writeEEprom(address+1,temp[1]);

  writeEEprom(address+2,p);
  writeEEprom(address+3,i);
  writeEEprom(address+4,d);
}


inline void readSetting(int address, float &TempSet, byte &p, byte &i, byte &d){
  // Temp will be written to the first two bytes. 
  TempSet= readEEprom(address+0);
  word x=readEEprom(address+1)<<8;
  TempSet=TempSet+ x;
  TempSet=TempSet/100;
  Serial.println("tempread=");
  Serial.println(TempSet);
  p=readEEprom(address+2);
  i=readEEprom(address+3);
  d=readEEprom(address+4);
  }



//The function that reads the SPI data from lm35
double inline readThermocouple() {
 return lm35.readCelsius();
}

  
void setup() {
  pinMode(PWM_pin, OUTPUT);
//  TCCR2B = TCCR2B & B11111000 | 0x03;    // pin 3 and 11 PWM frequency of 928.5 Hz

//TCCR2B = TCCR2B & B11111000 | 0x01;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | 0x02;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | 0x03;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  TCCR2B = TCCR2B & B11111000 | 0x04;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
//TCCR2B = TCCR2B & B11111000 | 0x05;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | 0x06;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | 0x07;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz



    
  Time = millis();
/*
  Last_State = (PINB & B00000001);      //Detect first state of the encoder

  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change. 
  PCMSK0 |= (1 << PCINT1);  //Set pin D9 trigger an interrupt on state change. 
  PCMSK0 |= (1 << PCINT3);  //Set pin D11 trigger an interrupt on state change.   
*/
 
  pinMode(clk, INPUT);   //clk
  pinMode(Ddata, INPUT);    //data
  pinMode(Sswitch, INPUT);   //Switch

// Initialize EEPROM library.
 eeprom.initialize();

 const word EEpromAddress = 0;
    
  lcd.begin(16, 2);
  lcd.setCursor(0, 1);
  Serial.begin(115200);
  readSetting(0,set_temperature,kp,ki,kd);
  last_set_temperature=set_temperature;  //They are the same in the begining.
}

void loop() {
  SwitchPressed();    //Check if push button is pressed 
  rotate();           //Check if rotation of the rotary encoder is detected
  
  //Display the results on the Screen
  if (menu_activated == 0)  
  {
    // First we read the real value of temperature
    temperature_read = readThermocouple();
    //Next we calculate the error between the setpoint and the real value
    PID_error = set_temperature - temperature_read + 1;
    //Calculate the P value
    PID_p = StepsToMaxPWMOutput*kp * PID_error;
    //Calculate the I value in a range on +- 0.1                                                                 //We have ±0.5 .. it was ±3
   // PID should work only between 0,5 , -0,5 error 
   if( PID_error<-0.5  ||  PID_error>0.5) 
      PID_i = PID_i + StepsToMaxPWMOutput*(ki * PID_error);
   else 
     PID_i=0;
     
    //For derivative we need real time to calculate speed change rate
    timePrev = Time;                            // the previous time is stored before the actual time read
    Time = millis();                            // actual time read
    elapsedTime = (Time - timePrev) / 1000;
    //Now we can calculate the D calue
    PID_d = StepsToMaxPWMOutput*kd*((PID_error - previous_error) / elapsedTime);
    //Final total PID value is the sum of P + I + D
    PID_value = PID_p + PID_i + PID_d;

    //We define PWM range between 0 and 255
    if (PID_value < 0)
    {
      PID_value = 0;
    }
    if (PID_value > 255)
    {
      PID_value = 255;
    }
    //Now we can write the PWM signal to the mosfet on digital pin D3
    //Since we activate the MOSFET with a 0 to the base of the BJT, we write 255-PID value (inverted)
    analogWrite(PWM_pin, 255 - PID_value);
     Serial.println("......");
    Serial.println(255 - PID_value);
    
    Serial.println(PID_p);
    Serial.println(PID_i );
    Serial.println(PID_d);

     Serial.println("......");
    previous_error = PID_error;     //Remember to store the previous error for next loop.

    delay(250); //Refresh rate + delay of LCD print
    //lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("PID TEMP control");
    lcd.setCursor(0, 1);
    lcd.print("S:");
    lcd.setCursor(2, 1);
    lcd.print(set_temperature, 1);
    lcd.setCursor(9, 1);
    lcd.print("R:");
    lcd.setCursor(11, 1);
    lcd.print(temperature_read, 1);
  }//end of menu 0 (PID control)

//First page of menu (temp setpoint)
  if (menu_activated == 1)
  {
    analogWrite(PWM_pin, 255);
    if (set_temperature != last_set_temperature || menuChanged==1)
    {
       if(menuChanged==1)  menuChanged=0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set temp    P1/5");
      lcd.setCursor(0, 1);
      lcd.print(set_temperature);
    }
    last_set_temperature = set_temperature;
  }//end of menu 1

  //Second page of menu (P set)
  if (menu_activated == 2)
  {
    if (kp != last_kp || menuChanged==1)
    {
      if(menuChanged==1)  menuChanged=0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set  P  val P2/5");
      lcd.setCursor(0, 1);
      lcd.print(kp);
    }
    last_kp = kp;
  }//end of menu 2

  //Third page of menu (I set)
  if (menu_activated == 3)
  {
    if (ki != last_ki || menuChanged==1)
    {
      if(menuChanged==1)  menuChanged=0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set  I  val P3/5");
      lcd.setCursor(0, 1);
      lcd.print(ki);
    }
    last_ki = ki;
  }//end of menu 3


  //Forth page of menu (D set)
  if (menu_activated == 4)
  {
    if (kd != last_kd  || menuChanged==1)
    {
      if(menuChanged==1)  menuChanged=0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set D  val  P4/5");
      lcd.setCursor(0, 1);
      lcd.print(kd);
    }
    last_kd = kd;
  }//end of menu 4


  //Fifth page of menu (Save setting)
  if (menu_activated == 5)
  {
    if (ksave != last_ksave || menuChanged==1)
    {
      if(menuChanged==1)  menuChanged=0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Setting   P5/5");
      lcd.setCursor(0, 1);
      if(ksave==1) 
       lcd.print("Save");
      else
       lcd.print("Cancel");
    }
    last_ksave = ksave;
  }//end of menu 5
  
 MenyAction();  //Update the status of the menus and do the action required. 
}//Loop end
