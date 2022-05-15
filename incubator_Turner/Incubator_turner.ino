
/*
Incubator flip 
2020-11-03
Mariwan Jalal
Using Stepper motor and stepper driver

 */

#include <Stepper.h>

const int stepsPerRevolution = 1200;  // change this to fit the number of steps per revolution
// for your motor


void turnoffMotor(){
    // Turn off motors - Initial state
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  }
// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

inline void delay_6Hours(){
 turnoffMotor();
 int period = 60*60*8;
 for(int i=0; i<period;i++)
  delay(1000);  
 }
void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(25);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  // step one revolution  in one direction:
  //delay_6Hours();
  Serial.println("anticlockwise");
     myStepper.step(-65);
     delay_6Hours();
       Serial.println("clockwise");
     myStepper.step(65);
}
