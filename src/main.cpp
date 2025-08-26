#include <Arduino.h>
#include <FastLED.h>

#define Button 2      //internal pull-up
#define M_PWM 3
#define M_IN1 4
#define M_IN2 5
#define LED 6
#define CurrentSensor A5

#define MAX_SPEED 200 //max speed of motor
#define MS_TOPSPEED 3000 //ms till top speed is reached


enum class MotorState { //setting up all possible states execpt error
    M_Wait,
    M_Starting,
    M_Running,
    M_SpinFreely,
    M_Stop
  };

enum class LEDState { //setting up all possible states execpt error
  LEDcomb1,
  LEDcomb2,
  LEDcomb3
  };

//keep track of current state and set first state
static MotorState M_State = MotorState::M_Wait;

//keep track of current state and set first state
static LEDState LED_State = LEDState::LEDcomb1;

unsigned long softstartStartTime = 0;
bool softstartActive = false;

boolean SoftstartFunction() {
    static int currentSpeed = 0;
    unsigned long now = millis();

    if (!softstartActive) {
        softstartStartTime = now;
        currentSpeed = 0;
        softstartActive = true;
        // Set motor direction (example: forward)
        digitalWrite(M_IN1, HIGH);
        digitalWrite(M_IN2, LOW);
    }

    unsigned long elapsed = now - softstartStartTime;
    if (elapsed < MS_TOPSPEED) {
        // Ramp up speed
        currentSpeed = map(elapsed, 0, MS_TOPSPEED, 0, MAX_SPEED);
        analogWrite(M_PWM, currentSpeed);
        return false; // Not finished yet
    } else {
        analogWrite(M_PWM, MAX_SPEED);
        softstartActive = false;
        return true; // Finished
    }
}

void setup() {
  pinMode(Button, INPUT_PULLUP);
  pinMode(M_PWM, OUTPUT);
  pinMode(M_IN1, OUTPUT);
  pinMode(M_IN2, OUTPUT);
  pinMode(LED, OUTPUT);
  Serial.begin(9600);


  //start init


  //end init    after init LEDcomb1 should be active
}

void loop() {
  
  switch(M_State) { //setting the motor states

    case MotorState::M_Wait:
      
      if(Button == HIGH){
        M_State = MotorState::M_Starting;
        LED_State = LEDState::LEDcomb1;
      }
      
      break;

    case MotorState::M_Starting:
      if(SoftstartFunction() == true){
        M_State = MotorState::M_Running;
        LED_State = LEDState::LEDcomb2;
      }

      break;
  
    case MotorState::M_Running:
      // Your code here
      break;

    case MotorState::M_SpinFreely:
      // Your code here
      break;

    case MotorState::M_Stop:
      LED_State = LEDState::LEDcomb3;
      break;
  
    default:  //Error state
      // Your code here
      break;
  }

  switch (LED_State)   //setting the LED states
  {
    case LEDState::LEDcomb1:
      // Your code here
      break;

    case LEDState::LEDcomb2:
      // Your code here
      break;

    case LEDState::LEDcomb3:
      // Your code here
      break;
  }
  
}
