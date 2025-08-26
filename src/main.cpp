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
#define THRESHOLD 2 //threshold for current sensor to detect if motor is stopped

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
  LEDcomb3,
  Error
  };

//keep track of current state and set first state
static MotorState M_State = MotorState::M_Wait;

//keep track of current state and set first state
static LEDState LED_State = LEDState::LEDcomb1;

unsigned long softstartStartTime = 0;
bool softstartActive = false;

unsigned long startTimeRunning = millis();
unsigned long randomSpinTime = 3000;
unsigned long stopStartTime = millis();
unsigned long errorStartTime = 0;
bool errorActive = false;

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

   // Default to 3 seconds

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
        startTimeRunning = millis();
        randomSpinTime = random(1.9, 8.5)*1000; // Random duration between 5 and 15 seconds
      }

      break;
  
    case MotorState::M_Running:
      if (millis() - startTimeRunning >= randomSpinTime) { //after 10 seconds switch to next state
        M_State = MotorState::M_SpinFreely;
      }
      break;

    case MotorState::M_SpinFreely:
      if(CurrentSensor < THRESHOLD){ //if current is below threshold switch to next state
        M_State = MotorState::M_Stop;
        stopStartTime = millis();
      }
      break;

    case MotorState::M_Stop:
      digitalWrite(M_IN1, LOW); //stop motor
      digitalWrite(M_IN2, LOW);
      LED_State = LEDState::LEDcomb3;
      if(millis() - stopStartTime >= 2000){ //after 5 seconds switch to wait state
        M_State = MotorState::M_Wait;
      }
      break;
  
    default:  //Error state
      if(errorActive = false){
        errorActive = true;
        errorStartTime = millis();
      }
      
      LED_State = LEDState::Error;
      if(millis() - errorStartTime >= 15000){ //after 15 seconds switch to wait state
        digitalWrite(M_IN1, LOW); //stop motor
        digitalWrite(M_IN2, LOW);
      } else {
        digitalWrite(M_IN1, HIGH); //laat motor gaan
        digitalWrite(M_IN2, HIGH);
      }
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
    
    case LEDState::Error:
      // Rode snel knipperende lichten
      break;
  }
  
}
