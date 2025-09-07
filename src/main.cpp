#include <Arduino.h>
#include <FastLED.h>
#include "OneButton.h"

#define Button 8      //internal pull-up
#define M_PWM 3
#define M_IN1 4
#define M_IN2 5
#define LED 6
#define CurrentSensor A5

OneButton button(Button,true);

#define MAX_SPEED 200 //max speed of motor
#define MS_TOPSPEED 3000 //ms till top speed is reached
#define MS_TOZERO 2000 //ms to go from top speed to zero
#define THRESHOLD 2 //threshold for current sensor to detect if motor is stopped
#define MODES [/*normal*/2,5,1,/*fast*/1,3,0,/*slow*/3,7,2] //array with different modes for random spin time in seconds
#define SELECTED_MODE 0 //select mode from MODES array
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

static int currentSpeed = 0;
int easingMode = 1; // 0=Linear, 1=Quadratic, 2=Cubic, 3=Exponential


unsigned long softstartStartTime = 0;
bool softstartActive = false;

unsigned long startTimeRunning = millis();
unsigned long randomSpinTime = 3000;
unsigned long stopStartTime = millis();
unsigned long errorStartTime = 0;
bool errorActive = false;

float offset = 512.03; // replace by value from calibration above
float sensitivity = 0.100; // for ACS712-20A. Use 0.185 for 5A or 0.066 for 30A
float current = 0;

const int sampleCount = 150;
int samplesTaken = 0;
float currentSum = 0;
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 3;  // ms between samples
float currentAmps = 0;

boolean pressed = false;

boolean SoftstartFunction() {
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

boolean softEndFunction() {
    unsigned long now = millis();

    if (!softstartActive) {
        softstartStartTime = now;
        currentSpeed = MAX_SPEED;
        softstartActive = true;
    }

    unsigned long elapsed = now - softstartStartTime;
    float progress = (float)elapsed / MS_TOZERO;
    if (progress > 1.0) progress = 1.0;

    // Pick easing function
    float factor;
    switch (easingMode) {
      case 0: // Linear
        factor = 1.0 - progress;
        break;
      case 1: // Quadratic ease-out
        factor = 1.0 - progress * progress;
        break;
      case 2: // Cubic ease-out
        factor = 1.0 - progress * progress * progress;
        break;
      case 3: // Exponential ease-out
        factor = 1.0 - pow(progress, 5);
        break;
      default: // fallback
        factor = 1.0 - progress;
        break;
    }

    currentSpeed = (int)(MAX_SPEED * factor);
    analogWrite(M_PWM, currentSpeed);

    if (progress >= 1.0) {
        softstartActive = false;
        return true; // finished
    }
    return false; // still spinning
    }

void currentReader(){
  unsigned long now = millis();
  
    // Take a sample every sampleInterval ms
  if (now - lastSampleTime >= sampleInterval) {
    lastSampleTime = now;
    currentSum += analogRead(CurrentSensor);
    samplesTaken++;
  }

  // When enough samples gathered, calculate current
  if (samplesTaken >= sampleCount) {
    float average = currentSum / sampleCount;
    float voltage = (average - offset) * (5.0 / 1024.0);
    currentAmps = voltage / sensitivity;

    Serial.print("Measured current: ");
    Serial.println(currentAmps, 3);

    // Reset for next batch
    samplesTaken = 0;
    currentSum = 0;
  }
  if (digitalRead(Button) == LOW && !pressed){
    Serial.println("Button Pressed");
    pressed = true;
    } else if (digitalRead(Button) == HIGH && pressed){
      Serial.println("Button up");
      pressed = false;
    }
}
void doubleClick(){Serial.println("doubleClick");}
void singleClick(){pressed = true;Serial.println("singleClick");}
void longClick(){Serial.println("longClick");}

void setup() {
  pinMode(Button, INPUT_PULLUP);
  pinMode(M_PWM, OUTPUT);
  pinMode(M_IN1, OUTPUT);
  pinMode(M_IN2, OUTPUT);
  pinMode(LED, OUTPUT);
  Serial.begin(9600);

  button.attachDoubleClick(doubleClick);
  button.attachClick(singleClick);
  button.attachLongPressStop(longClick);

   // Default to 3 seconds

  //start init


  //end init    after init LEDcomb1 should be active
}

void loop() {
  button.tick();
  switch(M_State) { //setting the motor states

    case MotorState::M_Wait:
      Serial.println("In wait state");
      if(pressed == true){
        M_State = MotorState::M_Starting;
        LED_State = LEDState::LEDcomb1;
        pressed == false;
      }
      
      break;

    case MotorState::M_Starting:
      Serial.println("In starting state");
      if(SoftstartFunction() == true){
        M_State = MotorState::M_Running;
        LED_State = LEDState::LEDcomb2;
        startTimeRunning = millis();
        randomSpinTime = random(3*SELECTED_MODE +1, 3*SELECTED_MODE +2)*1000; // Random duration between 5 and 15 seconds
      }

      break;
  
    case MotorState::M_Running:
      Serial.println("In running state");
      if (millis() - startTimeRunning >= randomSpinTime) { //after random seconds switch to next state
        M_State = MotorState::M_SpinFreely;
      }
      break;

    case MotorState::M_SpinFreely:
      Serial.println("In spin freely state");
      
      if(softEndFunction){ 
        M_State = MotorState::M_Stop;
        stopStartTime = millis();
      }
      break;

    case MotorState::M_Stop:
      Serial.println("In stop state");
      digitalWrite(M_IN1, LOW); //stop motor
      digitalWrite(M_IN2, LOW);
      LED_State = LEDState::LEDcomb3;
      if(millis() - stopStartTime >= 3*SELECTED_MODE +3){ //after 2 seconds switch to wait state
        M_State = MotorState::M_Wait;
      }
      break;
  
    default:  //Error state
      Serial.println("In error state");
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
