#include <AccelStepper.h>

// for the Arduino Uno + CNC shield V3 + A4988 + FL42STH47-1684A

#define MOTOR_X_ENABLE_PIN 8
#define MOTOR_X_STEP_PIN 2
#define MOTOR_X_DIR_PIN 5
#define MOTOR_Y_ENABLE_PIN 8
#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6
#define MOTOR_Z_ENABLE_PIN 8
#define MOTOR_Z_STEP_PIN 4
#define MOTOR_Z_DIR_PIN 7

void setup()
{
  Serial.begin(115200);
  motor_X.setEnablePin(MOTOR_X_ENABLE_PIN);
  motor_X.setPinsInverted(false, false, true);
  motor_X.setAcceleration(800000);  
  motor_X.setMaxSpeed(8000);
  motor_X.moveTo(0);
}

void loop()
{
  
}
