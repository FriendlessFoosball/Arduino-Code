#include <AccelStepper.h>

// for the Arduino Uno + CNC shield V3 + A4988 + FL42STH47-1684A

#define MOTOR_X_ENABLE_PIN 8
#define MOTOR_X_STEP_PIN 2
#define MOTOR_X_DIR_PIN 5
#define MOTOR_X_LOWER_LIMIT_PIN 51
#define MOTOR_X_UPPER_LIMIT_PIN 50
#define MOTOR_Y_ENABLE_PIN 8
#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6
#define MOTOR_Y_LOWER_LIMIT_PIN 53
#define MOTOR_Y_UPPER_LIMIT_PIN 52
#define MOTOR_Z_ENABLE_PIN 8
#define MOTOR_Z_STEP_PIN 4
#define MOTOR_Z_DIR_PIN 7
#define MOTOR_A_ENABLE_PIN 8
#define MOTOR_A_STEP_PIN 12
#define MOTOR_A_DIR_PIN 13
#define NUM_MOTORS 4

// initialize motor array
AccelStepper motor_X(1, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper motor_Y(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper motor_Z(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);
AccelStepper motor_A(1, MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);
AccelStepper *motors[NUM_MOTORS] = {&motor_X, &motor_Y, &motor_Z, &motor_A};

unsigned char serialBuffer[10];
unsigned char sendBuffer[8];
int bufferIndex = 0;
int sendIndex = 0;
int sendMax = 0;
bool dataReady = false;
bool positionControl[NUM_MOTORS] = {true, true, true, true};
short zeroPositions[NUM_MOTORS] = {0, 0, 0, 0};
bool movingForward[NUM_MOTORS] = {true, true, true, true};

void setup()
{
  SerialUSB.begin(1843200);

  pinMode(MOTOR_X_LOWER_LIMIT_PIN, INPUT);
  pinMode(MOTOR_Y_LOWER_LIMIT_PIN, INPUT);
  pinMode(MOTOR_X_UPPER_LIMIT_PIN, INPUT);
  pinMode(MOTOR_Y_UPPER_LIMIT_PIN, INPUT);

  // initialize motors
  (*motors[0]).setEnablePin(MOTOR_X_ENABLE_PIN);
  (*motors[1]).setEnablePin(MOTOR_Y_ENABLE_PIN);
  (*motors[2]).setEnablePin(MOTOR_Z_ENABLE_PIN);
  (*motors[3]).setEnablePin(MOTOR_A_ENABLE_PIN);
  for(int i=0; i < NUM_MOTORS; i++)
  {
    (*motors[i]).setPinsInverted(false, false, true);
    (*motors[i]).setAcceleration(800000);  
    (*motors[i]).setMaxSpeed(8000);
    (*motors[i]).moveTo(0);
    (*motors[i]).enableOutputs();
  }
}

void loop()
{
  if(dataReady) // if we have received a full command
  {
    // get 4-bit motor address and command
    unsigned char motor = serialBuffer[0] & 0x03;
    unsigned char command = (serialBuffer[0] >> 2) & 0x3F;

    switch(command)
    {
      case 0x00: // zero motor
      {
        (*motors[motor]).moveTo(zeroPositions[motor]);
        break;
      }
      case 0x01: // set zero position
      {
        zeroPositions[motor] = (*motors[motor]).currentPosition();
        break;
      }
      case 0x02: // set speed
      {
        (*motors[motor]).setSpeed( (float)(short)((((unsigned short)serialBuffer[1]) << 8) | (unsigned short)serialBuffer[2]) );
        positionControl[motor] = false;
        if((*motors[motor]).speed() >= 0) movingForward[motor] = true;
        else movingForward[motor] = false;
        break;
      }
      case 0x03: // set position
      {
        (*motors[motor]).moveTo( (float)(short)(((((unsigned short)serialBuffer[1]) << 8) | (unsigned short)serialBuffer[2])+zeroPositions[motor]) );
        positionControl[motor] = true; 
        if((*motors[motor]).distanceToGo() > 0) movingForward[motor] = true;
        else movingForward[motor] = false;
        break;
      }
      case 0x04: // get position
      {
        long currPos = (*motors[motor]).currentPosition();
        sendBuffer[0] = (unsigned char)(char)((currPos & 0xFF00) >> 8);
        sendBuffer[1] = (unsigned char)(char)(currPos & 0xFF);
        sendIndex = 0;
        sendMax = 2;
        break;
      }
      case 0x05: // set all positions
      {
        (*motors[0]).moveTo( (float)(short)(((((unsigned short)serialBuffer[1]) << 8) | (unsigned short)serialBuffer[2])+zeroPositions[0]) );
        (*motors[1]).moveTo( (float)(short)(((((unsigned short)serialBuffer[3]) << 8) | (unsigned short)serialBuffer[4])+zeroPositions[1]) );
        (*motors[2]).moveTo( (float)(short)(((((unsigned short)serialBuffer[5]) << 8) | (unsigned short)serialBuffer[6])+zeroPositions[2]) );
        (*motors[3]).moveTo( (float)(short)(((((unsigned short)serialBuffer[7]) << 8) | (unsigned short)serialBuffer[8])+zeroPositions[3]) );
        for(int i=0; i < NUM_MOTORS; i++)
        {
          positionControl[i] = true; 
          if((*motors[motor]).distanceToGo() > 0) movingForward[i] = true;
          else movingForward[i] = false;
        }
        break;
      }
      case 0x06: // set all speeds
      {
        (*motors[0]).setSpeed( (float)(short)((((unsigned short)serialBuffer[1]) << 8) | (unsigned short)serialBuffer[2]) );
        (*motors[1]).setSpeed( (float)(short)((((unsigned short)serialBuffer[3]) << 8) | (unsigned short)serialBuffer[4]) );
        (*motors[2]).setSpeed( (float)(short)((((unsigned short)serialBuffer[5]) << 8) | (unsigned short)serialBuffer[6]) );
        (*motors[3]).setSpeed( (float)(short)((((unsigned short)serialBuffer[7]) << 8) | (unsigned short)serialBuffer[8]) );
        for(int i = 0; i < NUM_MOTORS; i++)
        {
          positionControl[i] = false;
          if((*motors[i]).speed() >= 0) movingForward[i] = true;
          else movingForward[i] = false;
        }
        break;
      }
      case 0x07: // get all positions
      {
        long currPos = (*motors[0]).currentPosition() + zeroPositions[0];
        sendBuffer[0] = (unsigned char)(char)((currPos & 0xFF00) >> 8);
        sendBuffer[1] = (unsigned char)(char)(currPos & 0xFF);
        currPos = (*motors[1]).currentPosition() + zeroPositions[1];
        sendBuffer[2] = (unsigned char)(char)((currPos & 0xFF00) >> 8);
        sendBuffer[3] = (unsigned char)(char)(currPos & 0xFF);
        currPos = (*motors[2]).currentPosition() + zeroPositions[2];
        sendBuffer[4] = (unsigned char)(char)((currPos & 0xFF00) >> 8);
        sendBuffer[5] = (unsigned char)(char)(currPos & 0xFF);
        currPos = (*motors[3]).currentPosition() + zeroPositions[3];
        sendBuffer[6] = (unsigned char)(char)((currPos & 0xFF00) >> 8);
        sendBuffer[7] = (unsigned char)(char)(currPos & 0xFF);
        sendIndex = 0;
        sendMax = 8;
        break;
      }
      default: // invalid command
      {
        sendBuffer[0] = 0xFF;
        sendBuffer[1] = 0xFF;
        sendIndex = 0;
        break;
      }
    }
    dataReady = false;
    bufferIndex = 0;
  }

  // for each motor, update either speed or position control
  for(int i=0; i < NUM_MOTORS; i++)
  {
    // only send a tick if we are not hitting a limit switch (only for X and Y)
    bool limitHit = true;
    if(i > 1) limitHit = false;
    else if(i == 0 && ((movingForward[i] & digitalRead(MOTOR_X_UPPER_LIMIT_PIN)) || (!movingForward[i] & digitalRead(MOTOR_X_LOWER_LIMIT_PIN)))) limitHit = false;
    else if(i == 1 && ((movingForward[i] & digitalRead(MOTOR_Y_UPPER_LIMIT_PIN)) || (!movingForward[i] & digitalRead(MOTOR_Y_LOWER_LIMIT_PIN)))) limitHit = false;
    if(!limitHit)
    {
      if(positionControl[i])
      {
        (*motors[i]).run();
      }
      else
      {
        (*motors[i]).runSpeed();
      }
    }
  }
  
  // call SerialEvent manually because Arduino Due firmware is awful
  if (SerialUSB.available()) serialEvent();
  // call our writing function one byte at a time to prevent blocking
  if (SerialUSB.availableForWrite() && sendIndex < sendMax) serialWrite();
}

// "interrupt" for serial data receiving between iterations of loop()
void serialEvent() {
  while (SerialUSB.available()) {
    serialBuffer[bufferIndex] = (unsigned char)SerialUSB.read();
    if(serialBuffer[bufferIndex] == 0x0A)
    {
      dataReady = true;
    }
    bufferIndex++;
  }
}

void serialWrite() {
  while (SerialUSB.availableForWrite() && sendIndex < sendMax) {
    SerialUSB.write(sendBuffer[sendIndex]);
    sendIndex++;
  }
}
