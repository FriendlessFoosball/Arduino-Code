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

unsigned char serialBuffer[4];
unsigned char sendBuffer[2];
int bufferIndex = 0;
int sendIndex = 0;
bool dataReady = false;
bool positionControl[NUM_MOTORS] = {true, true, true, true};
long zeroPositions[NUM_MOTORS] = {0, 0, 0, 0};



void setup()
{
  SerialUSB.begin(1843200);

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
        break;
      }
      case 0x03: // set position
      {
        (*motors[motor]).moveTo( (float)(short)((((unsigned short)serialBuffer[1]) << 8) | (unsigned short)serialBuffer[2]) );
        positionControl[motor] = true; 
        break;
      }
      case 0x04: // get position
      {
        long currPos = (*motors[motor]).currentPosition();
        sendBuffer[0] = (unsigned char)(char)((currPos & 0xFF00) >> 8);
        sendBuffer[1] = (unsigned char)(char)(currPos & 0xFF);
        sendIndex = 0;
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
    if(positionControl[i])
    {
      (*motors[i]).run();
    }
    else
    {
      (*motors[i]).runSpeed();
    }
  }
  
  // call SerialEvent manually because Arduino Due firmware is awful
  if (SerialUSB.available()) serialEvent();
  // call our writing function one byte at a time to prevent blocking
  if (SerialUSB.availableForWrite() && sendIndex < 2) serialWrite();
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
  while (SerialUSB.availableForWrite() && sendIndex < 2) {
    SerialUSB.write(sendBuffer[sendIndex]);
    sendIndex++;
  }
}
