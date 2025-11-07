#include <Arduino.h>

/*
  MPU6050 DMP6

  Digital Motion Processor or DMP performs complex motion processing tasks.
  - Fuses the data from the accel, gyro, and external magnetometer if applied, 
  compensating individual sensor noise and errors.
  - Detect specific types of motion without the need to continuously monitor 
  raw sensor data with a microcontroller.
  - Reduce workload on the microprocessor.
  - Output processed data such as quaternions, Euler angles, and gravity vectors.

  The code includes an auto-calibration and offsets generator tasks. Different 
  output formats available.

  This code is compatible with the teapot project by using the teapot output format.

  Circuit: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
  depends on the MPU6050's INT pin being connected to the Arduino's
  external interrupt #0 pin.

  The teapot processing example may be broken due FIFO structure change if using DMP
  6.12 firmware version. 
    
  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki

*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_6Axis_MotionApps612.h" // Uncomment this library to work with DMP 6.12 and comment on the above library.

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/* OUTPUT FORMAT DEFINITION-------------------------------------------------------------------------------------------
- Use "OUTPUT_READABLE_QUATERNION" for quaternion commponents in [w, x, y, z] format. Quaternion does not 
suffer from gimbal lock problems but is harder to parse or process efficiently on a remote host or software 
environment like Processing.

- Use "OUTPUT_READABLE_EULER" for Euler angles (in degrees) output, calculated from the quaternions coming 
from the FIFO. EULER ANGLES SUFFER FROM GIMBAL LOCK PROBLEM.

- Use "OUTPUT_READABLE_YAWPITCHROLL" for yaw/pitch/roll angles (in degrees) calculated from the quaternions
coming from the FIFO. THIS REQUIRES GRAVITY VECTOR CALCULATION.
YAW/PITCH/ROLL ANGLES SUFFER FROM GIMBAL LOCK PROBLEM.

- Use "OUTPUT_READABLE_REALACCEL" for acceleration components with gravity removed. The accel reference frame
is not compensated for orientation. +X will always be +X according to the sensor.

- Use "OUTPUT_READABLE_WORLDACCEL" for acceleration components with gravity removed and adjusted for the world
reference frame. Yaw is relative if there is no magnetometer present.

-  Use "OUTPUT_TEAPOT" for output that matches the InvenSense teapot demo. 
-------------------------------------------------------------------------------------------------------------------------------*/ 
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
bool blinkState;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*-Packet structure for InvenSense teapot demo-*/ 
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

#define ENCA_left 2
#define ENCB_left 4
#define ENCA_right 3
#define ENCB_right 7

#define FWD_left 5 // connect all analog
#define BCK_left 10
#define FWD_right 6
#define BCK_right 9

// standard constants
#define PI 3.14159
#define HALF_OF_SQRT_3_MINUS_1 0.366025 // (sqrt(3) - 1) / 2

// useful evaluators
#define CTS_DECELERATION(setSpeed) (((setSpeed) / RATE_OF_ADJUSTMENT) * (setSpeed) * (CONTROL_STEP_MS / 1000.0) * 0.5 * ((setSpeed) < 0 ? (-1) : (1))) 

// CONSTS CONFIG

// chassis definition
#define LIENAR_DISTANCE_STEP_cm 30.0
#define WHEEL_DIAMETER_cm 6.0325
#define DISTANCE_BETWEEN_WHEELS_cm 10.8
#define TICKS_PER_REV 1920

#define LINEAR_TO_TICKS(x) ((x) / (WHEEL_DIAMETER_cm * PI) * TICKS_PER_REV)

// other consts
#define LINEAR_SPEED_SET 800
#define ROTATION_SPEED_SET 200

// instruction init
struct instruction {
  unsigned long leftTargetPos; // ticks
  bool leftTargetDir;

  unsigned long rightTargetPos; // ticks
  bool rightTargetDir;

  int absSetSpeed;
};

#define Forward instruction{LINEAR_TO_TICKS(LIENAR_DISTANCE_STEP_cm), true, LINEAR_TO_TICKS(LIENAR_DISTANCE_STEP_cm), true, LINEAR_SPEED_SET}
#define Backward instruction{LINEAR_TO_TICKS(LIENAR_DISTANCE_STEP_cm), false, LINEAR_TO_TICKS(LIENAR_DISTANCE_STEP_cm), false, LINEAR_SPEED_SET}
#define RotateLeft \ 
  instruction{LINEAR_TO_TICKS(DISTANCE_BETWEEN_WHEELS_cm*PI/8), false, LINEAR_TO_TICKS(DISTANCE_BETWEEN_WHEELS_cm*PI/8), true, LINEAR_SPEED_SET)}
  //instruction{0, true, LINEAR_TO_TICKS(DISTANCE_BETWEEN_WHEELS_cm*PI*(1.0/6)), true, ROTATION_SPEED_SET}, \
  instruction{LINEAR_TO_TICKS(DISTANCE_BETWEEN_WHEELS_cm*PI*(1.0/3)), false, 0, true, ROTATION_SPEED_SET}, \
  instruction{LINEAR_TO_TICKS(DISTANCE_BETWEEN_WHEELS_cm*HALF_OF_SQRT_3_MINUS_1), true, LINEAR_TO_TICKS(DISTANCE_BETWEEN_WHEELS_cm*HALF_OF_SQRT_3_MINUS_1), true, ROTATION_SPEED_SET}
#define RotateRight \
  instruction{LINEAR_TO_TICKS(DISTANCE_BETWEEN_WHEELS_cm*PI*(1.0/6)), true, 0, true, ROTATION_SPEED_SET}, \
  instruction{0, true, LINEAR_TO_TICKS(DISTANCE_BETWEEN_WHEELS_cm*PI*(1.0/3)), false, ROTATION_SPEED_SET}, \
  instruction{LINEAR_TO_TICKS(DISTANCE_BETWEEN_WHEELS_cm*HALF_OF_SQRT_3_MINUS_1), true, LINEAR_TO_TICKS(DISTANCE_BETWEEN_WHEELS_cm*HALF_OF_SQRT_3_MINUS_1), true, ROTATION_SPEED_SET}
// the sequences defined above are definitely impracticable unless the robot can be made quite accurate; 
// just do 90 degree turns, adjusting linear travel as needed if that doesn't work

instruction instructionQueue[] = {
   Forward
};

int sizeOfInstructionQueue = sizeof(instructionQueue) / sizeof(instructionQueue[0]);

int nextStep = 0;

// -- PID CONFIG -- //
const int CONTROL_STEP_MS = 100; 
const float kP_left = .1;//01;
const float kI_left = 0.19;//001;
const float kP_right = 0.1;
const float kI_right = 0.2;
const int RATE_OF_ADJUSTMENT = 50; // cts per 100 millis (per control_step_ms)

const int BASELINE = 0;

int setSpeed_left = 0; // only set at 50s
int setSpeed_right = 0;

int targetSpeed_left = 0; // cts / sec; target speed should prolly be lerped for real activity.
int targetSpeed_right = 0;

long targetPosition_left = 0;
long targetPosition_right = 0;

// const unsigned long targetPosition = 4000; // cts
// targetSpeed / RATE_OF_ADJUSTMENT * setSpeed * .5

// Gyro Control Vars
float initial_heading = 0;
const float kP_rot = .1;

// PID vars
long posLeft_total = 0;
long posLeft = 0;
long posRight_total = 0;
long posRight = 0;
long lastCheck = 0;

float prevLeftVel = 0;
float prevRightVel = 0;

float velLeftFilt = 0;
float velRightFilt = 0;

// PI Control vars
float integral_error_left = 0;
float integral_error_right = 0;


void setMotors(int forwardLeft, int forwardRight, int backwardLeft, int backwardRight) {
  analogWrite(FWD_left, constrain(forwardLeft, 0, 255));
  analogWrite(BCK_left, constrain(backwardLeft, 0, 255));
  analogWrite(FWD_right, constrain(forwardRight, 0, 255));
  analogWrite(BCK_right, constrain(backwardRight, 0, 255));
}

float recieveHeading() {
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print(ypr[0] * 180/M_PI); // operative value

    #endif

  /* Blink LED to indicate activity */
  blinkState = !blinkState;
  digitalWrite(LED_BUILTIN, blinkState);

  return ypr[0];
  }
}

void updateToNext() {
  instruction next = instructionQueue[nextStep];
  long leftSignal = next.leftTargetPos * (next.leftTargetDir ? (1) : (-1));
  long rightSignal = next.rightTargetPos * (next.rightTargetDir ? (1) : (-1));

  posLeft_total = 0;
  posRight_total = 0;
  initial_heading = recieveHeading();

  targetPosition_left = leftSignal;
  targetPosition_right = rightSignal;

  setSpeed_left = (next.leftTargetDir ? (1) : (-1)) * next.absSetSpeed;
  setSpeed_right = (next.rightTargetDir ? (1) : (-1)) * next.absSetSpeed;

  nextStep++;
}

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  Serial.begin(115200); //115200 is required for Teapot Demo output
  while (!Serial);

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  /*Wait for Serial input*/
  /*Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read()); // Empty buffer
  while (!Serial.available());                 // Wait for data
  while (Serial.available() && Serial.read()); // Empty buffer again*/

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(ENCA_left, INPUT);
  pinMode(ENCB_left, INPUT);
  pinMode(ENCA_right, INPUT);
  pinMode(ENCB_right, INPUT);

  pinMode(FWD_left, OUTPUT);
  pinMode(BCK_left, OUTPUT);
  pinMode(FWD_right, OUTPUT);
  pinMode(BCK_right, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_left), readEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_right), readEncoderRight, RISING);

  updateToNext();
}

void readEncoderLeft() {
  int b = digitalRead(ENCB_left);
  if (b > 0) {
    posLeft--;
    posLeft_total--;
  } else {
    posLeft++;
    posLeft_total++;
  }
}

void readEncoderRight() {
  int b = digitalRead(ENCB_right);
  if (b > 0) {
    posRight++; // signs adjusted to account for difference in motor orient.
    posRight_total++;
  } else {
    posRight--;
    posRight_total--;
  }
}

void adjustTargetSpeed() {
  if (setSpeed_left > targetSpeed_left) {
    targetSpeed_left = targetSpeed_left + RATE_OF_ADJUSTMENT;
  } else if (setSpeed_left < targetSpeed_left) {
    targetSpeed_left = targetSpeed_left - RATE_OF_ADJUSTMENT;
  }

  if (setSpeed_right > targetSpeed_right) {
    targetSpeed_right = targetSpeed_right + RATE_OF_ADJUSTMENT;
  } else if (setSpeed_right < targetSpeed_right) {
    targetSpeed_right = targetSpeed_right - RATE_OF_ADJUSTMENT;
  }
}


void loop() {
    if (!DMPReady) return; // Stop the program if DMP programming fails.
    
  /* Read a packet from FIFO */
  float current_heading = recieveHeading();

  if (millis() - lastCheck < CONTROL_STEP_MS) {
    return;
  }

  adjustTargetSpeed();
  
  float speedLeft = posLeft / (CONTROL_STEP_MS / 1000.0);
  float speedRight = posRight / (CONTROL_STEP_MS / 1000.0);
 
  velLeftFilt = velLeftFilt*0.6 + 0.2*speedLeft + .2*prevLeftVel;
  velRightFilt = velRightFilt*0.6 + 0.2*speedRight + .2*prevRightVel;

  float errorLeft = targetSpeed_left - velLeftFilt;
  float errorRight = targetSpeed_right - velRightFilt;
  float errorHeading = current_heading - initial_heading;

  integral_error_left = integral_error_left + (errorLeft * (CONTROL_STEP_MS / 1000.0));
  integral_error_right = integral_error_right + (errorRight * (CONTROL_STEP_MS / 1000.0));

  float controlLeft = BASELINE + kP_left * errorLeft + kI_left * integral_error_left - kP_rot * errorHeading;
  float controlRight = BASELINE + kP_right * errorRight + kI_right * integral_error_right + kP_rot * errorHeading;


  // force control down if 0
  if (targetPosition_left == 0 ) {
    controlLeft = 0;
  }
  if (targetPosition_right == 0) {
    controlRight = 0;
  }
  
  /*Serial.print(velLeftFilt);
  Serial.print("\t");
  Serial.print(posLeft_total);
  Serial.print("\t");
  Serial.print(velRightFilt);
  Serial.print("\t")  ;
  Serial.println(posRight_total);*/
  

  int forLeft = 0, forRight = 0, backLeft = 0, backRight = 0;
  if (controlLeft >= 0) {
    forLeft = controlLeft;
  } else {
    backLeft = controlLeft*(-1.0);
  }

  if (controlRight >= 0) {
    forRight = controlRight;
  } else {
    backRight = controlRight*(-1.0);
  }

  setMotors(forLeft, forRight, backLeft, backRight);

  lastCheck = millis();

  posLeft = 0;
  posRight = 0;
  
  prevLeftVel = speedLeft;
  prevRightVel = speedRight;

  // ending
  if ( abs(posLeft_total) > abs(targetPosition_left - CTS_DECELERATION(targetSpeed_left)) ) {
    setSpeed_left = 0;
  }
  if ( abs(posRight_total) > abs(targetPosition_right - CTS_DECELERATION(targetSpeed_right)) ) {
    setSpeed_right = 0;
  }

  // recurse next
  if (setSpeed_left == 0 && setSpeed_right == 0 && velLeftFilt < 1 && velRightFilt < 1 && sizeOfInstructionQueue >= nextStep + 1) {
    updateToNext();
  }
}
