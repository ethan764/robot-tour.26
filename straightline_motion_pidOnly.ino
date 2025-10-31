#include <Arduino.h>

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
const float kI_left = 0.2;//001;
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

void updateToNext() {
  instruction next = instructionQueue[nextStep];
  long leftSignal = next.leftTargetPos * (next.leftTargetDir ? (1) : (-1));
  long rightSignal = next.rightTargetPos * (next.rightTargetDir ? (1) : (-1));

  posLeft_total = 0;
  posRight_total = 0;

  targetPosition_left = leftSignal;
  targetPosition_right = rightSignal;

  setSpeed_left = (next.leftTargetDir ? (1) : (-1)) * next.absSetSpeed;
  setSpeed_right = (next.rightTargetDir ? (1) : (-1)) * next.absSetSpeed;

  nextStep++;
}

void setup() {
  Serial.begin(9600);

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

  integral_error_left = integral_error_left + (errorLeft * (CONTROL_STEP_MS / 1000.0));
  integral_error_right = integral_error_right + (errorRight * (CONTROL_STEP_MS / 1000.0));

  float controlLeft = BASELINE + kP_left * errorLeft + kI_left * integral_error_left;
  float controlRight = BASELINE + kP_right * errorRight + kI_right * integral_error_right;


  // force control down if 0
  if (targetPosition_left == 0 ) {
    controlLeft = 0;
  }
  if (targetPosition_right == 0) {
    controlRight = 0;
  }
  
  Serial.print(velLeftFilt);
  Serial.print("\t");
  Serial.print(posLeft_total);
  Serial.print("\t");
  Serial.print(velRightFilt);
  Serial.print("\t")  ;
  Serial.println(posRight_total);
  

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
