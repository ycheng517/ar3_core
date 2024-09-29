#include <AccelStepper.h>
#include <Encoder.h>
#include <avr/pgmspace.h>
#include <math.h>

#include <map>

// Firmware version
const char* VERSION = "0.1.0";

// Model of the AR4, i.e. mk1, mk2, mk3
String MODEL = "";

///////////////////////////////////////////////////////////////////////////////
// Physical Params
///////////////////////////////////////////////////////////////////////////////

const int STEP_PINS[] = {0, 2, 4, 6, 8, 10};
const int DIR_PINS[] = {1, 3, 5, 7, 9, 11};
const int LIMIT_PINS[] = {26, 27, 28, 29, 30, 31};

std::map<String, const float*> MOTOR_STEPS_PER_DEG;
const float MOTOR_STEPS_PER_DEG_MK1[] = {44.44444444, 55.55555556, 55.55555556,
                                         42.72664356, 21.86024888, 22.22222222};
const float MOTOR_STEPS_PER_DEG_MK2[] = {44.44444444, 55.55555556, 55.55555556,
                                         49.77777777, 21.86024888, 22.22222222};
const float MOTOR_STEPS_PER_DEG_MK3[] = {44.44444444, 55.55555556, 55.55555556,
                                         49.77777777, 21.86024888, 22.22222222};

const int MOTOR_STEPS_PER_REV[] = {400, 400, 400, 400, 800, 400};

// set encoder pins
Encoder encPos[6] = {Encoder(14, 15), Encoder(17, 16), Encoder(19, 18),
                     Encoder(20, 21), Encoder(23, 22), Encoder(24, 25)};
// +1 if encoder direction matches motor direction, -1 otherwise
int ENC_DIR[] = {-1, 1, 1, 1, 1, 1};
// +1 if encoder max value is at the minimum joint angle, 0 otherwise
int ENC_MAX_AT_ANGLE_MIN[] = {1, 0, 1, 0, 0, 1};
// motor steps * ENC_MULT = encoder steps
const float ENC_MULT[] = {10, 10, 10, 10, 5, 10};

// define axis limits in degrees, for calibration
std::map<String, const int*> JOINT_LIMIT_MIN;
int JOINT_LIMIT_MIN_MK1[] = {-170, -42, -89, -165, -105, -155};
int JOINT_LIMIT_MIN_MK2[] = {-170, -42, -89, -165, -105, -155};
int JOINT_LIMIT_MIN_MK3[] = {-170, -42, -89, -180, -105, -180};
std::map<String, const int*> JOINT_LIMIT_MAX;
int JOINT_LIMIT_MAX_MK1[] = {170, 90, 52, 165, 105, 155};
int JOINT_LIMIT_MAX_MK2[] = {170, 90, 52, 165, 105, 155};
int JOINT_LIMIT_MAX_MK3[] = {170, 90, 52, 180, 105, 180};

///////////////////////////////////////////////////////////////////////////////
// ROS Driver Params
///////////////////////////////////////////////////////////////////////////////

// roughly equals 0, -6, 0, 0, 0, 0 degrees
const int REST_ENC_POSITIONS[] = {75507, 20000, 49234, 70489, 11470, 34311};
enum SM { STATE_TRAJ, STATE_ERR };
SM STATE = STATE_TRAJ;

const int NUM_JOINTS = 6;
AccelStepper stepperJoints[NUM_JOINTS];

// calibration settings
const int LIMIT_SWITCH_HIGH[] = {
    1, 1, 1, 1, 1, 1};  // to account for both NC and NO limit switches
const int CAL_DIR[] = {-1, -1, 1,
                       -1, -1, 1};  // joint rotation direction to limit switch
const int CAL_SPEED = 500;          // motor steps per second
const int CAL_SPEED_MULT[] = {
    1, 1, 1, 2, 1, 1};  // multiplier to account for motor steps/rev

// speed and acceleration settings
float JOINT_MAX_SPEED[] = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0};  // deg/s
float JOINT_MAX_ACCEL[] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};  // deg/s^2
char JOINT_NAMES[] = {'A', 'B', 'C', 'D', 'E', 'F'};

// num of encoder steps in range of motion of joint
int ENC_RANGE_STEPS[NUM_JOINTS];

void setup() {
  MOTOR_STEPS_PER_DEG["mk1"] = MOTOR_STEPS_PER_DEG_MK1;
  MOTOR_STEPS_PER_DEG["mk2"] = MOTOR_STEPS_PER_DEG_MK2;
  MOTOR_STEPS_PER_DEG["mk3"] = MOTOR_STEPS_PER_DEG_MK3;

  JOINT_LIMIT_MIN["mk1"] = JOINT_LIMIT_MIN_MK1;
  JOINT_LIMIT_MIN["mk2"] = JOINT_LIMIT_MIN_MK2;
  JOINT_LIMIT_MIN["mk3"] = JOINT_LIMIT_MIN_MK3;

  JOINT_LIMIT_MAX["mk1"] = JOINT_LIMIT_MAX_MK1;
  JOINT_LIMIT_MAX["mk2"] = JOINT_LIMIT_MAX_MK2;
  JOINT_LIMIT_MAX["mk3"] = JOINT_LIMIT_MAX_MK3;

  Serial.begin(9600);

  for (int i = 0; i < NUM_JOINTS; ++i) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(LIMIT_PINS[i], INPUT);
  }
}

void setupSteppersMK1() {
  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(true, false, false);  // DM542T CW
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                     MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                 MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
  stepperJoints[3].setPinsInverted(false, false, false);  // J4 DM320T CCW
}

void setupSteppersMK2() {
  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(false, false,
                                     false);  // DM320T / DM332T --> CW
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                     MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                 MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
}

void setupSteppersMK3() {
  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(false, false,
                                     false);  // DM320T / DM332T --> CW
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                     MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                 MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
}

// initialize stepper motors and constants based on the model. Also verifies
// that the software version matches the firmware version
bool initStateTraj(String inData) {
  // parse initialisation message
  int idxVersion = inData.indexOf('A');
  int idxModel = inData.indexOf('B');
  String softwareVersion = inData.substring(idxVersion + 1, idxModel);
  int versionMatches = (softwareVersion == VERSION);

  String model = inData.substring(idxModel + 1, inData.length() - 1);
  int modelMatches = false;
  if (model == "mk1" || model == "mk2" || model == "mk3") {
    modelMatches = true;
    MODEL = model;

    for (int i = 0; i < NUM_JOINTS; ++i) {
      int joint_range = JOINT_LIMIT_MAX[MODEL][i] - JOINT_LIMIT_MIN[MODEL][i];
      ENC_RANGE_STEPS[i] = static_cast<int>(MOTOR_STEPS_PER_DEG[MODEL][i] *
                                            joint_range * ENC_MULT[i]);
    }

    if (model == "mk1") {
      setupSteppersMK1();
    } else if (model == "mk2") {
      setupSteppersMK2();
    } else if (model == "mk3") {
      setupSteppersMK3();
    }
  }

  // return acknowledgement with result
  String msg = String("ST") + "A" + versionMatches + "B" + VERSION + "C" +
               modelMatches + "D" + MODEL;
  Serial.println(msg);

  if (versionMatches && modelMatches) {
    return true;
  }
  return false;
}

void readMotorSteps(int* motorSteps) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    motorSteps[i] = encPos[i].read() / ENC_MULT[i];
  }
}

void encStepsToJointPos(int* encSteps, double* jointPos) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    jointPos[i] = encSteps[i] / MOTOR_STEPS_PER_DEG[MODEL][i] * ENC_DIR[i];
  }
}

void jointPosToEncSteps(double* jointPos, int* encSteps) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    encSteps[i] = jointPos[i] * MOTOR_STEPS_PER_DEG[MODEL][i] * ENC_DIR[i];
  }
}

String JointPosToString(double* jointPos) {
  String out;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    out += JOINT_NAMES[i];
    out += String(jointPos[i], 6);
  }
  return out;
}

void updateStepperSpeed(String inData) {
  int idxSpeedJ1 = inData.indexOf('A');
  int idxAccelJ1 = inData.indexOf('B');
  int idxSpeedJ2 = inData.indexOf('C');
  int idxAccelJ2 = inData.indexOf('D');
  int idxSpeedJ3 = inData.indexOf('E');
  int idxAccelJ3 = inData.indexOf('F');
  int idxSpeedJ4 = inData.indexOf('G');
  int idxAccelJ4 = inData.indexOf('H');
  int idxSpeedJ5 = inData.indexOf('I');
  int idxAccelJ5 = inData.indexOf('J');
  int idxSpeedJ6 = inData.indexOf('K');
  int idxAccelJ6 = inData.indexOf('L');

  JOINT_MAX_SPEED[0] = inData.substring(idxSpeedJ1 + 1, idxAccelJ1).toFloat();
  JOINT_MAX_ACCEL[0] = inData.substring(idxAccelJ1 + 1, idxSpeedJ2).toFloat();
  JOINT_MAX_SPEED[1] = inData.substring(idxSpeedJ2 + 1, idxAccelJ2).toFloat();
  JOINT_MAX_ACCEL[1] = inData.substring(idxAccelJ2 + 1, idxSpeedJ3).toFloat();
  JOINT_MAX_SPEED[2] = inData.substring(idxSpeedJ3 + 1, idxAccelJ3).toFloat();
  JOINT_MAX_ACCEL[2] = inData.substring(idxAccelJ3 + 1, idxSpeedJ4).toFloat();
  JOINT_MAX_SPEED[3] = inData.substring(idxSpeedJ4 + 1, idxAccelJ4).toFloat();
  JOINT_MAX_ACCEL[3] = inData.substring(idxAccelJ4 + 1, idxSpeedJ5).toFloat();
  JOINT_MAX_SPEED[4] = inData.substring(idxSpeedJ5 + 1, idxAccelJ5).toFloat();
  JOINT_MAX_ACCEL[4] = inData.substring(idxAccelJ5 + 1, idxSpeedJ6).toFloat();
  JOINT_MAX_SPEED[5] = inData.substring(idxSpeedJ6 + 1, idxAccelJ6).toFloat();
  JOINT_MAX_ACCEL[5] = inData.substring(idxAccelJ6 + 1).toFloat();
}

bool calibrateJoints(int* calJoints) {
  // check which joints to calibrate
  bool calAllDone = false;
  bool calJointsDone[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    calJointsDone[i] = !calJoints[i];
  }

  // first pass of calibration, fast speed
  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i]);
  }
  unsigned long startTime = millis();
  while (!calAllDone) {
    calAllDone = true;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      // if joint is not calibrated yet
      if (!calJointsDone[i]) {
        // check limit switches
        if (!reachedLimitSwitch(i)) {
          // limit switch not reached, continue moving
          stepperJoints[i].runSpeed();
          calAllDone = false;
        } else {
          // limit switch reached
          stepperJoints[i].setSpeed(0);  // redundancy
          calJointsDone[i] = true;
        }
      }
    }

    if (millis() - startTime > 20000) {
      return false;
    }
  }
  delay(2000);
  return true;
}

void moveAwayFromLimitSwitch() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i] * -1);
  }
  for (int j = 0; j < 10000000; j++) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      stepperJoints[i].runSpeed();
    }
  }
  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(0);
  }
  delay(2000);
  return;
}

bool reachedLimitSwitch(int joint) {
  int pin = LIMIT_PINS[joint];
  // check multiple times to deal with noise
  // possibly EMI from motor cables?
  for (int i = 0; i < 5; ++i) {
    if (digitalRead(pin) != LIMIT_SWITCH_HIGH[joint]) {
      return false;
    }
  }
  return true;
}

void doCalibrationRoutine() {
  // calibrate all joints
  int calJoints[] = {1, 1, 1, 1, 1, 1};
  int success = calibrateJoints(calJoints);
  if (!success) {
    // set all speeds to zero
    for (int i = 0; i < NUM_JOINTS; ++i) {
      stepperJoints[i].setSpeed(0);
    }
    String msg = String("JCRES0MSG") + "Failed to calibrate joints.";
    Serial.println(msg);
    return;
  }

  // record encoder steps
  int calSteps[6];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    calSteps[i] = encPos[i].read();
  }
  for (int i = 0; i < NUM_JOINTS; ++i) {
    encPos[i].write(ENC_RANGE_STEPS[i] * ENC_MAX_AT_ANGLE_MIN[i]);
  }

  // move away from the limit switches a bit so that if the next command
  // is in the wrong direction, the limit switches will not be run over
  // immediately and become damaged.
  moveAwayFromLimitSwitch();

  // return to original position
  int curMotorSteps[NUM_JOINTS];
  readMotorSteps(curMotorSteps);
  // repeatedly set pos, spd, acc 3x due to some joints occasionally not
  // moving
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                       MOTOR_STEPS_PER_DEG[MODEL][i]);
      stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                   MOTOR_STEPS_PER_DEG[MODEL][i]);
      float target_pos =
          (REST_ENC_POSITIONS[i] / ENC_MULT[i] - curMotorSteps[i]) * ENC_DIR[i];
      stepperJoints[i].move(target_pos);
      stepperJoints[i].run();
    }
  }
  bool restPosReached = false;
  unsigned long startTime = millis();
  while (!restPosReached && millis() - startTime < 10000) {
    restPosReached = true;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (abs(stepperJoints[i].distanceToGo()) > 5) {
        stepperJoints[i].run();
        restPosReached = false;
      }
    }
  }
  if (!restPosReached) {
    // set all speeds to zero
    for (int i = 0; i < NUM_JOINTS; ++i) {
      stepperJoints[i].setSpeed(0);
      stepperJoints[i].run();
    }
    String msg = String("JCRES0MSG") + "Failed to return to rest position.";
    Serial.println(msg);
    return;
  }

  // calibration done, send calibration values
  String msg = String("JCRES1") + "A" + calSteps[0] + "B" + calSteps[1] + "C" +
               calSteps[2] + "D" + calSteps[3] + "E" + calSteps[4] + "F" +
               calSteps[5];
  Serial.println(msg);
}

void stateTRAJ() {
  // clear message
  String inData = "";

  // initialise joint steps
  double curJointPos[NUM_JOINTS];
  int curMotorSteps[NUM_JOINTS];
  readMotorSteps(curMotorSteps);

  double cmdJointPos[NUM_JOINTS];
  int cmdEncSteps[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    cmdEncSteps[i] = curMotorSteps[i];
  }

  // start loop
  while (STATE == STATE_TRAJ) {
    char received = '\0';
    // check for message from host
    if (Serial.available()) {
      received = Serial.read();
      inData += received;
    }

    // process message when new line character is received
    if (received == '\n') {
      String function = inData.substring(0, 2);
      if (function == "ST") {
        if (!initStateTraj(inData)) {
          STATE = STATE_ERR;
          return;
        }
      } else if (MODEL == "") {
        // if model is not set, do not proceed with any other function
        STATE = STATE_ERR;
        return;
      }

      if (function == "MT") {
        // update trajectory information
        readMotorSteps(curMotorSteps);

        // update host with joint positions
        encStepsToJointPos(curMotorSteps, curJointPos);
        String msg = String("JP") + JointPosToString(curJointPos);
        Serial.println(msg);

        // get new position commands
        int msgIdxJ1 = inData.indexOf('A');
        int msgIdxJ2 = inData.indexOf('B');
        int msgIdxJ3 = inData.indexOf('C');
        int msgIdxJ4 = inData.indexOf('D');
        int msgIdxJ5 = inData.indexOf('E');
        int msgIdxJ6 = inData.indexOf('F');
        cmdJointPos[0] = inData.substring(msgIdxJ1 + 1, msgIdxJ2).toFloat();
        cmdJointPos[1] = inData.substring(msgIdxJ2 + 1, msgIdxJ3).toFloat();
        cmdJointPos[2] = inData.substring(msgIdxJ3 + 1, msgIdxJ4).toFloat();
        cmdJointPos[3] = inData.substring(msgIdxJ4 + 1, msgIdxJ5).toFloat();
        cmdJointPos[4] = inData.substring(msgIdxJ5 + 1, msgIdxJ6).toFloat();
        cmdJointPos[5] = inData.substring(msgIdxJ6 + 1).toFloat();
        jointPosToEncSteps(cmdJointPos, cmdEncSteps);

        // update target joint positions
        readMotorSteps(curMotorSteps);
        for (int i = 0; i < NUM_JOINTS; ++i) {
          int diffEncSteps = cmdEncSteps[i] - curMotorSteps[i];
          if (abs(diffEncSteps) > 2) {
            int diffMotSteps = diffEncSteps * ENC_DIR[i];
            // if (diffMotSteps < MOTOR_STEPS_PER_REV[i]) {
            //   // for the last rev of motor, introduce artificial
            //   decceleration
            //   // to help prevent overshoot
            //   diffMotSteps = diffMotSteps / 2;
            // }
            stepperJoints[i].move(diffMotSteps);
            stepperJoints[i].run();
          }
        }
      } else if (function == "JC") {
        doCalibrationRoutine();
      } else if (function == "JP") {
        readMotorSteps(curMotorSteps);
        encStepsToJointPos(curMotorSteps, curJointPos);
        String msg = String("JP") + JointPosToString(curJointPos);
        Serial.println(msg);
      } else if (function == "SS") {
        updateStepperSpeed(inData);
        // set motor speed and acceleration
        for (int i = 0; i < NUM_JOINTS; ++i) {
          stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                           MOTOR_STEPS_PER_DEG[MODEL][i]);
          stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                       MOTOR_STEPS_PER_DEG[MODEL][i]);
        }
        // read current joint positions
        readMotorSteps(curMotorSteps);
        encStepsToJointPos(curMotorSteps, curJointPos);

        // update host with joint positions
        String msg = String("JP") + JointPosToString(curJointPos);
        Serial.println(msg);
      }

      inData = "";  // clear message
    }
    for (int i = 0; i < NUM_JOINTS; ++i) {
      stepperJoints[i].run();
    }
  }
}

void stateERR() {
  // enter holding state
  for (int i = 0; i < NUM_JOINTS; ++i) {
    digitalWrite(STEP_PINS[i], LOW);
  }

  while (STATE == STATE_ERR) {
    Serial.println("DB: Unrecoverable error state entered. Please reset.");
    delay(1000);
  }
}

void loop() {
  STATE = STATE_TRAJ;

  switch (STATE) {
    case STATE_ERR:
      stateERR();
      break;
    default:
      stateTRAJ();
      break;
  }
}
