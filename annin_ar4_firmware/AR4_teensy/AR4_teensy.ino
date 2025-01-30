#include <AccelStepper.h>
#include <Bounce2.h>
#include <Encoder.h>
#include <avr/pgmspace.h>
#include <math.h>

#include <map>

// Firmware version
const char* VERSION = "2.0.1";

// Model of the AR4, i.e. mk1, mk2, mk3
String MODEL = "";

///////////////////////////////////////////////////////////////////////////////
// Physical Params
///////////////////////////////////////////////////////////////////////////////

const int ESTOP_PIN = 39;
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

// roughly equals 0, 0, 0, 0, 0, 0 degrees without any user-defined offsets.
std::map<String, const int*> REST_MOTOR_STEPS;
const int REST_MOTOR_STEPS_MK1[] = {7555, 2333, 4944, 7049, 2295, 3431};
const int REST_MOTOR_STEPS_MK2[] = {7555, 2333, 4944, 7049, 2295, 3431};
const int REST_MOTOR_STEPS_MK3[] = {7555, 2333, 4944, 8960, 2295, 4000};

enum SM { STATE_TRAJ, STATE_ERR };
SM STATE = STATE_TRAJ;

const int NUM_JOINTS = 6;
AccelStepper stepperJoints[NUM_JOINTS];
Bounce2::Button limitSwitches[NUM_JOINTS];
const int DEBOUCE_INTERVAL = 10;  // ms

// calibration settings
const int LIMIT_SWITCH_HIGH[] = {
    1, 1, 1, 1, 1, 1};  // to account for both NC and NO limit switches
const int CAL_DIR[] = {-1, -1, 1,
                       -1, -1, 1};  // joint rotation direction to limit switch
const int CAL_SPEED = 500;          // motor steps per second
const int CAL_SPEED_MULT[] = {
    1, 1, 1, 2, 1, 1};  // multiplier to account for motor steps/rev
// num of encoder steps in range of motion of joint
int ENC_RANGE_STEPS[NUM_JOINTS];

// speed and acceleration settings
float JOINT_MAX_SPEED[] = {60.0, 60.0, 60.0, 60.0, 60.0, 60.0};  // deg/s
float JOINT_MAX_ACCEL[] = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0};  // deg/s^2
char VARIABLE_NAMES[] = {'A', 'B', 'C', 'D', 'E', 'F',
                         'G', 'H', 'I', 'J', 'K', 'L'};

bool estop_pressed = false;

void estopPressed() { estop_pressed = true; }

void resetEstop() {
  // if ESTOP button is pressed still, do not reset the flag!
  if (digitalRead(ESTOP_PIN) == LOW) {
    return;
  }

  // reset any previously set MT commands
  for (int i = 0; i < NUM_JOINTS; ++i) {
    // NOTE: This may seem redundant but is the only permitted way to set
    // _stepInterval and _n to 0, which is required to avoid a jerk resume
    // when Estop is reset after interruption of an accelerated motion
    stepperJoints[i].setCurrentPosition(stepperJoints[i].currentPosition());
    stepperJoints[i].setSpeed(0);
  }

  estop_pressed = false;
}

bool safeRun(AccelStepper& stepperJoint) {
  if (estop_pressed) return false;
  return stepperJoint.run();
}

bool safeRunSpeed(AccelStepper& stepperJoint) {
  if (estop_pressed) return false;
  return stepperJoint.runSpeed();
}

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

  REST_MOTOR_STEPS["mk1"] = REST_MOTOR_STEPS_MK1;
  REST_MOTOR_STEPS["mk2"] = REST_MOTOR_STEPS_MK2;
  REST_MOTOR_STEPS["mk3"] = REST_MOTOR_STEPS_MK3;

  for (int i = 0; i < NUM_JOINTS; ++i) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(LIMIT_PINS[i], INPUT);
  }

  for (int i = 0; i < NUM_JOINTS; ++i) {
    limitSwitches[i] = Bounce2::Button();
    limitSwitches[i].attach(LIMIT_PINS[i], INPUT);
    limitSwitches[i].interval(DEBOUCE_INTERVAL);
    limitSwitches[i].setPressedState(LIMIT_SWITCH_HIGH[i]);
  }

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopPressed, FALLING);

  Serial.begin(38400);
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

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
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
    out += VARIABLE_NAMES[i];
    out += String(jointPos[i], 6);
  }
  return out;
}

String JointVelToString(double* lastVelocity) {
  String out;

  for (int i = 0; i < NUM_JOINTS; ++i) {
    out += VARIABLE_NAMES[i];
    out += String(lastVelocity[i], 6);
  }

  return out;
}

void ParseMessage(String& inData, double* cmdJointPos, int startIdx = 0) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    bool lastJoint = i == NUM_JOINTS - 1;
    int msgIdxJ_S = inData.indexOf(VARIABLE_NAMES[i + startIdx]);
    if (msgIdxJ_S == -1) {
      Serial.printf("ER: panic, missing variable %c\n",
                    VARIABLE_NAMES[i + startIdx]);
      return;
    }

    int msgIdxJ_E;
    if (!lastJoint) {
      msgIdxJ_E = inData.indexOf(VARIABLE_NAMES[i + startIdx + 1]);
    } else {
      msgIdxJ_E = inData.length();
    }

    cmdJointPos[i] = inData.substring(msgIdxJ_S + 1, msgIdxJ_E).toFloat();
  }
}

void MoveVelocity(String inData, int* motorSteps) {
  double cmdJointVel[NUM_JOINTS];
  double cmdJointPos[NUM_JOINTS];

  double curJointPos[NUM_JOINTS];
  encStepsToJointPos(motorSteps, curJointPos);

  // Parse velocity commands (A-F)
  ParseMessage(inData, cmdJointVel, 0);
  // Parse position commands (G-L)
  ParseMessage(inData, cmdJointPos, NUM_JOINTS);

  double tol = 0.001;
  for (int i = 0; i < NUM_JOINTS; i++) {
    // HACK to improve tracking performance: if position is getting away,
    // adjust velocity to catch up
    if ((abs(cmdJointVel[i]) > 0) &&
        (abs(cmdJointPos[i] - curJointPos[i]) > tol)) {
      cmdJointVel[i] += 50 * (cmdJointPos[i] - curJointPos[i]);
    }

    // clip joint velocity by max speed
    if (abs(cmdJointVel[i]) > JOINT_MAX_SPEED[i]) {
      Serial.printf("DB: joint %i speed %f > %f, clipping.\n", i + 1,
                    cmdJointVel[i], JOINT_MAX_SPEED[i]);
      cmdJointVel[i] = sgn(cmdJointVel[i]) * JOINT_MAX_SPEED[i];
    }
    cmdJointVel[i] *= MOTOR_STEPS_PER_DEG[MODEL][i];
    stepperJoints[i].setMaxSpeed(abs(cmdJointVel[i]));
    stepperJoints[i].setSpeed(cmdJointVel[i]);
    stepperJoints[i].move(sgn(cmdJointVel[i]) * __LONG_MAX__);
  }
}

void MoveTo(const int* cmdSteps, int* motorSteps) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    int diffEncSteps = cmdSteps[i] - motorSteps[i];
    if (abs(diffEncSteps) > 2) {
      int diffMotSteps = diffEncSteps * ENC_DIR[i];
      stepperJoints[i].move(diffMotSteps);
    }
  }
}

void MoveTo(String inData, int* motorSteps) {
  double cmdJointPos[NUM_JOINTS] = {0};
  ParseMessage(inData, cmdJointPos);

  for (int i = 0; i < NUM_JOINTS; i++) {
    if (abs(cmdJointPos[i] > 380.0)) {
      Serial.printf("ER: panic, joint %i value %f out of range\n", i + 1,
                    cmdJointPos[i]);
      return;
    }
  }

  // get current joint position
  double curJointPos[NUM_JOINTS];
  encStepsToJointPos(motorSteps, curJointPos);

  // update target joint position
  int cmdEncSteps[NUM_JOINTS] = {0};
  jointPosToEncSteps(cmdJointPos, cmdEncSteps);

  MoveTo(cmdEncSteps, motorSteps);
}

bool AtPosition(const int* targetMotorSteps, const int* currMotorSteps,
                const int maxDiff) {
  bool allDone = true;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    int diffEncSteps = targetMotorSteps[i] - currMotorSteps[i];
    if (abs(diffEncSteps) > maxDiff) {
      allDone = false;
    }
  }
  return allDone;
}

void setAllMaxSpeedAndAccel() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                 MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                     MOTOR_STEPS_PER_DEG[MODEL][i]);
  }
}

void updateAllLimitSwitches() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    limitSwitches[i].update();
  }
}

bool moveToLimitSwitches(int* calJoints) {
  // check which joints to calibrate
  bool calAllDone = false;
  bool calJointsDone[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    calJointsDone[i] = !calJoints[i];
  }

  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i]);
  }
  unsigned long startTime = millis();
  while (!calAllDone) {
    updateAllLimitSwitches();
    calAllDone = true;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      // if joint is not calibrated yet
      if (!calJointsDone[i]) {
        // check limit switches
        if (!limitSwitches[i].isPressed()) {
          // limit switch not reached, continue moving
          safeRunSpeed(stepperJoints[i]);
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

bool moveAwayFromLimitSwitch(int* calJoints) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (calJoints[i]) {
      stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i] *
                                -1);
    }
  }

  bool limitSwitchesActive = true;
  unsigned long startTime = millis();
  while (limitSwitchesActive || millis() - startTime < 4000) {
    limitSwitchesActive = false;
    updateAllLimitSwitches();
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (calJoints[i]) {
        if (limitSwitches[i].isPressed()) {
          limitSwitchesActive = true;
        }
        safeRunSpeed(stepperJoints[i]);
      }
    }

    if (millis() - startTime > 10000) {
      return false;
    }
  }

  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(0);  // redundancy
  }
  delay(1000);
  return true;
}

bool moveLimitedAwayFromLimitSwitch(int* calJoints) {
  // move the ones that already hit a limit away from it before start of
  // calibration
  int limitedJoints[NUM_JOINTS] = {0};
  updateAllLimitSwitches();
  for (int i = 0; i < NUM_JOINTS; i++) {
    limitedJoints[i] = (calJoints[i] && limitSwitches[i].isPressed());
  }
  return moveAwayFromLimitSwitch(limitedJoints);
}

bool doCalibrationRoutine(String& outputMsg) {
  // calibrate all joints
  int calJoints[] = {1, 1, 1, 1, 1, 1};
  if (!moveLimitedAwayFromLimitSwitch(calJoints)) {
    outputMsg = "ER: Failed to move away from limit switches at the start.";
    return false;
  }

  if (!moveToLimitSwitches(calJoints)) {
    outputMsg = "ER: Failed to move to limit switches.";
    return false;
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
  if (!moveAwayFromLimitSwitch(calJoints)) {
    outputMsg = "ER: Failed to move away from limit switches.";
    return false;
  }

  // return to original position
  unsigned long startTime = millis();
  int curMotorSteps[NUM_JOINTS];
  readMotorSteps(curMotorSteps);
  while (!AtPosition(REST_MOTOR_STEPS[MODEL], curMotorSteps, 5)) {
    if (millis() - startTime > 10000) {
      outputMsg = "ER: Failed to return to original position.";
      return false;
    }

    readMotorSteps(curMotorSteps);
    MoveTo(REST_MOTOR_STEPS[MODEL], curMotorSteps);
    for (int i = 0; i < NUM_JOINTS; ++i) {
      safeRun(stepperJoints[i]);
    }
  }

  // restore original max speed and acceleration
  setAllMaxSpeedAndAccel();

  // calibration done, send calibration values
  // N.B. calibration values aren't used right now
  outputMsg = String("JC") + "A" + calSteps[0] + "B" + calSteps[1] + "C" +
              calSteps[2] + "D" + calSteps[3] + "E" + calSteps[4] + "F" +
              calSteps[5];
  return true;
}

void updateMotorVelocities(int* motorSteps, int* lastMotorSteps,
                           int* checksteps, unsigned long* lastVelocityCalc,
                           double* lastVelocity) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    // for really small velocities we still get
    // artifacts, but quite manageable now!

    if (micros() - lastVelocityCalc[i] < 5000) {
      // we want to trigger calculation only after x ms but
      // immediately when steps change after that
      checksteps[i] = motorSteps[i];
      continue;
    }

    // have to add some sort of outlier-filter here , maybe moving average ..
    if (abs(stepperJoints[i].speed() / MOTOR_STEPS_PER_DEG[MODEL][i]) < 5) {
      // NB! trying to fix artifacts at low velocity
      if (abs(motorSteps[i] - checksteps[i]) > 0) {
        lastVelocity[i] =
            stepperJoints[i].speed() / MOTOR_STEPS_PER_DEG[MODEL][i];
        lastMotorSteps[i] = motorSteps[i];
        lastVelocityCalc[i] = micros();
      } else if (stepperJoints[i].speed() == 0) {
        lastVelocity[i] = 0;
      }
    } else {
      unsigned long currentMicros = micros();
      double delta = (currentMicros - lastVelocityCalc[i]);
      if (abs(motorSteps[i] - checksteps[i]) > 0) {
        // calculate TRUE motor velocity
        lastVelocity[i] = ENC_DIR[i] * (motorSteps[i] - lastMotorSteps[i]) /
                          MOTOR_STEPS_PER_DEG[MODEL][i] / (delta / 1000000.0);
        lastMotorSteps[i] = motorSteps[i];
        lastVelocityCalc[i] = currentMicros;
      }
    }
  }
}

void stateTRAJ() {
  // clear message
  String inData = "";

  // initialise joint steps
  double curJointPos[NUM_JOINTS];
  int curMotorSteps[NUM_JOINTS];
  int lastMotorSteps[NUM_JOINTS];
  int checksteps[NUM_JOINTS];
  double lastVelocity[NUM_JOINTS];
  unsigned long lastVelocityCalc[NUM_JOINTS];

  readMotorSteps(curMotorSteps);

  for (int i = 0; i < NUM_JOINTS; ++i) {
    lastVelocityCalc[i] = micros();
    lastMotorSteps[i] = curMotorSteps[i];
  }

  // start loop
  while (STATE == STATE_TRAJ) {
    char received = '\0';
    // check for message from host
    if (Serial.available()) {
      received = Serial.read();
      inData += received;
    }

    if (MODEL != "") {
      readMotorSteps(curMotorSteps);
      updateMotorVelocities(curMotorSteps, lastMotorSteps, checksteps,
                            lastVelocityCalc, lastVelocity);
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
        // clear speed counter
        for (int i = 0; i < NUM_JOINTS; i++) {
          if (stepperJoints[i].speed() == 0) {
            lastVelocityCalc[i] = micros();
          }
        }

        MoveTo(inData, curMotorSteps);

        // update the host about estop state
        String msg = String("ES") + estop_pressed;
        Serial.println(msg);

      } else if (function == "MV") {
        // clear speed counter
        for (int i = 0; i < NUM_JOINTS; i++) {
          if (stepperJoints[i].speed() == 0) {
            lastVelocityCalc[i] = micros();
          }
        }

        MoveVelocity(inData, curMotorSteps);

        // update the host about estop state
        String msg = String("ES") + estop_pressed;
        Serial.println(msg);

      } else if (function == "JP") {
        readMotorSteps(curMotorSteps);
        encStepsToJointPos(curMotorSteps, curJointPos);
        String msg = String("JP") + JointPosToString(curJointPos);
        Serial.println(msg);
      } else if (function == "JV") {
        String msg = String("JV") + JointVelToString(lastVelocity);
        Serial.println(msg);
      } else if (function == "JC") {
        String msg;
        if (!doCalibrationRoutine(msg)) {
          for (int i = 0; i < NUM_JOINTS; ++i) {
            stepperJoints[i].setSpeed(0);
          }
        }
        Serial.println(msg);
      } else if (function == "RE") {
        resetEstop();
        // update host with Estop status after trying to reset it
        String msg = String("ES") + estop_pressed;
        Serial.println(msg);
      }

      inData = "";  // clear message
    }

    for (int i = 0; i < NUM_JOINTS; ++i) {
      safeRun(stepperJoints[i]);
    }
  }
}

void stateERR() {
  // enter holding state
  for (int i = 0; i < NUM_JOINTS; ++i) {
    digitalWrite(STEP_PINS[i], LOW);
  }

  while (STATE == STATE_ERR) {
    Serial.println("ER: Unrecoverable error state entered. Please reset.");
    delay(1000);
  }
}

void loop() {
  switch (STATE) {
    case STATE_ERR:
      stateERR();
      break;
    default:
      stateTRAJ();
      break;
  }
}
