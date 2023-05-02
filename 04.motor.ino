const unsigned int MOTOR_POLE_PAIRS = 15;
const unsigned int MOTOR_VOLTAGE_LIMIT = 7;
const unsigned int MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT = 3;
const float MOTOR_FREEWHEEL_ASSIST_TARGET = .5;

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
Kalman veloFilter(0.01, 32, 1023, 0);

unsigned int lastMotorRequestMillis = 0;
float lastMotorTarget = 0;

void setMotorTarget(int magnitude) {
  int absMagnitude = abs(magnitude);

  if (absMagnitude > 10000) absMagnitude = 10000;

  float mapMagnitudeIn[] = {0, 1, 10000};
  float mapTargetOut[] = {0, .7, 7};
  float mappedTarget = multiMap<float>(
                         absMagnitude,
                         mapMagnitudeIn,
                         mapTargetOut,
                         3);

  // Apply back direction.
  if (magnitude < 0) mappedTarget *= -1;

  if (isMotorDebug) Serial.println(mappedTarget);
  lastMotorTarget = mappedTarget;
  lastMotorRequestMillis = millis();
}

float calculateFreewheelAssist() {
  if (lastMotorTarget > MOTOR_FREEWHEEL_ASSIST_TARGET) return 0;
  if (lastMotorTarget < MOTOR_FREEWHEEL_ASSIST_TARGET * -1) return 0;
  
  float velo = sensor.getVelocity();
  float filteredVelo = veloFilter.getFilteredValue(velo);
  float absVelo = abs(filteredVelo);

  float mapAbsVeloIn[] = {0, 0.1, 1};
  float mapTargetOut[] = {0, 0, 1};
  float mappedTarget = multiMap<float>(
                         absVelo,
                         mapAbsVeloIn,
                         mapTargetOut,
                         3);
  if (mappedTarget > MOTOR_FREEWHEEL_ASSIST_TARGET) mappedTarget = MOTOR_FREEWHEEL_ASSIST_TARGET;

  if (filteredVelo < 0) mappedTarget *= -1;
  return mappedTarget;
}

void motorSetup() {
  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);

  motor.target = 0;
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.voltage_sensor_align = MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT;
  motor.velocity_limit = 60;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.init();
  motor.initFOC();
}

void motorLoop() {
  // If motor disabled, just update the sensor value
  // to make the steering wheel still usable without force feedback.
  if (motor.enabled == false) {
    sensor.update();
    return;
  }

  // Safety:
  // Stop the motor if not receive any request after 500ms
  // since last command.
  if (millis() - lastMotorRequestMillis > 500) {
    lastMotorTarget = 0;
  }

  // Normal FOC routine.
  motor.target = lastMotorTarget + calculateFreewheelAssist();
  motor.loopFOC();
  motor.move();
}
