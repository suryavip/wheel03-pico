const unsigned int MOTOR_POLE_PAIRS = 15;
const unsigned int MOTOR_VOLTAGE_LIMIT = 7;
const unsigned int MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT = 3;

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
Kalman veloFilter(0.01, 32, 1023, 0);

unsigned int lastMotorRequestMillis = 0;
int lastMotorRequestMagnitude = 0;

void setRequestMagnitude(int magnitude) {
  lastMotorRequestMagnitude = magnitude;
  lastMotorRequestMillis = millis();
}

int veloReactiveMagnitude() {
  float velo = sensor.getVelocity();
  float filteredVelo = veloFilter.getFilteredValue(velo);

  int magnitude = savedVeloReactiveMagnitude;

  if (filteredVelo > .3) magnitude *= 1;
  else if (filteredVelo < -.3) magnitude *= -1;
  else magnitude = 0;

  return magnitude;
}

float magnitudeToTarget(int magnitude) {
  int absMagnitude = abs(magnitude);

  if (absMagnitude > 10000) absMagnitude = 10000;

  float mapMagnitudeIn[] = {0, 1, 10000};
  float mapTargetOut[] = {0, .8, 7};
  float mappedTarget = multiMap<float>(
                         absMagnitude,
                         mapMagnitudeIn,
                         mapTargetOut,
                         3);

  // Apply back direction.
  if (magnitude < 0) mappedTarget *= -1;

  if (isMotorDebug) Serial.println(mappedTarget);
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
  motor.foc_modulation = FOCModulationType::SinePWM;

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
    lastMotorRequestMagnitude = 0;
  }

  int magnitude = lastMotorRequestMagnitude + veloReactiveMagnitude();

  // Normal FOC routine.
  motor.target = magnitudeToTarget(magnitude);
  motor.loopFOC();
  motor.move();
}
