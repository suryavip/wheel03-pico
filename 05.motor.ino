const unsigned int MOTOR_POLE_PAIRS = 15;
const unsigned int MOTOR_VOLTAGE_LIMIT = 10;
const unsigned int MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT = 4;
const float MOTOR_ZERO_ELECTRICAL_ANGLE = 2.79; // DON'T FORGOT TO ADJUST THIS

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
Kalman veloFilter(0.01, 32, 1023, 0);

unsigned int lastMotorRequestMillis = 0;
int lastMotorRequestMagnitude = 0;
float filteredVelo = 0;

void setRequestMagnitude(int magnitude) {
  lastMotorRequestMagnitude = magnitude;
  lastMotorRequestMillis = millis();
}

void keepTrackVelo() {
  float velo = sensor.getVelocity();
  filteredVelo = veloFilter.getFilteredValue(velo);

  if (isMotorDebug) Serial.println(filteredVelo);
}

float additionalVoltageMultiplier() {
  float mapIn[]   = { -31, -30, -.5, .5, 30, 31};
  float mapOut[]  = {   2,   2,   1,  1,  2,  2};
  float mapResult = multiMap<float>(
                      filteredVelo,
                      mapIn,
                      mapOut,
                      6);

  return mapResult;
}

float calculateZeroElectricAngle() {
  float mapIn[]   = { -31, -30, -.5, .5,  30,  31};
  float mapOut[]  = {  .2,  .2,   0,  0, -.2, -.2};
  float mapResult = multiMap<float>(
                      filteredVelo,
                      mapIn,
                      mapOut,
                      6);

  return MOTOR_ZERO_ELECTRICAL_ANGLE + mapResult;
}

float magnitudeToTarget(int magnitude) {
  float mapIn[] =  { -10001, -10000,  -1, 0,  1, 10000, 10001};
  float mapOut[] = {     -7,     -7, -.9, 0, .9,     7,     7};
  float mapResult = multiMap<float>(
                      magnitude,
                      mapIn,
                      mapOut,
                      7);

  if (isMotorDebug) Serial.println(mapResult);
  return mapResult;
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
  motor.motion_downsample = 2;
  motor.zero_electric_angle = MOTOR_ZERO_ELECTRICAL_ANGLE;

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

  // Normal FOC routine.
  keepTrackVelo();
  motor.zero_electric_angle = calculateZeroElectricAngle();
  motor.target = magnitudeToTarget(lastMotorRequestMagnitude) * additionalVoltageMultiplier();
  motor.loopFOC();
  motor.move();
}
