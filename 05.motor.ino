const unsigned int MOTOR_POLE_PAIRS = 15;
const unsigned int MOTOR_VOLTAGE_LIMIT = 9;
const unsigned int MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT = 4;
const float MOTOR_ZERO_ELECTRICAL_ANGLE = 2.79; // DON'T FORGOT TO ADJUST THIS

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
Kalman veloFilter(0.01, 16, 1023, 0);

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

float additionalVoltage() {
  float mapIn[]  = {0.0, 4.0, 51.0, 52.0, 98.0, 98.0, 127.0, 135.0, 139.0, 139.0, 142.0, 184.0, 319.0, 324.0, 367.0, 368.0, 373.0, 412.0, 460.0, 462.0, 505.0, 508.0, 575.0, 586.0, 594.0, 631.0, 639.0, 640.0, 641.0, 681.0, 682.0, 682.0, 932.0, 951.0, 958.0, 959.0, 960.0, 1222.0, 1229.0, 1270.0, 1275.0, 1276.0, 1326.0, 1463.0, 1468.0, 1503.0, 1506.0, 1548.0, 1640.0, 1653.0, 1679.0, 1685.0, 1686.0, 1732.0, 1732.0, 1768.0, 1774.0, 1776.0, 1780.0, 1865.0, 1874.0, 1916.0, 1928.0, 1956.0, 1959.0, 1963.0, 1963.0, 2006.0, 2007.0, 2007.0, 2048.0, 2048.0, 2048.0, 2328.0, 2328.0, 2328.0, 2509.0, 2511.0, 2518.0, 2547.0, 2551.0, 2552.0, 2554.0, 2594.0, 2594.0, 2595.0, 2638.0, 2638.0, 2688.0, 2824.0, 2864.0, 2866.0, 2870.0, 2916.0, 2925.0, 2955.0, 2958.0, 3087.0, 3094.0, 3097.0, 3097.0, 3098.0, 3224.0, 3233.0, 3241.0, 3278.0, 3327.0, 3334.0, 3368.0, 3370.0, 3370.0, 3411.0, 3413.0, 3413.0, 3414.0, 3464.0, 3600.0, 3633.0, 3633.0, 3641.0, 3642.0, 3645.0, 3646.0, 3646.0, 3687.0, 3687.0, 3687.0, 3958.0, 3965.0, 3966.0, 3998.0, 3999.0, 3999.0, 4001.0, 4001.0, 4002.0};
  float mapOut[] = {0.147, 0.14, 0.175, 0.175, 0.196, 0.196, 0.077, 0.189, 0.224, 0.224, 0.21, 0.217, 0.161, 0.161, 0.231, 0.231, 0.203, 0.238, 0.182, 0.168, 0.182, 0.161, 0, 0.098, 0.168, 0.112, 0.203, 0.203, 0.196, 0.252, 0.252, 0.252, 0.077, 0.203, 0.266, 0.266, 0.266, 0.154, 0.231, 0.224, 0.231, 0.238, 0.147, 0.182, 0.154, 0.217, 0.217, 0.224, 0.133, 0.042, 0.119, 0.168, 0.168, 0.231, 0.231, 0.168, 0.245, 0.245, 0.231, 0.147, 0.133, 0.147, 0.042, 0.147, 0.161, 0.168, 0.161, 0.196, 0.203, 0.196, 0.252, 0.252, 0.252, 0.231, 0.231, 0.231, 0.147, 0.119, 0.042, 0.175, 0.203, 0.203, 0.203, 0.238, 0.238, 0.245, 0.252, 0.252, 0.189, 0.238, 0.217, 0.238, 0.245, 0.189, 0.098, 0.203, 0.203, 0.105, 0.231, 0.245, 0.245, 0.252, 0.049, 0.175, 0.112, 0.147, 0.154, 0.07, 0.196, 0.203, 0.203, 0.224, 0.245, 0.238, 0.245, 0.182, 0.147, 0.049, 0.042, 0.196, 0.203, 0.217, 0.217, 0.217, 0.273, 0.273, 0.273, 0.238, 0.189, 0.175, 0.21, 0.224, 0.224, 0.245, 0.245, 0.245};
  float mapResult = multiMap<float>(currentRawAngle, mapIn, mapOut, 136);

  if (lastMotorRequestMagnitude < 0) mapResult *= -1;

  return mapResult;
}

float additionalVoltageMultiplier() {
  float mapIn[]   = { -60, -20, -.5, .5,  20,  60};
  float mapOut[]  = { 1.3, 1.3,   1,  1, 1.3, 1.3};
  float mapResult = multiMap<float>(filteredVelo, mapIn, mapOut, 6);

  return mapResult;
}

float calculateZeroElectricAngle() {
  float mapIn[]   = { -60, -5, -.5, .5,   5,  60};
  float mapOut[]  = {  .8, .8,   0,  0, -.8, -.8};
  float mapResult = multiMap<float>(filteredVelo, mapIn, mapOut, 6);

  return MOTOR_ZERO_ELECTRICAL_ANGLE + mapResult;
}

float magnitudeToTarget() {
  float minPositive = savedMinimumOutputVoltage;
  float minNegative = savedMinimumOutputVoltage * -1;

  float mapIn[]   = { -10001, -10000,          -1, 0,           1, 10000, 10001};
  float mapOut[]  = {     -7,     -7, minNegative, 0, minPositive,     7,     7};
  float mapResult = multiMap<float>(lastMotorRequestMagnitude, mapIn, mapOut, 7);

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
  float baseTarget = magnitudeToTarget();
  float calibratedTarget = baseTarget + additionalVoltage();
  float speedAdjustedTarget = calibratedTarget * additionalVoltageMultiplier();
  motor.target = speedAdjustedTarget;
  motor.loopFOC();
  motor.move();
}
