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
}

float voltageMultiplierByVelo() {
  float mapIn[]   = { -60, -20, -.5, .5,  20,  60};
  float mapOut[]  = { 1.3, 1.3,   1,  1, 1.3, 1.3};
  float mapResult = multiMap<float>(filteredVelo, mapIn, mapOut, 6);

  return mapResult;
}

float voltageByMagnitude() {
  float minPositive = savedMinimumOutputVoltage;
  float minNegative = minPositive * -1;

  float mapIn[]   = { -10001, -10000,          -1, 0,           1, 10000, 10001};
  float mapOut[]  = {     -7,     -7, minNegative, 0, minPositive,     7,     7};
  float mapResult = multiMap<float>(lastMotorRequestMagnitude, mapIn, mapOut, 7);

  return mapResult;
}

float zeaOffsetByVelo() {
  float mapIn[]   = { -60, -5, -.5, .5,   5,  60};
  float mapOut[]  = {  .8, .8,   0,  0, -.8, -.8};
  float mapResult = multiMap<float>(filteredVelo, mapIn, mapOut, 6);

  return mapResult;
}

float zeaOffsetByMagnitude() {
  float mapIn[]   = { -1001, -1000,  -1, 1, 1000, 1001};
  float mapOut[]  = {     0,     0, -.4, 4,    0,    0};
  float mapResult = multiMap<float>(lastMotorRequestMagnitude, mapIn, mapOut, 6);

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

  float zeaAdjustment = zeaOffsetByMagnitude() + zeaOffsetByVelo();
  if (zeaAdjustment > .8) zeaAdjustment = .8;
  if (zeaAdjustment < -.8) zeaAdjustment = -.8;
  motor.zero_electric_angle = MOTOR_ZERO_ELECTRICAL_ANGLE + zeaAdjustment;

  float baseVoltage = voltageByMagnitude();
  float veloAdjustedVoltage = baseVoltage * voltageMultiplierByVelo();
  motor.target = veloAdjustedVoltage;

  motor.loopFOC();
  motor.move();
}
