const unsigned int MOTOR_POLE_PAIRS = 15;
const unsigned int MOTOR_VOLTAGE_LIMIT = 9;
const unsigned int MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT = 4;
//const float MOTOR_ZERO_ELECTRICAL_ANGLE = 2.80; // DON'T FORGOT TO ADJUST THIS

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

unsigned int lastMotorRequestMillis = 0;
int lastMotorRequestMagnitude = 0;
float zeroElectricalAngle;

void setRequestMagnitude(int magnitude) {
  if (magnitude > 10000) magnitude = 10000;
  if (magnitude < -10000) magnitude = -10000;
  lastMotorRequestMagnitude = magnitude;
  lastMotorRequestMillis = millis();
}

float voltageMultiplierByVelo() {
  float mapIn[]   = { -21, 0, 21};
  float mapOut[]  = {   2, 1,  2};
  float mapResult = multiMap<float>(lastVelo, mapIn, mapOut, 3);

  return mapResult;
}

float voltageByMagnitude() {
  float minPositive = savedMinimumOutputVoltage;
  float minNegative = minPositive * -1;

  float mapIn[]   = { -10000,          -1, 0,           1, 10000};
  float mapOut[]  = {     -7, minNegative, 0, minPositive,     7};
  float mapResult = multiMap<float>(lastMotorRequestMagnitude, mapIn, mapOut, 5);

  return mapResult;
}

float zeaOffsetByVelo() {
  float mapIn[]   = { -5, 0,   5};
  float mapOut[]  = { .8, 0, -.8};
  float mapResult = multiMap<float>(lastVelo, mapIn, mapOut, 3);

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
  //  motor.zero_electric_angle = MOTOR_ZERO_ELECTRICAL_ANGLE;

  motor.init();
  motor.initFOC();

  zeroElectricalAngle = motor.zero_electric_angle;
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
  float zeaAdjustment = zeaOffsetByVelo();
  if (zeaAdjustment > .8) zeaAdjustment = .8;
  if (zeaAdjustment < -.8) zeaAdjustment = -.8;
  motor.zero_electric_angle = zeroElectricalAngle + zeaAdjustment;

  float baseVoltage = voltageByMagnitude();
  float veloAdjustedVoltage = baseVoltage * voltageMultiplierByVelo();
  motor.target = veloAdjustedVoltage;

  motor.loopFOC();
  motor.move();
}
