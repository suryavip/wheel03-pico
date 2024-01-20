const unsigned int MOTOR_POLE_PAIRS = 15;
const unsigned int MOTOR_VOLTAGE_LIMIT = 9;
const unsigned int MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT = 4;
//const float MOTOR_ZERO_ELECTRICAL_ANGLE = 2.80; // DON'T FORGOT TO ADJUST THIS

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

unsigned int lastMotorRequestMillis = 0;
float lastMotorRequestVoltage = 0;
float zeroElectricalAngle;

void setRequestVoltage(float v) {
  lastMotorRequestVoltage = v;
  lastMotorRequestMillis = millis();
}

float voltageMultiplierByVelo() {
  float mapIn[]   = { -21, 0, 21};
  float mapOut[]  = {   2, 1,  2};
  float mapResult = multiMap<float>(lastVelo, mapIn, mapOut, 3);

  return mapResult;
}

float zeaOffsetByVelo() {
  float mapIn[]   = { -5, 0,   5};
  float mapOut[]  = { .8, 0, -.8};
  float mapResult = multiMap<float>(lastVelo, mapIn, mapOut, 3);

  if (mapResult > .8) mapResult = .8;
  if (mapResult < -.8) mapResult = -.8;

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
    lastMotorRequestVoltage = 0;
  }

  // Normal FOC routine.
  motor.zero_electric_angle = zeroElectricalAngle + zeaOffsetByVelo();
  motor.target = lastMotorRequestVoltage * voltageMultiplierByVelo();

  motor.loopFOC();
  motor.move();
}
