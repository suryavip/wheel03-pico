const unsigned int MOTOR_POLE_PAIRS = 15;
const unsigned int MOTOR_VOLTAGE_LIMIT = 6;
const unsigned int MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT = 2;

int ffbMapSize = 5;
float ffbMapIn[]  = { -10000, -5000, - 100, -10, 0, 10, 100, 5000, 10000};
float ffbMapOut[] = {     -3,  -1.2,   -.3, -.1, 0, .1,  .3,  1.2,     3};

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

unsigned int lastMotorRequestMillis = 0;

void setMotorTarget(int magnitude) {
  float mapped = multiMap<float>(magnitude,
                                 ffbMapIn,
                                 ffbMapOut,
                                 ffbMapSize);
  motor.target = mapped;
  lastMotorRequestMillis = millis();
}

void motorSetup() {
  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);

  motor.target = 0;
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.voltage_sensor_align = MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT;
  motor.velocity_limit = 40;
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
  // Stop the motor if not receive any request after 100ms
  // since last command.
  if (millis() - lastMotorRequestMillis > 100) {
    motor.target = 0;
  }

  // Normal FOC routine.
  motor.loopFOC();
  motor.move();
}
