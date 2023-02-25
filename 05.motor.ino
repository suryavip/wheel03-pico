const uint8_t MOTOR_POLE_PAIRS = 15;
const float MOTOR_PHASE_RESISTANCE = 0.5;
const float MOTOR_MAX_TARGET = 8;
const unsigned int MOTOR_VOLTAGE_LIMIT = 5;
const unsigned int MOTOR_CURRENT_LIMIT = 5;
const unsigned int MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT = 2;

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE);

unsigned long lastMotorRequestMillis = 0;

void setMotorTarget(double percentage) {
  motor.target = MOTOR_MAX_TARGET * percentage;
  lastMotorRequestMillis = millis();
}

void motorSetup() {
  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);

  motor.target = 0;
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.current_limit = MOTOR_CURRENT_LIMIT;
  motor.voltage_sensor_align = MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT;
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
