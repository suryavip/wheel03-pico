const unsigned int MOTOR_POLE_PAIRS = 15;
const unsigned int MOTOR_VOLTAGE_LIMIT = 6;
const unsigned int MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT = 2;
const float MOTOR_MIN_ROTATING_VOLTAGE = .5;

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

unsigned int lastMotorRequestMillis = 0;

void setMotorTarget(int magnitude) {
  float full = MOTOR_VOLTAGE_LIMIT * savedMaxPower;
  float quarter = full * .25;
  
  float ffbMapIn[]  = {0, 100, 2500, 10000};
  float ffbMapOut[] = {0, MOTOR_MIN_ROTATING_VOLTAGE, quarter, full};
  float mapped = multiMap<float>(abs(magnitude), ffbMapIn, ffbMapOut, 4);

  // Apply back direction.
  if (magnitude < 0) mapped *= -1;

  if (isDebug) Serial.println(mapped);
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
