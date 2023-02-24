const uint8_t MOTOR_POLE_PAIRS = 15;
const float MOTOR_PHASE_RESISTANCE = 0.5;
const float MOTOR_MAX_TARGET = 8;

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE);

void motorSetup() {
  debugLedSegment();

  driver.pwm_frequency = 3000;
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 5;
  driver.init();

  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);

  motor.target = 0;
  motor.voltage_limit = 5;
  motor.current_limit = 5;
  motor.voltage_sensor_align = 2;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SinePWM;

  debugLedSegment();

  motor.init();
  motor.initFOC();
}

unsigned long lastMotorRequestMillis = 0;

void motorLoop() {
  // Blinking LED to let user know that the motor is disabled.
  // Micro-controller need to be restarted.
  if (motor.enabled == false) {
    bool ledState = true;
    while (true) {
      digitalWrite(LED_BUILTIN, ledState);
      delay(200);
      ledState = !ledState;
    }
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

void requestMotorTarget(double percentage) {
  motor.target = MOTOR_MAX_TARGET * percentage;
  lastMotorRequestMillis = millis();
}
