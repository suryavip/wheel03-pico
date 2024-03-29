const unsigned int MOTOR_POLE_PAIRS = 15;
const unsigned int MOTOR_VOLTAGE_LIMIT = 9;
const unsigned int MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT = 2;

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

unsigned int lastMotorRequestMillis = 0;
float lastMotorRequestVoltage = 0;

// ZEA: Zero electrical angle
float zeaPositions[MOTOR_POLE_PAIRS + 2];
float zeas[MOTOR_POLE_PAIRS + 2];

void setRequestVoltage(float v) {
  lastMotorRequestVoltage = v;
  lastMotorRequestMillis = millis();
}

float voltageMultiplierByVelo() {
  float mapIn[] = { -21, 0, 21 };
  float mapOut[] = { 2, 1, 2 };
  float mapResult = multiMap<float>(lastVelo, mapIn, mapOut, 3);

  return mapResult;
}

float zeaOffsetByVelo() {
  float mapIn[] = { -5, 0, 5 };
  float mapOut[] = { .8, 0, -.8 };
  float mapResult = multiMap<float>(lastVelo, mapIn, mapOut, 3);

  if (mapResult > .8) mapResult = .8;
  if (mapResult < -.8) mapResult = -.8;

  return mapResult;
}

float zeaByPosition() {
  float mapResult = multiMap<float>(
    float(currentRawAngle),
    zeaPositions,
    zeas,
    MOTOR_POLE_PAIRS + 2);
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
  motor.motion_downsample = 1;

  motor.init();
  motor.initFOC();

  sensorLinearizer();
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
  motor.zero_electric_angle = zeaByPosition() + zeaOffsetByVelo();
  motor.target = lastMotorRequestVoltage * voltageMultiplierByVelo();

  motor.loopFOC();
  motor.move();
}

void sensorLinearizer() {
  float elAngle = _3PI_2;
  const int transition = 384;

  int p[MOTOR_POLE_PAIRS];
  float z[MOTOR_POLE_PAIRS];

  for (int i = 0; i < MOTOR_POLE_PAIRS; i++) {
    motor.setPhaseVoltage(MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT, 0, elAngle);
    _delay(700);

    sensor.update();
    motor.zero_electric_angle = 0;
    float thisEa = motor.electricalAngle();
    p[i] = currentRawAngle;
    z[i] = thisEa + .23;

    if (i == MOTOR_POLE_PAIRS - 1) break;

    for (int j = 0; j < transition; j++) {
      elAngle = elAngle + (_2PI / transition);
      motor.setPhaseVoltage(MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT, 0, elAngle);
      sensor.update();
      _delay(1);
    }
  }

  overRotation = 0;

  // find the starting index
  int l = SENSOR_PPR;
  int li = 0;
  for (int i = 0; i < MOTOR_POLE_PAIRS; i++) {
    if (p[i] < l) {
      l = p[i];
      li = i;
    }
  }

  // put sorted to global var
  float ps[MOTOR_POLE_PAIRS];
  float zs[MOTOR_POLE_PAIRS];
  for (int i = li; i < MOTOR_POLE_PAIRS; i++) {
    ps[i - li] = float(p[i]);
    zs[i - li] = z[i];
  }
  for (int i = 0; i < li; i++) {
    ps[i + MOTOR_POLE_PAIRS - li] = float(p[i]);
    zs[i + MOTOR_POLE_PAIRS - li] = z[i];
  }

  // add last ZEA to the front so it's connecting to the first ZEA when rotating over the position
  zeaPositions[0] = ps[MOTOR_POLE_PAIRS - 1] - SENSOR_PPR;
  zeas[0] = zs[MOTOR_POLE_PAIRS - 1];

  // moving sorted data to global variable
  for (int i = 0; i < MOTOR_POLE_PAIRS; i++) {
    zeaPositions[i + 1] = ps[i];
    zeas[i + 1] = zs[i];
  }

  // add first ZEA to the end so it's connecting to the last ZEA when rotating over the position
  zeaPositions[MOTOR_POLE_PAIRS + 1] = SENSOR_PPR + ps[0];
  zeas[MOTOR_POLE_PAIRS + 1] = zs[0];

  if (isMotorDebug) {
    for (int i = 0; i < MOTOR_POLE_PAIRS; i++) {
      Serial.print(p[i]);
      Serial.print(":");
      Serial.println(z[i]);
    }

    Serial.println("---");

    for (int i = 0; i < MOTOR_POLE_PAIRS + 2; i++) {
      Serial.print(zeaPositions[i]);
      Serial.print(":");
      Serial.println(zeas[i]);
    }
  }
}
