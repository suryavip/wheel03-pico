const unsigned int MOTOR_VOLTAGE_LIMIT = 9;
const unsigned int MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT = 2.5;

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

unsigned int lastMotorRequestMillis = 0;
float lastMotorRequestVoltage = 0;

// ZEA: Zero electrical angle
float zeaPositions[MOTOR_POLE_PAIRS + 2];
float zeas[MOTOR_POLE_PAIRS + 2];

void setRequestVoltage(float v) {
  lastMotorRequestVoltage = v * motor.sensor_direction;
  lastMotorRequestMillis = millis();
}

float voltageMultiplierByVelo() {
  return 1;
  float mapIn[] = { -21, 0, 21 };
  float mapOut[] = { 2, 1, 2 };
  float mapResult = multiMap<float>(lastVelo, mapIn, mapOut, 3);

  return mapResult;
}

float zeaOffsetByVelo() {
  return 0;
  float mapIn[] = { -5, 0, 5 };
  float mapOut[] = { .8, 0, -.8 };
  float mapResult = multiMap<float>(lastVelo, mapIn, mapOut, 3);

  if (mapResult > .8) mapResult = .8;
  if (mapResult < -.8) mapResult = -.8;

  return mapResult * motor.sensor_direction;
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
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
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
  // motor.zero_electric_angle = zeaByPosition() + zeaOffsetByVelo();
  motor.target = lastMotorRequestVoltage * voltageMultiplierByVelo();

  motor.loopFOC();
  motor.move();
}

void sensorLinearizer() {
  float elAngle = _3PI_2;
  const int transition = 384;
  int rp[MOTOR_POLE_PAIRS];
  int cp[MOTOR_POLE_PAIRS];
  int regulerDistance = SENSOR_PPR / MOTOR_POLE_PAIRS;

  for (int i = 0; i < MOTOR_POLE_PAIRS; i++) {
    motor.setPhaseVoltage(MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT, 0, elAngle);
    _delay(700);

    sensor.update();
    rp[i] = currentRawAngle;
    if (i == 0) cp[i] = rp[i];
    else {
      int correctedPos = cp[i - 1] + (regulerDistance * motor.sensor_direction);
      if (correctedPos < 0) correctedPos += SENSOR_PPR;
      else if (correctedPos >= SENSOR_PPR) correctedPos -= SENSOR_PPR;
      cp[i] = correctedPos;
    }

    for (int j = 0; j < transition; j++) {
      elAngle = elAngle + (_2PI / transition);
      motor.setPhaseVoltage(MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT, 0, elAngle);
      sensor.update();
      _delay(1);
    }
  }

  overRotation = 0;

  if (motor.sensor_direction == -1) {
    int rpArraySize = sizeof(rp) / sizeof(rp[0]);
    reverseArray(rp, rpArraySize);
    int cpArraySize = sizeof(cp) / sizeof(cp[0]);
    reverseArray(cp, cpArraySize);
  }

  int rpArraySize = sizeof(rp) / sizeof(rp[0]);
  KickSort<int>::insertionSort(rp, rpArraySize);
  int cpArraySize = sizeof(cp) / sizeof(cp[0]);
  KickSort<int>::insertionSort(cp, cpArraySize);

  // add last position to the front so it's connecting to the first position when rotating over
  rawPositions[0] = rp[MOTOR_POLE_PAIRS - 1] - SENSOR_PPR;
  correctedPositions[0] = cp[MOTOR_POLE_PAIRS - 1] - SENSOR_PPR;

  // moving sorted data to global variable
  for (int i = 0; i < MOTOR_POLE_PAIRS; i++) {
    rawPositions[i + 1] = rp[i];
    correctedPositions[i + 1] = cp[i];
  }

  // add first position to the end so it's connecting to the last position when rotating over
  rawPositions[MOTOR_POLE_PAIRS + 1] = SENSOR_PPR + rp[0];
  correctedPositions[MOTOR_POLE_PAIRS + 1] = SENSOR_PPR + cp[0];

  if (isMotorDebug) {
    for (int i = 0; i < MOTOR_POLE_PAIRS; i++) {
      Serial.print(rp[i]);
      Serial.print(":");
      Serial.println(cp[i]);
    }

    Serial.println("---");

    for (int i = 0; i < MOTOR_POLE_PAIRS + 2; i++) {
      Serial.print(rawPositions[i]);
      Serial.print(":");
      Serial.println(correctedPositions[i]);
    }
  }

  linearizationDone = true;
}

void reverseArray(int array[], int size) {
  int temp;
  for (int i = 0; i < size / 2; i++) {
    temp = array[i];
    array[i] = array[size - i - 1];
    array[size - i - 1] = temp;
  }
}
