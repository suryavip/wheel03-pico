const unsigned int MOTOR_POLE_PAIRS = 15;
const unsigned int MOTOR_VOLTAGE_LIMIT = 9;
const unsigned int MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT = 3;

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

unsigned int lastMotorRequestMillis = 0;
float lastMotorRequestVoltage = 0;

// ZEA: Zero electrical angle
float zea;

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

  return mapResult * motor.sensor_direction;
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
  sensorLinearizer();
  motor.initFOC();
  zea = motor.zero_electric_angle;
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
  motor.zero_electric_angle = zea + zeaOffsetByVelo();
  motor.target = lastMotorRequestVoltage * voltageMultiplierByVelo();

  motor.loopFOC();
  motor.move();
}

// call this after motor.init() and before motor.initFOC()
void sensorLinearizer() {
  float elAngle = _3PI_2;
  const int transition = 500;
  float rp[MOTOR_POLE_PAIRS];

  // collect raw positions for each home pole pair
  for (int i = 0; i < MOTOR_POLE_PAIRS; i++) {
    motor.setPhaseVoltage(MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT, 0, elAngle);
    _delay(700);

    sensor.update();
    rp[i] = currentRawPosition;

    for (int j = 0; j < transition; j++) {
      elAngle = elAngle + (_2PI / transition);
      motor.setPhaseVoltage(MOTOR_VOLTAGE_LIMIT_FOR_ALIGNMENT, 0, elAngle);
      sensor.update();
      _delay(1);
    }
  }

  // sort raw positions, ascending
  int rpArraySize = sizeof(rp) / sizeof(rp[0]);
  KickSort<float>::insertionSort(rp, rpArraySize);

  // calculating the corrected positions
  float cp[MOTOR_POLE_PAIRS];
  float regulerDistance = float(SENSOR_PPR) / float(MOTOR_POLE_PAIRS);
  for (int i = 0; i < MOTOR_POLE_PAIRS; i++) {
    if (i == 0) cp[i] = rp[i];
    else {
      float correctedPos = cp[i - 1] + regulerDistance;
      if (correctedPos < 0) correctedPos += float(SENSOR_PPR);
      else if (correctedPos >= float(SENSOR_PPR)) correctedPos -= float(SENSOR_PPR);
      cp[i] = correctedPos;
    }
  }

  float rp2[MOTOR_POLE_PAIRS + 2];
  float cp2[MOTOR_POLE_PAIRS + 2];

  // add last position to the front so it's connecting to the first position when rotating over
  rp2[0] = rp[MOTOR_POLE_PAIRS - 1] - float(SENSOR_PPR);
  cp2[0] = cp[MOTOR_POLE_PAIRS - 1] - float(SENSOR_PPR);

  // copy existing data
  for (int i = 0; i < MOTOR_POLE_PAIRS; i++) {
    rp2[i + 1] = rp[i];
    cp2[i + 1] = cp[i];
  }

  // add first position to the end so it's connecting to the last position when rotating over
  rp2[MOTOR_POLE_PAIRS + 1] = float(SENSOR_PPR) + rp[0];
  cp2[MOTOR_POLE_PAIRS + 1] = float(SENSOR_PPR) + cp[0];

  // map all the possible positions
  for (int i = 0; i < SENSOR_PPR; i++) {
    float p = multiMap<float>(i, rp2, cp2, MOTOR_POLE_PAIRS + 2);
    p = round(p);
    if (p < 0) p += float(SENSOR_PPR);
    else if (p >= float(SENSOR_PPR)) p -= float(SENSOR_PPR);
    linearized[i] = p;
  }

  overRotation = 0;
  linearizationDone = true;
}
