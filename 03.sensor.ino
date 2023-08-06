const unsigned int SENSOR_PPR = 4096;
const unsigned int SENSOR_MID_PPR = SENSOR_PPR / 2;
const unsigned int SENSOR_MID_PPR_MIN_1 = SENSOR_MID_PPR - 1;
const unsigned int SENSOR_I2C_CLOCK = 400 * 1000;

AS5600 as5600;
GenericSensor sensor;
Kalman veloFilter(0.01, 16, 1023, 0);

int currentRawAngle = -1;
int overRotation = 0;
float filteredVelo = 0;

float readMySensorCallback() {
  int newPosition = as5600.rawAngle();

  // Calculate over rotation.
  int distance = newPosition - currentRawAngle;
  if (abs(distance) > SENSOR_MID_PPR_MIN_1) {
    if (currentRawAngle > -1) {
      if (newPosition < SENSOR_MID_PPR) overRotation++;
      else overRotation--;
    }
  }

  currentRawAngle = newPosition;

  double rawAngleInDouble = currentRawAngle;
  double percent = rawAngleInDouble / SENSOR_PPR;
  return TWO_PI * percent;
}

void sensorSetup() {
  Wire.setSDA(PIN_SENSOR_SDA);
  Wire.setSCL(PIN_SENSOR_SCL);
  Wire.setClock(SENSOR_I2C_CLOCK);
  as5600.begin();
  sensor = GenericSensor(readMySensorCallback);
  sensor.init();
}

void sensorLoop() {
  float velo = sensor.getVelocity();
  filteredVelo = veloFilter.getFilteredValue(velo);
}
