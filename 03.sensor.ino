const unsigned int SENSOR_PPR = 4096;
const unsigned int SENSOR_MID_PPR = SENSOR_PPR / 2;
const unsigned int SENSOR_MID_PPR_MIN_1 = SENSOR_MID_PPR - 1;
const unsigned int SENSOR_I2C_CLOCK = 400 * 1000;

AS5600 as5600;
GenericSensor sensor;

float currentRawPosition = -1;
int overRotation = 0;

// Sensor linearization
float linearized[SENSOR_PPR];
bool linearizationDone = false;

// speed measurement
float lastRawPosition = -1;
int lastOverRotation = 0;
unsigned int lastVeloMillis = 0;
float lastVelo = 0;

float readMySensorCallback() {
  float newPosition = float(as5600.rawAngle());

  if (linearizationDone) {
    newPosition = linearized[int(newPosition)];
  }

  // Calculate over rotation.
  float distance = newPosition - currentRawPosition;
  if (abs(distance) > float(SENSOR_MID_PPR_MIN_1)) {
    if (currentRawPosition > -1) {
      if (newPosition < float(SENSOR_MID_PPR)) overRotation++;
      else overRotation--;
    }
  }

  currentRawPosition = newPosition;
  float percent = currentRawPosition / SENSOR_PPR;
  return TWO_PI * percent;
}

void keepTrackVelocity() {
  int td = millis() - lastVeloMillis;
  if (td < 10) return;

  if (lastRawPosition < 0) {
    // Initial recording
    lastRawPosition = currentRawPosition;
    lastOverRotation = overRotation;
    lastVeloMillis = millis();
    return;
  }

  float c = currentRawPosition + (float(SENSOR_PPR) * float(overRotation));
  float l = lastRawPosition + (float(SENSOR_PPR) * float(lastOverRotation));
  float d = c - l;
  float pd = d / float(SENSOR_PPR);
  float rd = TWO_PI * pd;
  float v = rd / float(td);  // rad/ms
  v *= 1000;                 // rad/s
  lastVelo = v;

  lastRawPosition = currentRawPosition;
  lastOverRotation = overRotation;
  lastVeloMillis = millis();
}

void sensorSetup() {
  Wire.setSDA(PIN_SENSOR_SDA);
  Wire.setSCL(PIN_SENSOR_SCL);
  Wire.setClock(SENSOR_I2C_CLOCK);
  Wire.begin();
  as5600.begin();
  sensor = GenericSensor(readMySensorCallback);
  sensor.init();
}

void sensorLoop() {
  keepTrackVelocity();
}
