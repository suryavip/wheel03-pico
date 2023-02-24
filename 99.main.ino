bool setupDone = false;

void setup() {
  debugLedStart();

  comSetup();
  sensorSetup();
//  motorSetup();

  debugLedStop();

  setupDone = true;
}

void loop() {
  sensor.update();
  comLoop();
}

void loop1() {
  if (setupDone == false) return;
  motorLoop();
}
