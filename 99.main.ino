bool setupDone = false;

void setup() {
  comSetup();
  sensorSetup();
  driverSetup();
  motorSetup();
  setupDone = true;
}

void loop() {
  comLoop();
}

void loop1() {
  if (setupDone == false) return;
  motorLoop();
}
