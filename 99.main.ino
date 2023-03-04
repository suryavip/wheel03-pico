bool setupDone = false;

void setup() {
  configSetup();
  pedalsSetup();
  comSetup();
  sensorSetup();
  driverSetup();
  motorSetup();
  setupDone = true;
}

void loop() {
  pedalsLoop();
  comLoop();
}

void loop1() {
  if (setupDone == false) return;
  motorLoop();
}
