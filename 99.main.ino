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

void setup1() {
  // just blink while setup

  pinMode(25, OUTPUT);
  bool blink = true;
  while (!setupDone) {
    digitalWrite(25, blink);
    blink = !blink;
    _delay(200);
  }
  digitalWrite(25, LOW);
}

void loop() {
  pedalsLoop();
  comLoop();
  sensorLoop();
}

void loop1() {
  if (setupDone == false) return;
  motorLoop();
}
