Kalman accFilter(0.1, 32, 4095, 0);
Kalman brkFilter(0.1, 32, 4095, 0);
Kalman cltFilter(0.1, 32, 4095, 0);

int accValue = 0;
int brkValue = 0;
int cltValue = 0;

void pedalsSetup() {
  analogReadResolution(12);
}

void pedalsLoop() {
  int acc = analogRead(PIN_ACCELERATOR);
  int brk = analogRead(PIN_BRAKE);
  int clt = analogRead(PIN_CLUTCH);

  accValue = accFilter.getFilteredValue(acc);
  brkValue = brkFilter.getFilteredValue(brk);
  cltValue = cltFilter.getFilteredValue(clt);

  if (isPedalsDebug) {
    Serial.print("Min:");
    Serial.print(0);
    Serial.print(",A:");
    Serial.print(acc);
    Serial.print(",AF:");
    Serial.print(accValue);
    Serial.print(",B:");
    Serial.print(brk);
    Serial.print(",BF:");
    Serial.print(brkValue);
    Serial.print(",C:");
    Serial.print(clt);
    Serial.print(",CF:");
    Serial.print(cltValue);
    Serial.print(",Max:");
    Serial.println(4095);
  }
}
