Kalman accFilter(0.125, 32, 1023, 0);
Kalman brkFilter(0.125, 32, 1023, 0);
Kalman cltFilter(0.125, 32, 1023, 0);

int accelerator_pedal = 0;
int brake_pedal = 0;
int clutch_pedal = 0;

void pedalsSetup() {
  analogReadResolution(12);
}

void pedalsLoop() {
  int acc = analogRead(PIN_ACCELERATOR);
  int brk = analogRead(PIN_BRAKE);
  int clt = analogRead(PIN_CLUTCH);

  accelerator_pedal = accFilter.getFilteredValue(acc);
  brake_pedal = brkFilter.getFilteredValue(brk);
  clutch_pedal = cltFilter.getFilteredValue(clt);

  //  Serial.print("Min:");
  //  Serial.print(0);
  //  Serial.print(",A:");
  //  Serial.print(acc);
  //  Serial.print(",B:");
  //  Serial.print(brk);
  //  Serial.print(",C:");
  //  Serial.print(clt);
  //  Serial.print(",Max:");
  //  Serial.println(4095);
}
