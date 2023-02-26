String getCurrentPositionCommand() {
  String cmd = "E:";
  String delimiter = ";";
  return cmd + getMultiRotationValueWithOffset() + delimiter;
}

void setAsZeroOffset() {
  setZeroOffset(getMultiRotationValue());
}

void setForceFeedback(String value) {
  int magnitude = value.toInt();
  setMotorTarget(magnitude);
  Serial.print(getCurrentPositionCommand());
}

void parser(String & cmdRef, String & valRef) {
  String cmd = "";
  String val = "";

  // skip if < 3. Need min. 3 for 1 command: 1 cmd char, 1 colon, 1 semi-colon.
  while (Serial.available() < 3) return;

  char read = Serial.read();
  while (read != ':') {
    cmd += read;
    if (Serial.available() < 1) return;
    read = Serial.read();
  }

  read = Serial.read();
  while (read != ';') {
    val += read;
    if (Serial.available() < 1) return;
    read = Serial.read();
  }

  cmdRef = cmd;
  valRef = val;
}

void comSetup() {
  Serial.begin(115200);
  Serial.setTimeout(4);
}

void comLoop() {
  String cmd = "";
  String val = "";
  parser(cmd, val);

  if (cmd == "F") setForceFeedback(val);
  if (cmd == "C") setAsZeroOffset();
}
