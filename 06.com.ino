String getCurrentPositionsCommand() {
  String cmd = "E:";
  String separator = ",";
  String delimiter = ";";

  String output = cmd + getMultiRotationValueWithOffset();
  output += separator + accelerator_pedal;
  output += separator + brake_pedal;
  output += separator + clutch_pedal;

  return output + delimiter;
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

  if (cmd == "F") {
    int magnitude = val.toInt();
    setMotorTarget(magnitude);
    Serial.print(getCurrentPositionCommand());
  }

  if (cmd == "C") {
    setSavedZeroOffset(getMultiRotationValue());
  }

  if (cmd == "P") {
    setSavedMaxPower(val.toFloat());
  }
}
