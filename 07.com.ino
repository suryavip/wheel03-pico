void replyWithPositions() {
  String cmd = "E:";
  String separator = ",";
  String delimiter = ";";

  String output = cmd + currentRawAngle;
  output += separator + overRotation;
  output += separator + accValue;
  output += separator + brkValue;
  output += separator + cltValue;

  output += delimiter;
  Serial.print(output);
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
    replyWithPositions();
    int magnitude = val.toInt();
    setRequestMagnitude(magnitude);
  }

  if (cmd == "M") {
    float minimumOutputVoltage = val.toFloat();
    setSavedMinimumOutputVoltage(minimumOutputVoltage);
  }
}
