void replyWithPositions() {
  String separator = ",";
  String delimiter = ";";

  String output = currentRawAngle + separator + overRotation;
  output += separator + accValue;
  output += separator + brkValue;
  output += separator + cltValue;
  output += delimiter;
  Serial.print(output);
}

void parser(String& valRef) {
  String val = "";

  char read = Serial.read();
  while (read != ';') {
    val += read;
    if (Serial.available() < 1) return;
    read = Serial.read();
  }

  valRef = val;
}

void comSetup() {
  Serial.begin(115200);
  Serial.setTimeout(4);
}

void comLoop() {
  String val = "";
  parser(val);

  if (val != "") {
    replyWithPositions();
    float v = val.toFloat();
    setRequestVoltage(v);
  }
}
