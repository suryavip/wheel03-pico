void replyWithPositions() {
  String separator = ",";
  String delimiter = ";";

  String output = int(currentRawPosition) + separator + overRotation;
  output += separator + accValue;
  output += separator + brkValue;
  output += separator + cltValue;
  output += delimiter;
  Serial.print(output);
}

void parser(String& valRef) {
  String val = "";

  if (Serial.available() < 1) return;
  char read = Serial.read();

  while (read != ';') {
    val += read;

    if (Serial.available() < 1) return;
    read = Serial.read();
  }

  valRef = val;
}

void comSetup() {
  Serial.begin();
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
