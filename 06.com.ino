const unsigned int FFB_MAX_VALUE = 10000;

bool freshStart = true;

String getCurrentPositionCommand() {
  String cmd = "E:";
  String delimiter = ";";
  return cmd + getMultiRotationValue() + delimiter;
}

void comSetup() {
  Serial.begin(115200);
  Serial.setTimeout(4);
}

void comLoop() {
  if (Serial.available() < 4) return;

  bool detectFFB = Serial.findUntil("F:", ";");
  if (detectFFB == false) return;

  if (freshStart) {
    freshStart = false;
    overRotation = 0;
  }

  // Calculating requested FFB percentage.
  String ffbValue = Serial.readString();
  ffbValue.replace(";", "");
  double readDouble = ffbValue.toDouble();
  double percentage = readDouble / FFB_MAX_VALUE;
  setMotorTarget(percentage);

  Serial.print(getCurrentPositionCommand());
}
