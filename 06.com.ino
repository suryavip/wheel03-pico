const unsigned int FFB_MAX_VALUE = 10000;

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

  bool detectSetZeroOffsetCmd = Serial.findUntil("C:", ";");
  if (detectSetZeroOffsetCmd) {
    setZeroOffset(currentRawAngle);
  }

  bool detectFfbCmd = Serial.findUntil("F:", ";");
  if (detectFfbCmd) {
    // Calculating requested FFB percentage.
    String ffbValue = Serial.readString();
    ffbValue.replace(";", "");
    double readDouble = ffbValue.toDouble();
    double percentage = readDouble / FFB_MAX_VALUE;
    setMotorTarget(percentage);

    Serial.print(getCurrentPositionCommand());
  }
}
