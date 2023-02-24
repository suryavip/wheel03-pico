void comSetup() {
  debugLedSegment();

  Serial.setTimeout(4);

  // Wait until receive first FFB command from PC.
  while (true) {
    if (Serial.available() <= 0) continue;
    bool detectFFB = Serial.findUntil("F:", ";");
    if (detectFFB) break;
  }
}

bool freshStart = true;

void comLoop() {
  if (Serial.available() <= 0) return;

  bool detectFFB = Serial.findUntil("F:", ";");
  if (detectFFB == false) return;

  if (freshStart) {
    freshStart = false;
    overRotation = 0;
  }

  // Calculating requested FFB percentage.
  String read = Serial.readString();
  read.replace(";", "");
  double readDouble = read.toDouble();
  double percentage = readDouble / 10000;
  requestMotorTarget(percentage);

  sendPositionToPC();
}

void sendPositionToPC() {
  String command = "E:";
  String delimiter = ";";
  String to_send = command + multiRotationValue() + delimiter;
  Serial.print(to_send);
}
