//const String CONFIG_PATH_VELO_REACTIVE_MAGNITUDE = "velo_reactive_magnitude";

LittleFSConfig cfg;

//int savedVeloReactiveMagnitude = 0;

//void setSavedVeloReactiveMagnitude(int newVeloReactiveMagnitude) {
//  savedVeloReactiveMagnitude = newVeloReactiveMagnitude;
//  String content = String(savedVeloReactiveMagnitude, DEC);
//  writeToFile(CONFIG_PATH_VELO_REACTIVE_MAGNITUDE, content);
//}

void writeToFile(String filePath, String content) {
  int toWriteLen = content.length() + 1;
  char chars[toWriteLen];
  content.toCharArray(chars, toWriteLen);

  File file = LittleFS.open(filePath, "w");
  if (isConfigDebug) {
    Serial.print("Writing ");
    Serial.print(content);
    Serial.print(" to ");
    Serial.println(filePath);
  }
  file.write(chars);
  file.close();
}

String readOrPrep(String filePath, String defaultContent) {
  String output = defaultContent;

  if (LittleFS.exists(filePath)) {
    File file = LittleFS.open(filePath, "r");
    output = file.readString();
    file.close();
  }
  else {
    writeToFile(filePath, defaultContent);
  }

  if (isConfigDebug) {
    Serial.print("Read/prep ");
    Serial.print(output);
    Serial.print(" from ");
    Serial.println(filePath);
  }

  return output;
}

void configSetup() {
  LittleFS.setConfig(cfg);
  LittleFS.begin();

  //  String savedVeloReactiveMagnitudeRead = readOrPrep(CONFIG_PATH_VELO_REACTIVE_MAGNITUDE, "0");
  //  savedVeloReactiveMagnitude = savedVeloReactiveMagnitudeRead.toInt();
}
