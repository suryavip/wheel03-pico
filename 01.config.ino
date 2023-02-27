const String CONFIG_PATH_ZERO_OFFSET = "zero_offset";
const String CONFIG_PATH_MAX_POWER = "max_power";

LittleFSConfig cfg;

int savedZeroOffset = 0;
float savedMaxPower = 1;

void setSavedZeroOffset(int newZeroOffset) {
  savedZeroOffset = newZeroOffset;
  String content = String(savedZeroOffset, DEC);
  writeToFile(CONFIG_PATH_ZERO_OFFSET, content);
}

void setSavedMaxPower(float newMaxPower) {
  savedMaxPower = newMaxPower;
  String content = String(savedMaxPower, 2);
  writeToFile(CONFIG_PATH_MAX_POWER, content);
}

void writeToFile(String filePath, String content) {
  int toWriteLen = content.length() + 1;
  char chars[toWriteLen];
  content.toCharArray(chars, toWriteLen);

  File file = LittleFS.open(filePath, "w");
  if (isDebug) {
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

  if (isDebug) {
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

  String savedZeroOffsetRead = readOrPrep(CONFIG_PATH_ZERO_OFFSET, "0");
  savedZeroOffset = savedZeroOffsetRead.toInt();

  String savedMaxPowerRead = readOrPrep(CONFIG_PATH_MAX_POWER, "1.00");
  savedMaxPower = savedMaxPowerRead.toFloat();
}
