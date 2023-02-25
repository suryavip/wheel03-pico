const String CONFIG_PATH_ZERO_OFFSET = "zero_offset";

LittleFSConfig cfg;

int zeroOffset = 0;

void setZeroOffset(int newZeroOffset) {
  zeroOffset = newZeroOffset;

  String toWrite = String(zeroOffset, DEC);
  int toWriteLen = toWrite.length() + 1;
  char chars[toWriteLen];
  toWrite.toCharArray(chars, toWriteLen);

  File file = LittleFS.open(CONFIG_PATH_ZERO_OFFSET, "w");
  file.write(chars);
  file.close();
}

void configSetup() {
  LittleFS.setConfig(cfg);
  LittleFS.begin();

  if (LittleFS.exists(CONFIG_PATH_ZERO_OFFSET)) {
    File file = LittleFS.open(CONFIG_PATH_ZERO_OFFSET, "r");
    String fileRead = file.readString();
    zeroOffset = fileRead.toInt();
    file.close();
  }
  else {
    File file = LittleFS.open(CONFIG_PATH_ZERO_OFFSET, "w");
    file.write("0");
    file.close();
  }
}
