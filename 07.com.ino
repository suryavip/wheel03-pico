const uint8_t COM_FRAME_MAX = 16;
const unsigned long COM_MIN_REPLY_INTERVAL_US = 2000;

// Receive accumulator. Kept across calls so a frame split over several reads
// isn't lost.
char comBuf[COM_FRAME_MAX];
uint8_t comLen = 0;
bool comOverflow = false;

bool comReplyPending = false;
unsigned long lastReplyMicros = 0;

void replyWithPositions() {
  char out[48];
  int n = snprintf(out, sizeof(out), "%d,%d,%d,%d,%d;",
                   int(currentRawPosition), overRotation, accValue, brkValue, cltValue);
  if (n <= 0) return;
  if (n >= (int)sizeof(out)) n = sizeof(out) - 1;

  // single write, so it goes out as one USB packet
  Serial.write((const uint8_t*)out, n);
}

// Drains everything currently waiting on the serial port.
// Returns true if at least one complete frame arrived, with the newest
// frame's value in `out`.
bool readLatestVoltage(float& out) {
  bool got = false;

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == ';') {
      if (!comOverflow && comLen > 0) {
        comBuf[comLen] = '\0';
        out = atof(comBuf);
        got = true;
      }
      comLen = 0;
      comOverflow = false;
    } else if (comLen < COM_FRAME_MAX - 1) {
      comBuf[comLen++] = c;
    } else {
      comOverflow = true;
    }
  }

  return got;
}

void comSetup() {
  Serial.begin();
}

void comLoop() {
  float v = 0;
  if (readLatestVoltage(v)) {
    setRequestVoltage(v);
    comReplyPending = true;
  }

  if (comReplyPending && micros() - lastReplyMicros >= COM_MIN_REPLY_INTERVAL_US) {
    replyWithPositions();
    lastReplyMicros = micros();
    comReplyPending = false;
  }
}
