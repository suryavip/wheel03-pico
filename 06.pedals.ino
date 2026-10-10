const unsigned int PEDALS_I2C_CLOCK = 400 * 1000;

int16_t accValue = 0;
int16_t brkValue = 0;
int16_t cltValue = 0;

ADS1115 ads1115(0x48, &Wire1);
// HX711 loadCell;

void pedalsSetup() {
  Wire1.setSDA(PIN_ADS1115_SDA);
  Wire1.setSCL(PIN_ADS1115_SCL);
  Wire1.begin();

  ads1115.begin();
  Wire1.setClock(PEDALS_I2C_CLOCK);
  ads1115.setDataRate(7);
  ads1115.setGain(1);

  // loadCell.begin(PIN_LOADCELL_DT, PIN_LOADCELL_CL);
}

// Non-blocking: readADC() waits out the whole conversion (x3 channels), which
// stalls comLoop() and paces the entire PC <-> Pico exchange. Instead, keep one
// single-shot conversion in flight and collect it on a later loop() pass.
void pedalsLoop() {
  static const uint8_t channels[3] = { PIN_ACCELERATOR, PIN_BRAKE, PIN_CLUTCH };
  static int16_t* const targets[3] = { &accValue, &brkValue, &cltValue };
  static uint8_t idx = 0;
  static bool pending = false;

  if (pending) {
    if (!ads1115.isReady()) return;  // conversion still running
    *targets[idx] = ads1115.getValue();
    idx = (idx + 1) % 3;
  }

  ads1115.requestADC(channels[idx]);
  pending = true;

  // if (loadCell.is_ready()) {
  //   brkValue = (loadCell.read() - 80000) / 64;
  // }
}
