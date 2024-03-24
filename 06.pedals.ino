int16_t accValue = 0;
int16_t brkValue = 0;
int16_t cltValue = 0;

ADS1115 ads1115(0x48, &Wire1);
HX711 loadCell;

void pedalsSetup() {
  Wire1.setSDA(PIN_ADS1115_SDA);
  Wire1.setSCL(PIN_ADS1115_SCL);
  Wire1.begin();

  ads1115.begin();
  ads1115.setDataRate(7);
  ads1115.setGain(1);

  loadCell.begin(PIN_LOADCELL_DT, PIN_LOADCELL_CL);
}

void pedalsLoop() {
  accValue = ads1115.readADC(PIN_ACCELERATOR);
  //  brkValue = ads1115.readADC(PIN_BRAKE);
  cltValue = ads1115.readADC(PIN_CLUTCH);

  if (loadCell.is_ready()) {
    brkValue = (loadCell.read() - 80000) / 64;
  }

  if (isPedalsDebug) {
    Serial.print("A:");
    Serial.print(accValue);
    Serial.print(",B:");
    Serial.print(brkValue);
    Serial.print(",C:");
    Serial.println(cltValue);
  }
}
