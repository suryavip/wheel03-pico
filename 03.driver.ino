const unsigned int DRIVER_PWM_FREQ = 3000;
const unsigned int DRIVER_VOLTAGE_SUPPLY = 12;
const unsigned int DRIVER_VOLTAGE_LIMIT = 5;

BLDCDriver3PWM driver;

void driverSetup() {
  driver = BLDCDriver3PWM(
             PIN_PA,
             PIN_PB,
             PIN_PC,
             PIN_PA_EN,
             PIN_PB_EN,
             PIN_PC_EN);

  driver.pwm_frequency = DRIVER_PWM_FREQ;
  driver.voltage_power_supply = DRIVER_VOLTAGE_SUPPLY;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driver.init();
}
