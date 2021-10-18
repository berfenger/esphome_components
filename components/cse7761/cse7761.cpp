#include "cse7761.h"

#include "esphome/core/log.h"

namespace esphome {
namespace cse7761 {

static const char* const TAG = "cse7761";

/*********************************************************************************************\
 * CSE7761 - Energy  (Sonoff Dual R3 Pow v1.x)
 *
 * Based on Tasmota source code
 * See https://github.com/arendst/Tasmota/discussions/10793
 * https://github.com/arendst/Tasmota/blob/development/tasmota/xnrg_19_cse7761.ino
\*********************************************************************************************/

#define CSE7761_UREF 42563  // RmsUc
#define CSE7761_IREF 52241  // RmsIAC
#define CSE7761_PREF 44513  // PowerPAC
#define CSE7761_FREF \
  3579545  // System clock (3.579545MHz) as used in frequency calculation

#define CSE7761_REG_SYSCON 0x00   // (2) System Control Register (0x0A04)
#define CSE7761_REG_EMUCON 0x01   // (2) Metering control register (0x0000)
#define CSE7761_REG_EMUCON2 0x13  // (2) Metering control register 2 (0x0001)
#define CSE7761_REG_PULSE1SEL \
  0x1D  // (2) Pin function output select register (0x3210)

#define CSE7761_REG_UFREQ 0x23  // (2) Voltage Frequency (0x0000)
#define CSE7761_REG_RMSIA \
  0x24  // (3) The effective value of channel A current (0x000000)
#define CSE7761_REG_RMSIB \
  0x25  // (3) The effective value of channel B current (0x000000)
#define CSE7761_REG_RMSU 0x26  // (3) Voltage RMS (0x000000)
#define CSE7761_REG_POWERFACTOR \
  0x27  // (3) Power factor register, select by command: channel A Power factor
        // or channel B power factor (0x7FFFFF)
#define CSE7761_REG_POWERPA \
  0x2C  // (4) Channel A active power, update rate 27.2Hz (0x00000000)
#define CSE7761_REG_POWERPB \
  0x2D  // (4) Channel B active power, update rate 27.2Hz (0x00000000)
#define CSE7761_REG_SYSSTATUS 0x43  // (1) System status register

#define CSE7761_REG_COEFFOFFSET \
  0x6E  // (2) Coefficient checksum offset (0xFFFF)
#define CSE7761_REG_COEFFCHKSUM 0x6F  // (2) Coefficient checksum
#define CSE7761_REG_RMSIAC \
  0x70  // (2) Channel A effective current conversion coefficient
#define CSE7761_REG_RMSIBC \
  0x71  // (2) Channel B effective current conversion coefficient
#define CSE7761_REG_RMSUC 0x72  // (2) Effective voltage conversion coefficient
#define CSE7761_REG_POWERPAC \
  0x73  // (2) Channel A active power conversion coefficient
#define CSE7761_REG_POWERPBC \
  0x74  // (2) Channel B active power conversion coefficient
#define CSE7761_REG_POWERSC 0x75  // (2) Apparent power conversion coefficient
#define CSE7761_REG_ENERGYAC \
  0x76  // (2) Channel A energy conversion coefficient
#define CSE7761_REG_ENERGYBC \
  0x77  // (2) Channel B energy conversion coefficient

#define CSE7761_SPECIAL_COMMAND 0xEA  // Start special command
#define CSE7761_CMD_RESET \
  0x96  // Reset command, after receiving the command, the chip resets
#define CSE7761_CMD_CHAN_A_SELECT \
  0x5A  // Current channel A setting command, which specifies the current used
        // to calculate apparent power,
        //   Power factor, phase angle, instantaneous active power,
        //   instantaneous apparent power and The channel indicated by the
        //   signal of power overload is channel A
#define CSE7761_CMD_CHAN_B_SELECT \
  0xA5  // Current channel B setting command, which specifies the current used
        // to calculate apparent power,
        //   Power factor, phase angle, instantaneous active power,
        //   instantaneous apparent power and The channel indicated by the
        //   signal of power overload is channel B
#define CSE7761_CMD_CLOSE_WRITE 0xDC   // Close write operation
#define CSE7761_CMD_ENABLE_WRITE 0xE5  // Enable write operation

enum CSE7761 {
  RmsIAC,
  RmsIBC,
  RmsUC,
  PowerPAC,
  PowerPBC,
  PowerSC,
  EnergyAC,
  EnergyBC
};

struct {
  uint32_t frequency = 0;
  uint32_t voltage_rms = 0;
  uint32_t current_rms[2] = {0};
  uint32_t energy[2] = {0};
  uint32_t active_power[2] = {0};
  uint16_t coefficient[8] = {0};
  uint8_t energy_update = 0;
  uint8_t init = 4;
  uint8_t ready = 0;
} CSE7761Data;

long last_init = 0;

inline int32_t time_difference(uint32_t prev, uint32_t next) {
  return ((int32_t)(next - prev));
}

int32_t time_passed_since(uint32_t timestamp) {
  // Compute the number of milliSeconds passed since timestamp given.
  // Note: value can be negative if the timestamp has not yet been reached.
  return time_difference(timestamp, millis());
}

bool time_reached(uint32_t timer) {
  // Check if a certain timeout has been reached.
  const long passed = time_passed_since(timer);
  return (passed >= 0);
}

uint32_t cse7761ref(uint32_t unit) {
  switch (unit) {
    case RmsUC:
      return 0x400000 * 100 / CSE7761Data.coefficient[RmsUC];
    case RmsIAC:
      return (0x800000 * 100 / CSE7761Data.coefficient[RmsIAC]) *
             10;  // Stay within 32 bits
    case PowerPAC:
      return 0x80000000 / CSE7761Data.coefficient[PowerPAC];
  }
  return 0;
}

void CSE7761Component::setup() { ESP_LOGCONFIG(TAG, "Setting up CSE7761..."); }

void CSE7761Component::loop() {
  if (CSE7761Data.init && millis() > last_init + 1000) {
    last_init = millis();
    if (3 == CSE7761Data.init) {
      this->write_(CSE7761_SPECIAL_COMMAND, CSE7761_CMD_RESET);
    } else if (2 == CSE7761Data.init) {
      uint16_t syscon = this->read_(0x00, 2);  // Default 0x0A04
      if ((0x0A04 == syscon) && this->chip_init_()) {
        CSE7761Data.ready = 1;
      }
    } else if (1 == CSE7761Data.init) {
      if (1 == CSE7761Data.ready) {
        this->write_(CSE7761_SPECIAL_COMMAND, CSE7761_CMD_CLOSE_WRITE);
        ESP_LOGD(TAG, "C61: CSE7761 found");
        CSE7761Data.ready = 2;
      }
    }
    CSE7761Data.init--;
  }
}

void CSE7761Component::dump_config() {
  ESP_LOGCONFIG(TAG, "CSE7761:");
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with CSE7761 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  this->check_uart_settings(38400, 1, uart::UART_CONFIG_PARITY_EVEN, 8);
}

float CSE7761Component::get_setup_priority() const {
  return setup_priority::DATA;
}

void CSE7761Component::update() {
  if (2 == CSE7761Data.ready) {
    this->get_data_();
  }
}

void CSE7761Component::write_(uint32_t reg, uint32_t data) {
  uint8_t buffer[5];

  buffer[0] = 0xA5;
  buffer[1] = reg;
  uint32_t len = 2;
  if (data) {
    if (data < 0xFF) {
      buffer[2] = data & 0xFF;
      len = 3;
    } else {
      buffer[2] = (data >> 8) & 0xFF;
      buffer[3] = data & 0xFF;
      len = 4;
    }
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
      crc += buffer[i];
    }
    buffer[len] = ~crc;
    len++;
  }

  this->write_array(buffer, len);
}

bool CSE7761Component::read_once_(uint32_t reg, uint32_t size,
                                  uint32_t* value) {
  while (this->available()) {
    this->read();
  }

  this->write_(reg, 0);

  uint8_t buffer[8] = {0};
  uint32_t rcvd = 0;
  uint32_t timeout = millis() + 3;

  while (!time_reached(timeout) && (rcvd <= size)) {
    int value = this->read();
    if ((value > -1) && (rcvd < sizeof(buffer) - 1)) {
      buffer[rcvd++] = value;
    }
  }

  if (!rcvd) {
    ESP_LOGD(TAG, PSTR("C61: Rx none"));
    return false;
  }
  if (rcvd > 5) {
    ESP_LOGD(TAG, PSTR("C61: Rx overflow"));
    return false;
  }

  rcvd--;
  uint32_t result = 0;
  uint8_t crc = 0xA5 + reg;
  for (uint32_t i = 0; i < rcvd; i++) {
    result = (result << 8) | buffer[i];
    crc += buffer[i];
  }
  crc = ~crc;
  if (crc != buffer[rcvd]) {
    return false;
  }

  *value = result;
  return true;
}

uint32_t CSE7761Component::read_(uint32_t reg, uint32_t size) {
  bool result = false;  // Start loop
  uint32_t retry = 3;   // Retry up to three times
  uint32_t value = 0;   // Default no value
  while (!result && retry) {
    retry--;
    result = this->read_once_(reg, size, &value);
  }
  return value;
}

uint32_t CSE7761Component::read_fallback_(uint32_t reg, uint32_t prev,
                                          uint32_t size) {
  uint32_t value = this->read_(reg, size);
  if (!value) {  // Error so use previous value read
    value = prev;
  }
  return value;
}

bool CSE7761Component::chip_init_(void) {
  uint16_t calc_chksum = 0xFFFF;
  for (uint32_t i = 0; i < 8; i++) {
    CSE7761Data.coefficient[i] = this->read_(CSE7761_REG_RMSIAC + i, 2);
    calc_chksum += CSE7761Data.coefficient[i];
  }
  calc_chksum = ~calc_chksum;
  //  uint16_t dummy = this->read_(CSE7761_REG_COEFFOFFSET, 2);
  uint16_t coeff_chksum = this->read_(CSE7761_REG_COEFFCHKSUM, 2);
  if ((calc_chksum != coeff_chksum) || (!calc_chksum)) {
    ESP_LOGD(TAG, PSTR("C61: Default calibration"));
    CSE7761Data.coefficient[RmsIAC] = CSE7761_IREF;
    //    CSE7761Data.coefficient[RmsIBC] = 0xCC05;
    CSE7761Data.coefficient[RmsUC] = CSE7761_UREF;
    CSE7761Data.coefficient[PowerPAC] = CSE7761_PREF;
    //    CSE7761Data.coefficient[PowerPBC] = 0xADD7;
  }

  this->write_(CSE7761_SPECIAL_COMMAND, CSE7761_CMD_ENABLE_WRITE);

  uint8_t sys_status = this->read_(CSE7761_REG_SYSSTATUS, 1);
  if (sys_status & 0x10) {  // Write enable to protected registers (WREN)
    this->write_(CSE7761_REG_SYSCON | 0x80, 0xFF04);
    this->write_(CSE7761_REG_EMUCON | 0x80,
                 0x1183);  // Tasmota enable zero cross detection on both
                           // positive and negative signal
    this->write_(CSE7761_REG_EMUCON2 | 0x80, 0x0FC1);  // Sonoff Dual R3 Pow
    this->write_(CSE7761_REG_PULSE1SEL | 0x80, 0x3290);
  } else {
    ESP_LOGD(TAG, PSTR("C61: Write failed"));
    return false;
  }
  return true;
}

void CSE7761Component::get_data_(void) {
  // The effective value of current and voltage Rms is a 24-bit signed number,
  // the highest bit is 0 for valid data,
  //   and when the highest bit is 1, the reading will be processed as zero
  // The active power parameter PowerA/B is in two’s complement format, 32-bit
  // data, the highest bit is Sign bit.
  uint32_t value =
      this->read_fallback_(CSE7761_REG_RMSU, CSE7761Data.voltage_rms, 3);
  CSE7761Data.voltage_rms = (value >= 0x800000) ? 0 : value;

  value =
      this->read_fallback_(CSE7761_REG_RMSIA, CSE7761Data.current_rms[0], 3);
  CSE7761Data.current_rms[0] = ((value >= 0x800000) || (value < 1600))
                                   ? 0
                                   : value;  // No load threshold of 10mA
  value =
      this->read_fallback_(CSE7761_REG_POWERPA, CSE7761Data.active_power[0], 4);
  CSE7761Data.active_power[0] = (0 == CSE7761Data.current_rms[0]) ? 0
                                : (value & 0x80000000)            ? (~value) + 1
                                                                  : value;

  value =
      this->read_fallback_(CSE7761_REG_RMSIB, CSE7761Data.current_rms[1], 3);
  CSE7761Data.current_rms[1] = ((value >= 0x800000) || (value < 1600))
                                   ? 0
                                   : value;  // No load threshold of 10mA
  value =
      this->read_fallback_(CSE7761_REG_POWERPB, CSE7761Data.active_power[1], 4);
  CSE7761Data.active_power[1] = (0 == CSE7761Data.current_rms[1]) ? 0
                                : (value & 0x80000000)            ? (~value) + 1
                                                                  : value;

  ESP_LOGD(TAG, PSTR("C61: F%d, U%d, I%d/%d, P%d/%d"), CSE7761Data.frequency,
           CSE7761Data.voltage_rms, CSE7761Data.current_rms[0],
           CSE7761Data.current_rms[1], CSE7761Data.active_power[0],
           CSE7761Data.active_power[1]);

  // convert values and publish to sensors

  float voltage = (float)CSE7761Data.voltage_rms / cse7761ref(RmsUC);
  if (this->voltage_sensor_ != nullptr) {
    this->voltage_sensor_->publish_state(voltage);
  }

  for (uint32_t channel = 0; channel < 2; channel++) {
    // Active power = PowerPA * PowerPAC * 1000 / 0x80000000
    float activePower =
        (float)CSE7761Data.active_power[channel] / cse7761ref(PowerPAC);  // W
    float amps =
        (float)CSE7761Data.current_rms[channel] / cse7761ref(RmsIAC);  // A
    ESP_LOGD(TAG, PSTR("C61: Channel %d power %f W, current %f A"), channel,
             activePower, amps);
    if (channel == 0) {
      if (this->power_sensor_1_ != nullptr) {
        this->power_sensor_1_->publish_state(activePower);
      }
      if (this->current_sensor_1_ != nullptr) {
        this->current_sensor_1_->publish_state(amps);
      }
    } else if (channel == 1) {
      if (this->power_sensor_2_ != nullptr) {
        this->power_sensor_2_->publish_state(activePower);
      }
      if (this->current_sensor_2_ != nullptr) {
        this->current_sensor_2_->publish_state(amps);
      }
    }
  }
}

}  // namespace cse7761
}  // namespace esphome