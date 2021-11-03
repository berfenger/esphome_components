#include "max44009.h"

#include "esphome/core/log.h"

namespace esphome {
namespace max44009 {

static const char *const TAG = "max44009.sensor";

// REGISTERS
static const uint8_t MAX44009_REGISTER_CONFIGURATION = 0x02;
static const uint8_t MAX44009_LUX_READING_HIGH = 0x03;
static const uint8_t MAX44009_LUX_READING_LOW = 0x04;
// CONFIGURATION MASKS
static const uint8_t MAX44009_CFG_CONTINUOUS = 0x80;
// ERROR CODES
static const uint8_t MAX44009_OK = 0;
static const uint8_t MAX44009_ERROR_WIRE_REQUEST = -10;
static const uint8_t MAX44009_ERROR_OVERFLOW = -20;
static const uint8_t MAX44009_ERROR_HIGH_BYTE = -30;
static const uint8_t MAX44009_ERROR_LOW_BYTE = -31;

inline int convert_to_lux(uint8_t datahigh, uint8_t datalow) {
  uint8_t exponent = datahigh >> 4;
  uint32_t mantissa = ((datahigh & 0x0F) << 4) + (datalow & 0x0F);
  float lux = ((0x0001 << exponent) * 0.045) * mantissa;
  return roundf(lux);
}

void MAX44009Sensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MAX44009...");
  bool state_ok = false;
  if (this->mode_ == MAX44009Mode::MAX44009_MODE_LOW_POWER) {
    state_ok = this->set_low_power_mode();
  } else if (this->mode_ == MAX44009Mode::MAX44009_MODE_CONTINUOUS) {
    state_ok = this->set_continuous_mode();
  } else {
    /*
     * Mode AUTO: Set mode depending on update interval
     * - On low power mode, the IC measures lux intensity only once every 800ms
     * regardless of integration time
     * - On continuous mode, the IC continuously measures lux intensity
     */
    if (this->get_update_interval() < 800) {
      state_ok = this->set_continuous_mode();
    } else {
      state_ok = this->set_low_power_mode();
    }
  }
  if (!state_ok)
    this->mark_failed();
}

void MAX44009Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX44009:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MAX44009 failed!");
  }
}

float MAX44009Sensor::get_setup_priority() const { return setup_priority::DATA; }

void MAX44009Sensor::update() {
  // update sensor illuminance value
  float lux = this->read_illuminance_();
  if (this->error_ != MAX44009_OK) {
    this->status_set_error();
  } else {
    this->status_clear_error();
    this->publish_state(lux);
  }
}

float MAX44009Sensor::read_illuminance_() {
  uint8_t datahigh = read(MAX44009_LUX_READING_HIGH);
  if (error_ != MAX44009_OK) {
    this->error_ = MAX44009_ERROR_HIGH_BYTE;
    return this->error_;
  }
  uint8_t datalow = read(MAX44009_LUX_READING_LOW);
  if (error_ != MAX44009_OK) {
    this->error_ = MAX44009_ERROR_LOW_BYTE;
    return this->error_;
  }
  uint8_t exponent = datahigh >> 4;
  if (exponent == 0x0F) {
    this->error_ = MAX44009_ERROR_OVERFLOW;
    return this->error_;
  }

  float lux = convert_to_lux(datahigh, datalow);
  return lux;
}

bool MAX44009Sensor::set_continuous_mode() {
  uint8_t config = read(MAX44009_REGISTER_CONFIGURATION);
  if (this->error_ == MAX44009_OK) {
    config |= MAX44009_CFG_CONTINUOUS;
    this->write(MAX44009_REGISTER_CONFIGURATION, config);
    this->status_clear_error();
    return true;
  } else {
    this->status_set_error();
    return false;
  }
}

bool MAX44009Sensor::set_low_power_mode() {
  uint8_t config = read(MAX44009_REGISTER_CONFIGURATION);
  if (this->error_ == MAX44009_OK) {
    config &= ~MAX44009_CFG_CONTINUOUS;
    this->write(MAX44009_REGISTER_CONFIGURATION, config);
    this->status_clear_error();
    return true;
  } else {
    this->status_set_error();
    return false;
  }
}

uint8_t MAX44009Sensor::read(uint8_t reg) {
  uint8_t data = 0;
  if (!this->read_byte(reg, &data)) {
    this->error_ = MAX44009_ERROR_WIRE_REQUEST;
  } else {
    this->error_ = MAX44009_OK;
  }
  return data;
}

void MAX44009Sensor::write(uint8_t reg, uint8_t value) {
  if (!this->write_byte(reg, value)) {
    this->error_ = MAX44009_ERROR_WIRE_REQUEST;
  } else {
    this->error_ = MAX44009_OK;
  }
}

void MAX44009Sensor::set_mode(MAX44009Mode mode) { this->mode_ = mode; }

}  // namespace max44009
}  // namespace esphome
