#include "max44009.h"
#include "esphome/core/log.h"

namespace esphome {
namespace max44009 {

static const char *const TAG = "max44009.sensor";

// REGISTERS
#define MAX44009_REGISTER_CONFIGURATION      0x02
#define MAX44009_LUX_READING_HIGH            0x03
#define MAX44009_LUX_READING_LOW             0x04
// CONFIGURATION MASKS
#define MAX44009_CFG_CONTINUOUS     0x80
// ERROR CODES
#define MAX44009_OK                     0
#define MAX44009_ERROR_WIRE_REQUEST    -10
#define MAX44009_ERROR_OVERFLOW        -20
#define MAX44009_ERROR_HIGH_BYTE       -30
#define MAX44009_ERROR_LOW_BYTE        -31

inline int convertToLux(uint8_t datahigh, uint8_t datalow) {
  uint8_t  exponent = datahigh >> 4;
  uint32_t mantissa = ((datahigh & 0x0F) << 4) + (datalow & 0x0F);
  float lux = ((0x0001 << exponent) * 0.045) * mantissa;
  return roundf(lux);
}

void MAX44009Sensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MAX44009...");
  this->raw_begin_transmission();
  if (!this->raw_end_transmission()) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }
  /* 
   * Set mode depending on update interval
   * - On low power mode, the IC measures lux intensity only once every 800ms regardless of integration time
   * - On continuous mode, the IC continuously measures lux intensity
   */
  if (this->get_update_interval() < 800) {
    this->setContinuousMode();
  } else {
    this->setLowPowerMode();
  }
}
void MAX44009Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX44009:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MAX44009 failed!");
  }
}
float MAX44009Sensor::get_setup_priority() const { return setup_priority::HARDWARE; }

void MAX44009Sensor::update() {
  // update sensor illuminance value
  float lux = this->read_illuminance_();
  if (_error != MAX44009_OK) {
    this->status_set_error();
    this->publish_state(NAN);
  } else {
    this->status_clear_error();
    this->publish_state(lux);
  }
}

float MAX44009Sensor::read_illuminance_() {
  uint8_t datahigh = read(MAX44009_LUX_READING_HIGH);
  if (_error != MAX44009_OK)
  {
    _error = MAX44009_ERROR_HIGH_BYTE;
    return _error;
  }
  uint8_t datalow = read(MAX44009_LUX_READING_LOW);
  if (_error != MAX44009_OK)
  {
    _error = MAX44009_ERROR_LOW_BYTE;
    return _error;
  }
  uint8_t exponent = datahigh >> 4;
  if (exponent == 0x0F)
  {
    _error = MAX44009_ERROR_OVERFLOW;
    return _error;
  }

  float lux = convertToLux(datahigh, datalow);
  return lux;
}

void MAX44009Sensor::setContinuousMode() {
  uint8_t config = read(MAX44009_REGISTER_CONFIGURATION);
  if (_error == MAX44009_OK) {
    config |= MAX44009_CFG_CONTINUOUS;
    write(MAX44009_REGISTER_CONFIGURATION, config);
    this->status_clear_error();
  } else {
    this->status_set_error();
  }
}

void MAX44009Sensor::setLowPowerMode() {
  uint8_t config = read(MAX44009_REGISTER_CONFIGURATION);
  if (_error == MAX44009_OK) {
    config &= ~MAX44009_CFG_CONTINUOUS;
    write(MAX44009_REGISTER_CONFIGURATION, config);
    this->status_clear_error();
  } else {
    this->status_set_error();
  }
}

uint8_t MAX44009Sensor::read(uint8_t reg)
{
  uint8_t data;
  if (!this->read_byte(reg, &data)) {
    _error = MAX44009_ERROR_WIRE_REQUEST;
  } else {
    _error = MAX44009_OK;
  }
  return data;
}


void MAX44009Sensor::write(uint8_t reg, uint8_t value)
{
  if (!this->write_byte(reg, value)) {
    _error = MAX44009_ERROR_WIRE_REQUEST;
  } else {
    _error = MAX44009_OK;
  }
}

}  // namespace max44009
}  // namespace esphome