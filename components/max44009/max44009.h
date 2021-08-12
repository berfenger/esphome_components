#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace max44009 {

/// This class implements support for the MAX44009 Illuminance i2c sensor.
class MAX44009Sensor : public sensor::Sensor,
                       public PollingComponent,
                       public i2c::I2CDevice {
 public:

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;
  void setContinuousMode();
  void setLowPowerMode();

 protected:
  /// Read the illuminance value
  float read_illuminance_();
  uint8_t read(uint8_t reg);
  void write(uint8_t reg, uint8_t value);

  int _error;
  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    WRONG_CHIP_ID,
  } error_code_{NONE};
};

}  // namespace max44009
}  // namespace esphome