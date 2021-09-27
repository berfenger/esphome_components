#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace cse7761 {

/// This class implements support for the CSE7761 UART power sensor.
class CSE7761Sensor : public PollingComponent,
                      public uart::UARTDevice {
 public:
  CSE7761Sensor() {}

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;
  void set_voltage_sensor(sensor::Sensor *voltage_sensor) {
    voltage_sensor_ = voltage_sensor;
  }
  void set_active_power_1_sensor(sensor::Sensor *power_sensor_1) {
    power_sensor_1_ = power_sensor_1;
  }
  void set_current_1_sensor(sensor::Sensor *current_sensor_1) {
    current_sensor_1_ = current_sensor_1;
  }
  void set_active_power_2_sensor(sensor::Sensor *power_sensor_2) {
    power_sensor_2_ = power_sensor_2;
  }
  void set_current_2_sensor(sensor::Sensor *current_sensor_2) {
    current_sensor_2_ = current_sensor_2;
  }

 protected:
  // Sensors
  sensor::Sensor *voltage_sensor_;
  sensor::Sensor *power_sensor_1_;
  sensor::Sensor *current_sensor_1_;
  sensor::Sensor *power_sensor_2_;
  sensor::Sensor *current_sensor_2_;

 private:
  void Cse7761Write(uint32_t reg, uint32_t data);
  bool Cse7761ReadOnce(uint32_t reg, uint32_t size, uint32_t *value);
  uint32_t Cse7761Read(uint32_t reg, uint32_t size);
  uint32_t Cse7761ReadFallback(uint32_t reg, uint32_t prev, uint32_t size);
  bool Cse7761ChipInit(void);
  void Cse7761GetData(void);
};

}  // namespace cse7761
}  // namespace esphome