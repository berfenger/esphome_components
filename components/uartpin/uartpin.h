#pragma once

#include "esphome/core/component.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace uartpin {

class UARTPINChannel;

class UARTPINComponent : public Component, public uart::UARTDevice {
 public:
  UARTPINComponent() {}

  UARTPINChannel *create_channel();

  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void set_init_data(const std::vector<uint8_t> &data);
  void set_init_delay(unsigned int delay);

 protected:
  friend UARTPINChannel;
  void write_to_uart(const std::vector<uint8_t> &data);

 private:
  std::vector<uint8_t> init_data_;
  unsigned int init_delay_ = 0;
  bool init_ = false;
};

class UARTPINChannel : public output::BinaryOutput {
 public:
  UARTPINChannel(UARTPINComponent *parent) : 
    parent_(parent) {
      
    }
  void set_data_high(const std::vector<uint8_t> &data);
  void set_data_low(const std::vector<uint8_t> &data);
  void write_state(bool state) override;

 private:
  std::vector<uint8_t> data_high_;
  std::vector<uint8_t> data_low_;
  UARTPINComponent *parent_;
};

}  // namespace uartpin
}  // namespace esphome
