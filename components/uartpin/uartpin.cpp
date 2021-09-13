#include "uartpin.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace uartpin {

static const char *const TAG = "uartpin";

void UARTPINComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up UARTPINComponent...");
}

void UARTPINComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "UARTPIN:");
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with UARTPIN failed!");
  }
}

void UARTPINComponent::loop() {
  if (!this->init_ && millis() > init_delay_) {
    this->init_ = true;
    if (this->init_data_.size() > 0) {
      this->write_to_uart(this->init_data_);
    }
  }
}

UARTPINChannel *UARTPINComponent::create_channel() {
  return new UARTPINChannel(this);
}

void UARTPINComponent::write_to_uart(const std::vector<uint8_t> &data) {
  if (this->init_) {
    this->write_array(&data[0], data.size());
  }
}

void UARTPINComponent::set_init_data(const std::vector<uint8_t> &data) {
  this->init_data_ = data;
}

void UARTPINComponent::set_init_delay(unsigned int delay) {
  this->init_delay_ = delay;
}

void UARTPINChannel::write_state(bool state) {
  if (state) {
    // write high
    this->parent_->write_to_uart(this->data_high_);
  } else {
    // write low
    this->parent_->write_to_uart(this->data_low_);
  }
}

void UARTPINChannel::set_data_high(const std::vector<uint8_t> &data) {
  this->data_high_ = data;
}

void UARTPINChannel::set_data_low(const std::vector<uint8_t> &data) {
  this->data_low_ = data;
}

}  // namespace uartpin
}  // namespace esphome
