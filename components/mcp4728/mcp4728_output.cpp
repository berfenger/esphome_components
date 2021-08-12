#include "mcp4728_output.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace mcp4728 {

static const char *const TAG = "mcp4728";

void MCP4728Output::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MCP4728OutputComponent...");
  this->raw_begin_transmission();
  if (!this->raw_end_transmission()) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  this->set_channel_value(0, 0);
  this->set_channel_value(1, 0);
  this->set_channel_value(2, 0);
  this->set_channel_value(3, 0);
}

void MCP4728Output::dump_config() {
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MCP4728 failed!");
  }
}

void MCP4728Output::loop() {
  if (this->update) {
    this->update = false;
    if (this->eeprom)
      this->seqWrite();
    else
      this->multiWrite();
  }
}

void MCP4728Output::set_channel_value(uint8_t channel, uint16_t value) {
  ESP_LOGD(TAG, "Setting MCP4728 channel %d to %d!", channel, value);
  reg_[channel].data = value;
  this->update = true;
}

uint8_t MCP4728Output::multiWrite() {
  this->raw_begin_transmission();
  for (uint8_t i = 0; i < 4; ++i)
  {
    uint8_t wd[3];
    wd[0] = ((uint8_t)CMD::MULTI_WRITE | (i << 1)) & 0xFE;
    wd[1] = ((uint8_t)reg_[i].vref << 7) | ((uint8_t)reg_[i].pd << 5) | ((uint8_t)reg_[i].gain << 4) | highByte(reg_[i].data);
    wd[2] = lowByte(reg_[i].data);
    this->raw_write(wd, sizeof(wd));
  }
  return this->raw_end_transmission();
}

uint8_t MCP4728Output::seqWrite() {
  this->raw_begin_transmission();
  uint8_t sb = (uint8_t)CMD::SEQ_WRITE;
  this->raw_write(&sb, sizeof(sb));
  for (uint8_t i = 0; i < 4; ++i) {
    uint8_t wd[2];
    wd[0] = ((uint8_t)reg_[i].vref << 7) | ((uint8_t)reg_[i].pd << 5) | ((uint8_t)reg_[i].gain << 4) | highByte(reg_[i].data);
    wd[1] = lowByte(reg_[i].data);
    this->raw_write(wd, sizeof(wd));
  }
  return this->raw_end_transmission();
}

void MCP4728Output::selectVref(uint8_t channel, VREF vref) {
  reg_[channel].vref = vref;

  this->update = true;
}

void MCP4728Output::selectPowerDown(uint8_t channel, PWR_DOWN pd) {
  reg_[channel].pd = pd;

  this->update = true;
}

void MCP4728Output::selectGain(uint8_t channel, GAIN gain) {
  reg_[channel].gain = gain;

  this->update = true;
}

MCP4728Channel *MCP4728Output::create_channel(uint8_t channel, bool use_vdd, int gain) {
  auto *c = new MCP4728Channel(this, channel, use_vdd, gain);
  return c;
}

void MCP4728Channel::write_state(float state) {
  const uint16_t max_duty = 4095;
  const float duty_rounded = roundf(state * max_duty);
  auto duty = static_cast<uint16_t>(duty_rounded);
  this->parent_->set_channel_value(this->channel_, duty);
}

}  // namespace mcp4728
}  // namespace esphome
