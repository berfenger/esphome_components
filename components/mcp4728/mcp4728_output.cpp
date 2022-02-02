#include "mcp4728_output.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mcp4728 {

static const char *const TAG = "mcp4728";

void MCP4728Output::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MCP4728OutputComponent...");
}

void MCP4728Output::dump_config() {
  ESP_LOGCONFIG(TAG, "MCP4728:");
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

void MCP4728Output::set_channel_value(MCP4728_CHANNEL channel, uint16_t value) {
  uint8_t cn = 0;
  if (channel == MCP4728_CHANNEL_A)
    cn = 'A';
  else if (channel == MCP4728_CHANNEL_B)
    cn = 'B';
  else if (channel == MCP4728_CHANNEL_C)
    cn = 'C';
  else
    cn = 'D';
  ESP_LOGD(TAG, "Setting MCP4728 channel %c to %d!", cn, value);
  reg_[channel].data = value;
  this->update = true;
}

uint8_t MCP4728Output::multiWrite() {
  for (uint8_t i = 0; i < 4; ++i) {
    uint8_t wd[3];
    wd[0] = ((uint8_t)CMD::MULTI_WRITE | (i << 1)) & 0xFE;
    wd[1] = ((uint8_t)reg_[i].vref << 7) | ((uint8_t)reg_[i].pd << 5) |
            ((uint8_t)reg_[i].gain << 4) | highByte(reg_[i].data);
    wd[2] = lowByte(reg_[i].data);
    this->write(wd, sizeof(wd));
  }
  return 0;
}

uint8_t MCP4728Output::seqWrite() {
  uint8_t wd[9];
  wd[0] = (uint8_t)CMD::SEQ_WRITE;
  for (uint8_t i = 0; i < 4; i++) {
    wd[i * 2 + 1] = ((uint8_t)reg_[i].vref << 7) | ((uint8_t)reg_[i].pd << 5) |
                    ((uint8_t)reg_[i].gain << 4) | highByte(reg_[i].data);
    wd[i * 2 + 2] = lowByte(reg_[i].data);
  }
  this->write(wd, sizeof(wd));
  return 0;
}

void MCP4728Output::selectVref(MCP4728_CHANNEL channel, MCP4728_VREF vref) {
  reg_[channel].vref = vref;

  this->update = true;
}

void MCP4728Output::selectPowerDown(MCP4728_CHANNEL channel, PWR_DOWN pd) {
  reg_[channel].pd = pd;

  this->update = true;
}

void MCP4728Output::selectGain(MCP4728_CHANNEL channel, MCP4728_GAIN gain) {
  reg_[channel].gain = gain;

  this->update = true;
}

MCP4728Channel *MCP4728Output::create_channel(MCP4728_CHANNEL channel,
                                              MCP4728_VREF vref,
                                              MCP4728_GAIN gain) {
  auto *c = new MCP4728Channel(this, channel, vref, gain);
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
