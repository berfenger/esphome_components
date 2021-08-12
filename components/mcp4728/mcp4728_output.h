#pragma once

#include "esphome/core/component.h"
#include "esphome/components/output/float_output.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace mcp4728 {

enum class CMD
{
    FAST_WRITE = 0x00,
    MULTI_WRITE = 0x40,
    SINGLE_WRITE = 0x58,
    SEQ_WRITE = 0x50,
    SELECT_VREF = 0x80,
    SELECT_GAIN = 0xC0,
    SELECT_PWRDOWN = 0xA0
};

enum class VREF { VDD, INTERNAL_2_8V };
enum class PWR_DOWN { NORMAL, GND_1KOHM, GND_100KOHM, GND_500KOHM };
enum class GAIN { X1, X2 };

struct DACInputData
{
    VREF vref;
    PWR_DOWN pd;
    GAIN gain;
    uint16_t data;
};

class MCP4728Channel;

/// MCP4728 float output component.
class MCP4728Output : public Component, public i2c::I2CDevice {
 public:
  MCP4728Output(bool eeprom): eeprom(eeprom) {}

  MCP4728Channel *create_channel(uint8_t channel, bool use_vdd, int gain);

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void loop() override;

 protected:
  enum ErrorCode { NONE = 0, COMMUNICATION_FAILED } error_code_{NONE};
  friend MCP4728Channel;
  void set_channel_value(uint8_t channel, uint16_t value);
  uint8_t multiWrite();
  uint8_t seqWrite();
  void selectVref(uint8_t channel, VREF vref);
  void selectPowerDown(uint8_t channel, PWR_DOWN pd);
  void selectGain(uint8_t channel, GAIN gain);

 private:
  DACInputData reg_[4];
  bool eeprom = false;
  bool update = false;
};

class MCP4728Channel : public output::FloatOutput {
 public:
  MCP4728Channel(MCP4728Output *parent, uint8_t channel, bool use_vdd, int gain) : 
    parent_(parent), channel_(channel), use_vdd_(use_vdd), gain_(gain) {
      // update VREF
      VREF vr = VREF::VDD;
      if (!use_vdd) {
        vr = VREF::INTERNAL_2_8V;
      }
      parent->selectVref(channel, vr);
      // update PD
      parent->selectPowerDown(channel, PWR_DOWN::NORMAL);
      // update GAIN
      GAIN ga = GAIN::X1;
      if (gain == 2) {
        ga = GAIN::X2;
      }
      parent->selectGain(channel, ga);
    }

 protected:
  void write_state(float state) override;

  MCP4728Output *parent_;
  uint8_t channel_;
  bool use_vdd_;
  bool gain_;
};

}  // namespace mcp4728
}  // namespace esphome
