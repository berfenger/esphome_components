#include "si1145.h"

#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace si1145 {

static const char *const TAG = "si1145.sensor";

inline float visible_temp_correction(uint16_t value, uint8_t range, uint8_t gain, uint16_t temp,
                                     uint16_t temp_at_begin) {
  float vis = value;
  if (range == Range::RANGE_LOW) {
    switch (gain) {
      case 0:
        vis = vis - 0.3f * (temp - temp_at_begin) / 35.0f;
        break;
      case 1:
        vis = vis - 0.11f * (temp - temp_at_begin) / 35.0f;
        break;
      case 2:
        vis = vis - 0.06f * (temp - temp_at_begin) / 35.0f;
        break;
      case 3:
        vis = vis - 0.03f * (temp - temp_at_begin) / 35.0f;
        break;
      case 4:
        vis = vis - 0.01f * (temp - temp_at_begin) / 35.0f;
        break;
      case 5:
        vis = vis - 0.008f * (temp - temp_at_begin) / 35.0f;
        break;
      case 6:
        vis = vis - 0.007f * (temp - temp_at_begin) / 35.0f;
        break;
      case 7:
        vis = vis - 0.008f * (temp - temp_at_begin) / 35.0f;
        break;
    }
  }
  return vis;
}

inline float infrared_temp_correction(uint16_t value, uint8_t range, uint8_t gain, uint16_t temp,
                                      uint16_t temp_at_begin) {
  float ir = value;
  if (range == Range::RANGE_LOW) {
    switch (gain) {
      case 0:
        ir = ir - 0.3f * (temp - temp_at_begin) / 35.0f;
        break;
      case 1:
        ir = ir - 0.06f * (temp - temp_at_begin) / 35.0f;
        break;
      case 2:
        ir = ir - 0.03f * (temp - temp_at_begin) / 35.0f;
        break;
      case 3:
        ir = ir - 0.01f * (temp - temp_at_begin) / 35.0f;
        break;
    }
  }
  return ir;
}

inline float illumination_combine_sensors(float vis_value, uint8_t vis_range, uint8_t vis_gain, float ir_value,
                                          uint8_t ir_range, uint8_t ir_gain) {
  float range_vis = (vis_range == Range::RANGE_LOW) ? 1.0f : 14.5f;  // 14.5 if high range, 1 otherwise
  float range_ir = (ir_range == Range::RANGE_LOW) ? 1.0f : 14.5f;    // 14.5 if high range, 1 otherwise
  float lux = (5.41f * vis_value * range_vis) / (1 << vis_gain) + (-0.08f * ir_value * range_ir) / (1 << ir_gain);
  if (lux < 0)
    lux = 0;
  return lux;
}

inline float apply_range_and_gain(float value, uint8_t range, uint8_t gain) {
  float range_factor = (range == Range::RANGE_LOW) ? 1.0f : 14.5f;
  return (value * range_factor) / (1 << gain);
}

void SI1145Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Si1145...");
  if (!this->begin_()) {
    this->mark_failed();
    return;
  }
  this->set_visible_gain_(this->visible_gain_);
  this->set_visible_range_(this->visible_range_);
  this->set_infrared_gain_(this->infrared_gain_);
  this->set_infrared_range_(this->infrared_range_);
}

void SI1145Component::dump_config() {
  ESP_LOGCONFIG(TAG, "SI1145:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with SI1145 failed!");
  }
}

float SI1145Component::get_setup_priority() const { return setup_priority::DATA; }

void SI1145Component::update() {
  // force measure
  write8_(SI1145_REG_COMMAND, SI1145_ALS_FORCE);
  delay(20);

  float vis;
  float ir;
  float tp;
  uint8_t resp = read8_(SI1145_REG_RESPONSE);
  switch (resp) {
    case 0x80:  // Invalid command
      vis = read_visible_();
      ir = read_infrared_();
      tp = read_temp_();
      write8_(SI1145_REG_COMMAND, SI1145_NOP);
      break;
    case 0x88:  // PS1 overflow
      vis = read_visible_();
      ir = read_infrared_();
      tp = read_temp_();
      write8_(SI1145_REG_COMMAND, SI1145_NOP);
      break;
    case 0x89:  // PS2 overflow
      vis = read_visible_();
      ir = read_infrared_();
      tp = read_temp_();
      write8_(SI1145_REG_COMMAND, SI1145_NOP);
      break;
    case 0x8A:  // PS3 overflow
      vis = read_visible_();
      ir = read_infrared_();
      tp = read_temp_();
      write8_(SI1145_REG_COMMAND, SI1145_NOP);
      break;
    case 0x8C:  // VIS overflow
      vis = OVERFLOW_VALUE;
      ir = read_infrared_();
      tp = read_temp_();
      write8_(SI1145_REG_COMMAND, SI1145_NOP);
      break;
    case 0x8D:  // IR overflow
      ir = OVERFLOW_VALUE;
      vis = read_visible_();
      tp = read_temp_();
      write8_(SI1145_REG_COMMAND, SI1145_NOP);
      break;
    case 0x8E:  // AUX overflow
      vis = read_visible_();
      ir = read_infrared_();
      tp = temp_at_begin_;
      write8_(SI1145_REG_COMMAND, SI1145_NOP);
      break;
    default:
      vis = read_visible_();
      ir = read_infrared_();
      tp = read_temp_();
      break;
  }

  // save raw values for auto range
  uint16_t visible_ar = vis;
  uint16_t infrared_ar = ir;

  // expected by IC
  uint8_t irq_status = read8_(SI1145_REG_IRQSTAT);
  write8_(SI1145_REG_IRQSTAT, irq_status);

  // temp correction
  if (this->visible_temp_correction_ && vis != OVERFLOW_VALUE) {
    vis = visible_temp_correction(vis, visible_range_, visible_gain_, tp, temp_at_begin_);
  }

  if (this->infrared_temp_correction_ && ir != OVERFLOW_VALUE) {
    ir = infrared_temp_correction(ir, infrared_range_, infrared_gain_, tp, temp_at_begin_);
  }

  // update sensors
  if (this->visible_sensor_ != nullptr && vis != OVERFLOW_VALUE) {
    this->visible_sensor_->publish_state(apply_range_and_gain(vis, visible_range_, visible_gain_));
  }

  if (this->infrared_sensor_ != nullptr && ir != OVERFLOW_VALUE) {
    this->infrared_sensor_->publish_state(apply_range_and_gain(ir, infrared_range_, infrared_gain_));
  }

  if (this->uvindex_sensor_ != nullptr && vis != OVERFLOW_VALUE && ir != OVERFLOW_VALUE) {
    uint8_t uf = read_uvindex_();
    this->uvindex_sensor_->publish_state(uf);
  }

  if (this->illuminance_sensor_ != nullptr && vis != OVERFLOW_VALUE && ir != OVERFLOW_VALUE) {
    float lux = illumination_combine_sensors(vis, visible_range_, visible_gain_, ir, infrared_range_, infrared_gain_);
    this->illuminance_sensor_->publish_state(lux);
  }

  // auto range
  if (visible_mode_auto_) {
    this->auto_range_visible_(visible_ar);
  }
  if (infrared_mode_auto_) {
    this->auto_range_infrared_(infrared_ar);
  }
  write8_(SI1145_REG_COMMAND, SI1145_NOP);
}

uint16_t SI1145Component::read_visible_() {
  uint16_t r = read16_(SI1145_REG_ALSVISDATA0);
  uint16_t vatzero = VALUE_AT_ZERO_HIGH;
  if (visible_range_ == Range::RANGE_LOW) {
    vatzero = VALUE_AT_ZERO_LOW;
  }
  if (r <= vatzero)
    r = 0;
  else
    r -= vatzero;
  return r;
}

uint16_t SI1145Component::read_infrared_() {
  uint16_t r = read16_(SI1145_REG_ALSIRDATA0);
  uint16_t vatzero = VALUE_AT_ZERO_HIGH;
  if (infrared_range_ == Range::RANGE_LOW) {
    vatzero = VALUE_AT_ZERO_LOW;
  }
  if (r <= vatzero)
    r = 0;
  else
    r -= vatzero;
  return r;
}

uint16_t SI1145Component::read_temp_() {
  uint16_t temp = read8_(SI1145_REG_UVINDEX0);
  temp |= (read8_(SI1145_REG_UVINDEX1) << 8);
  return temp;
}

uint8_t SI1145Component::read_uvindex_() {
  int uv = read16_(SI1145_REG_UVINDEX0);
  return (uint8_t) std::floor(uv / 100.0);
}

bool SI1145Component::begin_() {
  uint8_t id = read8_(SI1145_REG_PARTID);
  if (id != 0x45)
    return false;  // look for SI1145

  this->reset_();

  /***********************************/
  // enable UVindex measurement coefficients!
  write8_(SI1145_REG_UCOEFF0, 0x29);
  write8_(SI1145_REG_UCOEFF1, 0x89);
  write8_(SI1145_REG_UCOEFF2, 0x02);
  write8_(SI1145_REG_UCOEFF3, 0x00);

  // enable UV sensor
  write_param_(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV | SI1145_PARAM_CHLIST_ENALSIR |
                                        SI1145_PARAM_CHLIST_ENALSVIS | SI1145_PARAM_CHLIST_ENPS1);
  // enable interrupt on every sample
  write8_(SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);
  write8_(SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);

  /****************************** Prox Sense 1 */

  // program LED current
  write8_(SI1145_REG_PSLED21, 0x03);  // 20mA for LED 1 only
  write_param_(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
  // prox sensor #1 uses LED #1
  write_param_(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
  // fastest clocks, clock div 1
  write_param_(SI1145_PARAM_PSADCGAIN, 0);
  // take 511 clocks to measure
  write_param_(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in prox mode, high range
  write_param_(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE | SI1145_PARAM_PSADCMISC_PSMODE);

  write_param_(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);
  // fastest clocks, clock div 1
  write_param_(SI1145_PARAM_ALSIRADCGAIN, infrared_gain_);
  // take 511 clocks to measure
  write_param_(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // range mode
  write_param_(SI1145_PARAM_ALSIRADCMISC, infrared_range_);

  // fastest clocks, clock div 1
  write_param_(SI1145_PARAM_ALSVISADCGAIN, visible_gain_);
  // take 511 clocks to measure
  write_param_(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // range mode
  write_param_(SI1145_PARAM_ALSVISADCMISC, visible_range_);

  /************************/

  // measurement rate for auto
  write8_(SI1145_REG_MEASRATE0, 0xFF);  // 255 * 31.25uS = 8ms

  // auto run
  write8_(SI1145_REG_COMMAND, SI1145_PSALS_AUTO);

  temp_at_begin_ = read_temp_();

  return true;
}

void SI1145Component::reset_() {
  write8_(SI1145_REG_MEASRATE0, 0);
  write8_(SI1145_REG_MEASRATE1, 0);
  write8_(SI1145_REG_IRQEN, 0);
  write8_(SI1145_REG_IRQMODE1, 0);
  write8_(SI1145_REG_IRQMODE2, 0);
  write8_(SI1145_REG_INTCFG, 0);
  write8_(SI1145_REG_IRQSTAT, 0xFF);

  write8_(SI1145_REG_COMMAND, SI1145_RESET);
  delay(10);
  write8_(SI1145_REG_HWKEY, 0x17);

  delay(10);
}

void SI1145Component::set_visible_gain_(uint8_t gain) { write_param_(SI1145_PARAM_ALSVISADCGAIN, gain); }

void SI1145Component::set_infrared_gain_(uint8_t gain) { write_param_(SI1145_PARAM_ALSIRADCGAIN, gain); }

void SI1145Component::set_visible_range_(uint8_t range) { write_param_(SI1145_PARAM_ALSVISADCMISC, range); }

void SI1145Component::set_infrared_range_(uint8_t range) { write_param_(SI1145_PARAM_ALSIRADCMISC, range); }

void SI1145Component::auto_range_visible_(uint16_t read_value) {
  if (read_value > 25000) {
    if (visible_gain_ == 0) {
      // at lowest gain, increase range
      if (visible_range_ == Range::RANGE_LOW) {
        visible_range_ = Range::RANGE_HIGH;
        set_visible_range_(visible_range_);
      }
    } else if (visible_gain_ > 0) {
      // reduce gain
      visible_gain_ -= 1;
      set_visible_gain_(visible_gain_);
    }
  } else if (read_value < 1500) {
    if (visible_gain_ < 7) {
      // increase gain
      visible_gain_ += 1;
      set_visible_gain_(visible_gain_);
    } else if (visible_range_ == Range::RANGE_HIGH) {
      // at highest gain, reduce range
      visible_range_ = Range::RANGE_LOW;
      visible_gain_ = 0;
      set_visible_range_(visible_range_);
      set_visible_gain_(visible_gain_);
    }
  }
}

void SI1145Component::auto_range_infrared_(uint16_t read_value) {
  if (read_value > 65000 && infrared_range_ == Range::RANGE_HIGH) {
    infrared_range_ = Range::RANGE_LOW;
    infrared_gain_ = 0;
    set_infrared_range_(infrared_range_);
    set_infrared_gain_(infrared_gain_);
  } else if (read_value > 25000) {
    if (infrared_gain_ == 0) {
      if (infrared_range_ == Range::RANGE_LOW) {
        // at lowest gain, increase range
        infrared_range_ = Range::RANGE_HIGH;
        set_infrared_range_(infrared_range_);
      }
    } else if (infrared_gain_ > 0) {
      // reduce gain
      infrared_gain_ -= 1;
      set_infrared_gain_(infrared_gain_);
    }
  } else if (read_value < 1500) {
    if (infrared_gain_ < 7) {
      // increase gain
      infrared_gain_ += 1;
      set_infrared_gain_(infrared_gain_);
    } else if (infrared_range_ == Range::RANGE_HIGH) {
      // at highest gain, reduce range
      infrared_range_ = Range::RANGE_LOW;
      infrared_gain_ = 0;
      set_infrared_range_(infrared_range_);
      set_infrared_gain_(infrared_gain_);
    }
  }
}

void SI1145Component::write8_(uint8_t reg, uint8_t val) { this->write_byte(reg, val); }

uint8_t SI1145Component::read8_(uint8_t reg) {
  uint8_t d8 = 0;
  this->read_byte(reg, &d8);
  return d8;
}

uint16_t SI1145Component::read16_(uint8_t reg) {
  uint16_t d16 = 0;
  this->read_byte_16(reg, &d16);
  return (d16 >> 8) | ((d16 & 0xFF) << 8);
}

uint8_t SI1145Component::write_param_(uint8_t p, uint8_t v) {
  write8_(SI1145_REG_PARAMWR, v);
  write8_(SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
  return read8_(SI1145_REG_PARAMRD);
}

}  // namespace si1145
}  // namespace esphome
