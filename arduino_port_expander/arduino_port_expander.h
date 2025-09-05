#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>

namespace esphome {
namespace arduino_port_expander {

static const char *const TAGape = "ape";

#define APE_CMD_DIGITAL_READ 0
#define APE_CMD_WRITE_ANALOG 2
#define APE_CMD_WRITE_DIGITAL_HIGH 3
#define APE_CMD_WRITE_DIGITAL_LOW 4
#define APE_CMD_SETUP_PIN_OUTPUT 5
#define APE_CMD_SETUP_PIN_INPUT_PULLUP 6
#define APE_CMD_SETUP_PIN_INPUT 7

#define CMD_ANALOG_READ_A0 0x08
#define CMD_ANALOG_READ_A15 0x11

#define CMD_SETUP_ANALOG_INTERNAL 0x10
#define CMD_SETUP_ANALOG_DEFAULT 0x11

class ArduinoPortExpander;

class ApeSwitch : public switch_::Switch {
 public:
  ApeSwitch(ArduinoPortExpander *parent, uint8_t pin);
  void write_state(bool state) override;
  uint8_t get_pin() { return this->pin_; }
  
  // Interlock methods
  void set_interlock_switches(const std::vector<ApeSwitch*> &switches) { 
    this->interlock_switches_ = switches; 
  }
  const std::vector<ApeSwitch*> &get_interlock_switches() const { 
    return this->interlock_switches_; 
  }
  void set_interlock_wait_time(uint32_t wait_time) { this->interlock_wait_time_ = wait_time; }
  uint32_t get_interlock_wait_time() const { return this->interlock_wait_time_; }

 protected:
  ArduinoPortExpander *parent_;
  uint8_t pin_;
  std::vector<ApeSwitch*> interlock_switches_;
  uint32_t interlock_wait_time_{0};
};

class ApeBinarySensor : public binary_sensor::BinarySensor {
 public:
  ApeBinarySensor(ArduinoPortExpander *parent, uint8_t pin);
  uint8_t get_pin() { return this->pin_; }
  bool get_inverted() { return this->inverted_; }
  void set_inverted(bool inverted) { this->inverted_ = inverted; }

 protected:
  ArduinoPortExpander *parent_;
  uint8_t pin_;
  bool inverted_{false};
};

class ArduinoPortExpander : public Component, public i2c::I2CDevice {
 public:
  ArduinoPortExpander() = default;

  void setup() override;
  void loop() override;

  switch_::Switch *get_switch(uint8_t pin);
  binary_sensor::BinarySensor *get_binary_sensor(uint8_t pin);
  
  void register_binary_sensor(ApeBinarySensor *sensor) {
    this->input_pins_.push_back(sensor);
    ESP_LOGI(TAGape, "Registered binary sensor for pin %d", sensor->get_pin());
  }
  
  void register_switch(ApeSwitch *sw) {
    this->switch_pins_.push_back(sw);
    ESP_LOGI(TAGape, "Registered switch for pin %d (total: %d)", sw->get_pin(), this->switch_pins_.size());
  }

  void write_state(uint8_t pin, bool state);
  void force_turn_off_all_switches();
  void set_vref_default(bool vref_default) { this->vref_default_ = vref_default; }
  
  i2c::ErrorCode last_error() const { return last_error_; }
  void reset_connection();

 protected:
  i2c::ErrorCode write_register(uint8_t reg, uint8_t *data, uint8_t len);
  i2c::ErrorCode read_register(uint8_t reg, uint8_t *data, uint8_t len);
  
  bool configure_{true};
  bool initial_state_{true};
  uint8_t read_buffer_[9]{0};
  uint8_t last_states_[72]{0};
  unsigned long configure_timeout_{5000};
  bool vref_default_{false};
  i2c::ErrorCode last_error_{i2c::ERROR_OK};
  bool switches_turned_off_{false};
  unsigned long first_loop_time_{0};
  unsigned long switch_wait_start_{0};
  bool setup_complete_{false};

  std::vector<ApeSwitch *> switch_pins_;
  std::vector<ApeBinarySensor *> input_pins_;
};

}  // namespace arduino_port_expander
}  // namespace esphome