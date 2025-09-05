#include "arduino_port_expander.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace arduino_port_expander {

ApeSwitch::ApeSwitch(ArduinoPortExpander *parent, uint8_t pin) 
    : parent_(parent), pin_(pin) {}

ApeBinarySensor::ApeBinarySensor(ArduinoPortExpander *parent, uint8_t pin) 
    : parent_(parent), pin_(pin) {
  parent_->register_binary_sensor(this);
}

i2c::ErrorCode ArduinoPortExpander::write_register(uint8_t reg, uint8_t *data, uint8_t len) {
  last_error_ = i2c::I2CDevice::write_register(reg, data, len);
  if (last_error_ != i2c::ERROR_OK) {
    ESP_LOGE(TAGape, "Write register error: %d", last_error_);
  }
  return last_error_;
}

i2c::ErrorCode ArduinoPortExpander::read_register(uint8_t reg, uint8_t *data, uint8_t len) {
  last_error_ = i2c::I2CDevice::read_register(reg, data, len);
  if (last_error_ != i2c::ERROR_OK) {
    ESP_LOGE(TAGape, "Read register error: %d", last_error_);
  }
  return last_error_;
}

void ArduinoPortExpander::setup() {
  ESP_LOGCONFIG(TAGape, "=== ARDUINO PORT EXPANDER SETUP START ===");
  ESP_LOGCONFIG(TAGape, "Setting up ArduinoPortExpander at %#02x ...", this->address_);
  this->configure_timeout_ = millis() + 15000;
  this->initial_state_ = true;
  this->switches_turned_off_ = false;
  this->first_loop_time_ = 0;
  this->switch_wait_start_ = 0;
  this->setup_complete_ = false;
  
  for (int i = 0; i < 72; i++) {
    last_states_[i] = 0;
  }
  ESP_LOGCONFIG(TAGape, "=== ARDUINO PORT EXPANDER SETUP COMPLETE ===");
}

void ArduinoPortExpander::loop() {
  if (this->first_loop_time_ == 0) {
    this->first_loop_time_ = millis();
    this->switch_wait_start_ = this->first_loop_time_;
    ESP_LOGI(TAGape, "First loop at: %lu ms", this->first_loop_time_);
  }
  
  if (!this->setup_complete_ && millis() > 3000) {
    this->setup_complete_ = true;
    ESP_LOGI(TAGape, "Setup complete, switches count: %d", this->switch_pins_.size());
  }
  
  if (!this->switches_turned_off_ && this->setup_complete_ && !this->switch_pins_.empty()) {
    ESP_LOGI(TAGape, "Setup complete, turning off %d switches", this->switch_pins_.size());
    this->force_turn_off_all_switches();
  }
  
  if (millis() < this->configure_timeout_) {
    bool try_configure = millis() % 100 > 50;
    if (try_configure == this->configure_)
      return;
    this->configure_ = try_configure;

    if (this->read_register(APE_CMD_DIGITAL_READ, this->read_buffer_, 9) == i2c::ERROR_OK) {
      ESP_LOGCONFIG(TAGape, "ArduinoPortExpander found at %#02x", this->address_);
      
      delay(10);
      
      if (this->vref_default_) {
        ESP_LOGD(TAGape, "Setting default VREF");
        this->write_register(CMD_SETUP_ANALOG_DEFAULT, nullptr, 0);
      }
      
      this->configure_timeout_ = 0;
      this->status_clear_error();
      ESP_LOGI(TAGape, "Device configured successfully");

      // Configure input pins first
      for (auto *pin : this->input_pins_) {
        uint8_t pinNo = pin->get_pin();
        ESP_LOGI(TAGape, "Setting up input pin %d with pullup", pinNo);
        if (this->write_register(APE_CMD_SETUP_PIN_INPUT_PULLUP, &pinNo, 1) != i2c::ERROR_OK) {
          ESP_LOGE(TAGape, "Failed to configure pin %d", pinNo);
        }
        delay(20);
      }
      return;
    } else {
      ESP_LOGW(TAGape, "Device not responding during configuration");
    }
    return;
  }

  if (this->configure_timeout_ != 0 && millis() > this->configure_timeout_) {
    ESP_LOGE(TAGape, "ArduinoPortExpander NOT found at %#02x", this->address_);
    this->mark_failed();
    return;
  }

  if (this->read_register(APE_CMD_DIGITAL_READ, this->read_buffer_, 9) != i2c::ERROR_OK) {
    ESP_LOGE(TAGape, "Error reading from device. Reconfiguring...");
    this->status_set_error();
    this->configure_timeout_ = millis() + 15000;
    this->switches_turned_off_ = false;
    this->switch_wait_start_ = millis();
    return;
  }

  // Process input pins - PUBLISH STATES
  bool published_any_initial = false;

  for (auto *pin : this->input_pins_) {
    uint8_t pinNo = pin->get_pin();
    if (pinNo >= 72) {
      ESP_LOGW(TAGape, "Pin %d out of range", pinNo);
      continue;
    }
    
    uint8_t byte_index = pinNo / 8;
    uint8_t bit = pinNo % 8;
    uint8_t value = this->read_buffer_[byte_index];
    bool state = value & (1 << bit);
    
    // APPLY INVERT
    bool inverted = pin->get_inverted();
    if (inverted) {
      state = !state;
    }
    
    if (this->initial_state_) {
      // DURING INITIAL STATE: PUBLISH ALL STATES
      ESP_LOGI(TAGape, "PUBLISHING INITIAL STATE for pin %d: %d (inverted: %d)", 
               pinNo, state, inverted);
      pin->publish_initial_state(state);
      published_any_initial = true;
      last_states_[pinNo] = state;
    } else {
      // AFTER INITIAL STATE: CHECK FOR CHANGES AND PUBLISH
      if (last_states_[pinNo] != state) {
        ESP_LOGI(TAGape, "Pin %d state changed: %d -> %d (inverted: %d)", 
                 pinNo, last_states_[pinNo], state, inverted);
        ESP_LOGI(TAGape, "Publishing state change for pin %d: %d", pinNo, state);
        pin->publish_state(state);
        last_states_[pinNo] = state;
      }
    }
  }

  // Clear initial state flag only after ALL pins have been processed
  if (published_any_initial) {
    this->initial_state_ = false;
    ESP_LOGI(TAGape, "Initial states published for all %d pins", this->input_pins_.size());
  }
}

void ArduinoPortExpander::force_turn_off_all_switches() {
  if (this->is_failed()) {
    ESP_LOGW(TAGape, "Cannot turn off switches, device failed");
    return;
  }
  
  if (this->switches_turned_off_) {
    ESP_LOGD(TAGape, "Switches already turned off, skipping");
    return;
  }
  
  if (this->switch_pins_.empty()) {
    ESP_LOGW(TAGape, "No switch pins to turn off");
    this->switches_turned_off_ = true;
    return;
  }
  
  ESP_LOGI(TAGape, "=== FORCE TURNING OFF ALL SWITCHES ===");
  ESP_LOGI(TAGape, "Number of switch pins: %d", this->switch_pins_.size());
  
  bool all_success = true;
  
  for (auto *sw : this->switch_pins_) {
    uint8_t pin = sw->get_pin();
    ESP_LOGI(TAGape, "Force turning off pin %d", pin);
    
    // 1. Configure pin as output
    ESP_LOGI(TAGape, "Configuring pin %d as output", pin);
    i2c::ErrorCode error = this->write_register(APE_CMD_SETUP_PIN_OUTPUT, &pin, 1);
    if (error != i2c::ERROR_OK) {
      ESP_LOGE(TAGape, "Failed to setup pin %d as output, error: %d", pin, error);
      all_success = false;
      continue;
    } else {
      ESP_LOGI(TAGape, "Successfully configured pin %d as output", pin);
    }
    delay(10);
    
    // 2. Force turning off
    ESP_LOGI(TAGape, "Writing LOW to pin %d", pin);
    error = this->write_register(APE_CMD_WRITE_DIGITAL_LOW, &pin, 1);
    if (error != i2c::ERROR_OK) {
      ESP_LOGE(TAGape, "Failed to turn off pin %d, error: %d", pin, error);
      all_success = false;
    } else {
      ESP_LOGI(TAGape, "Successfully turned off pin %d", pin);
      sw->publish_state(false);
    }
    
    delay(10);
  }
  
  this->switches_turned_off_ = all_success;
  if (all_success) {
    ESP_LOGI(TAGape, "=== FINISHED FORCE TURN OFF ===");
  } else {
    ESP_LOGW(TAGape, "=== FORCE TURN OFF COMPLETED WITH ERRORS ===");
  }
}

switch_::Switch *ArduinoPortExpander::get_switch(uint8_t pin) {
  auto *sw = new ApeSwitch(this, pin);
  ESP_LOGI(TAGape, "Created switch for pin %d", pin);
  return sw;
}

binary_sensor::BinarySensor *ArduinoPortExpander::get_binary_sensor(uint8_t pin) {
  auto *binarySensor = new ApeBinarySensor(this, pin);
  ESP_LOGI(TAGape, "Created binary sensor for pin %d (total sensors: %d)", 
           pin, this->input_pins_.size());
  return binarySensor;
}

void ArduinoPortExpander::write_state(uint8_t pin, bool state) {
  if (this->configure_timeout_ != 0) {
    ESP_LOGW(TAGape, "Cannot write, device not configured");
    return;
  }
  
  if (this->is_failed()) {
    ESP_LOGW(TAGape, "Cannot write, device failed");
    return;
  }
  
  ESP_LOGI(TAGape, "Writing pin %d: state=%d", pin, state);
  
  uint8_t command = state ? APE_CMD_WRITE_DIGITAL_HIGH : APE_CMD_WRITE_DIGITAL_LOW;
  i2c::ErrorCode error = this->write_register(command, &pin, 1);
  
  if (error != i2c::ERROR_OK) {
    ESP_LOGE(TAGape, "Failed to write pin %d, error: %d", pin, error);
  } else {
    ESP_LOGI(TAGape, "Successfully wrote pin %d: %d", pin, state);
  }
}

void ApeSwitch::write_state(bool state) {
  ESP_LOGI(TAGape, "ApeSwitch pin %d: state=%d", this->pin_, state);
  
  if (state && !this->interlock_switches_.empty()) {
    bool found = false;
    for (auto *other : this->interlock_switches_) {
      if (other != this && other->state) {  
        ESP_LOGI(TAGape, "Interlock: turning off %d", other->get_pin());
        other->write_state(false);
        found = true;
      }
    }
    

    if (found && this->interlock_wait_time_ > 0) {
      ESP_LOGI(TAGape, "Waiting %dms before turning on pin %d", 
               this->interlock_wait_time_, this->pin_);
      delay(this->interlock_wait_time_);
    }
  }
  
  this->parent_->write_state(this->pin_, state);
  this->publish_state(state);
}

}  // namespace arduino_port_expander
}  // namespace esphome