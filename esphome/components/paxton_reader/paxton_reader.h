#pragma once
#include "esphome.h"
#include <Arduino.h>

namespace esphome {
namespace paxton {

class PaxtonReader : public Component {
 public:
  // Raw pins
  int clock_pin_{-1};
  int data_pin_{-1};
  int led_green_{-1};
  int led_yellow_{-1};
  int led_red_{-1};

  // Entities
  text_sensor::TextSensor *last_card_ts{nullptr};
  text_sensor::TextSensor *card_type_ts{nullptr};
  text_sensor::TextSensor *card_colour_ts{nullptr};
  sensor::Sensor *bit_count_s{nullptr};
  binary_sensor::BinarySensor *reading_bs{nullptr};

  // Config
  uint16_t net2_bits{75};
  uint16_t switch2_bits{220};
  uint32_t debounce_us{350};

  // Setters (called from codegen)
  void set_clock_pin(int p) { clock_pin_ = p; }
  void set_data_pin(int p) { data_pin_ = p; }
  void set_led_green(int p) { led_green_ = p; }
  void set_led_yellow(int p) { led_yellow_ = p; }
  void set_led_red(int p) { led_red_ = p; }

  void set_last_card(text_sensor::TextSensor *p) { last_card_ts = p; }
  void set_card_type(text_sensor::TextSensor *p) { card_type_ts = p; }
  void set_card_colour(text_sensor::TextSensor *p) { card_colour_ts = p; }
  void set_bit_count(sensor::Sensor *p) { bit_count_s = p; }
  void set_reading(binary_sensor::BinarySensor *p) { reading_bs = p; }

  void set_net2_bits(uint16_t v) { net2_bits = v; }
  void set_switch2_bits(uint16_t v) { switch2_bits = v; }
  void set_debounce_us(uint32_t v) { debounce_us = v; }

  void setup() override;
  void loop() override;

  static void IRAM_ATTR isr_trampoline(void *arg);

 protected:
  void IRAM_ATTR on_clock_falling();
  void pulse_led_(int pin, uint32_t ms);

  volatile uint16_t bit_count_{0};
  volatile bool processing_{false};
  volatile uint32_t last_edge_us_{0};
  volatile uint8_t bits_[256];

  bool check_leadin_10zeros_ending_one_() const;
  bool check_leadout_10zeros_() const;
  bool parse_net2_(std::string &card_no, std::string &bin);
  bool parse_switch2_(std::string &card_no, std::string &colour, std::string &bin);

  void publish_success_(const std::string &card_no,
                        const std::string &type,
                        const std::string &colour,
                        uint16_t bits,
                        const std::string &bin);
  void publish_error_(const char *msg);
};

}  // namespace paxton
}  // namespace esphhome
