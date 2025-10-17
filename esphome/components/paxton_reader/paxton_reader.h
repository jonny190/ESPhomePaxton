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
  text_sensor::TextSensor *raw_bits_ts{nullptr};
  sensor::Sensor *bit_count_s{nullptr};
  binary_sensor::BinarySensor *reading_bs{nullptr};

  // Config
  uint16_t net2_bits{75};
  uint16_t switch2_bits{220};
  uint32_t debounce_us{350};
  uint32_t frame_gap_us{4000};
  bool invert_data_{false};
  bool use_pullups_{true};

  // Setters (called from codegen)
  void set_clock_pin(int p) { clock_pin_ = p; }
  void set_data_pin(int p) { data_pin_ = p; }
  void set_led_green(int p) { led_green_ = p; }
  void set_led_yellow(int p) { led_yellow_ = p; }
  void set_led_red(int p) { led_red_ = p; }

  void set_last_card(text_sensor::TextSensor *p) { last_card_ts = p; }
  void set_card_type(text_sensor::TextSensor *p) { card_type_ts = p; }
  void set_card_colour(text_sensor::TextSensor *p) { card_colour_ts = p; }
  void set_raw_bits(text_sensor::TextSensor *p) { raw_bits_ts = p; }
  void set_bit_count(sensor::Sensor *p) { bit_count_s = p; }
  void set_reading(binary_sensor::BinarySensor *p) { reading_bs = p; }

  void set_net2_bits(uint16_t v) { net2_bits = v; }
  void set_switch2_bits(uint16_t v) { switch2_bits = v; }
  void set_debounce_us(uint32_t v) { debounce_us = v; }
  void set_frame_gap_us(uint32_t v) { frame_gap_us = v; }
  void set_invert_data(bool v) { invert_data_ = v; }
  void set_use_pullups(bool v) { use_pullups_ = v; } 

  void log_bits_preview_(uint16_t n);

  void setup() override;
  void loop() override;

  static void IRAM_ATTR isr_trampoline(void *arg);

 protected:
  void IRAM_ATTR on_clock_falling();

  // Non-blocking LED handling
  void led_on_for_(int pin, uint32_t ms);
  int led_on_pin_ = -1;
  uint32_t led_off_at_ms_ = 0;

  volatile uint16_t bit_count_{0};
  volatile bool processing_{false};
  volatile uint32_t last_edge_us_{0};
  volatile uint8_t bits_[256];

  bool check_leadin_10zeros_ending_one_() const;
  bool check_leadout_10zeros_() const;

  // Heuristic BCD (fallback for Net2)
  bool try_heuristic_bcd_(std::string &card_out);

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
}  // namespace esphome
