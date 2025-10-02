#pragma once
#include "esphome.h"

namespace esphome {
namespace paxton {

class PaxtonReader : public Component, public EntityBase {
 public:
  GPIOPin *clock_pin_{nullptr};
  GPIOPin *data_pin_{nullptr};
  GPIOPin *led_green_{nullptr};
  GPIOPin *led_yellow_{nullptr};
  GPIOPin *led_red_{nullptr};

  text_sensor::TextSensor *last_card_ts{nullptr};
  text_sensor::TextSensor *card_type_ts{nullptr};
  text_sensor::TextSensor *card_colour_ts{nullptr};
  sensor::Sensor *bit_count_s{nullptr};
  binary_sensor::BinarySensor *reading_bs{nullptr};

  uint16_t net2_bits{75};
  uint16_t switch2_bits{220};
  uint32_t debounce_us{350};

  void set_clock_pin(GPIOPin *p) { clock_pin_ = p; }
  void set_data_pin(GPIOPin *p) { data_pin_ = p; }
  void set_led_green(GPIOPin *p) { led_green_ = p; }
  void set_led_yellow(GPIOPin *p) { led_yellow_ = p; }
  void set_led_red(GPIOPin *p) { led_red_ = p; }

  // “set_parent” helpers used by Python glue
  void set_parent(text_sensor::TextSensor *p) { /* no-op for ts */ }
  void set_parent(sensor::Sensor *p) { /* no-op for sensor */ }
  void set_parent(binary_sensor::BinarySensor *p) { /* no-op for bs */ }

  void setup() override;
  void loop() override;

 protected:
  static void IRAM_ATTR isr_trampoline(PaxtonReader *self);
  void IRAM_ATTR on_clock_falling();
  void pulse_led(GPIOPin *pin, uint32_t ms);

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
}  // namespace esphome
