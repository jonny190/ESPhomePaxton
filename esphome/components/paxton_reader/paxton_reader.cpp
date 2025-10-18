#include "paxton_reader.h"
#include "driver/gpio.h"  // <-- IDF GPIO

namespace esphome {
namespace paxton {

void IRAM_ATTR PaxtonReader::isr_trampoline(void *arg) {
  static_cast<PaxtonReader*>(arg)->on_clock_falling();
}

void IRAM_ATTR PaxtonReader::on_clock_falling() {
  const uint32_t now = micros();
  if (now - last_edge_us_ < debounce_us) return;
  last_edge_us_ = now;
  if (processing_) return;

  int level = gpio_get_level((gpio_num_t) data_pin_);
  bool bit = invert_data_ ? !level : level;

  if (bit_count_ < sizeof(bits_)) {
    bits_[bit_count_++] = bit ? 1 : 0;
  } else {
    processing_ = true;
  }
}

void PaxtonReader::setup() {
  if (clock_pin_ < 0 || data_pin_ < 0) {
    ESP_LOGE("paxton", "Pins not configured");
    return;
  }

  // --- Configure CLOCK as input with falling-edge interrupt ---
  {
    gpio_config_t conf{};
    conf.pin_bit_mask = (1ULL << clock_pin_);
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = use_pullups_ ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_NEGEDGE;  // falling edge on clock
    ESP_ERROR_CHECK(gpio_config(&conf));
  }

  // --- Configure DATA as input (no interrupt) ---
  {
    gpio_config_t conf{};
    conf.pin_bit_mask = (1ULL << data_pin_);
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = use_pullups_ ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&conf));
  }

  // --- Configure LED pins as outputs (start LOW = off) ---
  auto cfg_out = [](int pin) {
    if (pin < 0) return;
    gpio_config_t conf{};
    conf.pin_bit_mask = (1ULL << pin);
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&conf));
    gpio_set_level((gpio_num_t) pin, 0);
  };
  cfg_out(led_green_);
  cfg_out(led_yellow_);
  cfg_out(led_red_);

  // --- Install ISR service (once) & attach ISR to CLOCK ---
  static bool isr_installed = false;
  if (!isr_installed) {
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    isr_installed = true;
  }
  ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t) clock_pin_, &PaxtonReader::isr_trampoline, this));
  ESP_ERROR_CHECK(gpio_set_intr_type((gpio_num_t) clock_pin_, GPIO_INTR_NEGEDGE));

  ESP_LOGI("paxton", "Ready (ESP-IDF ISR). debounce=%uus frame_gap=%uus invert_data=%s pullups=%s",
           (unsigned)debounce_us, (unsigned)frame_gap_us,
           invert_data_ ? "yes" : "no", use_pullups_ ? "yes" : "no");
}

void PaxtonReader::log_bits_preview_(uint16_t n) {
  const uint16_t show = std::min<uint16_t>(n, 64);
  std::string s; s.reserve(show);
  for (uint16_t i = 0; i < show; i++) s.push_back(bits_[i] ? '1' : '0');
  ESP_LOGD("paxton", "Bits(%u) preview[0..%u]: %s%s",
           (unsigned)n, (unsigned)(show - 1), s.c_str(), n > show ? "..." : "");
}

void PaxtonReader::led_on_for_(int pin, uint32_t ms) {
  if (pin < 0) return;
  // Note: many Paxton LED lines are active-low; adjust if needed.
  gpio_set_level((gpio_num_t) pin, 1);
  led_on_pin_ = pin;
  led_off_at_ms_ = millis() + ms;
}

bool PaxtonReader::check_leadin_10zeros_ending_one_() const {
  if (bit_count_ < 11) return false;
  for (int i = 0; i < 10; i++) if (bits_[i] != 0) return false;
  return bits_[10] == 1;
}

bool PaxtonReader::check_leadout_10zeros_() const {
  if (bit_count_ < 10) return false;
  for (int i = bit_count_ - 10; i < bit_count_; i++) if (bits_[i] != 0) return false;
  return true;
}

bool PaxtonReader::parse_net2_(std::string &card_no, std::string &bin) {
  const uint16_t n = bit_count_;
  if (n != net2_bits) return false;
  if (!check_leadin_10zeros_ending_one_()) return false;
  if (!check_leadout_10zeros_()) return false;

  bin.reserve(n);
  for (int i = 0; i < n; i++) bin.push_back(bits_[i] ? '1' : '0');

  card_no.clear();

  // Current strict attempt (kept) — adaptive fallback will kick in if this fails.
  int idx = 11;
  for (int d = 0; d < 8; d++) {
    if (idx + 3 >= n) return false;
    int val = (bits_[idx] << 3) | (bits_[idx + 1] << 2) | (bits_[idx + 2] << 1) | (bits_[idx + 3]);
    idx += 4;
    if (val > 9) return false;
    card_no.push_back(char('0' + val));
  }
  return true;
}

bool PaxtonReader::parse_switch2_(std::string &card_no, std::string &colour, std::string &bin) {
  const uint16_t n = bit_count_;
  if (n != switch2_bits) return false;
  if (!check_leadin_10zeros_ending_one_()) return false;
  if (!check_leadout_10zeros_()) return false;

  bin.reserve(n);
  for (int i = 0; i < n; i++) bin.push_back(bits_[i] ? '1' : '0');

  card_no.clear();
  int idx = 11;
  for (int d = 0; d < 8; d++) {
    if (idx + 3 >= n) return false;
    int val = (bits_[idx] << 3) | (bits_[idx + 1] << 2) | (bits_[idx + 2] << 1) | (bits_[idx + 3]);
    idx += 4;
    if (val > 9) return false;
    card_no.push_back(char('0' + val));
  }
  colour = "Unknown";
  return true;
}

void PaxtonReader::publish_success_(const std::string &card_no,
                                    const std::string &type,
                                    const std::string &colour,
                                    uint16_t bits,
                                    const std::string &bin) {
  if (last_card_ts) last_card_ts->publish_state(card_no);
  if (card_type_ts) card_type_ts->publish_state(type);
  if (card_colour_ts) card_colour_ts->publish_state(colour);
  if (bit_count_s) bit_count_s->publish_state(bits);
  if (reading_bs) reading_bs->publish_state(true);
  if (led_green_ >= 0) led_on_for_(led_green_, 60);
  if (reading_bs) reading_bs->publish_state(false);
  ESP_LOGI("paxton", "Card: %s | Type: %s | Colour: %s | Bits: %u",
           card_no.c_str(), type.c_str(), colour.c_str(), (unsigned) bits);
}

void PaxtonReader::publish_error_(const char *msg) {
  if (card_type_ts) card_type_ts->publish_state("Error");
  if (last_card_ts) last_card_ts->publish_state(msg);
  if (bit_count_s) bit_count_s->publish_state((float) bit_count_);
  if (led_red_ >= 0) led_on_for_(led_red_, 100);
  ESP_LOGW("paxton", "Parse error: %s (bits=%u)", msg, (unsigned) bit_count_);
}

void PaxtonReader::loop() {
  static uint32_t idle_since = millis();

  // Non-blocking LED off
  if (led_on_pin_ >= 0 && (int32_t) (millis() - led_off_at_ms_) >= 0) {
    gpio_set_level((gpio_num_t) led_on_pin_, 0);
    led_on_pin_ = -1;
  }

  if (bit_count_ > 0) {
    if ((micros() - last_edge_us_) > frame_gap_us) {  // frame complete
      processing_ = true;
      const uint16_t n = bit_count_;
      log_bits_preview_(n);

      // Publish full bitstring to HA
      std::string bin; bin.reserve(n);
      for (int i = 0; i < n; i++) bin.push_back(bits_[i] ? '1' : '0');
      if (raw_bits_ts) raw_bits_ts->publish_state(bin);

      std::string card_no, colour;
      bool ok = false;

      if (n == net2_bits) {
        ok = parse_net2_(card_no, bin);
        if (!ok) {
          if (try_adaptive_bcd_(card_no)) {
            publish_success_(card_no, "Net2 (adaptive)", "None", n, bin);
            ok = true;
          } else {
            publish_error_("Net2 parse fail");
          }
        } else {
          publish_success_(card_no, "Net2", "None", n, bin);
        }
      } else if (n == switch2_bits) {
        ok = parse_switch2_(card_no, colour, bin);
        if (ok) publish_success_(card_no, "Switch2 Knockout", colour, n, bin);
        else publish_error_("Switch2 parse fail");
      } else {
        publish_error_("Unknown bit length");
      }

      bit_count_ = 0;
      processing_ = false;
      idle_since = millis();
      if (!ok && led_yellow_ >= 0) led_on_for_(led_yellow_, 60);
    }
  } else {
    if (millis() - idle_since > 5000) {
      ESP_LOGV("paxton", "idle...");
      idle_since = millis();
    }
  }
}

// ---------------- Adaptive BCD fallback ----------------
// Try several plausible Net2 layouts: groups of 5 (BCD+parity) or 4 (tight BCD),
// various start offsets, optional bitstream/nibble reversal.
bool PaxtonReader::try_adaptive_bcd_(std::string &out) {
  const int N = bit_count_;
  auto get_bit = [&](int i, bool reverse) -> uint8_t {
    return reverse ? bits_[N - 1 - i] : bits_[i];
  };

  auto nibble_val = [&](int start, bool reverse_stream, bool reverse_nibble) -> int {
    uint8_t b0 = get_bit(start + 0, reverse_stream);
    uint8_t b1 = get_bit(start + 1, reverse_stream);
    uint8_t b2 = get_bit(start + 2, reverse_stream);
    uint8_t b3 = get_bit(start + 3, reverse_stream);
    if (reverse_nibble) { std::swap(b0, b3); std::swap(b1, b2); }
    return (b0 << 3) | (b1 << 2) | (b2 << 1) | (b3 << 0);
  };

  struct Try { int start; int group; bool rev_stream; bool rev_nibble; };
  std::vector<Try> tries;

  for (int s : {11, 12, 13, 14, 15}) {
    for (bool rs : {false, true}) {
      for (bool rn : {false, true}) {
        tries.push_back({s, 5, rs, rn});  // BCD+parity
        tries.push_back({s, 4, rs, rn});  // tight BCD
      }
    }
  }

  for (const auto &t : tries) {
    int idx = t.start;
    std::string digits; digits.reserve(8);
    bool ok = true;

    for (int d = 0; d < 8; d++) {
      if (idx + 3 >= N) { ok = false; break; }
      int val = nibble_val(idx, t.rev_stream, t.rev_nibble);
      if (val > 9) { ok = false; break; }
      digits.push_back(char('0' + val));
      idx += (t.group == 5 ? 5 : 4);
    }

    if (ok && (int)digits.size() == 8) {
      out = digits;
      ESP_LOGD("paxton", "Adaptive BCD matched: start=%d group=%d rev_stream=%s rev_nibble=%s -> %s",
               t.start, t.group, t.rev_stream ? "yes" : "no", t.rev_nibble ? "yes" : "no", out.c_str());
      return true;
    }
  }
  return false;
}

}  // namespace paxton
}  // namespace esphome
