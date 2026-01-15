#include "paxton_reader.h"
#include "driver/gpio.h"  // <-- IDF GPIO
#include <algorithm>      // for std::reverse
#include "esp_task_wdt.h" // ESP-IDF task watchdog
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

  // Net2 75-bit format (Paxtogeddon algorithm):
  // - 10 lead-in zeros (positions 0-9)
  // - 11 groups of 5 bits each (positions 10-64): START + 8 digits + STOP + LRC
  // - 10 lead-out zeros (positions 65-74)
  //
  // Each 5-bit group: 4 BCD bits (LSB first) + 1 parity bit
  // BCD value = 8*bit[i+3] + 4*bit[i+2] + 2*bit[i+1] + 1*bit[i+0]

  card_no.clear();
  int LRC[4] = {0, 0, 0, 0};

  for (int i = 10; i < n - 10; i += 5) {
    if (i + 4 >= n - 10) break;

    int b0 = bits_[i + 0];
    int b1 = bits_[i + 1];
    int b2 = bits_[i + 2];
    int b3 = bits_[i + 3];
    int b4 = bits_[i + 4];

    // BCD value (LSB first, like Paxtogeddon)
    int dval = 8*b3 + 4*b2 + 2*b1 + 1*b0;

    // Check row parity (odd)
    int row_parity = (b0 + b1 + b2 + b3) % 2 == 0 ? 1 : 0;
    if (row_parity != b4) {
      ESP_LOGD("paxton", "Net2: Row parity fail at position %d", i);
      return false;
    }

    // Check START marker (position 10)
    if (i == 10) {
      if (dval != 11) {  // 0xB
        ESP_LOGD("paxton", "Net2: Expected START (0xB=11) at pos 10, got %d", dval);
        return false;
      }
    }
    // Check STOP marker (position 55)
    else if (i == 55) {
      if (dval != 15) {  // 0xF
        ESP_LOGD("paxton", "Net2: Expected STOP (0xF=15) at pos 55, got %d", dval);
        return false;
      }
    }
    // Check LRC (position 60)
    else if (i == 60) {
      int c0 = LRC[0] % 2 == 0 ? 0 : 1;
      int c1 = LRC[1] % 2 == 0 ? 0 : 1;
      int c2 = LRC[2] % 2 == 0 ? 0 : 1;
      int c3 = LRC[3] % 2 == 0 ? 0 : 1;

      if (!(c0 == b0 && c1 == b1 && c2 == b2 && c3 == b3)) {
        ESP_LOGD("paxton", "Net2: LRC mismatch");
        return false;
      }
    }
    // Collect card digits (positions 15-50, which are 8 digits)
    else if (i >= 15 && i <= 50) {
      if (dval > 9) {
        ESP_LOGD("paxton", "Net2: Invalid BCD digit %d at position %d", dval, i);
        return false;
      }
      card_no.push_back(char('0' + dval));
    }

    // Accumulate LRC (skip LRC group itself)
    if (i < 60) {
      LRC[0] += b0;
      LRC[1] += b1;
      LRC[2] += b2;
      LRC[3] += b3;
    }
  }

  if (card_no.length() != 8) {
    ESP_LOGD("paxton", "Net2: Expected 8 digits, got %zu", card_no.length());
    return false;
  }

  ESP_LOGI("paxton", "Net2: Decoded card: %s", card_no.c_str());
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

bool PaxtonReader::parse_paxton90_(std::string &card_no, std::string &colour, std::string &bin) {
  const uint16_t n = bit_count_;
  if (n != paxton90_bits) return false;
  if (!check_leadin_10zeros_ending_one_()) return false;
  if (!check_leadout_10zeros_()) return false;

  bin.reserve(n);
  for (int i = 0; i < n; i++) bin.push_back(bits_[i] ? '1' : '0');

  // 90-bit format (Paxtogeddon-style parsing):
  // - 10 lead-in zeros (positions 0-9)
  // - 14 groups of 5 bits each (positions 10-79): START + 10 data digits + SEP(0xD) + STOP(0xF) + LRC
  // - 10 lead-out zeros (positions 80-89)
  //
  // Each 5-bit group: 4 BCD bits (LSB first) + 1 parity bit
  // BCD value = 8*bit[i+3] + 4*bit[i+2] + 2*bit[i+1] + 1*bit[i+0]
  //
  // NOTE: The encoded 10 digits do NOT match the printed card number!
  // Paxton uses a proprietary transformation algorithm.
  // We output the raw encoded digits for card identification.

  std::string all_digits;
  int LRC[4] = {0, 0, 0, 0};
  bool parity_ok = true;

  // Parse all 5-bit groups from position 10 to 75
  for (int i = 10; i < 80; i += 5) {
    if (i + 4 >= n) break;

    int b0 = bits_[i + 0];
    int b1 = bits_[i + 1];
    int b2 = bits_[i + 2];
    int b3 = bits_[i + 3];
    int b4 = bits_[i + 4];

    // BCD value (LSB first, like Paxtogeddon)
    int dval = 8*b3 + 4*b2 + 2*b1 + 1*b0;

    // Check row parity (odd): sum of data bits should make parity bit result in odd total
    int row_parity = (b0 + b1 + b2 + b3) % 2 == 0 ? 1 : 0;
    if (row_parity != b4) {
      ESP_LOGD("paxton", "90-bit: Row parity fail at position %d (expected %d, got %d)", i, row_parity, b4);
      parity_ok = false;
    }

    // Accumulate LRC (column parity) - skip LRC group itself (position 75)
    if (i < 75) {
      LRC[0] += b0;
      LRC[1] += b1;
      LRC[2] += b2;
      LRC[3] += b3;
    }

    // Check START marker (position 10)
    if (i == 10) {
      if (dval != 11) {  // 0xB
        ESP_LOGW("paxton", "90-bit: Expected START (0xB=11) at pos 10, got %d", dval);
      }
    }
    // Check SEP marker (position 65)
    else if (i == 65) {
      if (dval != 13) {  // 0xD
        ESP_LOGD("paxton", "90-bit: Expected SEP (0xD=13) at pos 65, got %d", dval);
      }
    }
    // Check STOP marker (position 70)
    else if (i == 70) {
      if (dval != 15) {  // 0xF
        ESP_LOGW("paxton", "90-bit: Expected STOP (0xF=15) at pos 70, got %d", dval);
      }
    }
    // Check LRC (position 75)
    else if (i == 75) {
      int c0 = LRC[0] % 2 == 0 ? 0 : 1;
      int c1 = LRC[1] % 2 == 0 ? 0 : 1;
      int c2 = LRC[2] % 2 == 0 ? 0 : 1;
      int c3 = LRC[3] % 2 == 0 ? 0 : 1;

      if (!(c0 == b0 && c1 == b1 && c2 == b2 && c3 == b3)) {
        ESP_LOGD("paxton", "90-bit: LRC mismatch (expected %d%d%d%d, got %d%d%d%d)",
                 c0, c1, c2, c3, b0, b1, b2, b3);
      }
    }
    // Collect data digits (positions 15-60, which are 10 digits)
    else if (i >= 15 && i <= 60 && dval <= 9) {
      all_digits.push_back(char('0' + dval));
    }
  }

  if (all_digits.length() >= 8) {
    // Output all decoded digits (typically 10 for 90-bit format)
    // First 2 digits are usually "03" (format identifier)
    // Remaining 8 digits are the encoded card data
    card_no = all_digits;
    colour = "None";

    ESP_LOGI("paxton", "90-bit: Decoded raw digits: %s (parity %s)",
             card_no.c_str(), parity_ok ? "OK" : "FAIL");
    ESP_LOGW("paxton", "90-bit: NOTE - Raw digits differ from printed card number due to Paxton encoding");

    return true;
  }

  ESP_LOGW("paxton", "90-bit: Failed to decode (got %zu digits)", all_digits.length());
  return false;
}

void PaxtonReader::publish_success_(const std::string &card_no,
                                    const std::string &type,
                                    const std::string &colour,
                                    uint16_t bits,
                                    const std::string &bin) {
  // Defer publishing to loop() to avoid watchdog timeout
  pending_card_no_ = card_no;
  pending_type_ = type;
  pending_colour_ = colour;
  pending_bits_ = bits;
  pending_bin_ = bin;
  pending_error_ = false;
  publish_step_ = 1;  // Start deferred publish

  if (led_green_ >= 0) led_on_for_(led_green_, 60);
  ESP_LOGI("paxton", "Card: %s | Type: %s | Colour: %s | Bits: %u",
           card_no.c_str(), type.c_str(), colour.c_str(), (unsigned) bits);
}

void PaxtonReader::publish_error_(const char *msg) {
  // Defer publishing to loop() to avoid watchdog timeout
  pending_error_ = true;
  pending_error_msg_ = msg;
  pending_bits_ = bit_count_;
  publish_step_ = 1;  // Start deferred publish

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

  // Handle deferred publishing with aggressive watchdog resets
  if (publish_step_ > 0) {
    // Reset watchdog before AND after each publish
    esp_task_wdt_reset();

    if (pending_error_) {
      // Error publishing
      switch (publish_step_) {
        case 1:
          if (card_type_ts) {
            esp_task_wdt_reset();
            card_type_ts->publish_state("Error");
            esp_task_wdt_reset();
          }
          break;
        case 2:
          if (last_card_ts) {
            esp_task_wdt_reset();
            last_card_ts->publish_state(pending_error_msg_);
            esp_task_wdt_reset();
          }
          break;
        case 3:
          if (bit_count_s) {
            esp_task_wdt_reset();
            bit_count_s->publish_state((float) pending_bits_);
            esp_task_wdt_reset();
          }
          publish_step_ = 0;  // Done
          return;
      }
    } else {
      // Success publishing
      switch (publish_step_) {
        case 1:
          ESP_LOGD("paxton", "Deferred publish step 1: raw_bits");
          if (raw_bits_ts) {
            esp_task_wdt_reset();
            raw_bits_ts->publish_state(pending_bin_);
            esp_task_wdt_reset();
          }
          break;
        case 2:
          ESP_LOGD("paxton", "Deferred publish step 2: last_card");
          if (last_card_ts) {
            esp_task_wdt_reset();
            last_card_ts->publish_state(pending_card_no_);
            esp_task_wdt_reset();
          }
          break;
        case 3:
          if (card_type_ts) {
            esp_task_wdt_reset();
            card_type_ts->publish_state(pending_type_);
            esp_task_wdt_reset();
          }
          break;
        case 4:
          if (card_colour_ts) {
            esp_task_wdt_reset();
            card_colour_ts->publish_state(pending_colour_);
            esp_task_wdt_reset();
          }
          break;
        case 5:
          if (bit_count_s) {
            esp_task_wdt_reset();
            bit_count_s->publish_state(pending_bits_);
            esp_task_wdt_reset();
          }
          break;
        case 6:
          if (reading_bs) {
            esp_task_wdt_reset();
            reading_bs->publish_state(true);
            esp_task_wdt_reset();
          }
          break;
        case 7:
          if (reading_bs) {
            esp_task_wdt_reset();
            reading_bs->publish_state(false);
            esp_task_wdt_reset();
          }
          publish_step_ = 0;  // Done
          return;
      }
    }
    publish_step_++;
    esp_task_wdt_reset();  // Reset after incrementing step
    return;  // Process one publish per loop iteration
  }

  if (bit_count_ > 0) {
    if ((micros() - last_edge_us_) > frame_gap_us) {  // frame complete
      processing_ = true;
      uint16_t n = bit_count_;

      // Handle double-reads: if we got exactly 2x the expected bits, use first half
      if (n == net2_bits * 2) {
        ESP_LOGD("paxton", "Detected double-read (150 bits), using first 75");
        n = net2_bits;
        bit_count_ = n;
      } else if (n == switch2_bits * 2) {
        ESP_LOGD("paxton", "Detected double-read (440 bits), using first 220");
        n = switch2_bits;
        bit_count_ = n;
      } else if (n == paxton90_bits * 2) {
        ESP_LOGD("paxton", "Detected double-read (180 bits), using first 90");
        n = paxton90_bits;
        bit_count_ = n;
      }

      log_bits_preview_(n);

      // Build bitstring for later publishing
      std::string bin; bin.reserve(n);
      for (int i = 0; i < n; i++) bin.push_back(bits_[i] ? '1' : '0');

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
      } else if (n == paxton90_bits) {
        ok = parse_paxton90_(card_no, colour, bin);
        if (ok) {
          // Note: 90-bit cards output raw encoded digits (not actual card number)
          // Paxton uses proprietary encoding - raw digits can still be used for card identification
          publish_success_(card_no, "Paxton 90-bit (raw)", colour, n, bin);
        } else {
          publish_error_("Paxton 90-bit parse fail (see raw_bits sensor)");
        }
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

  // For 90-bit cards, try wider range of start positions
  std::vector<int> start_positions = (N == 90) ?
    std::vector<int>{11, 12, 13, 14, 15, 16, 17, 18, 19, 20} :
    std::vector<int>{11, 12, 13, 14, 15};

  for (int s : start_positions) {
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
