#include "paxton_reader.h"
#include "driver/gpio.h"  // <-- IDF GPIO
#include <algorithm>      // for std::reverse

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

// Proper Net2 decoder with column/row parity (from Paxtogeddon)
bool PaxtonReader::parse_net2_proper_(std::string &card_no, int start_pos, int num_digits) {
  // Net2 uses groups of 5 bits: 4 data bits (BCD, LSB first) + 1 row parity (odd)
  // Column parity (even) is checked at the LRC position

  std::string result;
  result.reserve(num_digits);
  int LRC[4] = {0, 0, 0, 0};

  // Process groups of 5 bits
  for (int i = start_pos; i < bit_count_ - 10; i += 5) {
    if (i + 4 >= bit_count_ - 10) break;

    // Get 5 bits: b0-b3 are data (LSB first), b4 is row parity
    int b0 = bits_[i + 0];
    int b1 = bits_[i + 1];
    int b2 = bits_[i + 2];
    int b3 = bits_[i + 3];
    int b4 = bits_[i + 4];

    // BCD value (LSB first, like Paxtogeddon)
    int dval = 8*b3 + 4*b2 + 2*b1 + 1*b0;

    // Check row parity (odd): sum of data bits should make parity bit odd
    int rowParity = (b0 + b1 + b2 + b3) % 2 == 0 ? 1 : 0;
    if (rowParity != b4) {
      ESP_LOGD("paxton", "Net2: Row parity fail at bit %d", i);
      return false;
    }

    // Start bits check (position 10 in standard Net2 = start_pos in our case)
    if (i == start_pos && dval != 11) {
      ESP_LOGD("paxton", "Net2: Start bits != 11 (got %d)", dval);
      return false;
    }

    // Accumulate column parity (skip LRC group itself)
    int lrc_pos = start_pos + (num_digits + 2) * 5; // +2 for start/stop groups
    if (i < lrc_pos) {
      LRC[0] += b0;
      LRC[1] += b1;
      LRC[2] += b2;
      LRC[3] += b3;
    }

    // Check LRC (column parity, even)
    if (i == lrc_pos) {
      int c0 = LRC[0] % 2 == 0 ? 0 : 1;
      int c1 = LRC[1] % 2 == 0 ? 0 : 1;
      int c2 = LRC[2] % 2 == 0 ? 0 : 1;
      int c3 = LRC[3] % 2 == 0 ? 0 : 1;

      if (!(c0 == b0 && c1 == b1 && c2 == b2 && c3 == b3)) {
        ESP_LOGD("paxton", "Net2: Column parity (LRC) fail");
        return false;
      }
    }

    // Collect card digits (skip start/stop groups)
    int digit_start = start_pos + 5;  // First digit after start bits
    int digit_end = start_pos + (num_digits + 1) * 5;  // Before stop bits
    if (i >= digit_start && i < digit_end) {
      if (dval > 9) {
        ESP_LOGD("paxton", "Net2: Invalid BCD digit %d at bit %d", dval, i);
        return false;
      }
      result.push_back(char('0' + dval));
    }
  }

  if ((int)result.length() == num_digits) {
    card_no = result;
    return true;
  }

  return false;
}

bool PaxtonReader::parse_paxton90_(std::string &card_no, std::string &colour, std::string &bin) {
  const uint16_t n = bit_count_;
  if (n != paxton90_bits) return false;
  if (!check_leadin_10zeros_ending_one_()) return false;
  if (!check_leadout_10zeros_()) return false;

  bin.reserve(n);
  for (int i = 0; i < n; i++) bin.push_back(bits_[i] ? '1' : '0');

  // 90-bit format: 10 zeros + 1 + 69 data bits + 10 zeros
  // Data section is bits 11-79 (69 bits)
  // Encoding is complex - try multiple strategies and log ALL valid results

  // Scan all possible decodings for fallback (no longer logged verbosely)
  std::vector<std::string> all_valid;
  struct DecoderResult {
    std::string card;
    int start;
    int group;
    bool lsb;
  };
  std::vector<DecoderResult> results;

  // Try all combinations and collect ALL valid results
  for (int group_size = 4; group_size <= 9; group_size++) {
    for (int start = 11; start <= 20 && start + group_size * 8 < n - 10; start++) {
      // Try MSB first
      std::string candidate_msb;
      bool valid_msb = true;

      for (int d = 0; d < 8; d++) {
        int idx = start + d * group_size;
        if (idx + 3 >= n - 10) { valid_msb = false; break; }
        int val = (bits_[idx] << 3) | (bits_[idx+1] << 2) | (bits_[idx+2] << 1) | (bits_[idx+3]);
        if (val > 9) { valid_msb = false; break; }
        candidate_msb.push_back(char('0' + val));
      }

      if (valid_msb && candidate_msb.length() == 8) {
        results.push_back({candidate_msb, start, group_size, false});
        // Also try reversed digit order
        std::string reversed_msb = candidate_msb;
        std::reverse(reversed_msb.begin(), reversed_msb.end());
        results.push_back({reversed_msb + " (rev)", start, group_size, false});
      }

      // Try LSB first (reversed nibble)
      std::string candidate_lsb;
      bool valid_lsb = true;

      for (int d = 0; d < 8; d++) {
        int idx = start + d * group_size;
        if (idx + 3 >= n - 10) { valid_lsb = false; break; }
        int val = (bits_[idx+3] << 3) | (bits_[idx+2] << 2) | (bits_[idx+1] << 1) | (bits_[idx]);
        if (val > 9) { valid_lsb = false; break; }
        candidate_lsb.push_back(char('0' + val));
      }

      if (valid_lsb && candidate_lsb.length() == 8) {
        results.push_back({candidate_lsb, start, group_size, true});
        // Also try reversed digit order
        std::string reversed_lsb = candidate_lsb;
        std::reverse(reversed_lsb.begin(), reversed_lsb.end());
        results.push_back({reversed_lsb + " (rev)", start, group_size, true});
      }
    }
  }

  // Verbose decoding results removed - only log when a decoder succeeds

  // Try sequential decoder with skipped positions (for cheap/aftermarket cards)
  // Pattern: [15, 30, 35, 40, 45, 50, 55, 60] - skips positions 20 and 25
  std::vector<int> cheap_positions = {15, 30, 35, 40, 45, 50, 55, 60};
  std::string cheap_result;
  bool cheap_valid = true;

  for (int pos : cheap_positions) {
    if (pos + 3 < n - 10) {
      int b0 = bits_[pos];
      int b1 = bits_[pos+1];
      int b2 = bits_[pos+2];
      int b3 = bits_[pos+3];
      int dval = 8*b3 + 4*b2 + 2*b1 + 1*b0;  // LSB first

      if (dval <= 9) {
        cheap_result.push_back(char('0' + dval));
      } else {
        cheap_valid = false;
        break;
      }
    } else {
      cheap_valid = false;
      break;
    }
  }

  // Reject results starting with "01", "02", "03" which are likely incorrect decodings
  bool starts_with_invalid = cheap_result.length() >= 2 &&
                             cheap_result[0] == '0' &&
                             (cheap_result[1] >= '1' && cheap_result[1] <= '3');

  if (cheap_valid && cheap_result.length() == 8 && !starts_with_invalid) {
    ESP_LOGI("paxton", "90-bit: Sequential decoder (cheap cards) succeeded: %s", cheap_result.c_str());
    card_no = cheap_result;
    colour = "None";
    return true;
  } else if (cheap_valid && cheap_result.length() == 8 && starts_with_invalid) {
    ESP_LOGD("paxton", "90-bit: Sequential decoder result '%s' rejected (starts with 01/02/03), trying Switch2 Fob decoder",
             cheap_result.c_str());
  }

  // Try Switch2 Fob-style interleaved decoding (for official Paxton 90-bit cards)
  // Discovered pattern: positions [35,40,45,55,50,25,35,30] from analysis of real cards
  std::vector<int> fob_positions = {35, 40, 45, 55, 50, 25, 35, 30};
  std::string fob_result;
  bool fob_valid = true;

  for (int pos : fob_positions) {
    if (pos + 3 < n - 10) {
      int b0 = bits_[pos];
      int b1 = bits_[pos+1];
      int b2 = bits_[pos+2];
      int b3 = bits_[pos+3];
      int dval = 8*b3 + 4*b2 + 2*b1 + 1*b0;  // LSB first

      if (dval <= 9) {
        fob_result.push_back(char('0' + dval));
      } else {
        fob_valid = false;
        break;
      }
    } else {
      fob_valid = false;
      break;
    }
  }

  if (fob_valid && fob_result.length() == 8) {
    ESP_LOGI("paxton", "90-bit: Switch2 Fob-style decoder succeeded: %s", fob_result.c_str());
    card_no = fob_result;
    colour = "None";
    return true;
  }

  // Try proper Net2 decoder with parity checking (from Paxtogeddon)
  // 90-bit format might be extended Net2, try different start positions
  for (int start : {10, 11, 12, 13}) {
    for (int digits : {8, 9, 10}) {
      if (parse_net2_proper_(card_no, start, digits)) {
        ESP_LOGI("paxton", "90-bit: Net2 parity decoder succeeded at start=%d, digits=%d: %s",
                 start, digits, card_no.c_str());
        colour = "None";
        return true;
      }
    }
  }

  // Try adaptive BCD as fallback
  if (try_adaptive_bcd_(card_no)) {
    ESP_LOGI("paxton", "90-bit: Adaptive BCD succeeded: %s", card_no.c_str());
    colour = "None";
    return true;
  }

  // Return the first simple result if adaptive failed
  if (!results.empty()) {
    card_no = results[0].card;
    colour = "None";
    ESP_LOGW("paxton", "90-bit: Using first simple result: %s (may be incorrect!)", card_no.c_str());
    return true;
  }

  ESP_LOGW("paxton", "90-bit: No valid decoder found. Raw bits published to raw_bits sensor.");
  return false;
}

void PaxtonReader::publish_success_(const std::string &card_no,
                                    const std::string &type,
                                    const std::string &colour,
                                    uint16_t bits,
                                    const std::string &bin) {
  if (last_card_ts) last_card_ts->publish_state(card_no);
  App.feed_wdt();  // Reset watchdog between publishes
  if (card_type_ts) card_type_ts->publish_state(type);
  App.feed_wdt();
  if (card_colour_ts) card_colour_ts->publish_state(colour);
  App.feed_wdt();
  if (bit_count_s) bit_count_s->publish_state(bits);
  App.feed_wdt();
  if (reading_bs) reading_bs->publish_state(true);
  yield();  // Allow scheduler to run
  if (led_green_ >= 0) led_on_for_(led_green_, 60);
  if (reading_bs) reading_bs->publish_state(false);
  App.feed_wdt();
  ESP_LOGI("paxton", "Card: %s | Type: %s | Colour: %s | Bits: %u",
           card_no.c_str(), type.c_str(), colour.c_str(), (unsigned) bits);
}

void PaxtonReader::publish_error_(const char *msg) {
  if (card_type_ts) card_type_ts->publish_state("Error");
  App.feed_wdt();
  if (last_card_ts) last_card_ts->publish_state(msg);
  App.feed_wdt();
  if (bit_count_s) bit_count_s->publish_state((float) bit_count_);
  App.feed_wdt();
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
      if (raw_bits_ts) {
        raw_bits_ts->publish_state(bin);
        App.feed_wdt();  // Reset watchdog after publishing
        yield();         // Allow other tasks to run
      }

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
          publish_success_(card_no, "Paxton 90-bit", colour, n, bin);
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
