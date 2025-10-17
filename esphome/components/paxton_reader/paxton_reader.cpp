#include "paxton_reader.h"

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

  bool level = digitalRead(data_pin_);
  if (invert_data_) level = !level;

  if (bit_count_ < sizeof(bits_)) bits_[bit_count_++] = level ? 1 : 0;
  else processing_ = true;
}

void PaxtonReader::setup() {
  if (clock_pin_ < 0 || data_pin_ < 0) {
    ESP_LOGE("paxton", "Pins not configured");
    return;
  }
  pinMode(clock_pin_, use_pullups_ ? INPUT_PULLUP : INPUT);
  pinMode(data_pin_,  use_pullups_ ? INPUT_PULLUP : INPUT);

  if (led_green_ >= 0)  pinMode(led_green_, OUTPUT);
  if (led_yellow_ >= 0) pinMode(led_yellow_, OUTPUT);
  if (led_red_ >= 0)    pinMode(led_red_, OUTPUT);

  attachInterruptArg(clock_pin_, &PaxtonReader::isr_trampoline, this, FALLING);

  ESP_LOGI("paxton", "Ready. debounce=%uus frame_gap=%uus invert_data=%s pullups=%s",
           (unsigned)debounce_us, (unsigned)frame_gap_us,
           invert_data_ ? "yes" : "no", use_pullups_ ? "yes" : "no");
}

void PaxtonReader::log_bits_preview_(uint16_t n) {
  const uint16_t show = std::min<uint16_t>(n, 64);
  std::string s; s.reserve(show);
  for (uint16_t i=0;i<show;i++) s.push_back(bits_[i] ? '1' : '0');
  ESP_LOGD("paxton", "Bits(%u) preview[0..%u]: %s%s",
           (unsigned)n, (unsigned)(show-1), s.c_str(), n>show?"...":"");
}

void PaxtonReader::pulse_led_(int pin, uint32_t ms) {
  if (pin < 0) return;
  digitalWrite(pin, HIGH);
  delay(ms);
  digitalWrite(pin, LOW);
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
  int idx = 11;
  for (int d = 0; d < 8; d++) {
    if (idx + 3 >= n) return false;
    int val = (bits_[idx] << 3) | (bits_[idx+1] << 2) | (bits_[idx+2] << 1) | (bits_[idx+3]);
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
    int val = (bits_[idx] << 3) | (bits_[idx+1] << 2) | (bits_[idx+2] << 1) | (bits_[idx+3]);
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
  if (led_green_ >= 0) pulse_led_(led_green_, 80);
  if (reading_bs) reading_bs->publish_state(false);
  ESP_LOGI("paxton", "Card: %s | Type: %s | Colour: %s | Bits: %u",
           card_no.c_str(), type.c_str(), colour.c_str(), (unsigned)bits);
}

void PaxtonReader::publish_error_(const char *msg) {
  if (card_type_ts) card_type_ts->publish_state("Error");
  if (last_card_ts) last_card_ts->publish_state(msg);
  if (bit_count_s) bit_count_s->publish_state((float) bit_count_);
  if (led_red_ >= 0) pulse_led_(led_red_, 120);
  ESP_LOGW("paxton", "Parse error: %s (bits=%u)", msg, (unsigned)bit_count_);
}

void PaxtonReader::loop() {
  static uint32_t idle_since = millis();

  if (bit_count_ > 0) {
    if ((micros() - last_edge_us_) > frame_gap_us) {   // <-- use configurable gap
      processing_ = true;
      const uint16_t n = bit_count_;
      log_bits_preview_(n);                            // <-- show first bits

      std::string card_no, colour, bin;
      bool ok = false;

      if (n == net2_bits) {
        ok = parse_net2_(card_no, bin);
        if (ok) publish_success_(card_no, "Net2", "None", n, bin);
        else publish_error_("Net2 parse fail");
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
      if (!ok && led_yellow_ >= 0) pulse_led_(led_yellow_, 80);
    }
  } else {
    if (millis() - idle_since > 5000) {
      ESP_LOGV("paxton", "idle...");
      idle_since = millis();
    }
  }
}

}  // namespace paxton
}  // namespace esphome
