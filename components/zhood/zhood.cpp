#include "zhood.h"

#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cmath>

namespace esphome::zhood {

static const char *const TAG = "zhood";

/// If a side sends nothing for this long, a pending injection goes out anyway.
static constexpr uint32_t INJECT_IDLE_MS = 500;
static constexpr uint32_t MAX_INJECT_COUNT = 100;

/// Board frames that carry one remote key press. 3 is what worked in manual tests.
static constexpr uint8_t KEY_FRAMES = 3;
/// Panel frames to wait after a key press before checking its effect.
static constexpr uint8_t SETTLE_FRAMES = 1;
/// A command gives up after this many key presses (e.g. someone keeps using the hood's buttons).
static constexpr uint8_t MAX_PRESSES = 20;
/// The panel's frame type once it has talked to the board.
static constexpr uint8_t PANEL_FRAME_TYPE = 0x04;

static const char *const DIRECTION_NAMES[] = {"panel", "board"};
static const char *const MODE_NAMES[] = {"replace next frame", "extra frame"};

void ZhoodComponent::setup() {
  // Drop whatever arrived before the proxy started, so the first logged frame is a whole one.
  this->board_to_panel_.from.discard_available();
  this->panel_to_board_.from.discard_available();

  if (this->direction_select_ != nullptr && !this->direction_select_->has_state())
    this->direction_select_->publish_state(static_cast<size_t>(INJECT_TO_PANEL));
  if (this->mode_select_ != nullptr && !this->mode_select_->has_state())
    this->mode_select_->publish_state(static_cast<size_t>(INJECT_REPLACE));
  if (this->count_number_ != nullptr && !this->count_number_->has_state())
    this->count_number_->publish_state(1);
}

void ZhoodComponent::loop() {
  const uint32_t now = App.get_loop_component_start_time();
  this->forward_(this->board_to_panel_, now);
  this->forward_(this->panel_to_board_, now);
}

void ZhoodComponent::forward_(ZhoodLink &link, uint32_t now) {
  if (link.from.available() > 0) {
    // A new real frame starts while a replacement is pending: send ours instead and drop the real one.
    if (link.frame_len == 0 && !link.dropping && link.inject_remaining > 0 && link.inject_mode == INJECT_REPLACE) {
      link.dropping = true;
      this->send_injected_(link);
    }
    link.from.read_available_chunks([this, &link](const uint8_t *data, size_t len) {
      // Pass through first, so logging never delays the hood's own traffic.
      if (!link.dropping)
        link.to.write_array(data, len);
      for (size_t i = 0; i < len; i++) {
        link.frame[link.frame_len++] = data[i];
        if (link.frame_len == link.frame.size())
          this->log_frame_(link);
      }
      return true;
    });
    link.last_rx_ms = now;
    return;
  }

  if (link.frame_len > 0 && now - link.last_rx_ms >= this->frame_gap_ms_) {
    this->log_frame_(link);
    link.dropping = false;
    // The real frame just ended: an extra frame goes right behind it.
    if (link.inject_remaining > 0 && link.inject_mode == INJECT_EXTRA)
      this->send_injected_(link);
    return;
  }

  // The sending side is silent: inject anyway, at most once per INJECT_IDLE_MS.
  if (link.inject_remaining > 0 && link.frame_len == 0 && now - link.last_rx_ms >= INJECT_IDLE_MS &&
      now - link.inject_queued_ms >= INJECT_IDLE_MS)
    this->send_injected_(link);
}

void ZhoodComponent::send_injected_(ZhoodLink &link) {
  link.to.write_array(link.inject_frame.data(), link.inject_frame.size());
  link.inject_remaining--;
  link.inject_queued_ms = App.get_loop_component_start_time();

  char hex[format_hex_pretty_size(FRAME_LEN)];
  format_hex_pretty_to(hex, link.inject_frame.data(), link.inject_frame.size(), ' ');
  ESP_LOGI(TAG, "%s (%u): %s  [%" PRIu32 " left]", link.inject_label, static_cast<unsigned>(FRAME_LEN), hex,
           link.inject_remaining);
}

void ZhoodComponent::log_frame_(ZhoodLink &link) {
  char hex[format_hex_pretty_size(MAX_FRAME_LEN)];
  format_hex_pretty_to(hex, link.frame.data(), link.frame_len, ' ');
  ESP_LOGD(TAG, "%s (%u): %s%s", link.label, static_cast<unsigned>(link.frame_len), hex,
           link.dropping ? LOG_STR_LITERAL("  [dropped, replaced]") : LOG_STR_LITERAL(""));
  if (&link == &this->panel_to_board_ && link.frame_len == FRAME_LEN)
    this->on_panel_frame_(link.frame.data());
  link.frame_len = 0;
}

void ZhoodComponent::on_panel_frame_(const uint8_t *frame) {
  if (frame[0] != FRAME_START || frame[FRAME_LEN - 1] != FRAME_END || frame[FRAME_LEN - 2] != checksum(frame) ||
      frame[1] != PANEL_FRAME_TYPE)
    return;
  const uint8_t fan = frame[PANEL_FAN_BYTE];
  const uint8_t light = frame[PANEL_LIGHT_BYTE];
  if (fan > FAN_SPEEDS || light > LIGHT_LEVELS) {
    ESP_LOGW(TAG, "Panel frame has fan %u, light %u: out of range, ignored", fan, light);
    return;
  }
  this->state_known_ = true;
  this->fan_speed_ = fan;
  this->light_level_ = light;
  if (this->busy_) {
    this->step_controller_();
  } else {
    this->publish_entities_();
  }
}

bool ZhoodComponent::begin_command_() {
  if (!this->state_known_) {
    ESP_LOGW(TAG, "No frame from the panel yet, command ignored");
    return false;
  }
  if (!this->busy_) {
    this->target_fan_ = this->fan_speed_;
    this->target_light_ = this->light_level_;
    this->settle_frames_ = 0;
  }
  this->busy_ = true;
  this->presses_ = 0;
  return true;
}

void ZhoodComponent::on_fan_command(const fan::FanCall &call) {
  if (!this->begin_command_())
    return;
  auto state = call.get_state();
  auto speed = call.get_speed();
  // A speed without a state means "on at this speed", or "off" for speed 0.
  bool on = state.has_value() ? *state : speed.has_value() ? *speed > 0 : this->fan_->state;
  int new_speed = speed.value_or(this->fan_->speed);
  new_speed = std::clamp<int>(new_speed, 1, FAN_SPEEDS);
  this->target_fan_ = on ? new_speed : 0;
  ESP_LOGD(TAG, "Fan command: speed %u", this->target_fan_);

  // Show the wish right away; the real state is published when the command ends.
  this->fan_->state = on;
  if (on)
    this->fan_->speed = new_speed;
  this->fan_->publish_state();
  this->published_fan_ = this->target_fan_;
  this->step_controller_();
}

void ZhoodComponent::on_light_command(uint8_t level) {
  if (!this->state_known_)
    return;
  // Skip repeats, and the echo of a state this component published itself.
  if (this->busy_ ? level == this->target_light_
                  : level == this->light_level_ || level == static_cast<uint8_t>(this->published_light_))
    return;
  if (!this->begin_command_())
    return;
  this->target_light_ = level;
  this->published_light_ = level;
  ESP_LOGD(TAG, "Light command: level %u", level);
  this->step_controller_();
}

void ZhoodComponent::step_controller_() {
  // Wait until the key frames are out, then for the panel to show their effect.
  if (this->board_to_panel_.inject_remaining > 0)
    return;
  if (this->settle_frames_ > 0) {
    this->settle_frames_--;
    return;
  }

  RemoteKey key;
  if (this->target_fan_ == 0 && (this->fan_speed_ > 0 || (this->target_light_ == 0 && this->light_level_ > 0))) {
    // There is no "fan off" key: all off, then the light is brought back below.
    key = REMOTE_KEY_OFF;
  } else if (this->target_fan_ > 0 && this->fan_speed_ != this->target_fan_) {
    key = REMOTE_KEY_FAN;
  } else if (this->light_level_ != this->target_light_) {
    key = REMOTE_KEY_LIGHT;
  } else {
    ESP_LOGI(TAG, "Hood at fan %u, light %u after %u key presses", this->fan_speed_, this->light_level_,
             this->presses_);
    this->busy_ = false;
    this->publish_entities_();
    return;
  }

  if (this->presses_ >= MAX_PRESSES) {
    ESP_LOGW(TAG, "Gave up after %u key presses: wanted fan %u, light %u; hood is at fan %u, light %u", this->presses_,
             this->target_fan_, this->target_light_, this->fan_speed_, this->light_level_);
    this->busy_ = false;
    this->publish_entities_();
    return;
  }
  this->press_key_(key);
}

void ZhoodComponent::press_key_(RemoteKey key) {
  const uint8_t payload[] = {0x01, PANEL_FRAME_TYPE, key};
  this->inject(INJECT_TO_PANEL, INJECT_REPLACE, payload, sizeof(payload), KEY_FRAMES);
  this->presses_++;
  this->settle_frames_ = SETTLE_FRAMES;
}

void ZhoodComponent::publish_entities_() {
  if (this->fan_ != nullptr && this->published_fan_ != this->fan_speed_) {
    this->fan_->state = this->fan_speed_ > 0;
    // Off keeps the last speed, so turning on from Home Assistant goes back to it.
    if (this->fan_speed_ > 0)
      this->fan_->speed = this->fan_speed_;
    this->fan_->publish_state();
    this->published_fan_ = this->fan_speed_;
  }
  if (this->light_state_ != nullptr && this->published_light_ != this->light_level_) {
    auto call = this->light_state_->make_call();
    call.set_state(this->light_level_ > 0);
    if (this->light_level_ > 0)
      call.set_brightness(static_cast<float>(this->light_level_) / LIGHT_LEVELS);
    call.set_transition_length(0);
    call.perform();
    this->published_light_ = this->light_level_;
  }
}

light::LightTraits ZhoodLight::get_traits() {
  light::LightTraits traits;
  traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
  return traits;
}

void ZhoodLight::write_state(light::LightState *state) {
  // The target, not the value partway through a transition.
  const auto &values = state->remote_values;
  uint8_t level = 0;
  if (values.is_on())
    level = std::clamp<long>(lroundf(values.get_brightness() * LIGHT_LEVELS), 1, LIGHT_LEVELS);
  this->parent_->on_light_command(level);
}

uint8_t ZhoodComponent::checksum(const uint8_t *frame) {
  uint8_t sum = 0;
  for (size_t i = 0; i < FRAME_LEN - 2; i++)
    sum += frame[i];
  return sum ^ 0x55;
}

static bool is_separator(char c) { return c == ' ' || c == ',' || c == ':' || c == '-' || c == '\t'; }

bool ZhoodComponent::parse_payload(const std::string &text, std::array<uint8_t, PAYLOAD_LEN> &out, size_t &len) {
  std::array<uint8_t, FRAME_LEN> bytes{};
  size_t n = 0;
  size_t i = 0;
  while (i < text.size()) {
    if (is_separator(text[i])) {
      i++;
      continue;
    }
    // One token: optional 0x prefix, then hex digits.
    size_t start = i;
    while (i < text.size() && !is_separator(text[i]))
      i++;
    if (i - start > 2 && text[start] == '0' && (text[start + 1] == 'x' || text[start + 1] == 'X'))
      start += 2;
    const char *token = text.c_str() + start;
    const size_t token_len = i - start;
    // "1" or "01" is one byte; longer tokens like "0104" are read two digits at a time.
    if (token_len != 1 && token_len % 2 != 0)
      return false;
    for (size_t j = 0; j < token_len; j += 2) {
      auto byte = parse_hex<uint8_t>(token + j, token_len == 1 ? 1 : 2);
      if (!byte.has_value() || n == bytes.size())
        return false;
      bytes[n++] = *byte;
    }
  }

  const uint8_t *data = bytes.data();
  // A whole frame copied from the log: keep only its data bytes.
  if (n == FRAME_LEN && bytes[0] == FRAME_START && bytes[FRAME_LEN - 1] == FRAME_END) {
    data = bytes.data() + 1;
    n = PAYLOAD_LEN;
  }
  if (n == 0 || n > PAYLOAD_LEN)
    return false;
  out.fill(0);
  std::copy(data, data + n, out.begin());
  len = n;
  return true;
}

void ZhoodComponent::inject(InjectDirection direction, InjectMode mode, const uint8_t *payload, size_t len,
                            uint32_t count) {
  ZhoodLink &link = direction == INJECT_TO_PANEL ? this->board_to_panel_ : this->panel_to_board_;
  link.inject_frame.fill(0);
  link.inject_frame[0] = FRAME_START;
  std::copy(payload, payload + std::min(len, PAYLOAD_LEN), link.inject_frame.begin() + 1);
  link.inject_frame[FRAME_LEN - 2] = checksum(link.inject_frame.data());
  link.inject_frame[FRAME_LEN - 1] = FRAME_END;
  link.inject_mode = mode;
  link.inject_remaining = count;
  link.inject_queued_ms = App.get_loop_component_start_time();

  char hex[format_hex_pretty_size(FRAME_LEN)];
  format_hex_pretty_to(hex, link.inject_frame.data(), link.inject_frame.size(), ' ');
  ESP_LOGI(TAG, "Queued %" PRIu32 "x to %s (%s): %s", count, DIRECTION_NAMES[direction], MODE_NAMES[mode], hex);
}

void ZhoodComponent::on_inject_button_press() {
  if (this->payload_text_ == nullptr)
    return;
  std::array<uint8_t, PAYLOAD_LEN> payload{};
  size_t len = 0;
  if (!parse_payload(this->payload_text_->state, payload, len)) {
    ESP_LOGW(TAG, "Inject: payload '%s' is not valid hex (1-%u bytes)", this->payload_text_->state.c_str(),
             static_cast<unsigned>(PAYLOAD_LEN));
    return;
  }
  auto direction = INJECT_TO_PANEL;
  if (this->direction_select_ != nullptr && this->direction_select_->active_index().value_or(0) == INJECT_TO_BOARD)
    direction = INJECT_TO_BOARD;
  auto mode = INJECT_REPLACE;
  if (this->mode_select_ != nullptr && this->mode_select_->active_index().value_or(0) == INJECT_EXTRA)
    mode = INJECT_EXTRA;
  uint32_t count = 1;
  if (this->count_number_ != nullptr && this->count_number_->has_state()) {
    float value = this->count_number_->state;
    count = value < 1 ? 1 : value > MAX_INJECT_COUNT ? MAX_INJECT_COUNT : static_cast<uint32_t>(value);
  }
  this->inject(direction, mode, payload.data(), len, count);
}

void ZhoodPayloadText::control(const std::string &value) {
  std::array<uint8_t, PAYLOAD_LEN> payload{};
  size_t len = 0;
  if (!ZhoodComponent::parse_payload(value, payload, len)) {
    ESP_LOGW(TAG, "Payload '%s' rejected: use 1-%u hex bytes like '01 04 00 03', or a whole CD..DE frame",
             value.c_str(), static_cast<unsigned>(PAYLOAD_LEN));
    return;
  }
  this->publish_state(value);
}

void ZhoodComponent::dump_config() {
  ESP_LOGCONFIG(TAG,
                "zhood:\n"
                "  Frame gap: %" PRIu32 " ms\n"
                "  Injection: %s\n"
                "  Fan: %s\n"
                "  Light: %s",
                this->frame_gap_ms_, YESNO(this->payload_text_ != nullptr), YESNO(this->fan_ != nullptr),
                YESNO(this->light_state_ != nullptr));
}

}  // namespace esphome::zhood
