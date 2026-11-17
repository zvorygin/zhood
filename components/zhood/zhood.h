#pragma once

#include <array>

#include "esphome/components/button/button.h"
#include "esphome/components/fan/fan.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/text/text.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome::zhood {

/// Longest frame logged on one line; longer runs are split over several lines.
static constexpr size_t MAX_FRAME_LEN = 64;

/// Hood frame: CD <13 data bytes> <checksum> DE.
static constexpr size_t FRAME_LEN = 16;
static constexpr size_t PAYLOAD_LEN = 13;
static constexpr uint8_t FRAME_START = 0xCD;
static constexpr uint8_t FRAME_END = 0xDE;

/// Panel frame fields.
static constexpr size_t PANEL_FAN_BYTE = 2;
static constexpr size_t PANEL_LIGHT_BYTE = 3;
static constexpr uint8_t FAN_SPEEDS = 6;
static constexpr uint8_t LIGHT_LEVELS = 3;

/// Remote keys, sent to the panel in byte 3 of a board frame (payload 01 04 <key>).
enum RemoteKey : uint8_t {
  REMOTE_KEY_FAN = 0x01,     // off -> 1 -> ... -> 6 -> 1
  REMOTE_KEY_TOGGLE = 0x02,  // off <-> last fan and light state
  REMOTE_KEY_OFF = 0x03,     // fan and light off
  REMOTE_KEY_LIGHT = 0x04,   // off -> 1 -> 2 -> 3 -> off
};

/// Options of the direction select, in order.
enum InjectDirection : uint8_t { INJECT_TO_PANEL = 0, INJECT_TO_BOARD = 1 };
/// Options of the mode select, in order.
enum InjectMode : uint8_t { INJECT_REPLACE = 0, INJECT_EXTRA = 1 };

/// One direction of the proxy: bytes read from `from` are written to `to`.
struct ZhoodLink {
  ZhoodLink(uart::UARTComponent *from, uart::UARTComponent *to, const char *label, const char *inject_label)
      : from(from), to(to), label(label), inject_label(inject_label) {}

  uart::UARTDevice from;
  uart::UARTDevice to;
  const char *label;
  const char *inject_label;
  std::array<uint8_t, MAX_FRAME_LEN> frame{};
  size_t frame_len{0};
  uint32_t last_rx_ms{0};

  // Frame waiting to be injected towards `to`, and how many more times to send it.
  std::array<uint8_t, FRAME_LEN> inject_frame{};
  uint32_t inject_remaining{0};
  InjectMode inject_mode{INJECT_REPLACE};
  uint32_t inject_queued_ms{0};
  // True while the real frame being received is dropped because an injected one replaced it.
  bool dropping{false};
};

/// Sits between the hood's main board and control panel, passes all traffic through and logs it.
/// It can also inject frames towards either side.
class ZhoodComponent : public Component {
 public:
  ZhoodComponent(uart::UARTComponent *board_uart, uart::UARTComponent *panel_uart, uint32_t frame_gap_ms)
      : board_to_panel_(board_uart, panel_uart, "board>panel", "inject>panel"),
        panel_to_board_(panel_uart, board_uart, "panel>board", "inject>board"),
        frame_gap_ms_(frame_gap_ms) {}

  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_payload_text(text::Text *t) { this->payload_text_ = t; }
  void set_direction_select(select::Select *s) { this->direction_select_ = s; }
  void set_mode_select(select::Select *s) { this->mode_select_ = s; }
  void set_count_number(number::Number *n) { this->count_number_ = n; }
  void set_fan(fan::Fan *f) { this->fan_ = f; }
  void set_light_state(light::LightState *l) { this->light_state_ = l; }

  /// Called by the fan entity. Turns its wish into a fan speed target (0 = off).
  void on_fan_command(const fan::FanCall &call);
  /// Called by the light entity with the level it wants (0 = off).
  void on_light_command(uint8_t level);

  /// Called by the inject button: sends the payload text with the chosen direction, mode and count.
  void on_inject_button_press();
  /// Queues a frame built from `payload` (up to 13 bytes, zero padded) towards one side.
  void inject(InjectDirection direction, InjectMode mode, const uint8_t *payload, size_t len, uint32_t count);

  /// Parses hex text into payload bytes. Accepts "01 04", "0104", "0x01,0x04", or a whole
  /// 16-byte frame "CD .. DE" (its markers and checksum are dropped). Returns false if invalid.
  static bool parse_payload(const std::string &text, std::array<uint8_t, PAYLOAD_LEN> &out, size_t &len);
  /// Sum of bytes 0..13, XOR 0x55.
  static uint8_t checksum(const uint8_t *frame);

 protected:
  void forward_(ZhoodLink &link, uint32_t now);
  void send_injected_(ZhoodLink &link);
  void log_frame_(ZhoodLink &link);
  /// Reads fan and light from a whole panel frame, then runs the controller.
  void on_panel_frame_(const uint8_t *frame);
  /// Presses the next remote key that moves the hood towards the target, or finishes.
  void step_controller_();
  void press_key_(RemoteKey key);
  /// Copies the hood state to the fan and light entities.
  void publish_entities_();
  /// Starts a command from the current state, so the part not being changed stays as it is.
  bool begin_command_();

  ZhoodLink board_to_panel_;
  ZhoodLink panel_to_board_;
  uint32_t frame_gap_ms_;

  text::Text *payload_text_{nullptr};
  select::Select *direction_select_{nullptr};
  select::Select *mode_select_{nullptr};
  number::Number *count_number_{nullptr};
  fan::Fan *fan_{nullptr};
  light::LightState *light_state_{nullptr};

  // Hood state from the latest panel frame; false until the first one arrives.
  bool state_known_{false};
  uint8_t fan_speed_{0};
  uint8_t light_level_{0};
  // Controller: the state Home Assistant asked for. Busy while it differs from the hood state.
  bool busy_{false};
  uint8_t target_fan_{0};
  uint8_t target_light_{0};
  uint8_t presses_{0};
  uint8_t settle_frames_{0};
  // Last values given to the entities, so only changes are published.
  int8_t published_fan_{-1};
  int8_t published_light_{-1};
};

class ZhoodInjectButton : public button::Button, public Parented<ZhoodComponent> {
 protected:
  void press_action() override { this->parent_->on_inject_button_press(); }
};

class ZhoodFan : public fan::Fan, public Parented<ZhoodComponent> {
 public:
  fan::FanTraits get_traits() override { return fan::FanTraits(false, true, false, FAN_SPEEDS); }

 protected:
  void control(const fan::FanCall &call) override { this->parent_->on_fan_command(call); }
};

/// Light with 3 levels: brightness up to 1/3 is level 1, about 2/3 is level 2, full is level 3.
class ZhoodLight : public light::LightOutput, public Parented<ZhoodComponent> {
 public:
  light::LightTraits get_traits() override;
  void write_state(light::LightState *state) override;
};

/// Payload field. Only accepts text that parses as a payload.
class ZhoodPayloadText : public text::Text {
 protected:
  void control(const std::string &value) override;
};

/// Select that just keeps the chosen option.
class ZhoodSelect : public select::Select {
 protected:
  void control(size_t index) override { this->publish_state(index); }
};

/// Number that just keeps the entered value.
class ZhoodNumber : public number::Number {
 protected:
  void control(float value) override { this->publish_state(value); }
};

}  // namespace esphome::zhood
