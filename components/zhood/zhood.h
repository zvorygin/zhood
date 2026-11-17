#pragma once

#include <array>

#include "esphome/components/fan/fan.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome::zhood {

/// Longest frame logged on one line; longer runs are split over several lines.
static constexpr size_t MAX_FRAME_LEN = 64;

/// Hood frame: CD <13 data bytes> <checksum> DE.
static constexpr size_t FRAME_LEN = 16;
static constexpr uint8_t FRAME_START = 0xCD;
static constexpr uint8_t FRAME_END = 0xDE;

/// Panel frame fields.
static constexpr size_t PANEL_FAN_BYTE = 2;
static constexpr size_t PANEL_LIGHT_BYTE = 3;
static constexpr uint8_t FAN_SPEEDS = 6;
static constexpr uint8_t LIGHT_LEVELS = 3;

/// Remote keys, sent to the panel in byte 3 of a board frame (01 04 <key>).
enum RemoteKey : uint8_t {
  REMOTE_KEY_FAN = 0x01,     // off -> 1 -> ... -> 6 -> 1
  REMOTE_KEY_TOGGLE = 0x02,  // off <-> last fan and light state
  REMOTE_KEY_OFF = 0x03,     // fan and light off
  REMOTE_KEY_LIGHT = 0x04,   // off -> 1 -> 2 -> 3 -> off
};

/// One direction of the proxy: bytes read from `from` are written to `to`.
struct ZhoodLink {
  ZhoodLink(uart::UARTComponent *from, uart::UARTComponent *to, const char *label)
      : from(from), to(to), label(label) {}

  uart::UARTDevice from;
  uart::UARTDevice to;
  const char *label;
  std::array<uint8_t, MAX_FRAME_LEN> frame{};
  size_t frame_len{0};
  uint32_t last_rx_ms{0};

  // Frame that replaces the next real frames towards `to`, and how many more times to send it.
  std::array<uint8_t, FRAME_LEN> inject_frame{};
  uint32_t inject_remaining{0};
  uint32_t inject_queued_ms{0};
  // True while the real frame being received is dropped because an injected one replaced it.
  bool dropping{false};
};

/// Sits between the hood's main board and control panel, passes all traffic through and logs it.
/// Fan and light commands are sent to the panel as remote key presses.
class ZhoodComponent : public Component {
 public:
  ZhoodComponent(uart::UARTComponent *board_uart, uart::UARTComponent *panel_uart, uint32_t frame_gap_ms)
      : board_to_panel_(board_uart, panel_uart, "board>panel"),
        panel_to_board_(panel_uart, board_uart, "panel>board"),
        frame_gap_ms_(frame_gap_ms) {}

  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_fan(fan::Fan *f) { this->fan_ = f; }
  void set_light_state(light::LightState *l) { this->light_state_ = l; }

  /// Called by the fan entity. Turns its wish into a fan speed target (0 = off).
  void on_fan_command(const fan::FanCall &call);
  /// Called by the light entity with the level it wants (0 = off).
  void on_light_command(uint8_t level);

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

}  // namespace esphome::zhood
