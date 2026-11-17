"""Pass-through proxy between a range hood's main board and its control panel.

The ESP32 sits on both UART lines: every byte from one side is written to the
other side unchanged, and the traffic is logged one frame per line. A frame
ends when a side stays silent for `frame_gap`.

The optional fan and light entities control the hood by sending the remote
control's key codes to the panel, and read its state from the panel's frames.
"""

import esphome.codegen as cg
from esphome.components import fan, light, uart
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.types import ConfigType

CODEOWNERS = ["@zvorygin"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["fan", "light"]
MULTI_CONF = False

CONF_BOARD_UART_ID = "board_uart_id"
CONF_PANEL_UART_ID = "panel_uart_id"
CONF_FRAME_GAP = "frame_gap"
CONF_FAN = "fan"
CONF_LIGHT = "light"

BAUD_RATE = 4800

zhood_ns = cg.esphome_ns.namespace("zhood")
ZhoodComponent = zhood_ns.class_("ZhoodComponent", cg.Component)
ZhoodFan = zhood_ns.class_("ZhoodFan", fan.Fan)
ZhoodLight = zhood_ns.class_("ZhoodLight", light.LightOutput)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ZhoodComponent),
        cv.Required(CONF_BOARD_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_PANEL_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(
            CONF_FRAME_GAP, default="20ms"
        ): cv.positive_time_period_milliseconds,
        # The hood's own state is read from the panel; nothing is restored at boot.
        cv.Optional(CONF_FAN): fan.fan_schema(
            ZhoodFan, icon="mdi:fan", default_restore_mode="NO_RESTORE"
        ),
        # Levels are switched one key press at a time, so fading makes no sense.
        cv.Optional(CONF_LIGHT): light.light_schema(
            ZhoodLight, light.LightType.BRIGHTNESS_ONLY, default_restore_mode="ALWAYS_OFF"
        ).extend(
            {
                cv.Optional(light.CONF_DEFAULT_TRANSITION_LENGTH, default="0s"): cv.positive_time_period_milliseconds,
                cv.Optional(light.CONF_GAMMA_CORRECT, default=1.0): cv.positive_float,
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


def _uart_check(uart_bus: str) -> cv.Schema:
    return uart.final_validate_device_schema(
        "zhood",
        uart_bus=uart_bus,
        baud_rate=BAUD_RATE,
        require_tx=True,
        require_rx=True,
        parity="NONE",
    )


def _final_validate(config: ConfigType) -> ConfigType:
    if config[CONF_BOARD_UART_ID] == config[CONF_PANEL_UART_ID]:
        raise cv.Invalid(
            f"{CONF_BOARD_UART_ID} and {CONF_PANEL_UART_ID} must be different UARTs"
        )
    _uart_check(CONF_BOARD_UART_ID)(config)
    _uart_check(CONF_PANEL_UART_ID)(config)
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    board = await cg.get_variable(config[CONF_BOARD_UART_ID])
    panel = await cg.get_variable(config[CONF_PANEL_UART_ID])
    var = cg.new_Pvariable(
        config[CONF_ID], board, panel, config[CONF_FRAME_GAP].total_milliseconds
    )
    await cg.register_component(var, config)

    if (fan_config := config.get(CONF_FAN)) is not None:
        fan_var = await fan.new_fan(fan_config)
        cg.add(fan_var.set_parent(var))
        cg.add(var.set_fan(fan_var))

    if (light_config := config.get(CONF_LIGHT)) is not None:
        light_output = await light.new_light(light_config)
        cg.add(light_output.set_parent(var))
        light_state = await cg.get_variable(light_config[CONF_ID])
        cg.add(var.set_light_state(light_state))

