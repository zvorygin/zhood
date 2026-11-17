"""Pass-through proxy between a range hood's main board and its control panel.

The ESP32 sits on both UART lines: every byte from one side is written to the
other side unchanged, and the traffic is logged one frame per line. A frame
ends when a side stays silent for `frame_gap`.

With `inject:` set, Home Assistant gets a payload field, direction and mode
selects, a count and a button that sends `CD <payload> <checksum> DE` to the
chosen side.
"""

import esphome.codegen as cg
from esphome.components import button, fan, light, number, select, text, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_DISABLED_BY_DEFAULT,
    CONF_ID,
    CONF_OUTPUT_ID,
    ENTITY_CATEGORY_CONFIG,
)
from esphome.types import ConfigType

CODEOWNERS = ["@zvorygin"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["button", "fan", "light", "number", "select", "text"]
MULTI_CONF = False

CONF_BOARD_UART_ID = "board_uart_id"
CONF_PANEL_UART_ID = "panel_uart_id"
CONF_FRAME_GAP = "frame_gap"
CONF_INJECT = "inject"
CONF_FAN = "fan"
CONF_LIGHT = "light"
CONF_PAYLOAD = "payload"
CONF_DIRECTION = "direction"
CONF_MODE = "mode"
CONF_COUNT = "count"
CONF_BUTTON = "button"

# Order must match InjectDirection and InjectMode in zhood.h.
DIRECTION_OPTIONS = ["To panel", "To board"]
MODE_OPTIONS = ["Replace next frame", "Extra frame"]
MAX_INJECT_COUNT = 100
# Room for a whole pasted frame: 16 bytes as "CD 00 ..." is 47 characters.
MAX_PAYLOAD_TEXT = 64

BAUD_RATE = 4800

zhood_ns = cg.esphome_ns.namespace("zhood")
ZhoodComponent = zhood_ns.class_("ZhoodComponent", cg.Component)
ZhoodInjectButton = zhood_ns.class_("ZhoodInjectButton", button.Button)
ZhoodPayloadText = zhood_ns.class_("ZhoodPayloadText", text.Text)
ZhoodSelect = zhood_ns.class_("ZhoodSelect", select.Select)
ZhoodNumber = zhood_ns.class_("ZhoodNumber", number.Number)
ZhoodFan = zhood_ns.class_("ZhoodFan", fan.Fan)
ZhoodLight = zhood_ns.class_("ZhoodLight", light.LightOutput)

def _hidden(schema: cv.Schema) -> cv.Schema:
    """Disabled in Home Assistant until enabled there, and kept in the device's Configuration section."""
    return schema.extend(
        {cv.Optional(CONF_DISABLED_BY_DEFAULT, default=True): cv.boolean}
    )


INJECT_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_PAYLOAD, default={"name": "Inject Payload"}): _hidden(
            text.text_schema(
                ZhoodPayloadText,
                icon="mdi:code-brackets",
                mode="TEXT",
                entity_category=ENTITY_CATEGORY_CONFIG,
            )
        ),
        cv.Optional(CONF_DIRECTION, default={"name": "Inject Direction"}): _hidden(
            select.select_schema(
                ZhoodSelect,
                icon="mdi:swap-horizontal",
                entity_category=ENTITY_CATEGORY_CONFIG,
            )
        ),
        cv.Optional(CONF_MODE, default={"name": "Inject Mode"}): _hidden(
            select.select_schema(
                ZhoodSelect,
                icon="mdi:swap-vertical",
                entity_category=ENTITY_CATEGORY_CONFIG,
            )
        ),
        cv.Optional(
            CONF_COUNT, default={"name": "Inject Count", "mode": "BOX"}
        ): _hidden(
            number.number_schema(
                ZhoodNumber, icon="mdi:counter", entity_category=ENTITY_CATEGORY_CONFIG
            )
        ),
        cv.Optional(CONF_BUTTON, default={"name": "Inject"}): _hidden(
            button.button_schema(
                ZhoodInjectButton,
                icon="mdi:send",
                entity_category=ENTITY_CATEGORY_CONFIG,
            )
        ),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ZhoodComponent),
        cv.Required(CONF_BOARD_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_PANEL_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(
            CONF_FRAME_GAP, default="20ms"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_INJECT): INJECT_SCHEMA,
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

    if (inject := config.get(CONF_INJECT)) is not None:
        payload = await text.new_text(inject[CONF_PAYLOAD], max_length=MAX_PAYLOAD_TEXT)
        cg.add(var.set_payload_text(payload))
        direction = await select.new_select(inject[CONF_DIRECTION], options=DIRECTION_OPTIONS)
        cg.add(var.set_direction_select(direction))
        mode = await select.new_select(inject[CONF_MODE], options=MODE_OPTIONS)
        cg.add(var.set_mode_select(mode))
        count = await number.new_number(
            inject[CONF_COUNT], min_value=1, max_value=MAX_INJECT_COUNT, step=1
        )
        cg.add(var.set_count_number(count))
        inject_btn = await button.new_button(inject[CONF_BUTTON])
        cg.add(inject_btn.set_parent(var))

    if (fan_config := config.get(CONF_FAN)) is not None:
        fan_var = await fan.new_fan(fan_config)
        cg.add(fan_var.set_parent(var))
        cg.add(var.set_fan(fan_var))

    if (light_config := config.get(CONF_LIGHT)) is not None:
        light_output = await light.new_light(light_config)
        cg.add(light_output.set_parent(var))
        light_state = await cg.get_variable(light_config[CONF_ID])
        cg.add(var.set_light_state(light_state))

