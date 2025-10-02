# esphome/components/paxton_reader/__init__.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor, sensor, binary_sensor
from esphome import pins
from esphome.const import CONF_ID   # <-- add this import

DEPENDENCIES = ["logger"]
AUTO_LOAD = []

paxton_ns = cg.esphome_ns.namespace("paxton")
PaxtonReader = paxton_ns.class_("PaxtonReader", cg.Component)

# ... (constants unchanged)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(PaxtonReader),
        # pins/entities...
    }
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])  # <-- use CONF_ID from esphome.const

    clock = await cg.gpio_pin_expression(config[CONF_CLOCK_PIN])
    data  = await cg.gpio_pin_expression(config[CONF_DATA_PIN])
    cg.add(var.set_clock_pin(clock))
    cg.add(var.set_data_pin(data))

    if CONF_LED_GREEN in config:
        ledg = await cg.gpio_pin_expression(config[CONF_LED_GREEN])
        cg.add(var.set_led_green(ledg))
    if CONF_LED_YELLOW in config:
        ledy = await cg.gpio_pin_expression(config[CONF_LED_YELLOW])
        cg.add(var.set_led_yellow(ledy))
    if CONF_LED_RED in config:
        ledr = await cg.gpio_pin_expression(config[CONF_LED_RED])
        cg.add(var.set_led_red(ledr))

    last_card   = await cg.get_variable(config[CONF_LAST_CARD])
    card_type   = await cg.get_variable(config[CONF_CARD_TYPE])
    card_colour = await cg.get_variable(config[CONF_CARD_COLOUR])
    bit_count   = await cg.get_variable(config[CONF_BIT_COUNT])
    reading     = await cg.get_variable(config[CONF_READING])

    cg.add(var.set_last_card(last_card))
    cg.add(var.set_card_type(card_type))
    cg.add(var.set_card_colour(card_colour))
    cg.add(var.set_bit_count(bit_count))
    cg.add(var.set_reading(reading))

    cg.add(var.set_net2_bits(config[CONF_NET2_BITS]))
    cg.add(var.set_switch2_bits(config[CONF_SWITCH2_BITS]))
    cg.add(var.set_debounce_us(config[CONF_DEBOUNCE_US]))

    await cg.register_component(var, {})
