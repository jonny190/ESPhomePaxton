import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor, sensor, binary_sensor
from esphome import pins
from esphome.const import CONF_ID

DEPENDENCIES = ["logger"]
AUTO_LOAD = []

paxton_ns = cg.esphome_ns.namespace("paxton")
PaxtonReader = paxton_ns.class_("PaxtonReader", cg.Component)

CONF_CLOCK_PIN = "clock_pin"
CONF_DATA_PIN = "data_pin"
CONF_LED_GREEN = "led_green"
CONF_LED_YELLOW = "led_yellow"
CONF_LED_RED = "led_red"

CONF_LAST_CARD = "last_card"
CONF_CARD_TYPE = "card_type"
CONF_CARD_COLOUR = "card_colour"
CONF_BIT_COUNT = "bit_count"
CONF_READING = "reading"

CONF_NET2_BITS = "net2_bits"
CONF_SWITCH2_BITS = "switch2_bits"
CONF_DEBOUNCE_US = "debounce_us"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(PaxtonReader),

        cv.Required(CONF_CLOCK_PIN): pins.internal_gpio_input_pin_schema,
        cv.Required(CONF_DATA_PIN): pins.internal_gpio_input_pin_schema,
        cv.Optional(CONF_LED_GREEN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_LED_YELLOW): pins.gpio_output_pin_schema,
        cv.Optional(CONF_LED_RED): pins.gpio_output_pin_schema,

        cv.Required(CONF_LAST_CARD): cv.use_id(text_sensor.TextSensor),
        cv.Required(CONF_CARD_TYPE): cv.use_id(text_sensor.TextSensor),
        cv.Required(CONF_CARD_COLOUR): cv.use_id(text_sensor.TextSensor),
        cv.Required(CONF_BIT_COUNT): cv.use_id(sensor.Sensor),
        cv.Required(CONF_READING): cv.use_id(binary_sensor.BinarySensor),

        cv.Optional(CONF_NET2_BITS, default=75): cv.positive_int,
        cv.Optional(CONF_SWITCH2_BITS, default=220): cv.positive_int,
        cv.Optional(CONF_DEBOUNCE_US, default=350): cv.positive_int,
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    # Pins
    clock = await cg.gpio_pin_expression(config[CONF_CLOCK_PIN])
    data = await cg.gpio_pin_expression(config[CONF_DATA_PIN])
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

    # Entities
    last_card = await cg.get_variable(config[CONF_LAST_CARD])
    card_type = await cg.get_variable(config[CONF_CARD_TYPE])
    card_colour = await cg.get_variable(config[CONF_CARD_COLOUR])
    bit_count = await cg.get_variable(config[CONF_BIT_COUNT])
    reading = await cg.get_variable(config[CONF_READING])

    cg.add(var.set_last_card(last_card))
    cg.add(var.set_card_type(card_type))
    cg.add(var.set_card_colour(card_colour))
    cg.add(var.set_bit_count(bit_count))
    cg.add(var.set_reading(reading))

    # Config values
    cg.add(var.set_net2_bits(config[CONF_NET2_BITS]))
    cg.add(var.set_switch2_bits(config[CONF_SWITCH2_BITS]))
    cg.add(var.set_debounce_us(config[CONF_DEBOUNCE_US]))

    await cg.register_component(var, {})
