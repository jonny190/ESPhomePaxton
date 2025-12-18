import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor, sensor, binary_sensor
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
CONF_RAW_BITS = "raw_bits"

CONF_NET2_BITS = "net2_bits"
CONF_SWITCH2_BITS = "switch2_bits"
CONF_PAXTON90_BITS = "paxton90_bits"
CONF_DEBOUNCE_US = "debounce_us"

CONF_INVERT_DATA = "invert_data"
CONF_FRAME_GAP_US = "frame_gap_us"
CONF_USE_PULLUPS = "use_pullups"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(PaxtonReader),

        # raw GPIO numbers
        cv.Required(CONF_CLOCK_PIN): cv.int_range(min=0, max=48),
        cv.Required(CONF_DATA_PIN): cv.int_range(min=0, max=48),
        cv.Optional(CONF_LED_GREEN): cv.int_range(min=0, max=48),
        cv.Optional(CONF_LED_YELLOW): cv.int_range(min=0, max=48),
        cv.Optional(CONF_LED_RED): cv.int_range(min=0, max=48),

        cv.Required(CONF_LAST_CARD): cv.use_id(text_sensor.TextSensor),
        cv.Required(CONF_CARD_TYPE): cv.use_id(text_sensor.TextSensor),
        cv.Required(CONF_CARD_COLOUR): cv.use_id(text_sensor.TextSensor),
        cv.Required(CONF_BIT_COUNT): cv.use_id(sensor.Sensor),
        cv.Required(CONF_READING): cv.use_id(binary_sensor.BinarySensor),
        cv.Required(CONF_RAW_BITS): cv.use_id(text_sensor.TextSensor),

        cv.Optional(CONF_NET2_BITS, default=75): cv.positive_int,
        cv.Optional(CONF_SWITCH2_BITS, default=220): cv.positive_int,
        cv.Optional(CONF_PAXTON90_BITS, default=90): cv.positive_int,
        cv.Optional(CONF_DEBOUNCE_US, default=350): cv.positive_int,

        cv.Optional(CONF_INVERT_DATA, default=False): cv.boolean,
        cv.Optional(CONF_FRAME_GAP_US, default=4000): cv.positive_int,
        cv.Optional(CONF_USE_PULLUPS, default=True): cv.boolean,
    }
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    cg.add(var.set_clock_pin(config[CONF_CLOCK_PIN]))
    cg.add(var.set_data_pin(config[CONF_DATA_PIN]))
    if CONF_LED_GREEN in config:
        cg.add(var.set_led_green(config[CONF_LED_GREEN]))
    if CONF_LED_YELLOW in config:
        cg.add(var.set_led_yellow(config[CONF_LED_YELLOW]))
    if CONF_LED_RED in config:
        cg.add(var.set_led_red(config[CONF_LED_RED]))

    last_card   = await cg.get_variable(config[CONF_LAST_CARD])
    card_type   = await cg.get_variable(config[CONF_CARD_TYPE])
    card_colour = await cg.get_variable(config[CONF_CARD_COLOUR])
    bit_count   = await cg.get_variable(config[CONF_BIT_COUNT])
    reading     = await cg.get_variable(config[CONF_READING])
    raw_bits    = await cg.get_variable(config[CONF_RAW_BITS])

    cg.add(var.set_last_card(last_card))
    cg.add(var.set_card_type(card_type))
    cg.add(var.set_card_colour(card_colour))
    cg.add(var.set_bit_count(bit_count))
    cg.add(var.set_reading(reading))
    cg.add(var.set_raw_bits(raw_bits))

    cg.add(var.set_net2_bits(config[CONF_NET2_BITS]))
    cg.add(var.set_switch2_bits(config[CONF_SWITCH2_BITS]))
    cg.add(var.set_paxton90_bits(config[CONF_PAXTON90_BITS]))
    cg.add(var.set_debounce_us(config[CONF_DEBOUNCE_US]))

    cg.add(var.set_invert_data(config[CONF_INVERT_DATA]))
    cg.add(var.set_frame_gap_us(config[CONF_FRAME_GAP_US]))
    cg.add(var.set_use_pullups(config[CONF_USE_PULLUPS]))

    await cg.register_component(var, {})
