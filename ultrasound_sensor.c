#include "ultrasound_sensor.h"

#define TRIGGER_RE (TIMER_CTRL_REG |= (1 << TIMER_ESEL))
#define TRIGGER_FE (TIMER_CTRL_REG &= ~(1 << TIMER_ESEL))
#define ENABLE_TRIG(trig_port, trig_pin) (trig_port |= (1 << trig_pin))
#define DISABLE_TRIG(trig_port, trig_pin) (trig_port &= ~(1 << trig_pin))

#define CLEAR_TRIG_FLAG (TIMER_FLAG_REG |= (1 << TIMER_IPT_FLAG))
#define EDGE_TRIGGER_DETECTED (TIMER_FLAG_REG & (1 << TIMER_IPT_FLAG))

#define CLEAR_OVFL_FLAG (TIMER_FLAG_REG |= (1 << TIMER_OVFL_FLAG))
#define TIMER_OVFL_DETECTED (TIMER_FLAG_REG & (1 << TIMER_OVFL_FLAG))

#define TIMER_ENABLE (TIMER_CTRL_REG |= 0x01)
#define TIMER_DISABLE (TIMER_CTRL_REG &= ~(0x01))

/*
 * FUNCTION DECLARATIONS
 */
uint16_t _ultrasoundsensor_read_dist(volatile uint8_t *sensor_port, volatile uint8_t sensor_pin);
uint16_t _calculate_distance(uint32_t ticks);

/*
 * TYPE DEFINITIONS
 */
static enum edge_detection_t {
    RE_DETECTION, FE_DETECTION
};

void ultrasoundsensor_init() {
    // init echo_pin
    ECHO_DDR &= ~(1 << ECHO_PIN);  // ECHO_PIN as tri-state input
}

uint16_t ultrasoundsensor_read_dist(volatile uint8_t *sensor_port, volatile uint8_t sensor_pin) {
    uint16_t distance = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        distance += _ultrasoundsensor_read_dist(sensor_port, sensor_pin);
    }
    distance = distance >> 3UL;  // calc mean distance
    return distance;
}

static uint16_t _ultrasoundsensor_read_dist(volatile uint8_t *sensor_port, volatile uint8_t sensor_pin) {
    TRIGGER_RE;  // trigger on rising edge

    // TODO: config max runtime timer

    // Send trigger signal to ultrasound sensor
    ENABLE_TRIG(*sensor_port, sensor_pin);
    _delay_ms(100);
    DISABLE_TRIG(*sensor_port, sensor_pin);

    // TODO: start max runtime timer

    // Measure echo pulse
    uint16_t start_ticks = 0;
    uint16_t end_ticks = 0;
    uint8_t timer_overflows = 0;

    enum edge_detection_t detection_status = RE_DETECTION;
    uint8_t edge_detected = 0;

    TIMER_ENABLE;
    do {
        if (EDGE_TRIGGER_DETECTED)  // Edge detected
        {
            switch (detection_status) {
                case RE_DETECTION:
                    start_ticks = TIMER_IPT_CAPT_REG;
                    CLEAR_TRIG_FLAG;  // clear input capture flag
                    TRIGGER_FE;  // trigger on falling edge
                    detection_status = FE_DETECTION;
                    break;
                case FE_DETECTION:
                    end_ticks = TIMER_IPT_CAPT_REG;
                    CLEAR_TRIG_FLAG;  // clear input capture flag
                    edge_detected = 1;
                    break;
            }
        }

        if (TIMER_OVFL_DETECTED) {
            if (!edge_detected && detection_status != FE_DETECTION)
                timer_overflows++;
            CLEAR_OVFL_FLAG;
        }

    } while (!edge_detected);
    TIMER_DISABLE;

    uint32_t total_end_ticks = ((uint32_t) timer_overflows * (uint32_t) UINT16_MAX) + (uint32_t) end_ticks;
    uint32_t total_ticks = total_end_ticks - (uint32_t) start_ticks;

    return _calculate_distance(total_ticks);
}

static uint16_t _calculate_distance(uint32_t ticks) {
    ticks = ticks >> 1UL;  // divide ticks by 2
    return (uint16_t) (((float) ticks * 42.75) / 1000.0); // return distance in [mm]
}
