/*
* hcsr04.h
*
* Created: 07.05.2017 22:49:41
*  Author: philipp
*/


#ifndef ULTRASOUND_SENSOR_H_
#define ULTRASOUND_SENSOR_H_

#include <avr/io.h>
#include <util/delay.h>

#define ECHO_PIN PINB0
#define ECHO_DDR DDRB
#define ECHO_PORT PORTB

#ifndef TIMER_CTRL_REG
    #define TIMER_CTRL_REG TCCR1B
#endif

#ifndef TIMER_ESEL
    #define TIMER_ESEL ICES1
#endif

#ifndef TIMER_FLAG_REG
    #define TIMER_FLAG_REG TIFR
#endif

#ifndef TIMER_IPT_FLAG
    #define TIMER_IPT_FLAG ICF1
#endif

#ifndef TIMER_OVFL_FLAG
    #define TIMER_OVFL_FLAG TOV1
#endif

#ifndef TIMER_IPT_CAPT_REG
    #define TIMER_IPT_CAPT_REG ICR1
#endif

void ultrasoundsensor_init(void);

uint16_t ultrasoundsensor_read_dist(volatile uint8_t *sensor_port, volatile uint8_t sensor_pin);

#endif /* ULTRASOUND_SENSOR_H_ */
