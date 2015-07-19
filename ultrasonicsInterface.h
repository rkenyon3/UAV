#ifndef ULTRASONICSINTERFACE_H
#define ULTRASONICSINTERFACE_H

#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_i2c.h"
#include "coOS.h"

#include "system.h"
#include "util.h"

#define ULTRASONIC_SENSOR_TOP_ADDRESS		0
#define ULTRASONIC_SENSOR_BOTTOM_ADDRESS	1
#define ULTRASONIC_SENSOR_FRONT_ADDRESS		2
#define ULTRASONIC_SENSOR_BACK_ADDRESS		3
#define ULTRASONIC_SENSOR_LEFT_ADDRESS		4
#define ULTRASONIC_SENSOR_RIGHT_ADDRESS		5

// Sets up the sensor interface
void ultrasonicsSetupSensors(void);

/* Reads the range from the sensor,
 * and returns a value in cm. Returns
 * -1 if the sensor is unresponsive.
 */
int ultrasonicsReadSensor(uint8_t SensorAddressToRead);

#endif
