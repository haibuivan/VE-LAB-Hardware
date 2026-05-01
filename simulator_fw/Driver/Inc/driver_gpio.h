#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "stm32f10x.h"

void gpio_open  (GPIO_TypeDef* GPIOx, uint16_t pin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed);
bool gpio_read  (GPIO_TypeDef* GPIOx, uint16_t pin);
void gpio_write (GPIO_TypeDef* GPIOx, uint16_t pin, bool state);
void gpio_toggle(GPIO_TypeDef* GPIOx, uint16_t pin);
void gpio_close (GPIO_TypeDef* GPIOx, uint16_t pin);

#endif /* GPIO_DRIVER_H */