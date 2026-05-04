#ifndef DRIVER_GPIO_H
#define DRIVER_GPIO_H

#include "stm32f10x.h"
#include <stdbool.h>

/* ================================================================
 *  GPIO DRIVER API
 *
 *  gpio_open   : cau hinh 1 chan GPIO (bat clock, set mode/speed)
 *  gpio_read   : doc trang thai chan input
 *  gpio_write  : ghi muc logic ra chan output
 *  gpio_toggle : dao trang thai chan output
 *  gpio_close  : reset chan ve input floating (an toan)
 * ================================================================ */
void gpio_open  (GPIO_TypeDef *GPIOx, uint16_t pin,
                 GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed);
bool gpio_read  (GPIO_TypeDef *GPIOx, uint16_t pin);
void gpio_write (GPIO_TypeDef *GPIOx, uint16_t pin, bool state);
void gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t pin);
void gpio_close (GPIO_TypeDef *GPIOx, uint16_t pin);

#endif /* DRIVER_GPIO_H */