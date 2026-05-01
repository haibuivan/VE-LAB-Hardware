#include "gpio_driver.h"

/* 
 * gpio_open - cấu hình 1 chân GPIO
 * GPIOx  : GPIOA, GPIOB, ...
 * pin    : GPIO_Pin_0, GPIO_Pin_1, ...
 * mode   : GPIO_Mode_AIN, GPIO_Mode_IN_FLOATING, GPIO_Mode_IPD, GPIO_Mode_IPU,
 *          GPIO_Mode_Out_PP, GPIO_Mode_Out_OD, GPIO_Mode_AF_PP, GPIO_Mode_AF_OD
 * speed  : GPIO_Speed_10MHz, GPIO_Speed_2MHz, GPIO_Speed_50MHz
 */
void gpio_open(GPIO_TypeDef* GPIOx, uint16_t pin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Bật clock cho port tương ứng */
    if      (GPIOx == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (GPIOx == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (GPIOx == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (GPIOx == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    else if (GPIOx == GPIOE) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    else if (GPIOx == GPIOF) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    else if (GPIOx == GPIOG) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

    /* Cấu hình chân */
    GPIO_InitStructure.GPIO_Pin   = pin;
    GPIO_InitStructure.GPIO_Mode  = mode;
    GPIO_InitStructure.GPIO_Speed = speed;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/*
 * gpio_read - đọc trạng thái chân input
 * Trả về: true (HIGH), false (LOW)
 */
bool gpio_read(GPIO_TypeDef* GPIOx, uint16_t pin)
{
    return (GPIO_ReadInputDataBit(GPIOx, pin) != Bit_RESET);
}

/*
 * gpio_write - ghi mức logic ra chân output
 * state: true (SET), false (RESET)
 */
void gpio_write(GPIO_TypeDef* GPIOx, uint16_t pin, bool state)
{
    if (state)
        GPIO_SetBits(GPIOx, pin);
    else
        GPIO_ResetBits(GPIOx, pin);
}

/*
 * gpio_toggle - đảo trạng thái chân output
 */
void gpio_toggle(GPIO_TypeDef* GPIOx, uint16_t pin)
{
    /* Đọc trạng thái hiện tại rồi ghi ngược lại */
    if (GPIO_ReadOutputDataBit(GPIOx, pin))
        GPIO_ResetBits(GPIOx, pin);
    else
        GPIO_SetBits(GPIOx, pin);
}

/*
 * gpio_close - reset chân về input floating (mặc định an toàn)
 */
void gpio_close(GPIO_TypeDef* GPIOx, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin   = pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}
