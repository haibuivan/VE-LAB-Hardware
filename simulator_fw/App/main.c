#include "stm32f10x.h" // Device header
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "queue.h"
#include "stdio.h"
#include "sensor_dht11.h"

/* ================================================================
 *  IRQ HANDLERS — đặt trong stm32f10x_it.c
 * ================================================================ */
void dht11_tim1_cc_irq(void);
void dht11_tim2_ic_irq(void);

void TIM1_CC_IRQHandler(void)
{
    #ifdef DHT11
    dht11_tim1_cc_irq();
    #endif
}
void TIM2_IRQHandler(void) { 
    dht11_tim2_ic_irq(); 
}

/* ================================================================
 *  MAIN — app chỉ cần 2 dòng để chạy lab DHT11
 * ================================================================ */
int main(void)
{
    SystemInit();
    SystemCoreClockUpdate();

    dht11_sensor_config();
    dht11_sensor_run();

    vTaskStartScheduler();
    for (;;);
}
