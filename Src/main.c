#include "stm32f10x.h"                  // Device header
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "queue.h"
#include "stdio.h"
#include "dht11_app.h"


/* ================================================================
 *  IRQ HANDLERS — đặt trong stm32f10x_it.c
 * ================================================================ */
void dht11_tim1_cc_irq(void);
void dht11_tim2_ic_irq(void);

void TIM1_CC_IRQHandler(void) { dht11_tim1_cc_irq(); }
void TIM2_IRQHandler(void)    { dht11_tim2_ic_irq(); }

/* ================================================================
 *  MAIN — app chỉ cần 2 dòng để chạy lab DHT11
 * ================================================================ */
int main(void)
{
    SystemInit();
    SystemCoreClockUpdate();

    dht11_app_config();
    dht11_app_run();

    vTaskStartScheduler();
    for (;;) {}
}
