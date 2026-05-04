#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"

#if (ENABLE_DS1307 == 1)
#include "sensor_ds1307.h"
#endif

#if (ENABLE_DHT11 == 1)
#include "sensor_dht11.h"
#endif

int main(void)
{
    /* 1. Cau hinh he thong co ban */
    SystemInit();
    SystemCoreClockUpdate();
    
    /* Dam bao NVIC Priority Group 4 cho FreeRTOS */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 2. Khoi tao cac Sensor duoc enable */
#if (ENABLE_DS1307 == 1)
    sensor_ds1307_init();
    sensor_ds1307_run();
#endif

#if (ENABLE_DHT11 == 1)
    sensor_dht11_init();
    sensor_dht11_run();
#endif

    /* 3. Chay RTOS Scheduler */
    vTaskStartScheduler();

    /* 4. Fallback neu thieu RAM tao Idle task */
    while (1)
    {
    }
}
