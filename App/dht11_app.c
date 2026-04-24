#include "dht11_app.h"
#include "dht11_sim.h"

/* ================================================================
 *  INTERNAL DEFAULTS — không expose ra ngoài
 * ================================================================ */
#define DHT11_APP_TASK_PRIORITY   2u
#define DHT11_APP_TASK_STACK      128u

/* ================================================================
 *  dht11_app_config()
 *  Gọi 1 lần trước vTaskStartScheduler().
 *  Không nhận tham số — mọi giá trị lấy từ #define trong .h
 * ================================================================ */
int dht11_app_config(void)
{
    DHT11_Sim_Cfg_t sim_cfg;
    sim_cfg.init_data.humidity    = DHT11_APP_HUMIDITY;
    sim_cfg.init_data.temperature = DHT11_APP_TEMPERATURE;
    sim_cfg.task_priority         = DHT11_APP_TASK_PRIORITY;
    sim_cfg.task_stack            = DHT11_APP_TASK_STACK;

    if (dht11_sim_config(&sim_cfg) != DHT11_SIM_OK)
        return DHT11_APP_EFAIL;

    return DHT11_APP_OK;
}

/* ================================================================
 *  dht11_app_run()
 *  Enable IC interrupt + start FreeRTOS task.
 *  Gọi sau config(). Từ thời điểm này simulator tự chạy hoàn toàn.
 * ================================================================ */
int dht11_app_run(void)
{
    if (dht11_sim_run() != DHT11_SIM_OK)
        return DHT11_APP_EFAIL;

    return DHT11_APP_OK;
}
