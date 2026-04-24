#ifndef DHT11_SIM_H
#define DHT11_SIM_H

/* ================================================================
 *  DHT11 SIMULATOR — SENSOR LAYER
 *
 *  Phần cứng cố định (PA0 — single-wire, open-drain):
 *    TIM2 CH1 (IC) : phát hiện start signal từ host (falling/rising)
 *    TIM1 CH1 (OC) : timing engine phát 40 bit data
 *    PA0           : dây dữ liệu duy nhất, switch mode lúc runtime
 *                    IC mode  → GPIO_Mode_IPU   (chờ host kéo LOW)
 *                    TX mode  → GPIO_Mode_Out_OD (đang phát data)
 *
 *  FreeRTOS objects (tạo bên trong dht11_sim_config):
 *    Task            : DHT11 state machine vòng lặp vô hạn
 *    BinarySemaphore : IC ISR → Task báo start signal hợp lệ
 *    Queue (depth=1) : dht11_sim_set_data() → Task cập nhật data
 *
 *  ┌──────────────┐  dht11_sim_config()   ┌──────────────────────┐
 *  │  APP LAYER   │ ───────────────────►  │   SENSOR LAYER       │
 *  │              │  dht11_sim_run()      │   FreeRTOS Task      │
 *  │              │ ───────────────────►  │  ┌────────────────┐  │
 *  │              │  dht11_sim_set_data() │  │ block semaphore│  │
 *  │              │ ───────────────────►  │  │ → build data   │  │
 *  └──────────────┘                       │  │ → phát 40 bit  │  │
 *                                         │  └───────┬────────┘  │
 *                                         └──────────┼───────────┘
 *                                                    │ register direct
 *                                          ┌─────────▼──────────┐
 *                                          │  DRIVER LAYER      │
 *                                          │ TIM1(OC) TIM2(IC)  │
 *                                          │ PA0 open-drain     │
 *                                          └────────────────────┘
 * ================================================================ */

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ================================================================
 *  ERROR CODES
 * ================================================================ */
#define DHT11_SIM_OK       ( 0)
#define DHT11_SIM_EINVAL   (-1)
#define DHT11_SIM_ENOMEM   (-2)   /* FreeRTOS alloc thất bại        */
#define DHT11_SIM_EBUSY    (-3)   /* gọi run() khi đang chạy        */
#define DHT11_SIM_ENOTCFG  (-4)   /* gọi run() trước config()       */

/* ================================================================
 *  DATA
 * ================================================================ */
typedef struct {
    uint8_t humidity;     /* %RH — 0..99  */
    uint8_t temperature;  /* °C  — 0..50  */
} DHT11_Data_t;

/* ================================================================
 *  CONFIG — truyền vào dht11_sim_config()
 *
 *  init_data     : giá trị hum/temp hard-code ban đầu
 *  task_priority : FreeRTOS priority (khuyến nghị tskIDLE_PRIORITY+2)
 *  task_stack    : stack size tính bằng word (khuyến nghị 128)
 * ================================================================ */
typedef struct {
    DHT11_Data_t  init_data;
    UBaseType_t   task_priority;
    uint16_t      task_stack;
} DHT11_Sim_Cfg_t;

/* ================================================================
 *  APP-LAYER API — app chỉ gọi 3 hàm này
 *
 *  dht11_sim_config()   : cấu hình hardware + tạo FreeRTOS objects.
 *                         Chưa start task, chưa enable timer IRQ.
 *                         Gọi trước vTaskStartScheduler().
 *
 *  dht11_sim_run()      : enable IC/OC interrupt, start task.
 *                         Gọi sau config(). Trả EBUSY nếu gọi lại.
 *
 *  dht11_sim_set_data() : cập nhật hum/temp cho lần trigger tiếp.
 *                         Thread-safe, gọi từ task bất kỳ hoặc ISR.
 * ================================================================ */
int dht11_sim_config(const DHT11_Sim_Cfg_t *cfg);
int dht11_sim_run(void);
int dht11_sim_set_data(const DHT11_Data_t *data);

#endif /* DHT11_SIM_H */
