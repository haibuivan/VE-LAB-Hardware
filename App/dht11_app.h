#ifndef DHT11_APP_H
#define DHT11_APP_H

/* ================================================================
 *  DHT11 APP LAYER
 *
 *  Caller không cần biết gì về timer, FreeRTOS, protocol, GPIO.
 *  Toàn bộ tham số được định nghĩa sẵn bên trong (#define bên dưới).
 *  Thầy/TA chỉ cần chỉnh số ở đây để thay đổi kịch bản lab.
 *
 *  Cách dùng từ main.c — chỉ 2 dòng:
 *
 *      dht11_app_config();
 *      dht11_app_run();
 * ================================================================ */

/* ================================================================
 *  SCENARIO PARAMETERS
 *  Thầy/TA chỉnh tại đây trước khi build cho từng bài lab
 * ================================================================ */
#define DHT11_APP_HUMIDITY      30u   /* %RH — 0..99  */
#define DHT11_APP_TEMPERATURE   28u   /* °C  — 0..50  */

/* ================================================================
 *  ERROR CODES
 * ================================================================ */
#define DHT11_APP_OK     ( 0)
#define DHT11_APP_EFAIL  (-1)

/* ================================================================
 *  API
 * ================================================================ */
int dht11_app_config(void);
int dht11_app_run(void);

#endif /* DHT11_APP_H */
