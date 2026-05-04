#ifndef DRIVER_I2C_SLAVE_H
#define DRIVER_I2C_SLAVE_H

#include "stm32f10x.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    DRIVER_I2C_SLAVE_STATE_IDLE = 0,
    DRIVER_I2C_SLAVE_STATE_RX,
    DRIVER_I2C_SLAVE_STATE_TX    
} driver_i2c_slave_state_t;

typedef enum
{
    DRIVER_I2C_SLAVE_EVT_WRITE_START = 0,
    DRIVER_I2C_SLAVE_EVT_READ_START,
    DRIVER_I2C_SLAVE_EVT_BYTE_RECEIVED,
    DRIVER_I2C_SLAVE_EVT_BYTE_SENT,
    DRIVER_I2C_SLAVE_EVT_WRITE_DONE,
    DRIVER_I2C_SLAVE_EVT_READ_DONE,
    DRIVER_I2C_SLAVE_EVT_ERROR
} driver_i2c_slave_evt_type_t;

typedef struct 
{
    driver_i2c_slave_evt_type_t type;
    uint8_t data;
    uint32_t error;
} driver_i2c_slave_evt_t;

typedef struct driver_i2c_slave driver_i2c_slave_t;

typedef void (*driver_i2c_slave_event_cb_t) (void *ctx,
                                            const driver_i2c_slave_evt_t *evt);

typedef uint8_t (*driver_i2c_slave_tx_byte_cb_t)(void *ctx);

struct driver_i2c_slave
{
    I2C_TypeDef *instance;

    uint32_t clock_speed;
    uint16_t own_address;
    uint16_t duty_cycle;

    volatile driver_i2c_slave_state_t state;

    /* Callback khi có sự kiện */
    driver_i2c_slave_event_cb_t event_cb;
    void *event_ctx;

    /* Callback khi cần byte để gửi */
    driver_i2c_slave_tx_byte_cb_t tx_byte_cb;
    void *tx_byte_ctx;
};

/* Hàm khởi tạo */
void driver_i2c_slave_init(driver_i2c_slave_t *dev,
                        I2C_TypeDef *instance,
                        uint16_t own_address,
                        uint32_t clock_speed,
                        uint16_t duty_cycle);

/* Hàm start và stop ngoại vi */
void driver_i2c_slave_start(driver_i2c_slave_t *dev);
void driver_i2c_slave_stop(driver_i2c_slave_t *dev);

/* Hàm đăng kí callback (được các layer trên gọi) */
void driver_i2c_slave_set_event_callback(driver_i2c_slave_t *dev,
                                        driver_i2c_slave_event_cb_t cb,
                                        void *ctx);

void driver_i2c_slave_set_tx_byte_callback(driver_i2c_slave_t *dev,
                                            driver_i2c_slave_tx_byte_cb_t cb,
                                            void *ctx);

void driver_i2c_slave_ev_handler(driver_i2c_slave_t *dev);
void driver_i2c_slave_er_handler(driver_i2c_slave_t *dev);

#ifdef __cplusplus
}
#endif

#endif