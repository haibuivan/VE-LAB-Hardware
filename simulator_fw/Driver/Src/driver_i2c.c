#include "driver_i2c.h"
#include <string.h>

/* Hàm này sẽ gọi hàm Callback nếu muốn báo sự kiện */
static void driver_i2c_slave_emit_event(driver_i2c_slave_t *dev,
                                        driver_i2c_slave_evt_type_t type,
                                        uint8_t data,
                                        uint32_t error)
{
    if (dev == NULL)
    {
        return;
    }

    driver_i2c_slave_event_cb_t cb = dev->event_cb;
    void *ctx = dev->event_ctx;

    if (cb != NULL)
    {
        driver_i2c_slave_evt_t evt;

        evt.type = type;
        evt.data = data;
        evt.error = error;

        cb(ctx, &evt);
    }
}

/* Hàm này sẽ gọi hàm callback để lấy byte mỗi khi cần byte */
static uint8_t driver_i2c_slave_get_tx_byte(driver_i2c_slave_t *dev)
{
    if (dev == NULL)
    {
        return 0xFFu;
    }

    driver_i2c_slave_tx_byte_cb_t cb = dev->tx_byte_cb;
    void *ctx = dev->tx_byte_ctx;

    if (cb != NULL)
    {
        return cb(ctx);
    }
    return 0xFFu;
}

void driver_i2c_slave_init(driver_i2c_slave_t *dev,
                           I2C_TypeDef *instance,
                           uint16_t own_address,
                           uint32_t clock_speed,
                           uint16_t duty_cycle)
{
    if (dev == NULL)
    {
        return;
    }

    memset(dev, 0, sizeof(*dev));

    dev->instance = instance;
    dev->own_address = own_address;
    dev->clock_speed = clock_speed;
    dev->duty_cycle = duty_cycle;
    dev->state = DRIVER_I2C_SLAVE_STATE_IDLE;
}

void driver_i2c_slave_start(driver_i2c_slave_t *dev)
{
    if ((dev == NULL) || (dev->instance == NULL))
    {
        return;
    }

    I2C_InitTypeDef i2c_init;

    i2c_init.I2C_Mode = I2C_Mode_I2C;
    i2c_init.I2C_ClockSpeed = dev->clock_speed;
    i2c_init.I2C_DutyCycle = dev->duty_cycle;
    i2c_init.I2C_OwnAddress1 = (uint16_t)(dev->own_address << 1);
    i2c_init.I2C_Ack = I2C_Ack_Enable;
    i2c_init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Init(dev->instance, &i2c_init);
    I2C_AcknowledgeConfig(dev->instance, ENABLE);   

    I2C_ITConfig(dev->instance, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);

    I2C_Cmd(dev->instance, ENABLE);

    dev->state = DRIVER_I2C_SLAVE_STATE_IDLE;
}

void driver_i2c_slave_stop(driver_i2c_slave_t *dev)
{
    if ((dev == NULL) || (dev->instance == NULL))
    {
        return;
    }

    I2C_ITConfig(dev->instance, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);

    I2C_Cmd(dev->instance, DISABLE);

    dev->state = DRIVER_I2C_SLAVE_STATE_IDLE;
}

void driver_i2c_slave_set_event_callback(driver_i2c_slave_t *dev,
                                         driver_i2c_slave_event_cb_t cb,
                                         void *ctx)
{
    if (dev == NULL)
    {
        return;
    }

    dev->event_cb = cb;
    dev->event_ctx = ctx;
}

void driver_i2c_slave_set_tx_byte_callback(driver_i2c_slave_t *dev,
                                           driver_i2c_slave_tx_byte_cb_t cb,
                                           void *ctx)
{
    if (dev == NULL)
    {
        return;
    }

    dev->tx_byte_cb = cb;
    dev->tx_byte_ctx = ctx;    
}

void driver_i2c_slave_ev_handler(driver_i2c_slave_t *dev)
{
    if ((dev == NULL) || (dev->instance == NULL))
    {
        return;
    }

    I2C_TypeDef *i2c = dev->instance;
    uint16_t sr1 = i2c->SR1;

    if ((sr1 & I2C_SR1_ADDR) != 0u)
    {
        uint16_t sr2;

        sr2 = i2c->SR2;

        if ((sr2 & I2C_SR2_TRA) != 0u)
        {
            uint8_t tx_data;

            dev->state = DRIVER_I2C_SLAVE_STATE_TX;

            driver_i2c_slave_emit_event(dev,
                                        DRIVER_I2C_SLAVE_EVT_READ_START,
                                        0u,
                                        0u);

            tx_data = driver_i2c_slave_get_tx_byte(dev);
            i2c->DR = tx_data;

            driver_i2c_slave_emit_event(dev,
                                        DRIVER_I2C_SLAVE_EVT_BYTE_SENT,
                                        tx_data,
                                        0u);
        }
        else
        {
            dev->state = DRIVER_I2C_SLAVE_STATE_RX;

            driver_i2c_slave_emit_event(dev,
                                        DRIVER_I2C_SLAVE_EVT_WRITE_START,
                                        0u,
                                        0u);
        }

        return;
    }

    if ((sr1 & I2C_SR1_STOPF) != 0u)
    {
        i2c->CR1 |= I2C_CR1_PE;

        if (dev->state == DRIVER_I2C_SLAVE_STATE_RX)
        {
            driver_i2c_slave_emit_event(dev,
                                        DRIVER_I2C_SLAVE_EVT_WRITE_DONE,
                                        0u,
                                        0u);
        }

        dev->state = DRIVER_I2C_SLAVE_STATE_IDLE;
        return;
    }

    if ((sr1 & I2C_SR1_RXNE) != 0u)
    {
        uint8_t rx_data;

        rx_data = (uint8_t)i2c->DR;

        if (dev->state != DRIVER_I2C_SLAVE_STATE_RX)
        {
            dev->state = DRIVER_I2C_SLAVE_STATE_RX;
        }

        driver_i2c_slave_emit_event(dev,
                                    DRIVER_I2C_SLAVE_EVT_BYTE_RECEIVED,
                                    rx_data,
                                    0u);

        return;
    }

    if ((sr1 & I2C_SR1_TXE) != 0u)
    {
        uint8_t tx_data;

        if (dev->state != DRIVER_I2C_SLAVE_STATE_TX)
        {
            dev->state = DRIVER_I2C_SLAVE_STATE_TX;
        }

        tx_data = driver_i2c_slave_get_tx_byte(dev);
        i2c->DR = tx_data;

        driver_i2c_slave_emit_event(dev,
                                    DRIVER_I2C_SLAVE_EVT_BYTE_SENT,
                                    tx_data,
                                    0u);

        return;
    }

    if ((sr1 & I2C_SR1_BTF) != 0u)
    {
        if (dev->state == DRIVER_I2C_SLAVE_STATE_TX)
        {
            uint8_t tx_data = driver_i2c_slave_get_tx_byte(dev);
            i2c->DR = tx_data;

            driver_i2c_slave_emit_event(dev,
                                        DRIVER_I2C_SLAVE_EVT_BYTE_SENT,
                                        tx_data,
                                        0u);
        }
        else if (dev->state == DRIVER_I2C_SLAVE_STATE_RX)
        {
            uint8_t rx_data = (uint8_t)i2c->DR;
            
            driver_i2c_slave_emit_event(dev,
                                        DRIVER_I2C_SLAVE_EVT_BYTE_RECEIVED,
                                        rx_data,
                                        0u);
        }
        return;
    }
}

void driver_i2c_slave_er_handler(driver_i2c_slave_t *dev)
{
    I2C_TypeDef *i2c;
    uint32_t error;

    if ((dev == NULL) || (dev->instance == NULL))
    {
        return;
    }

    i2c = dev->instance;
    error = 0u;

    if ((i2c->SR1 & I2C_SR1_AF) != 0u)
    {
        i2c->SR1 &= (uint16_t)~I2C_SR1_AF;

        if (dev->state == DRIVER_I2C_SLAVE_STATE_TX)
        {
            driver_i2c_slave_emit_event(dev,
                                        DRIVER_I2C_SLAVE_EVT_READ_DONE,
                                        0u,
                                        0u);

            dev->state = DRIVER_I2C_SLAVE_STATE_IDLE;
        }
        else
        {
            error |= I2C_SR1_AF;
        }
    }

    if ((i2c->SR1 & I2C_SR1_BERR) != 0u)
    {
        i2c->SR1 &= (uint16_t)~I2C_SR1_BERR;
        error |= I2C_SR1_BERR;
    }

    if ((i2c->SR1 & I2C_SR1_ARLO) != 0u)
    {
        i2c->SR1 &= (uint16_t)~I2C_SR1_ARLO;
        error |= I2C_SR1_ARLO;
    }

    if ((i2c->SR1 & I2C_SR1_OVR) != 0u)
    {
        i2c->SR1 &= (uint16_t)~I2C_SR1_OVR;
        error |= I2C_SR1_OVR;
    }

    if (error != 0u)
    {
        dev->state = DRIVER_I2C_SLAVE_STATE_IDLE;

        driver_i2c_slave_emit_event(dev,
                                    DRIVER_I2C_SLAVE_EVT_ERROR,
                                    0u,
                                    error);
    }
}
