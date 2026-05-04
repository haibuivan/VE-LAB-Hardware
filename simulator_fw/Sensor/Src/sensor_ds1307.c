#include "sensor_ds1307.h"
#include "app_config.h"

#if (ENABLE_DS1307 == 1)

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "driver_i2c.h"
#include "driver_gpio.h"
#include <string.h>

#define APP_DS1307_I2C_OWN_ADDRESS     DS1307_I2C_ADDR
#define APP_DS1307_I2C_CLOCK_SPEED     100000u
#define APP_DS1307_TASK_PRIORITY_OFFSET 1u
#define APP_DS1307_TASK_STACK_SIZE      256u
#define APP_DS1307_TICK_PERIOD_MS       1000u
#define APP_DS1307_SIMU_DEBUG           0u

/* ============================================================
 * DS1307 COMMON DEFINITIONS
 * ============================================================
 * Phần định nghĩa chung theo datasheet DS1307:
 * - địa chỉ I2C
 * - register map
 * - bit mask
 * - số lượng register
 */

/* Địa chỉ I2C 7-bit mặc định của DS1307 */
#define DS1307_I2C_ADDR      0x68u

/* DS1307 register map */
#define DS1307_REG_SECONDS   0x00u
#define DS1307_REG_MINUTES   0x01u
#define DS1307_REG_HOURS     0x02u
#define DS1307_REG_DAY       0x03u
#define DS1307_REG_DATE      0x04u
#define DS1307_REG_MONTH     0x05u
#define DS1307_REG_YEAR      0x06u
#define DS1307_REG_CONTROL   0x07u
#define DS1307_REG_RAM_BASE  0x08u
#define DS1307_REG_COUNT     64u

/* DS1307 bit definitions */
#define DS1307_BIT_CH        (1u << 7)
#define DS1307_BIT_12H       (1u << 6)
#define DS1307_BIT_PM        (1u << 5)
#define DS1307_BIT_OUT       (1u << 7)

/* DS1307 register masks */
#define DS1307_MASK_SECONDS  0x7Fu
#define DS1307_MASK_MINUTES  0x7Fu
#define DS1307_MASK_HOURS_12 0x1Fu
#define DS1307_MASK_HOURS_24 0x3Fu
#define DS1307_MASK_DAY      0x07u
#define DS1307_MASK_DATE     0x3Fu
#define DS1307_MASK_MONTH    0x1Fu

/* Số bank dùng cho cơ chế triple-buffer khi phục vụ I2C read */
#define DS1307_SIMU_BANK_COUNT 3u

/* ============================================================
 * DS1307 CORE LOGIC
 * ============================================================
 * Nhóm này đại diện cho logic lõi của DS1307:
 * - lưu thời gian dạng binary
 * - quản lý register DS1307
 * - convert time sang BCD để lưu vào thanh ghi của DS1307
 * - tick tăng 1 giây
 * - read/write register
 *
 * Không liên quan ISR, driver hay RTOS
 */

/* Thời gian dạng binary, dễ xử lý trong code.
* Khi ghi ra register DS1307 sẽ được convert sang BCD.
*/
typedef struct
{
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day_of_week;
    uint8_t date;
    uint8_t month;
    uint16_t year;
} ds1307_simu_time_t;

typedef struct ds1307_simu ds1307_simu_t;

/* ============================================================
 * DS1307 I2C SIMULATION / DRIVER ADAPTER
 * ============================================================
 * Nhóm này dùng để nối DS1307 core logic với driver_i2c_slave:
 * - nhận event từ driver I2C slave
 * - quản lý register pointer theo chuẩn DS1307
 * - phục vụ master read/write qua I2C
 * - staging write transaction
 * - triple-buffer để tránh race khi đang I2C read
 * - callback báo app khi master write done
 *
 * Lưu ý:
 * - ISR nằm ở driver.
 * - RTOS vẫn nằm ở app layer.
 */

/* Callback báo cho app biết master đã write xong.
* App có thể dùng callback này để give semaphore cho task xử lý.
*/
typedef void (*ds1307_simu_write_done_cb_t)(void *ctx);

/* ============================================================
 * DS1307 SIMULATOR CONTEXT
 * ============================================================
 * Struct này chứa cả:
 * 1. Core state của DS1307
 * 2. State phục vụ I2C transaction
 */
struct ds1307_simu
{
    /* ============ Core state ================ */
    ds1307_simu_time_t current_time;        /* Thời gian hiện tại dạng bin */
    uint8_t regs[DS1307_REG_COUNT];         /* Các thanh ghi time lưu dạng BCD giống DS1307 thật */

    /* ============= I2C Simu state =========== */
    driver_i2c_slave_t *i2c;
    uint8_t current_reg;           /* Register pointer hiện tại */
    uint8_t tx_reg;                /* Register pointer khi master read */
    uint8_t got_reg_pointer;       /* Cờ báo đã nhận byte register pointer khi write hay chưa */

    /* Triple-buffer register bank.
     * Dùng để master read ổn định trong khi core có thể update regs.
     */
    uint8_t reg_bank[DS1307_SIMU_BANK_COUNT][DS1307_REG_COUNT];
    volatile uint8_t active_bank;   /* Bank mới nhất sẵn sàng dùng cho lần tiếp */
    volatile uint8_t tx_bank;      /* Bank đang được dùng trong phiên read hiện tại */
    volatile uint8_t tx_active;     /* Cờ báo đang có phiên read/TX diễn ra */

    uint8_t write_start_reg;        /* Regs bắt đầu của transaction write */
    uint8_t write_len;             
    uint8_t write_buf[DS1307_REG_COUNT];    /* Buffer lưu data tạm thời */
    volatile uint8_t write_pending;        /* Cờ báo cáo transaction write chờ xử lí */

    /* Callbacl báo write done lên app */
    ds1307_simu_write_done_cb_t write_done_cb;
    void *write_done_ctx;
};

/* ============================================================
 * DS1307 MODULE INIT / ATTACH
 * ============================================================
 */

/* Khởi tạo DS1307 simu */

/* =======================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ======================================================= */

/* Callback nội bộ nhận event từ driver I2C slave */
static void ds1307_simu_on_i2c_event(void *ctx,
                                    const driver_i2c_slave_evt_t *evt);

/* Callback nội bộ đẻ driver lấy byte */
static uint8_t ds1307_simu_get_tx_byte(void *ctx);

/* ============================================================
 * DS1307 CORE LOGIC - BCD HELPER
 * ============================================================
 * DS1307 lưu time register ở dạng BCD.
 * Core logic dùng binary để dễ tính toán.
 */

/* Chuyển bin sang BCD */
static uint8_t ds1307_simu_bin_to_bcd(uint8_t val)
{
    return (uint8_t)(((val / 10u) << 4u) | (val % 10u));
}

/* Chuyenr BCD sang bin */
static uint8_t ds1307_simu_bcd_to_bin(uint8_t val)
{
    return (uint8_t)(((val >> 4u) * 10u) + (val & 0x0Fu));
}

/* ============================================================
 * DS1307 CORE LOGIC - DATE/TIME HELPER
 * ============================================================ */

/* Tính số ngày trong tháng */
static uint8_t ds1307_simu_days_in_month(uint8_t month, uint16_t year)
{
    if ((month == 4u) || (month == 6u) ||
        (month == 9u) || (month == 11u))
    {
        return 30u;
    }
    
    if (month == 2u)
    {
        uint8_t is_leap;

        is_leap = (uint8_t)((((year % 4u) == 0u) &&
                             ((year % 100u) != 0u)) ||
                            ((year % 400u) == 0u));

        return is_leap ? 29u : 28u;
    }

    return 31u;
}

/* Giới hạn giá trị trong khoảng min -> max */
static uint8_t ds1307_simu_limit_u8(uint8_t value,
                                    uint8_t min,
                                    uint8_t max)
{
    if (value < min)
    {
        return min;
    }

    if (value > max)
    {
        return max;
    }

    return value;
}

static void ds1307_simu_normalize_time(ds1307_simu_time_t *time)
{
    if (time == NULL)
    {
        return;
    }

    time->second = ds1307_simu_limit_u8(time->second, 0u, 59u);
    time->minute = ds1307_simu_limit_u8(time->minute, 0u, 59u);
    time->hour = ds1307_simu_limit_u8(time->hour, 0u, 23u);
    time->day_of_week = ds1307_simu_limit_u8(time->day_of_week, 1u, 7u);
    time->month = ds1307_simu_limit_u8(time->month, 1u, 12u);

    // if (time->year < 2000u)
    // {
    //     time->year = 2000u;
    // }

    // if (time->year > 2099u)
    // {
    //     time->year = 2099u;
    // }

    uint8_t max_date = ds1307_simu_days_in_month(time->month, time->year);
    time->date = ds1307_simu_limit_u8(time->date, 1u, max_date);
}

/* ============================================================
 * DS1307 CORE LOGIC - HOUR 12H/24H HELPER
 * ============================================================ */

/* Decode thanh ghi hour DS1307 sang hour dạng 24h */
static uint8_t ds1307_simu_decode_hour_to_24h(uint8_t hour_reg)
{
    if ((hour_reg & DS1307_BIT_12H) != 0u)
    {
        uint8_t h12 = ds1307_simu_bcd_to_bin((uint8_t)(hour_reg & DS1307_MASK_HOURS_12));
        uint8_t is_pm = (uint8_t)((hour_reg & DS1307_BIT_PM) ? 1u : 0u);   
        
        if (h12 == 12u)
        {
            return is_pm ? 12u : 0u;
        }

        return is_pm ? (uint8_t)(h12 + 12u) : h12;
    }

    return ds1307_simu_bcd_to_bin((uint8_t)(hour_reg & DS1307_MASK_HOURS_24));
}
/* Encode hour 24h về đúng format thanh ghi hour DS1307 */
static uint8_t ds1307_simu_encode_hour_from_24h(uint8_t hour24,
                                                uint8_t old_hour_reg)
{
    if ((old_hour_reg & DS1307_BIT_12H) != 0u)
    {
        uint8_t h12;
        uint8_t pm;

        if (hour24 == 0u)
        {
            h12 = 12u;
        }
        else if (hour24 < 12u)
        {
            h12 = hour24;
        }
        else if (hour24 == 12u)
        {
            h12 = 12u;
            pm = DS1307_BIT_PM;
        }
        else
        {
            h12 = (uint8_t)(hour24 - 12u);
            pm = DS1307_BIT_PM;
        }

        return (uint8_t)(DS1307_BIT_12H | pm |
                        (ds1307_simu_bin_to_bcd(h12) & DS1307_MASK_HOURS_12));
    }

    return (uint8_t)(ds1307_simu_bin_to_bcd(hour24) & DS1307_MASK_HOURS_24);
}

/* ============================================================
 * DS1307 CORE LOGIC - SYNC TIME <-> REGISTER
 * ============================================================ */

/* Đồng bộ register DS1307 sang current_time */
static void ds1307_simu_sync_time_from_regs(ds1307_simu_t *sim)
{
    if (sim == NULL)
    {
        return;
    }

    sim->current_time.second = 
        ds1307_simu_bcd_to_bin((uint8_t)(sim->regs[DS1307_REG_SECONDS] &
                                            DS1307_MASK_SECONDS));
    sim->current_time.minute = 
        ds1307_simu_bcd_to_bin((uint8_t)(sim->regs[DS1307_REG_MINUTES] &
                                            DS1307_MASK_MINUTES));
    sim->current_time.hour = 
        ds1307_simu_decode_hour_to_24h(sim->regs[DS1307_REG_HOURS]);
    sim->current_time.day_of_week = 
        (uint8_t)(sim->regs[DS1307_REG_DAY] & DS1307_MASK_DAY);
    sim->current_time.date = 
        ds1307_simu_bcd_to_bin((uint8_t)(sim->regs[DS1307_REG_DATE] &
                                            DS1307_MASK_DATE));
    sim->current_time.month = 
        ds1307_simu_bcd_to_bin((uint8_t)(sim->regs[DS1307_REG_MONTH] &
                                            DS1307_MASK_MONTH));
    sim->current_time.year =
        (uint16_t)(2000u + ds1307_simu_bcd_to_bin(sim->regs[DS1307_REG_YEAR]));

    ds1307_simu_normalize_time(&sim->current_time);
}

/* Đồng bộ current_time sang register DS1307 */
static void ds1307_simu_sync_regs_from_time(ds1307_simu_t *sim)
{
    uint8_t sec_ch;

    if (sim == NULL)
    {
        return;
    }

    ds1307_simu_normalize_time(&sim->current_time);

    /* Giữ lại bit CH khi update seconds */
    sec_ch = (uint8_t)(sim->regs[DS1307_REG_SECONDS] & DS1307_BIT_CH);

    sim->regs[DS1307_REG_SECONDS] =
        (uint8_t)(sec_ch |
                  (ds1307_simu_bin_to_bcd(sim->current_time.second) &
                   DS1307_MASK_SECONDS));

    sim->regs[DS1307_REG_MINUTES] =
        (uint8_t)(ds1307_simu_bin_to_bcd(sim->current_time.minute) &
                  DS1307_MASK_MINUTES);

    sim->regs[DS1307_REG_HOURS] =
        ds1307_simu_encode_hour_from_24h(sim->current_time.hour,
                                         sim->regs[DS1307_REG_HOURS]);

    sim->regs[DS1307_REG_DAY] =
        (uint8_t)(sim->current_time.day_of_week & DS1307_MASK_DAY);

    sim->regs[DS1307_REG_DATE] =
        (uint8_t)(ds1307_simu_bin_to_bcd(sim->current_time.date) &
                  DS1307_MASK_DATE);

    sim->regs[DS1307_REG_MONTH] =
        (uint8_t)(ds1307_simu_bin_to_bcd(sim->current_time.month) &
                  DS1307_MASK_MONTH);

    sim->regs[DS1307_REG_YEAR] =
        ds1307_simu_bin_to_bcd((uint8_t)(sim->current_time.year - 2000u));
}

/* ============================================================
 * DS1307 I2C ADAPTER - REGISTER BANK HELPER
 * ============================================================
 * Triple-buffer giúp master đọc ổn định trong khi core update register.
 */

/* Mask register về range 0x00 -> 0x3F */
static uint8_t ds1307_simu_mask_reg(uint8_t reg)
{
    return (uint8_t)(reg % DS1307_REG_COUNT);
}

/* Tăng register pointer và tự wrap về 0 nếu quá 0x3F */
static uint8_t ds1307_simu_next_reg(uint8_t reg)
{
    return ds1307_simu_mask_reg((uint8_t)(reg + 1u));
}

/* Tìm bank rảnh để publish register mới */
static uint8_t ds1307_simu_find_free_bank(ds1307_simu_t *sim)
{
    uint8_t bank;

    for (bank = 0u; bank < DS1307_SIMU_BANK_COUNT; bank++)
    {
        if ((sim->tx_active == 0u) || (bank != sim->tx_bank))
        {
            if (bank != sim->active_bank)
            {
                return bank;
            }
        }
    }

    return sim->active_bank;
}

/* Copy regs hiện tại sang bank mới để phục vụ I2C read */
static void ds1307_simu_publish_regs(ds1307_simu_t *sim)
{
    uint8_t new_bank;

    if (sim == NULL)
    {
        return;
    }

    new_bank = ds1307_simu_find_free_bank(sim);

    if ((sim->tx_active != 0u) && (new_bank == sim->tx_bank))
    {
        return;
    }

    memcpy(sim->reg_bank[new_bank], sim->regs, DS1307_REG_COUNT);

    sim->active_bank = new_bank;
}

/* ============================================================
 * DS1307 I2C ADAPTER - WRITE DONE HELPER
 * ============================================================ */

/* Báo lên app rằng master đã write xong */
static void ds1307_simu_emit_write_done(ds1307_simu_t *sim)
{
    if ((sim != NULL) && (sim->write_done_cb != NULL))
    {
        sim->write_done_cb(sim->write_done_ctx);
    }
}

/* ============================================================
 * DS1307 MODULE INIT / ATTACH
 * ============================================================ */

void ds1307_simu_init(ds1307_simu_t *sim,
                      driver_i2c_slave_t *i2c)
{
    if (sim == NULL)
    {
        return;
    }

    memset(sim, 0, sizeof(*sim));

    sim->i2c = i2c;

    /* Thời gian mặc định: 00:00:00 01/01/2000 Day 1 */
    sim->current_time.second = 0u;
    sim->current_time.minute = 0u;
    sim->current_time.hour = 0u;
    sim->current_time.day_of_week = 1u;
    sim->current_time.date = 1u;
    sim->current_time.month = 1u;
    sim->current_time.year = 2000u;

    /* Register mặc định */
    sim->regs[DS1307_REG_SECONDS] = 0x00u;
    sim->regs[DS1307_REG_MINUTES] = 0x00u;
    sim->regs[DS1307_REG_HOURS] = 0x00u;
    sim->regs[DS1307_REG_DAY] = 0x01u;
    sim->regs[DS1307_REG_DATE] = 0x01u;
    sim->regs[DS1307_REG_MONTH] = 0x01u;
    sim->regs[DS1307_REG_YEAR] = 0x00u;
    sim->regs[DS1307_REG_CONTROL] = 0x00u;

    ds1307_simu_sync_regs_from_time(sim);

    /* State cho I2C adapter */
    sim->current_reg = 0u;
    sim->tx_reg = 0u;
    sim->got_reg_pointer = 0u;

    sim->active_bank = 0u;
    sim->tx_bank = 0u;
    sim->tx_active = 0u;

    sim->write_start_reg = 0u;
    sim->write_len = 0u;
    sim->write_pending = 0u;

    ds1307_simu_publish_regs(sim);
}

void ds1307_simu_attach(ds1307_simu_t *sim)
{
    if ((sim == NULL) || (sim->i2c == NULL))
    {
        return;
    }

    /* Đăng ký callback với I2C */
    driver_i2c_slave_set_event_callback(sim->i2c,
                                        ds1307_simu_on_i2c_event,
                                        sim);

    driver_i2c_slave_set_tx_byte_callback(sim->i2c,
                                          ds1307_simu_get_tx_byte,
                                          sim);  
}

/* ============================================================
 * DS1307 CORE API
 * ============================================================ */

void ds1307_simu_set_time(ds1307_simu_t *sim,
                          const ds1307_simu_time_t *time)
{
    if ((sim == NULL) || (time == NULL))
    {
        return;
    }

    sim->current_time = *time;

    ds1307_simu_sync_regs_from_time(sim);
    ds1307_simu_publish_regs(sim);
}

void ds1307_simu_get_time(const ds1307_simu_t *sim,
                          ds1307_simu_time_t *time)
{
    if ((sim == NULL) || (time == NULL))
    {
        return;
    }

    *time = sim->current_time;
}

uint8_t ds1307_simu_is_clock_halted(const ds1307_simu_t *sim)
{
    if (sim == NULL)
    {
        return 1u;
    }

    return (uint8_t)((sim->regs[DS1307_REG_SECONDS] & DS1307_BIT_CH) ? 1u : 0u);
}

void ds1307_simu_tick_1s(ds1307_simu_t *sim)
{
    ds1307_simu_time_t *t;

    if (sim == NULL)
    {
        return;
    }

    /* Nếu bit CH = 1 thì clock halt, không tick */
    if (ds1307_simu_is_clock_halted(sim))
    {
        return;
    }

    t  = &sim->current_time;

     /* Tăng giây */
    t->second++;

    if (t->second >= 60u)
    {
        t->second = 0u;
        t->minute++;

        if (t->minute >= 60u)
        {
            t->minute = 0u;
            t->hour++;

            if (t->hour >= 24u)
            {
                t->hour = 0u;

                t->day_of_week++;
                if (t->day_of_week > 7u)
                {
                    t->day_of_week = 1u;
                }

                t->date++;
                if (t->date > ds1307_simu_days_in_month(t->month, t->year))
                {
                    t->date = 1u;
                    t->month++;

                    if (t->month > 12u)
                    {
                        t->month = 1u;
                        t->year++;

                        if (t->year > 2099u)
                        {
                            t->year = 2000u;
                        }
                    }
                }
            }
        }
    }

    /* Sau khi tick, sync time sang register và publish ra bank */
    ds1307_simu_sync_regs_from_time(sim);
    ds1307_simu_publish_regs(sim);
}

#if (APP_DS1307_SIMU_DEBUG == 1u)
uint8_t ds1307_simu_read_reg(const ds1307_simu_t *sim,
                             uint8_t reg)
{
    if (sim == NULL)
    {
        return 0xFFu;
    }

    reg = (uint8_t)(reg & 0x3Fu);

    return sim->regs[reg];
}

void ds1307_simu_write_reg(ds1307_simu_t *sim,
                           uint8_t reg,
                           uint8_t value)
{
    if (sim == NULL)
    {
        return;
    }

    reg = (uint8_t)(reg & 0x3Fu);

    switch (reg)
    {
    case DS1307_REG_SECONDS:
        /* Giữ đúng format seconds, bao gồm bit CH */
        sim->regs[reg] = (uint8_t)(value & 0x7Fu);
        sim->regs[reg] |= (uint8_t)(value & DS1307_BIT_CH);
        break;

    case DS1307_REG_MINUTES:
        sim->regs[reg] = (uint8_t)(value & DS1307_MASK_MINUTES);
        break;

    case DS1307_REG_HOURS:
    {
        uint8_t new_hour_24;
        uint8_t h12;

        if ((value & DS1307_BIT_12H) != 0u)
        {
            h12 = ds1307_simu_bcd_to_bin((uint8_t)(value & DS1307_MASK_HOURS_12));

            if ((h12 < 1u) || (h12 > 12u))
            {
                sim->regs[DS1307_REG_HOURS] =
                    ds1307_simu_encode_hour_from_24h(sim->current_time.hour,
                                                     DS1307_BIT_12H);
            }
            else
            {
                new_hour_24 = ds1307_simu_decode_hour_to_24h(value);
                sim->current_time.hour = new_hour_24;

                sim->regs[DS1307_REG_HOURS] =
                    ds1307_simu_encode_hour_from_24h(sim->current_time.hour,
                                                     value);
            }
        }
        else
        {
            new_hour_24 =
                ds1307_simu_bcd_to_bin((uint8_t)(value & DS1307_MASK_HOURS_24));

            if (new_hour_24 > 23u)
            {
                new_hour_24 = 23u;
            }

            sim->current_time.hour = new_hour_24;

            sim->regs[DS1307_REG_HOURS] =
                ds1307_simu_encode_hour_from_24h(sim->current_time.hour,
                                                 value);
        }

        break;
    }

    case DS1307_REG_DAY:
        sim->regs[reg] = (uint8_t)(value & DS1307_MASK_DAY);
        break;

    case DS1307_REG_DATE:
        sim->regs[reg] = (uint8_t)(value & DS1307_MASK_DATE);
        break;

    case DS1307_REG_MONTH:
        sim->regs[reg] = (uint8_t)(value & DS1307_MASK_MONTH);
        break;

    case DS1307_REG_YEAR:
        sim->regs[reg] = value;
        break;

    case DS1307_REG_CONTROL:
        sim->regs[reg] = value;
        break;

    default:
        /* Vùng RAM user từ 0x08 -> 0x3F */
        if (reg >= DS1307_REG_RAM_BASE)
        {
            sim->regs[reg] = value;
        }
        break;
    }

    /* Nếu ghi register thời gian thì cập nhật lại current_time */
    if ((reg <= DS1307_REG_YEAR) && (reg != DS1307_REG_HOURS))
    {
        ds1307_simu_sync_time_from_regs(sim);
    }

    ds1307_simu_publish_regs(sim);
}

void ds1307_simu_read_regs(const ds1307_simu_t *sim,
                           uint8_t start_reg,
                           uint8_t *buffer,
                           size_t length)
{
    size_t i;
    uint8_t reg;

    if ((sim == NULL) || (buffer == NULL))
    {
        return;
    }

    reg = (uint8_t)(start_reg & 0x3Fu);

    for (i = 0u; i < length; i++)
    {
        buffer[i] = ds1307_simu_read_reg(sim, reg);
        reg = (uint8_t)((reg + 1u) & 0x3Fu);
    }
}

void ds1307_simu_write_regs(ds1307_simu_t *sim,
                            uint8_t start_reg,
                            const uint8_t *buffer,
                            size_t length)
{
    size_t i;
    uint8_t reg;

    if ((sim == NULL) || (buffer == NULL))
    {
        return;
    }

    reg = (uint8_t)(start_reg & 0x3Fu);

    for (i = 0u; i < length; i++)
    {
        ds1307_simu_write_reg(sim, reg, buffer[i]);
        reg = (uint8_t)((reg + 1u) & 0x3Fu);
    }

    ds1307_simu_publish_regs(sim);
}
#endif

void ds1307_simu_write_transaction(ds1307_simu_t *sim,
                                   uint8_t start_reg,
                                   const uint8_t *data,
                                   uint8_t length)
{
    uint8_t i;
    uint8_t reg;

    if ((sim == NULL) || (data == NULL))
    {
        return;
    }

    reg = (uint8_t)(start_reg & 0x3Fu);

    for (i = 0u; i < length; i++)
    {
        sim->regs[reg] = data[i];
        reg = (uint8_t)((reg + 1u) & 0x3Fu);
    }

    /* Nếu transaction chạm vùng time thì sync lại time/register */
    if (start_reg <= DS1307_REG_YEAR)
    {
        ds1307_simu_sync_time_from_regs(sim);
        ds1307_simu_sync_regs_from_time(sim);
    }

    ds1307_simu_publish_regs(sim);
}

/* ============================================================
 * DS1307 I2C TRANSACTION / STAGING API
 * ============================================================ */

void ds1307_simu_set_write_done_callback(ds1307_simu_t *sim,
                                         ds1307_simu_write_done_cb_t cb,
                                         void *ctx)
{
    if (sim == NULL)
    {
        return;
    }

    sim->write_done_cb = cb;
    sim->write_done_ctx = ctx;
}

uint8_t ds1307_simu_has_pending_write(const ds1307_simu_t *sim)
{
    if (sim == NULL)
    {
        return 0u;
    }

    return sim->write_pending;
}

void ds1307_simu_get_staged_transaction(ds1307_simu_t *sim,
                                        uint8_t *start_reg,
                                        uint8_t buffer[DS1307_REG_COUNT],
                                        uint8_t *length)
{
    if ((sim == NULL) ||
        (start_reg == NULL) ||
        (buffer == NULL) ||
        (length == NULL))
    {
        return;
    }

    *start_reg = sim->write_start_reg;
    *length = sim->write_len;

    if (sim->write_len > 0u)
    {
        memcpy(buffer, sim->write_buf, sim->write_len);
    }
}

void ds1307_simu_clear_staged_transaction(ds1307_simu_t *sim)
{
    if (sim == NULL)
    {
        return;
    }

    sim->write_len = 0u;
    sim->write_pending = 0u;
}

void ds1307_simu_apply_pending_write(ds1307_simu_t *sim)
{
    uint8_t start_reg;
    uint8_t length;
    uint8_t buffer[DS1307_REG_COUNT];

    if (sim == NULL)
    {
        return;
    }

    if (ds1307_simu_has_pending_write(sim) == 0u)
    {
        return;
    }

    ds1307_simu_get_staged_transaction(sim,
                                       &start_reg,
                                       buffer,
                                       &length);

    ds1307_simu_clear_staged_transaction(sim);

    ds1307_simu_write_transaction(sim,
                                  start_reg,
                                  buffer,
                                  length);
}

/* ============================================================
 * DS1307 I2C DRIVER CALLBACKS
 * ============================================================
 * Các hàm này là private static.
 * App không gọi trực tiếp.
 * App chỉ gọi ds1307_simu_attach().
 */

static void ds1307_simu_on_i2c_event(void *ctx,
                                     const driver_i2c_slave_evt_t *evt)
{
    ds1307_simu_t *sim;

    if ((ctx == NULL) || (evt == NULL))
    {
        return;
    }

    sim = (ds1307_simu_t *)ctx;

    switch (evt->type)
    {
    case DRIVER_I2C_SLAVE_EVT_WRITE_START:
        /* Bắt đầu phiên master write */
        if (sim->write_pending != 0u)
        {
            ds1307_simu_emit_write_done(sim);
        }

        sim->got_reg_pointer = 0u;
        sim->tx_active = 0u;
        break;

    case DRIVER_I2C_SLAVE_EVT_BYTE_RECEIVED:
        if (sim->got_reg_pointer == 0u)
        {
            /* Byte đầu tiên là register pointer */
            sim->current_reg = ds1307_simu_mask_reg(evt->data);
            sim->write_start_reg = sim->current_reg;
            sim->write_len = 0u;
            sim->got_reg_pointer = 1u;
        }
        else
        {
            /* Các byte sau là data, chỉ staging, chưa apply ngay */
            if (sim->write_len < DS1307_REG_COUNT)
            {
                sim->write_buf[sim->write_len] = evt->data;
                sim->write_len++;
                sim->write_pending = 1u;
            }

            sim->current_reg = ds1307_simu_next_reg(sim->current_reg);
        }
        break;

    case DRIVER_I2C_SLAVE_EVT_WRITE_DONE:
        /* STOP sau write: báo app/task apply transaction */
        sim->got_reg_pointer = 0u;

        if (sim->write_pending != 0u)
        {
            ds1307_simu_emit_write_done(sim);
        }
        break;

    case DRIVER_I2C_SLAVE_EVT_READ_START:
        /* Bắt đầu phiên master read */
        sim->tx_bank = sim->active_bank;
        sim->tx_reg = sim->current_reg;
        sim->tx_active = 1u;
        break;

    case DRIVER_I2C_SLAVE_EVT_BYTE_SENT:
        /* Không cần xử lý thêm, tx_reg đã tăng trong get_tx_byte */
        break;

    case DRIVER_I2C_SLAVE_EVT_READ_DONE:
        /* Kết thúc read, cập nhật lại current_reg */
        sim->current_reg = sim->tx_reg;
        sim->tx_active = 0u;
        break;

    case DRIVER_I2C_SLAVE_EVT_ERROR:
    default:
        /* Lỗi I2C: reset state adapter */
        sim->tx_active = 0u;
        sim->got_reg_pointer = 0u;
        break;
    }
}

static uint8_t ds1307_simu_get_tx_byte(void *ctx)
{
    ds1307_simu_t *sim;
    uint8_t bank;
    uint8_t reg;
    uint8_t data;

    if (ctx == NULL)
    {
        return 0xFFu;
    }

    sim = (ds1307_simu_t *)ctx;

    bank = sim->tx_bank;
    reg = ds1307_simu_mask_reg(sim->tx_reg);

    data = sim->reg_bank[bank][reg];

    sim->tx_reg = ds1307_simu_next_reg(sim->tx_reg);

    return data;
}








/* ============================================================
 * HARDWARE MAPPING
 * ============================================================ */

#define APP_DS1307_I2C_INSTANCE         I2C1
#define APP_DS1307_I2C_DUTY_CYCLE       I2C_DutyCycle_2

#define APP_DS1307_GPIO_PORT            GPIOB
#define APP_DS1307_GPIO_CLK             RCC_APB2Periph_GPIOB
#define APP_DS1307_GPIO_AF_CLK          RCC_APB2Periph_AFIO

#define APP_DS1307_GPIO_PIN_SCL         GPIO_Pin_6
#define APP_DS1307_GPIO_PIN_SDA         GPIO_Pin_7

#define APP_DS1307_I2C_CLK              RCC_APB1Periph_I2C1

#define APP_DS1307_I2C_EV_IRQ           I2C1_EV_IRQn
#define APP_DS1307_I2C_ER_IRQ           I2C1_ER_IRQn

#define APP_DS1307_IRQ_PREEMPT_PRIO     6u
#define APP_DS1307_IRQ_SUB_PRIO         0u

#define APP_DS1307_TASK_PRIORITY        (tskIDLE_PRIORITY + APP_DS1307_TASK_PRIORITY_OFFSET)


/* ============================================================
 * PRIVATE OBJECTS
 * ============================================================ */

static driver_i2c_slave_t g_app_ds1307_i2c;
static ds1307_simu_t g_app_ds1307_simu;

static SemaphoreHandle_t g_app_ds1307_write_sem;
static SemaphoreHandle_t g_app_ds1307_mutex;

static TaskHandle_t g_app_ds1307_task_handle;
static uint8_t g_app_ds1307_initialized;
static uint8_t g_app_ds1307_running;


/* ============================================================
 * PRIVATE PROTOTYPES
 * ============================================================ */

static void app_ds1307_hw_init(void);
static void app_ds1307_write_done_callback(void *ctx);
static void app_ds1307_task(void *argument);
static void ds1307_sim_start_task(void);


/* ============================================================
 * INIT
 * ============================================================ */

void sensor_ds1307_init(void)
{
    if (g_app_ds1307_initialized != 0u)
    {
        return;
    }

    /* Init hardware I2C1 GPIO/RCC/NVIC */
    app_ds1307_hw_init();

    /* Init I2C slave driver */
    driver_i2c_slave_init(&g_app_ds1307_i2c,
                          APP_DS1307_I2C_INSTANCE,
                          APP_DS1307_I2C_OWN_ADDRESS,
                          APP_DS1307_I2C_CLOCK_SPEED,
                          APP_DS1307_I2C_DUTY_CYCLE);

    /* Init DS1307 simulator */
    ds1307_simu_init(&g_app_ds1307_simu,
                     &g_app_ds1307_i2c);

    /* Attach simulator vào driver callback */
    ds1307_simu_attach(&g_app_ds1307_simu);

    /* Semaphore báo master write xong */
    g_app_ds1307_write_sem = xSemaphoreCreateBinary();

    /* Mutex bảo vệ DS1307 simulator khi task tick/apply write */
    g_app_ds1307_mutex = xSemaphoreCreateMutex();

    /* Callback này được gọi khi DS1307 simulator nhận write done */
    ds1307_simu_set_write_done_callback(&g_app_ds1307_simu, app_ds1307_write_done_callback, NULL);

    g_app_ds1307_initialized = 1u;
}

void sensor_ds1307_run(void)
{
    if (g_app_ds1307_initialized == 0u)
    {
        sensor_ds1307_init();
    }

    if (g_app_ds1307_running != 0u)
    {
        return;
    }

    ds1307_sim_start_task();
    g_app_ds1307_running = 1u;
}

void ds1307_sim_init(void)
{
    sensor_ds1307_init();
    sensor_ds1307_run();
}


/* ============================================================
 * START
 * ============================================================ */

static void ds1307_sim_start_task(void)
{
    /* Tạo task quản lý DS1307 */
    xTaskCreate(app_ds1307_task,
                "app_ds1307",
                APP_DS1307_TASK_STACK_SIZE,
                NULL,
                APP_DS1307_TASK_PRIORITY,
                &g_app_ds1307_task_handle);

    /* Start I2C slave sau khi callback/task đã sẵn sàng */
    driver_i2c_slave_start(&g_app_ds1307_i2c);
}


/* ============================================================
 * HARDWARE INIT
 * ============================================================
 * Cấu hình phần cứng cho I2C1 slave:
 * - RCC
 * - GPIO PB6/PB7 alternate open-drain
 * - NVIC I2C event/error interrupt
 */

static void app_ds1307_hw_init(void)
{
    RCC_APB2PeriphClockCmd(APP_DS1307_GPIO_CLK | APP_DS1307_GPIO_AF_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(APP_DS1307_I2C_CLK, ENABLE);

    gpio_open(APP_DS1307_GPIO_PORT, APP_DS1307_GPIO_PIN_SCL, GPIO_Mode_AF_OD, GPIO_Speed_50MHz);
    gpio_open(APP_DS1307_GPIO_PORT, APP_DS1307_GPIO_PIN_SDA, GPIO_Mode_AF_OD, GPIO_Speed_50MHz);

    /* Set priority group trước khi enable interrupt */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = APP_DS1307_I2C_EV_IRQ;
    nvic.NVIC_IRQChannelPreemptionPriority = APP_DS1307_IRQ_PREEMPT_PRIO;
    nvic.NVIC_IRQChannelSubPriority = APP_DS1307_IRQ_SUB_PRIO;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = APP_DS1307_I2C_ER_IRQ;
    NVIC_Init(&nvic);
}


/* ============================================================
 * WRITE DONE CALLBACK
 * ============================================================
 * Callback này đi từ luồng I2C event.
 * Vì event gốc đến từ ISR, dùng xSemaphoreGiveFromISR().
 */

static void app_ds1307_write_done_callback(void *ctx)
{
    BaseType_t higher_priority_task_woken;

    (void)ctx;

    higher_priority_task_woken = pdFALSE;

    if (g_app_ds1307_write_sem != NULL)
    {
        xSemaphoreGiveFromISR(g_app_ds1307_write_sem,
                              &higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}


/* ============================================================
 * DS1307 TASK
 * ============================================================
 * Task này thay service task cũ.
 *
 * Nhiệm vụ:
 * 1. Nếu master write xong:
 *      - apply pending write vào DS1307 register.
 *
 * 2. Nếu timeout 1 giây:
 *      - tick DS1307 lên 1 giây.
 */

static void app_ds1307_task(void *argument)
{
    TickType_t period_ticks;

    (void)argument;

    period_ticks = pdMS_TO_TICKS(APP_DS1307_TICK_PERIOD_MS);

    while (1)
    {
        if ((g_app_ds1307_write_sem != NULL) &&
            (xSemaphoreTake(g_app_ds1307_write_sem,
                            period_ticks) == pdTRUE))
        {
            do
            {
                if (g_app_ds1307_mutex != NULL)
                {
                    xSemaphoreTake(g_app_ds1307_mutex,
                                   portMAX_DELAY);
                }

                ds1307_simu_apply_pending_write(&g_app_ds1307_simu);

                if (g_app_ds1307_mutex != NULL)
                {
                    xSemaphoreGive(g_app_ds1307_mutex);
                }
            }
            while (xSemaphoreTake(g_app_ds1307_write_sem, 0u) == pdTRUE);
        }
        else
        {
            if (g_app_ds1307_mutex != NULL)
            {
                xSemaphoreTake(g_app_ds1307_mutex,
                               portMAX_DELAY);
            }

            ds1307_simu_tick_1s(&g_app_ds1307_simu);

            if (g_app_ds1307_mutex != NULL)
            {
                xSemaphoreGive(g_app_ds1307_mutex);
            }
        }
    }
}


/* ============================================================
 * I2C IRQ HANDLERS
 * ============================================================
 * ISR chỉ gọi driver.
 * Driver sẽ emit event lên ds1307_simu thông qua callback đã attach.
 */

void I2C1_EV_IRQHandler(void)
{
    driver_i2c_slave_ev_handler(&g_app_ds1307_i2c);
}

void I2C1_ER_IRQHandler(void)
{
    driver_i2c_slave_er_handler(&g_app_ds1307_i2c);
}

#endif // ENABLE_DS1307

