#include "dht11_sim.h"

/* ================================================================
 *  DHT11 PROTOCOL TIMING (µs)
 *  Giữ nguyên từ code test gốc đã chạy được
 * ================================================================ */
#define DHT_RESP_LOW      80u
#define DHT_RESP_HIGH     80u
#define DHT_BIT_LOW       50u
#define DHT_BIT_0_HIGH    26u
#define DHT_BIT_1_HIGH    70u
#define DHT_STOP_LOW      50u
#define DHT_STOP_CLEANUP  10u

/* FreeRTOS ISR-safe API chỉ được gọi từ ngắt có priority số >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY. */
#define DHT_TIM1_CC_IRQ_PRIO   6u
#define DHT_TIM2_IC_IRQ_PRIO   7u

/* Start signal từ host: LOW 18ms..30ms */
#define DHT_START_MIN  18000u
#define DHT_START_MAX  30000u

/* ================================================================
 *  TX STATE MACHINE — chạy hoàn toàn trong TIM1 CC ISR
 * ================================================================ */
typedef enum {
    TX_IDLE = 0,
    TX_RESP_LOW,
    TX_RESP_HIGH,
    TX_BIT_LOW,
    TX_BIT_HIGH,
    TX_STOP_LOW,
    TX_DONE
} prv_TxState_t;

/* ================================================================
 *  MODULE STATE — tất cả private, không expose ra ngoài
 * ================================================================ */
static struct {
    /* FreeRTOS */
    TaskHandle_t   task_handle;
    SemaphoreHandle_t start_sem;   /* IC ISR → Task: start hợp lệ   */
    QueueHandle_t  data_queue;     /* set_data() → Task: update data */

    /* Config */
    DHT11_Sim_Cfg_t cfg;
    uint8_t         initialized;
    uint8_t         running;

    /* DHT11 data frame */
    uint8_t data[5];               /* [hum_int, hum_dec, tmp_int, tmp_dec, cksum] */

    /* TX state machine (dialed từ ISR) */
    volatile prv_TxState_t tx_state;
    volatile uint8_t       bit_idx;   /* 0..39 */

    /* IC state (dialed từ ISR) */
    volatile uint8_t  ic_edge;        /* 0=chờ falling, 1=chờ rising */
    volatile uint16_t ic_t_fall;
} s;

/* ================================================================
 *  PRIVATE — HARDWARE HELPERS
 *  Giữ nguyên logic từ code gốc, chỉ đổi tên hàm
 * ================================================================ */

/* PA0 → Output Open-Drain (đang phát data) */
static void prv_pa0_tx_mode(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_0);   /* giữ HIGH tránh glitch khi chuyển mode */
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetBits(GPIOA, GPIO_Pin_0);   /* idle HIGH */
}

/* PA0 → Input Pull-Up (chờ host) */
static void prv_pa0_ic_mode(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* Reset IC state, về bắt Falling edge */
    s.ic_edge = 0;
    TIM2->CCER |= TIM_CCER_CC1P;              /* CC1P=1 → Falling   */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);  /* tránh fire giả      */
}

/* Schedule OC event sau `us` µs (tính từ CNT hiện tại) */
static inline void prv_oc_schedule(uint16_t us)
{
    TIM1->CCR1 = (uint16_t)(TIM1->CNT + us);
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
}

/* Lấy bit thứ bit_idx trong frame 40 bit (MSB first) */
static inline uint8_t prv_current_bit(void)
{
    uint8_t byte_idx = s.bit_idx >> 3;
    uint8_t shift    = 7u - (s.bit_idx & 0x07u);
    return (s.data[byte_idx] >> shift) & 0x01u;
}

/* Build 5 byte data frame từ hum/tmp hiện tại */
static void prv_build_frame(uint8_t hum, uint8_t tmp)
{
    s.data[0] = hum;
    s.data[1] = 0;
    s.data[2] = tmp;
    s.data[3] = 0;
    s.data[4] = (s.data[0] + s.data[1] + s.data[2] + s.data[3]) & 0xFFu;
}

/* ================================================================
 *  PRIVATE — HARDWARE INIT
 *  Tách ra khỏi timer_driver để giữ nguyên timing chính xác từ
 *  code gốc (dùng register trực tiếp, free-running counter 0xFFFF)
 * ================================================================ */
static void prv_tim1_oc_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef tb;
    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler   = (uint16_t)((SystemCoreClock / 1000000u) - 1u); /* 1 tick = 1µs */
    tb.TIM_Period      = 0xFFFFu;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tb);

    TIM_OCInitTypeDef oc;
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_Timing;        /* không kéo chân, chỉ interrupt */
    oc.TIM_OutputState = TIM_OutputState_Disable;
    oc.TIM_Pulse       = 0;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);

    /* CC1 interrupt disable — chỉ enable khi bắt đầu phát */
    TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);

    /* FIX: TIM1 advanced timer cần enable MOE */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    /* TIM1 CC vector — priority cao hơn IC để timing chính xác */
    NVIC_SetPriority(TIM1_CC_IRQn, DHT_TIM1_CC_IRQ_PRIO);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}

static void prv_tim2_ic_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA0 = Input Pull-Up mặc định */
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseInitTypeDef tb;
    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = (uint16_t)((SystemCoreClock / 1000000u) - 1u); /* 1 tick = 1µs */
    tb.TIM_Period    = 0xFFFFu;
    TIM_TimeBaseInit(TIM2, &tb);

    TIM_ICInitTypeDef ic;
    ic.TIM_Channel     = TIM_Channel_1;
    ic.TIM_ICPolarity  = TIM_ICPolarity_Falling; /* bắt đầu chờ host kéo LOW */
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter    = 0x0Fu;                  /* filter noise */
    TIM_ICInit(TIM2, &ic);

    /* IC interrupt disable — enable sau khi run() được gọi */
    TIM_ITConfig(TIM2, TIM_IT_CC1, DISABLE);
    TIM_Cmd(TIM2, ENABLE);

    NVIC_SetPriority(TIM2_IRQn, DHT_TIM2_IC_IRQ_PRIO);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/* ================================================================
 *  PRIVATE — DHT11 TASK
 *
 *  Vòng lặp:
 *    1. Block indefinitely chờ start_sem (từ IC ISR)
 *    2. Kiểm tra queue xem có data mới không (non-blocking)
 *    3. Build frame
 *    4. Kick-off TX (disable IC, switch PA0 → OD, schedule OC)
 *    5. Chờ TX_DONE qua notification từ TIM1 ISR
 *    6. Switch PA0 → IC, re-enable IC
 *    7. Lặp lại từ 1
 * ================================================================ */
static void prv_dht11_task(void *arg)
{
    (void)arg;

    /* Data khởi tạo từ config */
    uint8_t cur_hum = s.cfg.init_data.humidity;
    uint8_t cur_tmp = s.cfg.init_data.temperature;

    for (;;)
    {
        /* ── BƯỚC 1: Chờ start signal hợp lệ từ IC ISR ── */
        xSemaphoreTake(s.start_sem, portMAX_DELAY);

        /* ── BƯỚC 2: Nhận data mới nếu có (non-blocking) ── */
        DHT11_Data_t new_data;
        if (xQueueReceive(s.data_queue, &new_data, 0) == pdTRUE)
        {
            cur_hum = new_data.humidity;
            cur_tmp = new_data.temperature;
        }

        /* ── BƯỚC 3: Build 40-bit frame ── */
        prv_build_frame(cur_hum, cur_tmp);

        /* ── BƯỚC 4: Disable IC, switch PA0 → TX, kick-off OC ── */
        TIM_ITConfig(TIM2, TIM_IT_CC1, DISABLE);
        TIM_Cmd(TIM2, DISABLE);

        prv_pa0_tx_mode();

        s.bit_idx  = 0;
        s.tx_state = TX_RESP_LOW;

        GPIO_ResetBits(GPIOA, GPIO_Pin_0);   /* bắt đầu RESP_LOW */
        prv_oc_schedule(DHT_RESP_LOW);

        /* ── BƯỚC 5: Chờ TX hoàn thành ──
         * TIM1 ISR sẽ gửi notification khi đến TX_DONE */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* ── BƯỚC 6: Switch PA0 → IC, re-enable IC ──
         * (đã được thực hiện trong ISR TX_DONE, task chỉ cần loop lại) */
    }
}

/* ================================================================
 *  IRQ HANDLERS — đặt trong stm32f10x_it.c
 *
 *  void TIM1_CC_IRQHandler(void) { dht11_tim1_cc_irq(); }
 *  void TIM2_IRQHandler(void)    { dht11_tim2_ic_irq(); }
 * ================================================================ */

/* TIM1 CC ISR — toàn bộ TX state machine, giữ nguyên logic code gốc */
void dht11_tim1_cc_irq(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == RESET) return;
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

    BaseType_t higher_woken = pdFALSE;

    switch (s.tx_state)
    {
        case TX_RESP_LOW:
            GPIO_SetBits(GPIOA, GPIO_Pin_0);
            s.tx_state = TX_RESP_HIGH;
            prv_oc_schedule(DHT_RESP_HIGH);
            break;

        case TX_RESP_HIGH:
            GPIO_ResetBits(GPIOA, GPIO_Pin_0);
            s.tx_state = TX_BIT_LOW;
            prv_oc_schedule(DHT_BIT_LOW);
            break;

        case TX_BIT_LOW:
            GPIO_SetBits(GPIOA, GPIO_Pin_0);
            s.tx_state = TX_BIT_HIGH;
            prv_oc_schedule(prv_current_bit() ? DHT_BIT_1_HIGH : DHT_BIT_0_HIGH);
            break;

        case TX_BIT_HIGH:
            s.bit_idx++;
            if (s.bit_idx < 40u)
            {
                GPIO_ResetBits(GPIOA, GPIO_Pin_0);
                s.tx_state = TX_BIT_LOW;
                prv_oc_schedule(DHT_BIT_LOW);
            }
            else
            {
                GPIO_ResetBits(GPIOA, GPIO_Pin_0);
                s.tx_state = TX_STOP_LOW;
                prv_oc_schedule(DHT_STOP_LOW);
            }
            break;

        case TX_STOP_LOW:
            GPIO_SetBits(GPIOA, GPIO_Pin_0);
            s.tx_state = TX_DONE;
            prv_oc_schedule(DHT_STOP_CLEANUP);
            break;

        case TX_DONE:
            TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
            s.tx_state = TX_IDLE;

            /* Switch PA0 về IC mode, re-enable TIM2 */
            prv_pa0_ic_mode();
            TIM_SetCounter(TIM2, 0);
            TIM_Cmd(TIM2, ENABLE);
            TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

            /* Notify task rằng TX hoàn thành */
            vTaskNotifyGiveFromISR(s.task_handle, &higher_woken);
            portYIELD_FROM_ISR(higher_woken);
            break;

        default:
            TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
            s.tx_state = TX_IDLE;
            break;
    }
}

/* TIM2 IC ISR — phát hiện start signal, giữ nguyên logic code gốc */
void dht11_tim2_ic_irq(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) == RESET) return;
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

    BaseType_t higher_woken = pdFALSE;

    if (s.ic_edge == 0)
    {
        /* Falling edge — host bắt đầu kéo LOW */
        s.ic_t_fall = TIM_GetCapture1(TIM2);
        s.ic_edge   = 1;
        TIM2->CCER &= ~TIM_CCER_CC1P;   /* CC1P=0 → Rising edge */
    }
    else
    {
        /* Rising edge — host thả HIGH, tính độ rộng pulse */
        uint16_t t_rise    = TIM_GetCapture1(TIM2);
        uint32_t pulse_us  = (t_rise >= s.ic_t_fall)
                             ? (uint32_t)(t_rise - s.ic_t_fall)
                             : (uint32_t)(0xFFFFu - s.ic_t_fall + t_rise + 1u);
        s.ic_edge = 0;
        TIM2->CCER |= TIM_CCER_CC1P;    /* CC1P=1 → Falling edge */

        /* Start signal hợp lệ: LOW 18ms..30ms */
        if (pulse_us >= DHT_START_MIN && pulse_us <= DHT_START_MAX)
        {
            xSemaphoreGiveFromISR(s.start_sem, &higher_woken);
            portYIELD_FROM_ISR(higher_woken);
        }
    }
}

/* ================================================================
 *  PUBLIC API
 * ================================================================ */

int dht11_sim_config(const DHT11_Sim_Cfg_t *cfg)
{
    if (!cfg) return DHT11_SIM_EINVAL;
    if (cfg->init_data.humidity     > 99u) return DHT11_SIM_EINVAL;
    if (cfg->init_data.temperature  > 50u) return DHT11_SIM_EINVAL;

    s.cfg         = *cfg;
    s.initialized = 0;
    s.running     = 0;

    /* Tạo FreeRTOS objects */
    s.start_sem = xSemaphoreCreateBinary();
    if (!s.start_sem) return DHT11_SIM_ENOMEM;

    /* Queue depth=1: chỉ cần giữ giá trị mới nhất */
    s.data_queue = xQueueCreate(1, sizeof(DHT11_Data_t));
    if (!s.data_queue)
    {
        vSemaphoreDelete(s.start_sem);
        return DHT11_SIM_ENOMEM;
    }

    /* FreeRTOS Cortex-M yêu cầu toàn bộ bit priority là preemption bits. */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* Khởi tạo hardware — chưa enable interrupt */
    prv_tim1_oc_init();
    prv_tim2_ic_init();

    s.initialized = 1;
    return DHT11_SIM_OK;
}

int dht11_sim_run(void)
{
    if (!s.initialized)  return DHT11_SIM_ENOTCFG;
    if (s.running)       return DHT11_SIM_EBUSY;

    /* Tạo task */
    BaseType_t res = xTaskCreate(
        prv_dht11_task,
        "dht11_sim",
        s.cfg.task_stack,
        NULL,
        s.cfg.task_priority,
        &s.task_handle
    );
    if (res != pdPASS) return DHT11_SIM_ENOMEM;

    /* Enable IC interrupt — bắt đầu lắng nghe host */
    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

    s.running = 1;
    return DHT11_SIM_OK;
}

int dht11_sim_set_data(const DHT11_Data_t *data)
{
    if (!data)              return DHT11_SIM_EINVAL;
    if (!s.initialized)     return DHT11_SIM_ENOTCFG;

    /* Overwrite queue (không block): chỉ giữ giá trị mới nhất */
    xQueueOverwrite(s.data_queue, data);
    return DHT11_SIM_OK;
}

