#include "sensor_dht11.h"

/*
 * Timing giao thức DHT11 (đơn vị: microsecond):
 *
 *   Host start signal : kéo LOW 18–30 ms
 *   Sensor response   : LOW 80 µs → HIGH 80 µs (báo sẵn sàng)
 *   Mỗi bit dữ liệu  : LOW 50 µs → HIGH 26 µs (bit=0) hoặc 70 µs (bit=1)
 *   Stop bit          : LOW 50 µs → dọn bus HIGH 10 µs
 */
#define DHT_RESP_LOW 80u     /* Response: kéo LOW  80 µs */
#define DHT_RESP_HIGH 80u    /* Response: kéo HIGH 80 µs */
#define DHT_BIT_LOW 50u      /* Preamble mỗi bit: LOW 50 µs */
#define DHT_BIT_0_HIGH 26u   /* Bit-0: HIGH 26 µs */
#define DHT_BIT_1_HIGH 70u   /* Bit-1: HIGH 70 µs */
#define DHT_STOP_LOW 50u     /* Stop bit: LOW 50 µs */
#define DHT_STOP_CLEANUP 10u /* Dọn bus: HIGH 10 µs sau stop */

#define DHT_TIM1_CC_IRQ_PRIO 6u
#define DHT_TIM2_IC_IRQ_PRIO 7u

/* Giới hạn độ rộng xung start hợp lệ từ host (µs) */
#define DHT_START_MIN 18000u
#define DHT_START_MAX 30000u

#define SENSOR_DHT11_DEFAULT_HUMIDITY   65u
#define SENSOR_DHT11_DEFAULT_TEMPERATURE 28u
#define SENSOR_DHT11_TASK_PRIORITY      (tskIDLE_PRIORITY + 2u)
#define SENSOR_DHT11_TASK_STACK_WORDS   128u

/**
 * enum dht11_tx_state - Các trạng thái của TX state machine
 * @TX_IDLE:      Bus rảnh, chờ host
 * @TX_RESP_LOW:  Đang kéo LOW 80 µs (response giai đoạn 1)
 * @TX_RESP_HIGH: Đang kéo HIGH 80 µs (response giai đoạn 2)
 * @TX_BIT_LOW:   Đang kéo LOW 50 µs preamble cho bit hiện tại
 * @TX_BIT_HIGH:  Đang kéo HIGH cho bit-0 (26 µs) hoặc bit-1 (70 µs)
 * @TX_STOP_LOW:  Đang kéo LOW 50 µs (stop bit)
 * @TX_DONE:      Truyền xong, đang chuyển về chế độ nhận
 */
typedef enum
{
    TX_IDLE = 0,
    TX_RESP_LOW,
    TX_RESP_HIGH,
    TX_BIT_LOW,
    TX_BIT_HIGH,
    TX_STOP_LOW,
    TX_DONE
} dht11_tx_state_t;

/**
 * struct dht11_ctx - Context singleton của toàn module DHT11
 * @task_handle:       Handle của FreeRTOS task
 * @start_sem:         Semaphore báo hiệu start signal hợp lệ (give từ IC ISR)
 * @data_queue:        Queue nhận humidity/temperature mới từ application
 * @cfg:               Cấu hình được truyền vào qua dht11_sim_config()
 * @initialized:       Đặt thành 1 sau khi dht11_sim_config() thành công
 * @running:           Đặt thành 1 sau khi dht11_sim_run() thành công
 * @frame:             40-bit data frame [hum_int, hum_dec, tmp_int, tmp_dec, cksum]
 * @tx_state:          Trạng thái hiện tại của TX state machine (volatile, truy cập từ ISR)
 * @bit_index:         Vị trí bit đang truyền: 0..39, MSB-first (volatile, truy cập từ ISR)
 * @ic_awaiting_rise:  0 = đang chờ falling edge, 1 = đang chờ rising edge (volatile)
 * @ic_fall_capture:   Giá trị TIM2 capture tại falling edge (volatile)
 */
static struct dht11_ctx
{
    /* FreeRTOS */
    TaskHandle_t task_handle;
    SemaphoreHandle_t start_sem;
    QueueHandle_t data_queue;

    /* Cấu hình & trạng thái module */
    DHT11_Sim_Cfg_t cfg;
    uint8_t initialized;
    uint8_t running;

    /* DHT11 protocol frame */
    uint8_t frame[5];

    /* TX state machine — truy cập từ ISR */
    volatile dht11_tx_state_t tx_state;
    volatile uint8_t bit_index;

    /* Input capture state — truy cập từ ISR */
    volatile uint8_t ic_awaiting_rise;
    volatile uint16_t ic_fall_capture;
} g_dht11;

static uint8_t g_sensor_dht11_initialized;
static uint8_t g_sensor_dht11_running;

/* ------------------------------------------------------------------ */

/**
 * prv_pa0_set_tx_mode - Chuyển PA0 sang open-drain output để phát data
 *
 * Pre-set HIGH trước khi đổi mode để tránh glitch trên bus.
 */
static void prv_pa0_set_tx_mode(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_0);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_SetBits(GPIOA, GPIO_Pin_0); /* Idle HIGH */
}

/**
 * prv_pa0_set_rx_mode - Chuyển PA0 sang input pull-up để lắng nghe host
 *
 * Reset IC state và arm lại để bắt falling edge tiếp theo.
 */
static void prv_pa0_set_rx_mode(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    g_dht11.ic_awaiting_rise = 0;
    TIM2->CCER |= TIM_CCER_CC1P;             /* CC1P=1 → bắt falling edge */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1); /* Xóa pending cũ tránh fire giả */
}

/* ------------------------------------------------------------------ */

/**
 * prv_oc_schedule - Lên lịch CC1 interrupt sau @us microsecond kể từ bây giờ
 * @us: Độ trễ tính bằng microsecond
 */
static inline void prv_oc_schedule(uint16_t us)
{
    TIM1->CCR1 = (uint16_t)(TIM1->CNT + us);
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
}

/* ------------------------------------------------------------------ */

/**
 * prv_frame_current_bit - Trả về giá trị (0 hoặc 1) của bit đang truyền
 *
 * Frame được truyền MSB-first trong từng byte.
 */
static inline uint8_t prv_frame_current_bit(void)
{
    uint8_t byte_idx = g_dht11.bit_index >> 3;
    uint8_t bit_pos = 7u - (g_dht11.bit_index & 0x07u);

    return (g_dht11.frame[byte_idx] >> bit_pos) & 0x01u;
}

/**
 * prv_frame_build - Đóng gói humidity và temperature vào 5-byte DHT11 frame
 * @humidity:    Độ ẩm nguyên (0..99)
 * @temperature: Nhiệt độ nguyên (0..50)
 *
 * DHT11 không có phần thập phân nên byte[1] và byte[3] luôn bằng 0.
 * Byte[4] là checksum = tổng 4 byte đầu, truncate về 8 bit.
 */
static void prv_frame_build(uint8_t humidity, uint8_t temperature)
{
    g_dht11.frame[0] = humidity;
    g_dht11.frame[1] = 0u;
    g_dht11.frame[2] = temperature;
    g_dht11.frame[3] = 0u;
    g_dht11.frame[4] = (g_dht11.frame[0] + g_dht11.frame[1] + g_dht11.frame[2] + g_dht11.frame[3]) & 0xFFu;
}

/* ------------------------------------------------------------------ */

/**
 * prv_tim1_oc_init - Khởi tạo TIM1 làm output-compare timer cho TX
 *
 * TIM1 chạy ở 1 tick = 1 µs. CC1 dùng làm one-shot delay: mỗi trạng
 * thái trong TX state machine lập lịch interrupt tiếp theo qua
 * prv_oc_schedule(). GPIO được kéo thủ công trong ISR, TIM1 không
 * điều khiển chân trực tiếp (TIM_OCMode_Timing).
 *
 * TIM1 là advanced timer nên cần gọi TIM_CtrlPWMOutputs() để enable MOE,
 * dù không dùng PWM output.
 */
static void prv_tim1_oc_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef tb;
    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = (uint16_t)((SystemCoreClock / 1000000u) - 1u);
    tb.TIM_Period = 0xFFFFu;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tb);

    TIM_OCInitTypeDef oc;
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_Timing;
    oc.TIM_OutputState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 0u;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);

    TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE); /* Chỉ enable khi đang TX */

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    NVIC_SetPriority(TIM1_CC_IRQn, DHT_TIM1_CC_IRQ_PRIO);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}

/**
 * prv_tim2_ic_init - Khởi tạo TIM2 input-capture trên PA0 để phát hiện start signal
 *
 * TIM2 đo độ rộng xung LOW từ host. Xung hợp lệ nằm trong [18 ms, 30 ms].
 * IC ISR dùng kỹ thuật two-edge: bắt falling để ghi timestamp, bắt rising
 * để tính pulse width, sau đó đổi chiều bắt cho lần kế tiếp.
 */
static void prv_tim2_ic_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseInitTypeDef tb;
    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = (uint16_t)((SystemCoreClock / 1000000u) - 1u);
    tb.TIM_Period = 0xFFFFu;
    TIM_TimeBaseInit(TIM2, &tb);

    TIM_ICInitTypeDef ic;
    TIM_ICStructInit(&ic);
    ic.TIM_Channel = TIM_Channel_1;
    ic.TIM_ICPolarity = TIM_ICPolarity_Falling;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter = 0x0Fu; /* Lọc nhiễu */
    TIM_ICInit(TIM2, &ic);

    TIM_ITConfig(TIM2, TIM_IT_CC1, DISABLE); /* Enable sau dht11_sim_run() */
    TIM_Cmd(TIM2, ENABLE);

    NVIC_SetPriority(TIM2_IRQn, DHT_TIM2_IC_IRQ_PRIO);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/* ------------------------------------------------------------------ */

/**
 * prv_dht11_task - Task chính điều phối toàn bộ chu kỳ giao tiếp DHT11
 * @arg: Không sử dụng
 *
 * Vòng lặp chính:
 *   1. Block trên start_sem cho đến khi IC ISR báo start signal hợp lệ.
 *   2. Lấy data mới từ data_queue (non-blocking, giữ giá trị cũ nếu không có).
 *   3. Build 40-bit frame.
 *   4. Chuyển PA0 sang TX, tắt TIM2, kick-off TIM1 OC sequence.
 *   5. Block chờ notification từ TIM1 ISR khi TX_DONE.
 *   6. TIM1 ISR đã restore PA0 và TIM2, task chỉ cần loop lại.
 */
static void prv_dht11_task(void *arg)
{
    (void)arg;

    uint8_t cur_humidity = g_dht11.cfg.init_data.humidity;
    uint8_t cur_temperature = g_dht11.cfg.init_data.temperature;

    for (;;)
    {
        xSemaphoreTake(g_dht11.start_sem, portMAX_DELAY);

        DHT11_Data_t new_data;
        if (xQueueReceive(g_dht11.data_queue, &new_data, 0) == pdTRUE)
        {
            cur_humidity = new_data.humidity;
            cur_temperature = new_data.temperature;
        }

        prv_frame_build(cur_humidity, cur_temperature);

        TIM_ITConfig(TIM2, TIM_IT_CC1, DISABLE);
        TIM_Cmd(TIM2, DISABLE);

        prv_pa0_set_tx_mode();

        g_dht11.bit_index = 0u;
        g_dht11.tx_state = TX_RESP_LOW;

        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        prv_oc_schedule(DHT_RESP_LOW);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

/* ------------------------------------------------------------------ */

/*
 * IRQ handlers — khai báo trong stm32f10x_it.c:
 *
 *   void TIM1_CC_IRQHandler(void) { dht11_tim1_cc_irq(); }
 *   void TIM2_IRQHandler(void)    { dht11_tim2_ic_irq(); }
 */

/**
 * dht11_tim1_cc_irq - TIM1 CC ISR, thực thi TX state machine
 *
 * Mỗi trạng thái kéo PA0 và lên lịch interrupt tiếp theo.
 * Khi đến TX_DONE: restore PA0 về RX, re-enable TIM2, notify task.
 */
void dht11_tim1_cc_irq(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == RESET)
        return;
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

    BaseType_t higher_woken = pdFALSE;

    switch (g_dht11.tx_state)
    {
    case TX_RESP_LOW:
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
        g_dht11.tx_state = TX_RESP_HIGH;
        prv_oc_schedule(DHT_RESP_HIGH);
        break;

    case TX_RESP_HIGH:
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        g_dht11.tx_state = TX_BIT_LOW;
        prv_oc_schedule(DHT_BIT_LOW);
        break;

    case TX_BIT_LOW:
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
        g_dht11.tx_state = TX_BIT_HIGH;
        prv_oc_schedule(prv_frame_current_bit() ? DHT_BIT_1_HIGH : DHT_BIT_0_HIGH);
        break;

    case TX_BIT_HIGH:
        g_dht11.bit_index++;
        if (g_dht11.bit_index < 40u)
        {
            GPIO_ResetBits(GPIOA, GPIO_Pin_0);
            g_dht11.tx_state = TX_BIT_LOW;
            prv_oc_schedule(DHT_BIT_LOW);
        }
        else
        {
            GPIO_ResetBits(GPIOA, GPIO_Pin_0);
            g_dht11.tx_state = TX_STOP_LOW;
            prv_oc_schedule(DHT_STOP_LOW);
        }
        break;

    case TX_STOP_LOW:
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
        g_dht11.tx_state = TX_DONE;
        prv_oc_schedule(DHT_STOP_CLEANUP);
        break;

    case TX_DONE:
        TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
        g_dht11.tx_state = TX_IDLE;

        prv_pa0_set_rx_mode();
        TIM_SetCounter(TIM2, 0u);
        TIM_Cmd(TIM2, ENABLE);
        TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

        vTaskNotifyGiveFromISR(g_dht11.task_handle, &higher_woken);
        portYIELD_FROM_ISR(higher_woken);
        break;

    default:
        TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
        g_dht11.tx_state = TX_IDLE;
        break;
    }
}

/**
 * dht11_tim2_ic_irq - TIM2 IC ISR, phát hiện start signal từ host
 *
 * Dùng kỹ thuật two-edge capture:
 *   - Falling edge: ghi timestamp vào ic_fall_capture, chuyển bắt rising.
 *   - Rising edge: tính pulse width, nếu hợp lệ thì give start_sem.
 *
 * Wrap-around của timer 16-bit được xử lý trong phép tính pulse_us.
 */
void dht11_tim2_ic_irq(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) == RESET)
        return;
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

    BaseType_t higher_woken = pdFALSE;

    if (!g_dht11.ic_awaiting_rise)
    {
        /* Falling edge: host bắt đầu kéo bus LOW */
        g_dht11.ic_fall_capture = TIM_GetCapture1(TIM2);
        g_dht11.ic_awaiting_rise = 1u;
        TIM2->CCER &= ~TIM_CCER_CC1P; /* Chuyển sang bắt rising edge */
    }
    else
    {
        /* Rising edge: host thả bus, tính độ rộng xung LOW */
        uint16_t t_rise = TIM_GetCapture1(TIM2);
        uint32_t pulse_us = (t_rise >= g_dht11.ic_fall_capture) ? (uint32_t)(t_rise - g_dht11.ic_fall_capture) : (uint32_t)(0xFFFFu - g_dht11.ic_fall_capture + t_rise + 1u);

        g_dht11.ic_awaiting_rise = 0u;
        TIM2->CCER |= TIM_CCER_CC1P; /* Chuyển lại bắt falling edge */

        if (pulse_us >= DHT_START_MIN && pulse_us <= DHT_START_MAX)
        {
            xSemaphoreGiveFromISR(g_dht11.start_sem, &higher_woken);
            portYIELD_FROM_ISR(higher_woken);
        }
    }
}

/* ------------------------------------------------------------------ */

/**
 * dht11_sim_config - Cấu hình module và khởi tạo hardware
 * @cfg: Con trỏ đến cấu hình, không được NULL
 *
 * Tạo FreeRTOS semaphore và queue, khởi tạo TIM1/TIM2.
 * Chưa enable interrupt và chưa tạo task.
 *
 * Return: DHT11_SIM_OK nếu thành công, mã lỗi âm nếu thất bại.
 */
int dht11_sim_config(const DHT11_Sim_Cfg_t *cfg)
{
    if (!cfg)
        return DHT11_SIM_EINVAL;
    if (cfg->init_data.humidity > 99u)
        return DHT11_SIM_EINVAL;
    if (cfg->init_data.temperature > 50u)
        return DHT11_SIM_EINVAL;

    g_dht11.cfg = *cfg;
    g_dht11.initialized = 0u;
    g_dht11.running = 0u;

    g_dht11.start_sem = xSemaphoreCreateBinary();
    if (!g_dht11.start_sem)
        return DHT11_SIM_ENOMEM;

    /* Depth=1: chỉ cần giữ giá trị mới nhất */
    g_dht11.data_queue = xQueueCreate(1u, sizeof(DHT11_Data_t));
    if (!g_dht11.data_queue)
    {
        vSemaphoreDelete(g_dht11.start_sem);
        return DHT11_SIM_ENOMEM;
    }

    /* FreeRTOS trên Cortex-M yêu cầu toàn bộ priority bits là preemption bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    prv_tim1_oc_init();
    prv_tim2_ic_init();

    g_dht11.initialized = 1u;
    return DHT11_SIM_OK;
}

/**
 * dht11_sim_run - Tạo task và bắt đầu lắng nghe host
 *
 * Phải gọi dht11_sim_config() trước. Chỉ được gọi một lần.
 *
 * Return: DHT11_SIM_OK nếu thành công, mã lỗi âm nếu thất bại.
 */
int dht11_sim_run(void)
{
    if (!g_dht11.initialized)
        return DHT11_SIM_ENOTCFG;
    if (g_dht11.running)
        return DHT11_SIM_EBUSY;

    BaseType_t res = xTaskCreate(
        prv_dht11_task,
        "dht11_sim",
        g_dht11.cfg.task_stack,
        NULL,
        g_dht11.cfg.task_priority,
        &g_dht11.task_handle);

    if (res != pdPASS)
        return DHT11_SIM_ENOMEM;

    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

    g_dht11.running = 1u;
    return DHT11_SIM_OK;
}

/**
 * dht11_sim_set_data - Cập nhật giá trị humidity/temperature sẽ trả về
 * @data: Con trỏ đến data mới, không được NULL
 *
 * Có thể gọi từ bất kỳ context nào (task hoặc ISR không quan trọng).
 * Dùng xQueueOverwrite nên không block, luôn giữ giá trị mới nhất.
 *
 * Return: DHT11_SIM_OK nếu thành công, mã lỗi âm nếu thất bại.
 */
int dht11_sim_set_data(const DHT11_Data_t *data)
{
    if (!data)
        return DHT11_SIM_EINVAL;
    if (!g_dht11.initialized)
        return DHT11_SIM_ENOTCFG;

    xQueueOverwrite(g_dht11.data_queue, data);
    return DHT11_SIM_OK;
}

void sensor_dht11_init(void)
{
    DHT11_Sim_Cfg_t cfg;

    if (g_sensor_dht11_initialized != 0u)
    {
        return;
    }

    cfg.init_data.humidity = SENSOR_DHT11_DEFAULT_HUMIDITY;
    cfg.init_data.temperature = SENSOR_DHT11_DEFAULT_TEMPERATURE;
    cfg.task_priority = SENSOR_DHT11_TASK_PRIORITY;
    cfg.task_stack = SENSOR_DHT11_TASK_STACK_WORDS;

    if (dht11_sim_config(&cfg) == DHT11_SIM_OK)
    {
        g_sensor_dht11_initialized = 1u;
    }
}

void sensor_dht11_run(void)
{
    if (g_sensor_dht11_initialized == 0u)
    {
        sensor_dht11_init();
    }

    if ((g_sensor_dht11_initialized == 0u) || (g_sensor_dht11_running != 0u))
    {
        return;
    }

    if (dht11_sim_run() == DHT11_SIM_OK)
    {
        g_sensor_dht11_running = 1u;
    }
}
