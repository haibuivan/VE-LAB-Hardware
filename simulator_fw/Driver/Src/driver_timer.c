#include "driver_timer.h"

/* ================================================================
 *  INTERNAL STATE
 *
 *  cb_table      [4][4] : IRQ callback   — 4 timer x 4 channel
 *  dma_cb_table  [4][4] : DMA callback   — 4 timer x 4 channel
 *  mode_table    [4][4] : mode (uint8_t) — 4 timer x 4 channel
 *  dma_ch_table  [4][4] : DMA channel pointer tương ứng
 *
 *  Tổng RAM:
 *    cb_table     = 16 x 4 = 64 bytes
 *    dma_cb_table = 16 x 4 = 64 bytes
 *    mode_table   = 16 x 1 = 16 bytes
 *    dma_ch_table = 16 x 4 = 64 bytes
 *    ──────────────────────────────
 *    Tổng         = 208 bytes
 * ================================================================ */
static timer_cb_t cb_table[4][4];
static timer_dma_cb_t dma_cb_table[4][4];
static uint8_t mode_table[4][4];
static DMA_Channel_TypeDef *dma_ch_table[4][4];

/* ================================================================
 *  DMA MAP — STM32F103 Reference Manual Table 78
 *
 *  TIM2_CH3 -> DMA1_Ch1   TIM3_CH3 -> DMA1_Ch2
 *  TIM4_CH1 -> DMA1_Ch1   TIM3_CH4 -> DMA1_Ch3
 *  TIM1_CH1 -> DMA1_Ch2   TIM4_CH2 -> DMA1_Ch4
 *  TIM1_CH2 -> DMA1_Ch3   TIM2_CH1 -> DMA1_Ch5
 *  TIM4_CH3 -> DMA1_Ch5   TIM3_CH1 -> DMA1_Ch6
 *  TIM1_CH4 -> DMA1_Ch4   TIM2_CH2 -> DMA1_Ch7
 *  TIM1_CH3 -> DMA1_Ch6   TIM3_CH2 -> DMA1_Ch7
 *  TIM2_CH4 -> DMA1_Ch7   TIM4_CH4 -> không có DMA
 *
 *  Lưu ý: một số channel DMA bị share (ví dụ CH7 share TIM2_CH2,
 *  TIM2_CH4, TIM3_CH2) — không dùng DMA đồng thời trên các timer
 *  share cùng DMA channel.
 * ================================================================ */
typedef struct
{
    DMA_Channel_TypeDef *ch;
    uint32_t flag_tc; /* Transfer Complete flag */
    uint32_t flag_gl; /* Global flag để clear   */
    IRQn_Type irqn;
} prv_dma_map_t;

static const prv_dma_map_t dma_map[4][4] = {
    /* TIM1 */
    {
        {DMA1_Channel2, DMA1_FLAG_TC2, DMA1_FLAG_GL2, DMA1_Channel2_IRQn}, /* CH1 */
        {DMA1_Channel3, DMA1_FLAG_TC3, DMA1_FLAG_GL3, DMA1_Channel3_IRQn}, /* CH2 */
        {DMA1_Channel6, DMA1_FLAG_TC6, DMA1_FLAG_GL6, DMA1_Channel6_IRQn}, /* CH3 */
        {DMA1_Channel4, DMA1_FLAG_TC4, DMA1_FLAG_GL4, DMA1_Channel4_IRQn}, /* CH4 */
    },
    /* TIM2 */
    {
        {DMA1_Channel5, DMA1_FLAG_TC5, DMA1_FLAG_GL5, DMA1_Channel5_IRQn}, /* CH1 */
        {DMA1_Channel7, DMA1_FLAG_TC7, DMA1_FLAG_GL7, DMA1_Channel7_IRQn}, /* CH2 */
        {DMA1_Channel1, DMA1_FLAG_TC1, DMA1_FLAG_GL1, DMA1_Channel1_IRQn}, /* CH3 */
        {DMA1_Channel7, DMA1_FLAG_TC7, DMA1_FLAG_GL7, DMA1_Channel7_IRQn}, /* CH4 */
    },
    /* TIM3 */
    {
        {DMA1_Channel6, DMA1_FLAG_TC6, DMA1_FLAG_GL6, DMA1_Channel6_IRQn}, /* CH1 */
        {DMA1_Channel7, DMA1_FLAG_TC7, DMA1_FLAG_GL7, DMA1_Channel7_IRQn}, /* CH2 */
        {DMA1_Channel2, DMA1_FLAG_TC2, DMA1_FLAG_GL2, DMA1_Channel2_IRQn}, /* CH3 */
        {DMA1_Channel3, DMA1_FLAG_TC3, DMA1_FLAG_GL3, DMA1_Channel3_IRQn}, /* CH4 */
    },
    /* TIM4 */
    {
        {DMA1_Channel1, DMA1_FLAG_TC1, DMA1_FLAG_GL1, DMA1_Channel1_IRQn}, /* CH1 */
        {DMA1_Channel4, DMA1_FLAG_TC4, DMA1_FLAG_GL4, DMA1_Channel4_IRQn}, /* CH2 */
        {DMA1_Channel5, DMA1_FLAG_TC5, DMA1_FLAG_GL5, DMA1_Channel5_IRQn}, /* CH3 */
        {0, 0, 0, 0},                                                      /* CH4: không có DMA */
    },
};

/* ================================================================
 *  PRIVATE HELPERS
 * ================================================================ */
static int prv_tim_idx(TIM_TypeDef *TIMx)
{
    if (TIMx == TIM1)
        return 0;
    if (TIMx == TIM2)
        return 1;
    if (TIMx == TIM3)
        return 2;
    if (TIMx == TIM4)
        return 3;
    return -1;
}

static void prv_rcc_tim(TIM_TypeDef *TIMx, FunctionalState s)
{
    if (TIMx == TIM1)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, s);
    else if (TIMx == TIM2)
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, s);
    else if (TIMx == TIM3)
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, s);
    else if (TIMx == TIM4)
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, s);
}

/* FIX #7: TIM1 có 2 vector IRQ riêng biệt:
 *   TIM1_UP_IRQn  — update interrupt (TIMER_MODE_BASE)
 *   TIM1_CC_IRQn  — capture/compare (IC / OC / PWM)
 * Các timer khác (TIM2/3/4) chỉ có 1 vector dùng chung.
 */
static IRQn_Type prv_tim_cc_irqn(TIM_TypeDef *TIMx)
{
    if (TIMx == TIM1)
        return TIM1_CC_IRQn;
    if (TIMx == TIM2)
        return TIM2_IRQn;
    if (TIMx == TIM3)
        return TIM3_IRQn;
    if (TIMx == TIM4)
        return TIM4_IRQn;
    return TIM2_IRQn;
}

static IRQn_Type prv_tim_up_irqn(TIM_TypeDef *TIMx)
{
    if (TIMx == TIM1)
        return TIM1_UP_IRQn; /* TIM1 UP vector riêng */
    if (TIMx == TIM2)
        return TIM2_IRQn;
    if (TIMx == TIM3)
        return TIM3_IRQn;
    if (TIMx == TIM4)
        return TIM4_IRQn;
    return TIM2_IRQn;
}

static uint16_t prv_spl_ch(Timer_Ch_t ch)
{
    static const uint16_t tbl[4] = {
        TIM_Channel_1,
        TIM_Channel_2,
        TIM_Channel_3,
        TIM_Channel_4,
    };
    return tbl[ch];
}

static uint16_t prv_it_flag(Timer_Ch_t ch)
{
    static const uint16_t tbl[4] = {
        TIM_IT_CC1,
        TIM_IT_CC2,
        TIM_IT_CC3,
        TIM_IT_CC4,
    };
    return tbl[ch];
}

static uint16_t prv_dma_src(Timer_Ch_t ch)
{
    static const uint16_t tbl[4] = {
        TIM_DMA_CC1,
        TIM_DMA_CC2,
        TIM_DMA_CC3,
        TIM_DMA_CC4,
    };
    return tbl[ch];
}

static uint32_t prv_ccr_addr(TIM_TypeDef *TIMx, Timer_Ch_t ch)
{
    switch (ch)
    {
    case TIMER_CH1:
        return (uint32_t)&TIMx->CCR1;
    case TIMER_CH2:
        return (uint32_t)&TIMx->CCR2;
    case TIMER_CH3:
        return (uint32_t)&TIMx->CCR3;
    case TIMER_CH4:
        return (uint32_t)&TIMx->CCR4;
    default:
        return 0;
    }
}

static uint32_t prv_get_ccr(TIM_TypeDef *TIMx, Timer_Ch_t ch)
{
    switch (ch)
    {
    case TIMER_CH1:
        return TIM_GetCapture1(TIMx);
    case TIMER_CH2:
        return TIM_GetCapture2(TIMx);
    case TIMER_CH3:
        return TIM_GetCapture3(TIMx);
    case TIMER_CH4:
        return TIM_GetCapture4(TIMx);
    default:
        return 0;
    }
}

static void prv_set_ccr(TIM_TypeDef *TIMx, Timer_Ch_t ch, uint16_t val)
{
    switch (ch)
    {
    case TIMER_CH1:
        TIM_SetCompare1(TIMx, val);
        break;
    case TIMER_CH2:
        TIM_SetCompare2(TIMx, val);
        break;
    case TIMER_CH3:
        TIM_SetCompare3(TIMx, val);
        break;
    case TIMER_CH4:
        TIM_SetCompare4(TIMx, val);
        break;
    }
}

/* ----------------------------------------------------------------
 *  GPIO auto-config
 *
 *  Default pin map STM32F103C8T6 (no remap):
 *    TIM1: CH1=PA8  CH2=PA9  CH3=PA10 CH4=PA11
 *    TIM2: CH1=PA0  CH2=PA1  CH3=PA2  CH4=PA3
 *    TIM3: CH1=PA6  CH2=PA7  CH3=PB0  CH4=PB1
 *    TIM4: CH1=PB6  CH2=PB7  CH3=PB8  CH4=PB9
 * ---------------------------------------------------------------- */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} prv_pin_t;

static const prv_pin_t pin_map[4][4] = {
    {{GPIOA, GPIO_Pin_8}, {GPIOA, GPIO_Pin_9}, {GPIOA, GPIO_Pin_10}, {GPIOA, GPIO_Pin_11}},
    {{GPIOA, GPIO_Pin_0}, {GPIOA, GPIO_Pin_1}, {GPIOA, GPIO_Pin_2}, {GPIOA, GPIO_Pin_3}},
    {{GPIOA, GPIO_Pin_6}, {GPIOA, GPIO_Pin_7}, {GPIOB, GPIO_Pin_0}, {GPIOB, GPIO_Pin_1}},
    {{GPIOB, GPIO_Pin_6}, {GPIOB, GPIO_Pin_7}, {GPIOB, GPIO_Pin_8}, {GPIOB, GPIO_Pin_9}},
};

static void prv_gpio_open(TIM_TypeDef *TIMx, Timer_Ch_t ch, Timer_Mode_t mode)
{
    GPIO_InitTypeDef gpio;
    int tidx = prv_tim_idx(TIMx);
    GPIO_TypeDef *port;
    uint16_t pin;

    if (tidx < 0)
        return;

    port = pin_map[tidx][ch].port;
    pin = pin_map[tidx][ch].pin;

    if (port == GPIOA)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = pin;

    if (mode == TIMER_MODE_PWM || mode == TIMER_MODE_OC)
    {
        gpio.GPIO_Mode = GPIO_Mode_AF_PP;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;
    }
    else if (mode == TIMER_MODE_IC)
    {
        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    else
    {
        return; /* BASE: không cần GPIO */
    }

    GPIO_Init(port, &gpio);
}

static void prv_gpio_close(TIM_TypeDef *TIMx, Timer_Ch_t ch)
{
    GPIO_InitTypeDef gpio;
    int tidx = prv_tim_idx(TIMx);
    if (tidx < 0)
        return;

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = pin_map[tidx][ch].pin;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(pin_map[tidx][ch].port, &gpio);
}

/* ----------------------------------------------------------------
 *  DMA open / close
 * ---------------------------------------------------------------- */
static int prv_dma_open(TIM_TypeDef *TIMx, Timer_Ch_t ch,
                        uint16_t *buf, uint16_t len,
                        uint32_t dir) /* DMA_DIR_PeripheralSRC / DST */
{
    DMA_InitTypeDef dma;
    int tidx = prv_tim_idx(TIMx);
    const prv_dma_map_t *m;

    if (tidx < 0 || !buf || !len)
        return TIMER_EINVAL;

    m = &dma_map[tidx][ch];
    if (!m->ch)
        return TIMER_ENOTSUP; /* TIM4_CH4 không có DMA */

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(m->ch);
    dma.DMA_PeripheralBaseAddr = prv_ccr_addr(TIMx, ch);
    dma.DMA_MemoryBaseAddr = (uint32_t)buf;
    dma.DMA_DIR = dir;
    dma.DMA_BufferSize = len;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(m->ch, &dma);

    /* Bật TC interrupt để gọi dma_cb */
    DMA_ITConfig(m->ch, DMA_IT_TC, ENABLE);

    /* Bật NVIC cho DMA channel */
    NVIC_SetPriority(m->irqn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(m->irqn);

    /* Kết nối TIM -> DMA */
    TIM_DMACmd(TIMx, prv_dma_src(ch), ENABLE);
    DMA_Cmd(m->ch, ENABLE);

    /* Lưu DMA channel vào table để timer_dma_process() tra ngược */
    dma_ch_table[tidx][ch] = m->ch;

    return TIMER_OK;
}

static void prv_dma_close(TIM_TypeDef *TIMx, Timer_Ch_t ch)
{
    int tidx = prv_tim_idx(TIMx);
    const prv_dma_map_t *m;

    if (tidx < 0)
        return;

    m = &dma_map[tidx][ch];
    if (!m->ch)
        return;

    TIM_DMACmd(TIMx, prv_dma_src(ch), DISABLE);
    DMA_ITConfig(m->ch, DMA_IT_TC, DISABLE);
    DMA_Cmd(m->ch, DISABLE);
    NVIC_DisableIRQ(m->irqn);
    DMA_DeInit(m->ch);

    dma_ch_table[tidx][ch] = 0;
}

/* ----------------------------------------------------------------
 *  OC init helper
 * ---------------------------------------------------------------- */
static void prv_oc_init(TIM_TypeDef *TIMx, Timer_Ch_t ch,
                        TIM_OCInitTypeDef *oc)
{
    switch (ch)
    {
    case TIMER_CH1:
        TIM_OC1Init(TIMx, oc);
        TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
        break;
    case TIMER_CH2:
        TIM_OC2Init(TIMx, oc);
        TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
        break;
    case TIMER_CH3:
        TIM_OC3Init(TIMx, oc);
        TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
        break;
    case TIMER_CH4:
        TIM_OC4Init(TIMx, oc);
        TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
        break;
    }
}

/* ================================================================
 *  PRIVATE - MODE INIT
 * ================================================================ */
static int prv_timebase_init(TIM_TypeDef *TIMx,
                             const Timer_Base_Cfg_t *cfg)
{
    TIM_TimeBaseInitTypeDef tb;

    /* FIX #4: guard underflow — prescaler/period = 0 sẽ wrap thành 0xFFFF */
    if (cfg->prescaler == 0 || cfg->period == 0)
        return TIMER_EINVAL;

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = cfg->prescaler - 1;
    tb.TIM_Period = cfg->period - 1;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &tb);
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
    return TIMER_OK;
}

static int prv_pwm_init(TIM_TypeDef *TIMx, Timer_Ch_t ch,
                        const Timer_PWM_Cfg_t *cfg)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;
    int ret = TIMER_OK;

    /* FIX #4: guard underflow */
    if (cfg->prescaler == 0 || cfg->period == 0)
        return TIMER_EINVAL;

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = cfg->prescaler - 1;
    tb.TIM_Period = cfg->period - 1;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = cfg->pulse;
    oc.TIM_OCPolarity = cfg->polarity;
    prv_oc_init(TIMx, ch, &oc);
    TIM_ARRPreloadConfig(TIMx, ENABLE);

    /* FIX #6: TIM1 là Advanced Timer — phải enable Main Output Enable (MOE)
     * Nếu không có dòng này, output PWM/OC của TIM1 sẽ hoàn toàn câm. */
    if (TIMx == TIM1)
        TIM_CtrlPWMOutputs(TIMx, ENABLE);

    if (cfg->dma_enable)
    {
        if (!cfg->dma_buf || cfg->dma_len == 0)
            return TIMER_EINVAL;
        ret = prv_dma_open(TIMx, ch, cfg->dma_buf, cfg->dma_len,
                           DMA_DIR_PeripheralDST);
    }
    return ret;
}

static int prv_ic_init(TIM_TypeDef *TIMx, Timer_Ch_t ch,
                       const Timer_IC_Cfg_t *cfg)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef ic;
    int ret = TIMER_OK;

    /* FIX #4: guard underflow */
    if (cfg->prescaler == 0 || cfg->period == 0)
        return TIMER_EINVAL;

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = cfg->prescaler - 1;
    tb.TIM_Period = cfg->period - 1;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &tb);

    TIM_ICStructInit(&ic);
    ic.TIM_Channel = prv_spl_ch(ch);
    ic.TIM_ICPolarity = cfg->polarity;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter = cfg->filter;
    TIM_ICInit(TIMx, &ic);

    if (cfg->dma_enable)
    {
        /* DMA mode: không dùng CC interrupt, DMA tự lấy CCR */
        if (!cfg->dma_buf || cfg->dma_len == 0)
            return TIMER_EINVAL;
        ret = prv_dma_open(TIMx, ch, cfg->dma_buf, cfg->dma_len,
                           DMA_DIR_PeripheralSRC);
    }
    else
    {
        TIM_ITConfig(TIMx, prv_it_flag(ch), ENABLE);
    }
    return ret;
}

static int prv_oc_cfg_init(TIM_TypeDef *TIMx, Timer_Ch_t ch,
                           const Timer_OC_Cfg_t *cfg)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;
    int ret = TIMER_OK;

    /* FIX #4: guard underflow */
    if (cfg->prescaler == 0 || cfg->period == 0)
        return TIMER_EINVAL;

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = cfg->prescaler - 1;
    tb.TIM_Period = cfg->period - 1;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = cfg->oc_mode;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = cfg->pulse;
    oc.TIM_OCPolarity = cfg->polarity;
    prv_oc_init(TIMx, ch, &oc);
    TIM_ARRPreloadConfig(TIMx, ENABLE);

    /* FIX #6: TIM1 Advanced Timer — enable MOE */
    if (TIMx == TIM1)
        TIM_CtrlPWMOutputs(TIMx, ENABLE);

    if (cfg->dma_enable)
    {
        if (!cfg->dma_buf || cfg->dma_len == 0)
            return TIMER_EINVAL;
        ret = prv_dma_open(TIMx, ch, cfg->dma_buf, cfg->dma_len,
                           DMA_DIR_PeripheralDST);
    }
    else
    {
        TIM_ITConfig(TIMx, prv_it_flag(ch), ENABLE);
    }
    return ret;
}

/* ================================================================
 *  PUBLIC API
 * ================================================================ */
int timer_open(TIM_TypeDef *TIMx, Timer_Ch_t ch,
               Timer_Mode_t mode, const void *cfg)
{
    int tidx, ret;

    if (!TIMx || !cfg)
        return TIMER_EINVAL;
    if (ch < TIMER_CH1 || ch > TIMER_CH4)
        return TIMER_EINVAL;

    tidx = prv_tim_idx(TIMx);
    if (tidx < 0)
        return TIMER_EINVAL;

    /* FIX #5: guard channel đã open — tránh leak DMA/NVIC */
    if (mode_table[tidx][ch] != 0)
        return TIMER_EBUSY;

    prv_rcc_tim(TIMx, ENABLE);
    prv_gpio_open(TIMx, ch, mode);

    switch (mode)
    {
    case TIMER_MODE_BASE:
    {
        const Timer_Base_Cfg_t *c = (const Timer_Base_Cfg_t *)cfg;
        ret = prv_timebase_init(TIMx, c); /* FIX #4: now returns error code */
        if (ret != TIMER_OK)
            return ret;
        cb_table[tidx][ch] = c->callback;
        dma_cb_table[tidx][ch] = 0;
        break;
    }
    case TIMER_MODE_PWM:
    {
        const Timer_PWM_Cfg_t *c = (const Timer_PWM_Cfg_t *)cfg;
        ret = prv_pwm_init(TIMx, ch, c);
        cb_table[tidx][ch] = 0;
        dma_cb_table[tidx][ch] = c->dma_enable ? c->dma_cb : 0;
        break;
    }
    case TIMER_MODE_IC:
    {
        const Timer_IC_Cfg_t *c = (const Timer_IC_Cfg_t *)cfg;
        ret = prv_ic_init(TIMx, ch, c);
        cb_table[tidx][ch] = c->dma_enable ? 0 : c->callback;
        dma_cb_table[tidx][ch] = c->dma_enable ? c->dma_cb : 0;
        break;
    }
    case TIMER_MODE_OC:
    {
        const Timer_OC_Cfg_t *c = (const Timer_OC_Cfg_t *)cfg;
        ret = prv_oc_cfg_init(TIMx, ch, c);
        cb_table[tidx][ch] = c->dma_enable ? 0 : c->callback;
        dma_cb_table[tidx][ch] = c->dma_enable ? c->dma_cb : 0;
        break;
    }
    default:
        return TIMER_EINVAL;
    }

    if (ret != TIMER_OK)
        return ret;

    mode_table[tidx][ch] = (uint8_t)mode;

    /* FIX #7: TIM1 có 2 vector IRQ riêng (UP và CC).
     * TIMER_MODE_BASE dùng Update interrupt -> enable TIM1_UP_IRQn.
     * Các mode khác dùng CC interrupt     -> enable TIM1_CC_IRQn.
     * TIM2/3/4 chỉ có 1 vector nên prv_tim_up_irqn == prv_tim_cc_irqn. */
    if (mode == TIMER_MODE_BASE)
    {
        NVIC_SetPriority(prv_tim_up_irqn(TIMx),
                         NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
        NVIC_EnableIRQ(prv_tim_up_irqn(TIMx));
    }
    else
    {
        NVIC_SetPriority(prv_tim_cc_irqn(TIMx),
                         NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
        NVIC_EnableIRQ(prv_tim_cc_irqn(TIMx));
    }

    TIM_Cmd(TIMx, ENABLE);
    return TIMER_OK;
}

/* ---------------------------------------------------------------- */
int timer_read(TIM_TypeDef *TIMx, Timer_Ch_t ch, uint32_t *out)
{
    int tidx;

    if (!TIMx || !out)
        return TIMER_EINVAL;
    if (ch < TIMER_CH1 || ch > TIMER_CH4)
        return TIMER_EINVAL;

    tidx = prv_tim_idx(TIMx);
    if (tidx < 0)
        return TIMER_EINVAL;

    switch ((Timer_Mode_t)mode_table[tidx][ch])
    {
    case TIMER_MODE_IC:
        *out = prv_get_ccr(TIMx, ch);
        return TIMER_OK;
    case TIMER_MODE_BASE:
        *out = TIM_GetCounter(TIMx);
        return TIMER_OK;
    default:
        return TIMER_ENOTSUP;
    }
}

/* ---------------------------------------------------------------- */
int timer_write(TIM_TypeDef *TIMx, Timer_Ch_t ch, uint16_t val)
{
    int tidx;

    if (!TIMx)
        return TIMER_EINVAL;
    if (ch < TIMER_CH1 || ch > TIMER_CH4)
        return TIMER_EINVAL;

    tidx = prv_tim_idx(TIMx);
    if (tidx < 0)
        return TIMER_EINVAL;

    switch ((Timer_Mode_t)mode_table[tidx][ch])
    {
    case TIMER_MODE_PWM:
    case TIMER_MODE_OC:
        prv_set_ccr(TIMx, ch, val);
        return TIMER_OK;
    default:
        return TIMER_ENOTSUP;
    }
}

/* ---------------------------------------------------------------- */
int timer_close(TIM_TypeDef *TIMx, Timer_Ch_t ch)
{
    int tidx, i, still_used_cc, still_used_up;
    uint8_t cur_mode;

    if (!TIMx)
        return TIMER_EINVAL;
    if (ch < TIMER_CH1 || ch > TIMER_CH4)
        return TIMER_EINVAL;

    tidx = prv_tim_idx(TIMx);
    if (tidx < 0)
        return TIMER_EINVAL;

    cur_mode = mode_table[tidx][ch];
    if (cur_mode == 0)
        return TIMER_OK; /* channel chưa open, không làm gì */

    /* FIX #1: dừng DMA TRƯỚC khi dừng timer
     * Nếu dừng timer trước, DMA có thể vẫn fire TC interrupt 1 lần cuối
     * sau khi callback đã bị NULL hóa -> undefined behaviour. */
    if (dma_ch_table[tidx][ch])
        prv_dma_close(TIMx, ch);

    /* Sau khi DMA đã dừng hẳn mới disable timer CC/Update IT và timer */
    TIM_ITConfig(TIMx, prv_it_flag(ch) | TIM_IT_Update, DISABLE);
    TIM_Cmd(TIMx, DISABLE);

    /* FIX #2: still_used dựa vào mode_table thay vì callback pointer.
     * Lý do: BASE mode có thể không có callback (NULL) nhưng timer vẫn
     * đang chạy — nếu chỉ check callback sẽ tắt clock sai.
     * Tách riêng: still_used_up (BASE) và still_used_cc (IC/OC/PWM)
     * để handle TIM1 có 2 vector IRQ riêng. */
    still_used_up = 0;
    still_used_cc = 0;
    for (i = 0; i < 4; i++)
    {
        if (i == (int)ch)
            continue;
        if (mode_table[tidx][i] == 0)
            continue;
        if (mode_table[tidx][i] == (uint8_t)TIMER_MODE_BASE)
            still_used_up = 1;
        else
            still_used_cc = 1;
    }

    /* FIX #7: tắt đúng vector NVIC cho TIM1 */
    if (!still_used_up && cur_mode == (uint8_t)TIMER_MODE_BASE)
        NVIC_DisableIRQ(prv_tim_up_irqn(TIMx));

    if (!still_used_cc && cur_mode != (uint8_t)TIMER_MODE_BASE)
        NVIC_DisableIRQ(prv_tim_cc_irqn(TIMx));

    /* Tắt clock chỉ khi không còn channel nào đang dùng timer */
    if (!still_used_up && !still_used_cc)
        prv_rcc_tim(TIMx, DISABLE);

    prv_gpio_close(TIMx, ch);

    cb_table[tidx][ch] = 0;
    dma_cb_table[tidx][ch] = 0;
    mode_table[tidx][ch] = 0;

    return TIMER_OK;
}

/* ================================================================
 *  IRQ PROCESS — gọi trong TIMx_IRQHandler()
 *
 *  FIX #7: TIM1 có 2 vector riêng, phải khai báo cả 2:
 *
 *  void TIM1_UP_IRQHandler(void) { timer_irq_process(TIM1); }  <- BASE
 *  void TIM1_CC_IRQHandler(void) { timer_irq_process(TIM1); }  <- IC/OC/PWM
 *  void TIM2_IRQHandler(void)    { timer_irq_process(TIM2); }
 *  void TIM3_IRQHandler(void)    { timer_irq_process(TIM3); }
 *  void TIM4_IRQHandler(void)    { timer_irq_process(TIM4); }
 * ================================================================ */
void timer_irq_process(TIM_TypeDef *TIMx)
{
    static const uint16_t it_flags[4] = {
        TIM_IT_CC1,
        TIM_IT_CC2,
        TIM_IT_CC3,
        TIM_IT_CC4,
    };
    int tidx = prv_tim_idx(TIMx);
    int ch;

    if (tidx < 0)
        return;

    /* TIME BASE: update interrupt */
    if (TIM_GetITStatus(TIMx, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIMx, TIM_IT_Update);
        for (ch = 0; ch < 4; ch++)
        {
            if (mode_table[tidx][ch] == (uint8_t)TIMER_MODE_BASE &&
                cb_table[tidx][ch])
            {
                cb_table[tidx][ch](TIMx, (Timer_Ch_t)ch,
                                   TIM_GetCounter(TIMx));
            }
        }
    }

    /* IC / OC: capture compare interrupt */
    for (ch = 0; ch < 4; ch++)
    {
        if (TIM_GetITStatus(TIMx, it_flags[ch]) != SET)
            continue;
        TIM_ClearITPendingBit(TIMx, it_flags[ch]);
        if (cb_table[tidx][ch])
            cb_table[tidx][ch](TIMx, (Timer_Ch_t)ch,
                               prv_get_ccr(TIMx, (Timer_Ch_t)ch));
    }
}

/* ================================================================
 *  DMA PROCESS — gọi trong DMA1_ChannelX_IRQHandler()
 *
 *  void DMA1_Channel1_IRQHandler(void) { timer_dma_process(DMA1_Channel1); }
 *  void DMA1_Channel2_IRQHandler(void) { timer_dma_process(DMA1_Channel2); }
 *  void DMA1_Channel3_IRQHandler(void) { timer_dma_process(DMA1_Channel3); }
 *  void DMA1_Channel4_IRQHandler(void) { timer_dma_process(DMA1_Channel4); }
 *  void DMA1_Channel5_IRQHandler(void) { timer_dma_process(DMA1_Channel5); }
 *  void DMA1_Channel6_IRQHandler(void) { timer_dma_process(DMA1_Channel6); }
 *  void DMA1_Channel7_IRQHandler(void) { timer_dma_process(DMA1_Channel7); }
 * ================================================================ */
void timer_dma_process(DMA_Channel_TypeDef *dma_ch)
{
    int tidx, ch;
    const prv_dma_map_t *m;
    static TIM_TypeDef *const tim_lut[4] = {TIM1, TIM2, TIM3, TIM4};

    /* FIX #3: DMA channel bị share (ví dụ DMA1_Ch7 share TIM2_CH2,
     * TIM2_CH4, TIM3_CH2). Không thể dừng ở entry đầu tiên match
     * m->ch == dma_ch vì có thể sai timer/channel.
     *
     * Giải pháp: kiểm tra thêm dma_ch_table[tidx][ch] == dma_ch,
     * tức là channel này đang THỰC SỰ sử dụng DMA đó (được ghi bởi
     * prv_dma_open). Điều này phân biệt đúng khi có share. */
    for (tidx = 0; tidx < 4; tidx++)
    {
        for (ch = 0; ch < 4; ch++)
        {
            /* Bỏ qua nếu channel này không dùng DMA hoặc dùng DMA khác */
            if (dma_ch_table[tidx][ch] != dma_ch)
                continue;

            m = &dma_map[tidx][ch];
            if (DMA_GetITStatus(m->flag_tc) != SET)
                continue;

            DMA_ClearITPendingBit(m->flag_gl);

            if (dma_cb_table[tidx][ch])
                dma_cb_table[tidx][ch](tim_lut[tidx], (Timer_Ch_t)ch);

            /* Không return sớm: với shared channel, lý thuyết chỉ 1
             * entry trong dma_ch_table trỏ vào dma_ch tại 1 thời điểm
             * (hardware không cho phép 2 DMA request cùng lúc trên
             * cùng 1 DMA channel), nhưng vòng lặp tiếp tục an toàn. */
        }
    }
}
