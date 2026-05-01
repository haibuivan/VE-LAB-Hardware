#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

#include "stm32f10x.h"

/* ================================================================
 *  ERROR CODES
 * ================================================================ */
#define TIMER_OK (0)
#define TIMER_EINVAL (-1)
#define TIMER_EBUSY (-2)
#define TIMER_ENOTSUP (-3)

/* ================================================================
 *  ENUMS
 * ================================================================ */
typedef enum
{
    TIMER_CH1 = 0,
    TIMER_CH2 = 1,
    TIMER_CH3 = 2,
    TIMER_CH4 = 3,
} Timer_Ch_t;

typedef enum
{
    TIMER_MODE_BASE = 0,
    TIMER_MODE_PWM = 1,
    TIMER_MODE_IC = 2,
    TIMER_MODE_OC = 3,
} Timer_Mode_t;

typedef enum
{
    TIMER_IC_RISING = TIM_ICPolarity_Rising,
    TIMER_IC_FALLING = TIM_ICPolarity_Falling,
    TIMER_IC_BOTH = TIM_ICPolarity_BothEdge,
} Timer_IC_Polarity_t;

typedef enum
{
    TIMER_OC_TIMING = TIM_OCMode_Timing,
    TIMER_OC_ACTIVE = TIM_OCMode_Active,
    TIMER_OC_INACTIVE = TIM_OCMode_Inactive,
    TIMER_OC_TOGGLE = TIM_OCMode_Toggle,
    TIMER_OC_PWM1 = TIM_OCMode_PWM1,
    TIMER_OC_PWM2 = TIM_OCMode_PWM2,
} Timer_OC_Mode_t;

/* ================================================================
 *  CALLBACK
 *
 *  IRQ callback  : gọi từ TIMx CC/Update interrupt
 *  DMA callback  : gọi khi DMA transfer complete
 *
 *  val : CCR (IC) / CNT (BASE) / 0 (OC/PWM)
 * ================================================================ */
typedef void (*timer_cb_t)(TIM_TypeDef *TIMx, Timer_Ch_t ch, uint32_t val);
typedef void (*timer_dma_cb_t)(TIM_TypeDef *TIMx, Timer_Ch_t ch);

/* ================================================================
 *  CONFIG STRUCTS
 *
 *  DMA fields (IC / PWM / OC):
 *    dma_enable = 0 : dùng interrupt bình thường
 *    dma_enable = 1 : bật DMA, dma_buf + dma_len bắt buộc có giá trị
 *    dma_cb         : gọi khi DMA transfer complete (có thể NULL)
 * ================================================================ */

/* --- TIME BASE ---
 * Không có DMA vì BASE chỉ dùng update interrupt
 * freq = SYS_CLK / (prescaler * period)
 */
typedef struct
{
    uint16_t prescaler;
    uint32_t period;
    timer_cb_t callback;
} Timer_Base_Cfg_t;

/* --- PWM ---
 * dma_enable=1: DMA tự nạp dma_buf[] vào CCR theo chu kỳ
 * -> thay đổi duty cycle tự động không cần CPU
 * duty(%) = pulse / period * 100
 */
typedef struct
{
    uint16_t prescaler;
    uint32_t period;
    uint16_t pulse;   /* duty ban đầu                  */
    uint8_t polarity; /* TIM_OCPolarity_High / Low     */
    uint8_t dma_enable;
    uint16_t *dma_buf; /* bảng giá trị CCR              */
    uint16_t dma_len;
    timer_dma_cb_t dma_cb; /* gọi sau mỗi vòng DMA          */
} Timer_PWM_Cfg_t;

/* --- INPUT CAPTURE ---
 * dma_enable=0: mỗi capture -> IRQ -> callback
 * dma_enable=1: DMA gom capture vào dma_buf, sau đó gọi dma_cb
 * prescaler=72 -> 1 tick = 1us
 */
typedef struct
{
    uint16_t prescaler;
    uint32_t period;
    Timer_IC_Polarity_t polarity;
    uint8_t filter;      /* 0x00 - 0x0F                   */
    timer_cb_t callback; /* dùng khi dma_enable=0         */
    uint8_t dma_enable;
    uint16_t *dma_buf; /* buffer nhận dữ liệu capture   */
    uint16_t dma_len;
    timer_dma_cb_t dma_cb; /* gọi khi đầy buffer            */
} Timer_IC_Cfg_t;

/* --- OUTPUT COMPARE ---
 * dma_enable=1: DMA nạp dma_buf[] vào CCR tự động
 * -> tạo chuỗi pulse với timing khác nhau không cần CPU
 */
typedef struct
{
    uint16_t prescaler;
    uint32_t period;
    uint16_t pulse;
    Timer_OC_Mode_t oc_mode;
    uint8_t polarity;    /* TIM_OCPolarity_High / Low     */
    timer_cb_t callback; /* dùng khi dma_enable=0         */
    uint8_t dma_enable;
    uint16_t *dma_buf;
    uint16_t dma_len;
    timer_dma_cb_t dma_cb;
} Timer_OC_Cfg_t;

/* ================================================================
 *  API
 *
 *  timer_open  : cấu hình TIMx + GPIO + DMA (nếu enable), start luôn
 *  timer_read  : đọc CCR (IC) hoặc CNT (BASE)
 *  timer_write : cập nhật CCR (PWM / OC) khi không dùng DMA
 *  timer_close : dừng timer + DMA, tắt clock, reset GPIO
 *
 *  IRQ  : gọi timer_irq_process(TIMx)  trong TIMx_IRQHandler()
 *  DMA  : gọi timer_dma_process(DMAch) trong DMAx_Channely_IRQHandler()
 *
 *  --- Đặt trong stm32f10x_it.c ---
 *
 *  void TIM1_CC_IRQHandler(void) { timer_irq_process(TIM1); }
 *  void TIM2_IRQHandler(void)    { timer_irq_process(TIM2); }
 *  void TIM3_IRQHandler(void)    { timer_irq_process(TIM3); }
 *  void TIM4_IRQHandler(void)    { timer_irq_process(TIM4); }
 *
 *  void DMA1_Channel1_IRQHandler(void) { timer_dma_process(DMA1_Channel1); }
 *  void DMA1_Channel2_IRQHandler(void) { timer_dma_process(DMA1_Channel2); }
 *  void DMA1_Channel3_IRQHandler(void) { timer_dma_process(DMA1_Channel3); }
 *  void DMA1_Channel4_IRQHandler(void) { timer_dma_process(DMA1_Channel4); }
 *  void DMA1_Channel5_IRQHandler(void) { timer_dma_process(DMA1_Channel5); }
 *  void DMA1_Channel6_IRQHandler(void) { timer_dma_process(DMA1_Channel6); }
 *  void DMA1_Channel7_IRQHandler(void) { timer_dma_process(DMA1_Channel7); }
 * ================================================================ */
int timer_open(TIM_TypeDef *TIMx, Timer_Ch_t ch,
               Timer_Mode_t mode, const void *cfg);
int timer_read(TIM_TypeDef *TIMx, Timer_Ch_t ch, uint32_t *out);
int timer_write(TIM_TypeDef *TIMx, Timer_Ch_t ch, uint16_t val);
int timer_close(TIM_TypeDef *TIMx, Timer_Ch_t ch);
void timer_irq_process(TIM_TypeDef *TIMx);
void timer_dma_process(DMA_Channel_TypeDef *dma_ch);

#endif /* TIMER_DRIVER_H */
