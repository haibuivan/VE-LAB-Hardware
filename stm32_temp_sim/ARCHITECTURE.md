các API trong driver sẽ thống nhất cách gọi như sau:
- init:
- write
- read
- stop

nếu ngắt thì là
- irq_process

nếu dma thì là 
- dma_process

ví dụ với timer sẽ có các API driver như sau:

```c
int timer_init(TIM_TypeDef *TIMx, Timer_Ch_t ch,
               Timer_Mode_t mode, const void *cfg);
int timer_read(TIM_TypeDef *TIMx, Timer_Ch_t ch, uint32_t *out);
int timer_write(TIM_TypeDef *TIMx, Timer_Ch_t ch, uint16_t val);
int timer_stop(TIM_TypeDef *TIMx, Timer_Ch_t ch);
void timer_irq_process(TIM_TypeDef *TIMx);
void timer_dma_process(DMA_Channel_TypeDef *dma_ch);
```

đối với sensor - FreeRTOS layer sẽ có 2 api để lớp app (main.c) gọi là:

ví dụ như dht11 sẽ có 2 API là:

```c
void sensor_dht11_init();
void sensor_dht11_run();
```

tương tự với ds1307 sẽ có 2 API là:

```c
void sensor_ds1307_init();
void sensor_ds1307_run();
```

lớp app khi được define nó sẽ chỉ gọi API đấy và nạp xuống stm32 mô phỏng thôi.