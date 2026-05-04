# Tài liệu Giải thích Kiến trúc Các lớp Driver

Dự án được thiết kế phân lớp rõ ràng nhằm tạo sự tách biệt giữa tầng ứng dụng (App), tầng cảm biến/mô phỏng (Sensor) và tầng giao tiếp phần cứng (Driver).
Tất cả các API Driver đều tuân theo chuẩn quy ước API như đã nêu trong `ARCHITECTURE.md`.

## 1. Triết lý chung của lớp Driver
Mọi Driver đều cung cấp một tập các API thống nhất với định dạng chung:
- `[tên_driver]_init()` (hoặc `open`): Dùng để cấp clock, cấu hình phần cứng (GPIO, Speed, Mode) cho ngoại vi.
- `[tên_driver]_read()`: Dùng để đọc giá trị từ ngoại vi đó (polling).
- `[tên_driver]_write()`: Dùng để ghi giá trị ra ngoại vi.
- `[tên_driver]_stop()` (hoặc `close`): Hủy quá trình đang chạy, tiết kiệm điện, trả chân về trạng thái floating.

Trường hợp sử dụng Interrupt (Ngắt) hoặc DMA:
- `[tên_driver]_irq_process()`: Được gọi tại hàm ngắt (trong file `stm32f10x_it.c`) để xử lý các event ngắt cơ bản.
- `[tên_driver]_dma_process()`: Được gọi tại DMA IRQ Handler để tiếp tục xử lý buffer DMA.

## 2. Chi tiết các Driver

### A. Driver Timer (`driver_timer.h` / `driver_timer.c`)
- **Nhiệm vụ:** Trừu tượng hóa các Timer của STM32F1 (TIM1, TIM2, TIM3, TIM4) ở các chế độ TIME_BASE, PWM, IC (Input Capture), OC (Output Compare).
- **Cách dùng:**
  - `timer_open(...)`: Bật timer với cấu hình struct cụ thể. Cấu hình tự động GPIO nếu đó là PWM/IC/OC.
  - `timer_read(...)`: Lấy Counter của Timer hoặc Capture value.
  - `timer_write(...)`: Đổi độ rộng xung (CCR) dùng cho PWM.
  - `timer_close(...)`: Tắt an toàn Timer.
- **Tích hợp:** Nó kết nối rất chặt chẽ với tầng Sensor (`sensor_dht11`) vì DHT11 yêu cầu dùng độ rộng xung mức micro-second để phân tích chuỗi giao tiếp.

### B. Driver GPIO (`driver_gpio.h` / `driver_gpio.c`)
- **Nhiệm vụ:** Trừu tượng hóa việc bật/tắt và cài đặt mode của các chân I/O.
- **Cách dùng:** 
  - `gpio_open(...)`: Kích hoạt clock cho Port cần thiết và khởi tạo theo `GPIOMode_TypeDef`.
  - `gpio_write(...)`: Set/Reset một Pin.
  - `gpio_read(...)`: Trả về kiểu bool mức logic thực của Pin.
  - `gpio_toggle(...)`: Đảo trạng thái hiện tại.

### C. Driver I2C (`driver_i2c.h` / `driver_i2c.c`)
- **Nhiệm vụ:** Đóng vai trò là I2C Slave Driver, cho phép STM32 giả lập một I2C Slave trên bus (DS1307). 
- **Cách dùng:**
  - `driver_i2c_slave_init(...)` / `driver_i2c_slave_start(...)`: Cấu hình module I2C phần cứng thành dạng ngắt Slave.
  - Cung cấp callback cơ chế Event-driven (`event_cb` và `tx_byte_cb`) cho tầng Sensor nắm bắt khi Master tiến hành Read/Write. Tầng Sensor (`sensor_ds1307`) sẽ phản ứng trả về Time hoặc Update Register.

### D. Driver UART (`driver_uart.h` / `driver_uart.c`)
- **Nhiệm vụ:** Quản lý đường truyền bất đồng bộ UART.
- **Cách dùng:** 
  - `uart_init(...)`: Cấu hình baudrate và các chân TX (Push-pull) RX (Floating) cho UART tương ứng (ví dụ USART1 hoặc USART2).
  - `uart_write(...)`: Đẩy từng byte ra bộ đệm phần cứng liên tục có Blocking (chờ cờ TXE).
  - `uart_read(...)`: Lấy từng byte từ thanh ghi phần cứng (chờ cờ RXNE).
  - `uart_stop(...)`: Ngắt kết nối UART, disable clock. 

## 3. Mối liên kết tới tầng Sensor & Tầng App
Lớp Driver **không chứa** Business Logic.
Nó đảm bảo việc thiết lập đúng các bit thanh ghi (thông qua SPL). Sensor DHT11 và DS1307 sẽ liên kết các lệnh của nó với các driver này để thao tác.

Ví dụ, đối với `sensor_ds1307`: 
1. Hàm `ds1307_sim_init` sử dụng trực tiếp các lệnh `gpio_open` để cấu hình chân I2C thành Alternate Function Open Drain. 
2. Sau đó nó gọi API của `driver_i2c` để kích hoạt giao tiếp I2C. 
3. Cuối cùng, tầng App (`main.c`) chỉ cần gọi `sensor_ds1307_init()` và `sensor_ds1307_run()` ở đầu hàm main, sau đó `FreeRTOS` task sẽ đảm bảo các giá trị thời gian được cập nhật tự động.
