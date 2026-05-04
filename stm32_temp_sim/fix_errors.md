# Danh sách các lỗi đã sửa (Bug Fixes)

Trong quá trình bảo trì và làm cho hệ thống đồng bộ theo thiết kế kiến trúc phân lớp, các vấn đề và lỗi sau đã được giải quyết:

## 1. Lỗi cấu trúc đường dẫn (Path Issues)
- **Vấn đề:** `Makefile` trong `stm32_temp_sim` chỉ định vị trí các file code nằm ở các thư mục như `App/`, `Driver/`, `Sensor/` nhưng thực tế bộ mã nguồn đã được tổ chức lại và di chuyển vào thư mục `simulator_fw`.
- **Cách khắc phục:** Cập nhật biến `C_SOURCES` và `C_INCLUDES` trong `Makefile` để trỏ đúng tới `../simulator_fw/App`, `../simulator_fw/Driver/Src`, `../simulator_fw/Sensor/Src`, v.v.

## 2. Inconsistencies Include Paths (Sai tên file Header)
- **Vấn đề:** Tên file header và source không đồng nhất hoặc bị sai lệch tên. 
- **Cách khắc phục:** Đổi include path trong toàn bộ file:
  - Trong `driver_timer.c`: `#include "timer_driver.h"` ➔ `#include "driver_timer.h"`
  - Trong `driver_gpio.c`: `#include "gpio_driver.h"` ➔ `#include "driver_gpio.h"`
  - Trong `driver_i2c.c`: `#include "driver_i2c_slave.h"` ➔ `#include "driver_i2c.h"`
  - Trong `sensor_dht11.c`: `#include "dht11_sim.h"` ➔ `#include "sensor_dht11.h"`
  - Trong `sensor_ds1307.c`: `#include "ds1307.h"` ➔ `#include "sensor_ds1307.h"`
  - Trong `main.c`: `#include "ds1307.h"` ➔ `#include "sensor_ds1307.h"`

## 3. Các lỗi cú pháp (Syntax Errors) trong `sensor_ds1307.c`
- **Vấn đề:** Lớp `sensor_ds1307.c` mắc nhiều lỗi cú pháp nghiêm trọng làm cản trở việc biên dịch.
- **Cách khắc phục:**
  - Sửa `ds1307-DS1307_BIT_12H` thành `DS1307_BIT_12H`.
  - Sửa `ds1307_simu-ds1307_simu_bcd_to_bin` thành `ds1307_simu_bcd_to_bin`.
  - Sửa lỗi thiếu chấm phẩy `;` trong hàm `ds1307_simu_decode_hour_to_24h`.
  - Thay đổi phép tính bitwise lỗi `7` thành phép AND `&`.
  - Sửa lỗi khai báo hàm thiếu kiểu trả về: `static ds1307_simu_sync_regs_from_time` ➔ `static void ds1307_simu_sync_regs_from_time`.
  - Sửa lỗi gán struct trực tiếp: `sim->current_time = time;` ➔ `sim->current_time = *time;`.
  - Cập nhật đúng các lệnh call API I2C như `driver_i2c_slave_set_tx_byte_callback` và `ds1307_simu_publish_regs`.

## 4. Các lỗi định nghĩa Macro và biến trong DS1307
- **Vấn đề:** Quên định nghĩa các hằng số liên quan tới FreeRTOS như Stack Size, Priority, hay tốc độ Clock I2C. Ngoài ra gọi API của rcc và nvic không đúng chuẩn.
- **Cách khắc phục:** 
  - Khai báo thêm các hằng số bị thiếu (APP_DS1307_TASK_STACK_SIZE, v.v.).
  - Thay thế thư viện/hàm rcc_enable, gpio_init giả định bằng thư viện chuẩn Standard Peripheral Library (SPL) của STM32 và API của `driver_gpio` mới (gpio_open).

## 5. Lỗi thiếu Driver UART và GPIO 
- **Vấn đề:** Thiết kế có `driver_gpio` và `driver_uart` nhưng chúng chưa được viết hàm đồng bộ theo kiến trúc. File `uart_driver.c` rỗng.
- **Cách khắc phục:** 
  - Sửa đổi hàm của `driver_gpio.c`/`driver_gpio.h` thành `gpio_open`, `gpio_read`, `gpio_write`. Bổ sung `#include <stdbool.h>`.
  - Xóa bỏ `uart_driver.c`, tạo file `driver_uart.c` và `driver_uart.h` theo đúng chuẩn kiến trúc `init`, `read`, `write`, `stop`.
  - Tích hợp tất cả vào `Makefile` và tiến hành biên dịch thành công (0 error).
