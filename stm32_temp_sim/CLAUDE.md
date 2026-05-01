# AI Assistant Guide cho Embedded Projects

Tài liệu hướng dẫn chung cho AI assistant khi làm việc trên các project embedded (STM32, Linux kernel, firmware...).
Mục tiêu: code nhất quán, dễ đọc, dễ bảo trì, theo triết lý Linux kernel.

## 1) Coding Style: Linux Kernel Convention

Tất cả code C viết theo **Linux kernel coding style**:

|  |  |
| --- | --- |
| Indent | **Tab** (width = 8), không dùng space |
| Dòng | Tối đa **80 ký tự** (linh hoạt đến 100 nếu code readability bị ảnh hưởng nghiêm trọng) |
| Tên biến, hàm | `snake_case` viết thường toàn bộ |
| Macro, constant | `UPPER_CASE` |
| Struct, enum | `snake_case` (không typedef che giấu) |
| Con trỏ | Dấu `*` sát tên biến: `char *ptr` |
| Mở ngoặc `{` | Cùng dòng với statement (riêng function: xuống dòng mới) |
| Comment | `/* */` cho block, `//` cho inline ngắn |

Lưu ý: tên biến đặt ra phải rõ ràng, có nghĩa, tránh viết tắt không cần thiết. Tránh dùng `typedef` để che giấu struct, enum vì dễ gây nhầm lẫn về kiểu dữ liệu. Luôn ưu tiên readability hơn việc tuân thủ cứng nhắc nếu có trường hợp ngoại lệ.

**Ví dụ:**

```c
/* Good */
static int dht11_read_data(struct dht11_dev *dev, uint8_t *buf)
{
    if (!dev || !buf)
        return -EINVAL;

    for (int i = 0; i < DHT11_DATA_SIZE; i++) {
        buf[i] = dev->reg[i];          // copy from shadow reg
    }
    return 0;
}
/* Bad — không dùng */
int DHT11_ReadData(struct DHT11_Dev* pDev, uint8_t* pBuf) {
    if (!pDev || !pBuf) return -1;
    ...
}
```

Tham khảo: https://www.kernel.org/doc/html/latest/process/coding-style.html

## 2) Kiến trúc phân lớp

```
┌─────────────────────────┐
│  App Layer              │  State machine, workflow bài lab
│  (app_config.h, main.c)     │  KHÔNG truy cập thanh ghi trực tiếp
├─────────────────────────┤
│  Sensor Layer           │  Mô phỏng sensor, normalize data
│  (sensor_dht11.c, ...)  │  Che giấu protocol/register mapping
├─────────────────────────┤
│  Driver Layer           │  HAL mức thấp: init, read, write, irq
│  (driver_gpio.c, ...)   │  KHÔNG chứa business logic
└─────────────────────────┘
```

Nguyên tắc:

- Mỗi module có cặp .c/.h rõ ràng
- API public dùng prefix theo module: driver_timer_init(), sensor_ds1307_read(), app_dht11_start()
- Hàm static/private không cần prefix module
- Ưu tiên static buffer, hạn chế malloc
- Hàm trả về 0 thành công, -Exxx (errno) khi lỗi

## 3) Cấu trúc thư mục

```text
├───simulator_fw
│   ├───App
│   ├───Driver
│   │   ├───Inc
│   │   └───Src
│   └───Sensor
│       ├───Inc
│       └───Src
└───stm32_temp_sim
    ├───build
    ├───FreeRTOS
    │   ├───include
    │   ├───portable
    │   │   ├───GCC
    │   │   │   └───ARM_CM3
    │   │   └───MemMang
    │   └───source
    ├───Inc
    ├───Libraries
    │   ├───CMSIS
    │   │   ├───Device
    │   │   │   └───ST
    │   │   │       └───STM32F1xx
    │   │   │           └───Include
    │   │   └───Include
    │   └───STM32F10x_StdPeriph_Driver
    │       ├───inc
    │       └───src
    └───Src
```

## 4) Build

```bash
make                    # build
make clean              # dọn dẹp
make flash              # nạp xuống chip
make reset              # reset board
```

## 5) Quy tắc khi AI assistant sửa code

1. Thay đổi nhỏ, đúng phạm vi — không sửa những phần không liên quan
2. Không đổi tên API public nếu không có yêu cầu
3. Không format lại toàn bộ file nếu chỉ sửa vài dòng logic
4. Cập nhật comment/module note nếu thay đổi hành vi
5. Giữ đúng coding style của file đang chỉnh sửa
6. Khi có nhiều cách, ưu tiên cách dễ đọc, dễ bảo trì
7. Ghi chú test case tối thiểu (happy path + lỗi cơ bản) sau khi thêm tính năng mới

## 6) Checklist khi thêm module/bài lab mới

- Tạo cặp .c/.h đúng layer (App/Driver/Sensor)
- Đăng ký vào Makefile (C_SOURCES, include path)
- Thêm lời gọi vào main.c nếu cần
- Build không warning (bật -Wall -Wextra)
- Test: init ok, timeout, invalid input, return code
- Nếu dùng interrupt + RTOS: kiểm tra race condition
- Lưu conversation log vào conversations/YYYY-MM-DD_<chủ-đề>.md

## 7) Ngôn ngữ

| Thành phần | Ngôn ngữ |
| --- | --- |
| Code comment | Tiếng Anh |
| Commit message | Tiếng Việt (ưu tiên) |
| Học liệu markdown | Tiếng Việt xen tiếng Anh (thuật ngữ kỹ thuật giữ nguyên) |
| Conversation log | Tiếng Việt |

