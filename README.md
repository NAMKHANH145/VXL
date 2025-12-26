# VXL-V2: Hệ Thống Kiểm Soát Độ Ẩm & Nhiệt Độ Khép Kín (ESP32)

Dự án phát triển hệ thống ổn định độ ẩm và nhiệt độ cho buồng kín (Test Chamber) sử dụng vi điều khiển **ESP32** và công nghệ **Peltier**. Hệ thống có khả năng tự động tăng ẩm (bằng bơm khí/sục) và giảm ẩm (bằng ngưng tụ lạnh), đồng thời giám sát an toàn nhiệt độ nước.

![Language](https://img.shields.io/badge/Language-C-blue.svg)
![Platform](https://img.shields.io/badge/Platform-ESP--IDF-red.svg)
![Hardware](https://img.shields.io/badge/Hardware-ESP32-green.svg)

## 🌟 Tính Năng Chính

1.  **Kiểm soát Độ ẩm Hai chiều:**
    *   **Tăng ẩm (Humidify):** Kích hoạt máy bơm khí sục vào nước nóng.
    *   **Giảm ẩm (Dehumidify):** Kích hoạt Sò lạnh (Peltier) và Quạt sên (Blower) để ngưng tụ hơi nước thừa.
2.  **Điều khiển Thông minh:**
    *   Thuật toán **Soft-Start** (Khởi động mềm) cho quạt Blower và Case Fan để bảo vệ nguồn và giảm tiếng ồn.
    *   Cơ chế **Hysteresis** (Vùng trễ) giúp thiết bị không bật tắt liên tục.
    *   Chế độ **After-Cool**: Chạy quạt làm khô tản nhiệt sau khi giảm ẩm để tránh đọng nước.
3.  **Giám sát & An toàn:**
    *   Theo dõi nhiệt độ nước làm mát Sò nóng bằng cảm biến **DS18B20**.
    *   Tự động ngắt hệ thống và bật quạt tản nhiệt hết công suất nếu nước quá nóng (> 70°C).
4.  **Giao diện & Lưu trữ:**
    *   Hiển thị thông số Real-time trên màn hình **LCD 1602**.
    *   Cài đặt mức ẩm mong muốn (Target) trực tiếp qua nút bấm.
    *   Ghi dữ liệu (Data Logging) vào thẻ nhớ **MicroSD** (file `log.csv`) mỗi 10 giây.

## 🛠️ Phần Cứng Yêu Cầu

Xem hướng dẫn lắp ráp chi tiết tại file: [HUONG_DAN_LAP_DAT.txt](HUONG_DAN_LAP_DAT.txt)

*   **MCU:** ESP32 WROOM-32.
*   **Cảm biến:** SHT35 (Nhiệt/Ẩm môi trường), DS18B20 (Nhiệt nước).
*   **Actuators (12V):** Sò nóng lạnh Peltier (12706), Máy bơm khí 370, Quạt Sên 5015, Quạt tản nhiệt 4x4.
*   **Driver:** 5x Module MOSFET D4184.
*   **Khác:** Màn hình LCD 1602 (I2C), Module MicroSD (SPI), Nguồn tổ ong 12V-10A.

## 🔌 Sơ Đồ Kết Nối (Pinout)

| Chức Năng | Linh Kiện | GPIO (ESP32) | Ghi Chú |
| :--- | :--- | :--- | :--- |
| **I2C Bus** | LCD & SHT35 | **21 (SDA), 22 (SCL)** | Dùng chung bus I2C |
| **SPI Bus** | Thẻ nhớ SD | **5 (CS), 23 (MOSI), 19 (MISO), 18 (CLK)** | |
| **OneWire** | DS18B20 | **4** | Cần trở kéo lên 4.7k |
| **Output** | Sò Peltier | **13** | MOSFET 1 |
| **Output** | Máy Bơm | **17** | MOSFET 2 |
| **PWM** | Quạt Sên (Blower) | **14** | MOSFET 3 |
| **PWM** | Quạt Tản Vỏ | **26** | MOSFET 4 |
| **Output** | Quạt Đối Lưu | **27** | MOSFET 5 (Luôn bật) |
| **Input** | Nút MODE | **32** | Kích âm (GND) |
| **Input** | Nút UP | **33** | Kích âm (GND) |
| **Input** | Nút DOWN | **25** | Kích âm (GND) |

## 🚀 Cài Đặt & Build

Dự án được xây dựng trên **Espressif IoT Development Framework (ESP-IDF)**.

1.  **Cài đặt môi trường:**
    Đảm bảo bạn đã cài đặt ESP-IDF (VS Code Extension hoặc dòng lệnh).

2.  **Cấu hình:**
    ```bash
    idf.py set-target esp32
    idf.py menuconfig
    ```
    *Lưu ý: Cần chỉnh cấu hình Flash size lên 4MB nếu cần thiết.*

3.  **Biên dịch & Nạp code:**
    ```bash
    idf.py build
    idf.py -p /dev/ttyUSB0 flash monitor
    ```
    *(Thay `/dev/ttyUSB0` bằng cổng COM tương ứng trên máy bạn).*

## 📊 Cấu Trúc Thư Mục

```text
VXL/
├── main/
│   ├── VXL.c                # Mã nguồn chính (Logic điều khiển)
│   └── CMakeLists.txt       # Cấu hình build cho thư mục main
├── components/              # Các thư viện phụ thuộc (nếu có)
├── HUONG_DAN_LAP_DAT.txt    # Hướng dẫn thi công phần cứng chi tiết
├── README.md                # Tài liệu dự án
└── sdkconfig                # Cấu hình dự án ESP-IDF
```

## 📝 Nhật Ký Thay Đổi (Changelog)

*   **26/12/2025:**
    *   Tối ưu hóa sơ đồ chân (Chuyển Pump sang GPIO 17 để tránh lỗi Boot).
    *   Thêm tính năng Soft-Start cho quạt.
    *   Hoàn thiện bảng hướng dẫn lắp đặt.
    *   Thêm Macro định nghĩa chân (PINs) trong code để dễ bảo trì.
