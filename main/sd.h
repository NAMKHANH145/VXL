/**
 * @file sd_card.h
 * @brief Driver giao tiếp thẻ nhớ SD qua chuẩn SPI cho ESP32.
 * 
 * @note Sơ đồ nối dây (Wemos D1 MicroSD Shield):
 * | Chức năng | Chân ESP32 (Code) | Chân trên Shield (Ký hiệu) |
 * | :---      | :---              | :---                      |
 * | **CS**     | GPIO 5            | **D8**                    |
 * | **MOSI**   | GPIO 23           | **D7**                    |
 * | **MISO**   | GPIO 19           | **D6**                    |
 * | **CLK**    | GPIO 18           | **D5**                    |
 * | **VCC**    | 3.3V              | **3V3**                   |
 * | **GND**    | GND               | **G**                     |
 */

#ifndef SD_H
#define SD_H

#include <stdbool.h>
#include <stdio.h>
#include "esp_err.h"
#include "config.h" 

// Note: Pin Definitions (PIN_SD_CS, etc.) are now imported from config.h

/**
 * @brief Khởi tạo bus SPI và mount hệ thống tập tin FATFS lên thẻ nhớ.
 * 
 * @return 
 *      - ESP_OK: Mount thành công.
 *      - ESP_FAIL: Lỗi hệ thống tập tin hoặc định dạng.
 *      - Các mã lỗi khác từ driver SPI.
 */
esp_err_t sd_card_init(void);

/**
 * @brief Hủy mount thẻ nhớ và giải phóng tài nguyên bus SPI.
 * 
 * @return esp_err_t ESP_OK nếu thành công.
 */
esp_err_t sd_card_unmount(void);

/**
 * @brief Kiểm tra xem thẻ nhớ hiện tại có đang trong trạng thái mounted hay không.
 * 
 * @return true Nếu đã mount thành công.
 * @return false Nếu chưa mount hoặc đã bị tháo.
 */
bool sd_card_is_mounted(void);

/**
 * @brief Tự động tìm tên file log khả dụng tiếp theo (log_0.txt, log_1.txt...)
 * để tránh việc ghi đè dữ liệu sau khi hệ thống reset hoặc mất nguồn.
 * 
 * @param base_name Tên gốc của file (ví dụ: "log").
 * @param ext Đuôi mở rộng của file (ví dụ: ".txt").
 * @param out_name Buffer để lưu tên file hoàn chỉnh tìm được.
 * @param len Độ dài của buffer.
 */
void sd_get_next_filename(const char* base_name, const char* ext, char *out_name, size_t len);

/**
 * @brief Ghi thêm dữ liệu (Append) vào cuối file chỉ định.
 * 
 * @param path Đường dẫn đầy đủ của file (ví dụ: "/sdcard/log_0.txt").
 * @param data Chuỗi dữ liệu cần ghi.
 * @return esp_err_t ESP_OK nếu thành công.
 */
esp_err_t sd_card_append_file(const char *path, const char *data);

/**
 * @brief Đọc toàn bộ nội dung của một file và in trực tiếp ra Serial Console.
 * Dùng để kiểm tra dữ liệu nhanh mà không cần đầu đọc thẻ.
 * 
 * @param path Đường dẫn file cần đọc.
 * @return esp_err_t ESP_OK nếu đọc thành công.
 */
esp_err_t sd_card_dump_file_to_console(const char *path);

/**
 * @brief Ghi dữ liệu cảm biến (Nhiệt độ, Độ ẩm) theo format yêu cầu.
 * 
 * @param filename Tên file đang thực hiện logging.
 * @param temp Giá trị nhiệt độ.
 * @param hum Giá trị độ ẩm.
 */
void sd_log_sensor_data(const char* filename, float temp, float hum);

/**
 * @brief In thông tin chi tiết của thẻ nhớ (Dung lượng tổng, dung lượng còn trống).
 */
void sd_card_print_info(void);

#endif // SD_H