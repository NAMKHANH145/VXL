#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "esp_log.h"
#include "sd.h"

static const char *TAG = "SD_DRIVER";

#define MOUNT_POINT "/sdcard"

static sdmmc_card_t *card;
static bool mounted = false;

// --- INIT & MOUNT --- (Giữ nguyên logic khởi tạo ổn định từ trước)
esp_err_t sd_card_init(void) {
    if (mounted) return ESP_OK;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,  // CRITICAL FIX: Don't format, it blocks for minutes!
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 5000; // 5MHz

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_SD_MOSI,
        .miso_io_num = PIN_SD_MISO,
        .sclk_io_num = PIN_SD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    // Init BUS chỉ nếu chưa init (đề phòng gọi nhiều lần)
    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI Bus Init Failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // If INVALID_STATE, it means already initialized, which is fine.

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_SD_CS;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret == ESP_OK) {
        mounted = true;
        ESP_LOGI(TAG, "SD Card Mounted Successfully");
        sd_card_print_info();
    } else {
        ESP_LOGE(TAG, "SD Mount Failed: %s", esp_err_to_name(ret));
        // Cleanup if mount failed but bus was just initialized? 
        // In complex app, we might keep bus. Here we leave it.
    }
    return ret;
}

esp_err_t sd_card_unmount(void) {
    if (!mounted) return ESP_OK;

    esp_err_t ret = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    if (ret == ESP_OK) {
        mounted = false;
        ESP_LOGI(TAG, "Card Unmounted");
        // Deinit bus SPI if needed, usually we keep it
    }
    return ret;
}

bool sd_card_is_mounted(void) {
    return mounted;
}

// --- FILE LOGIC MỚI ---

void sd_get_next_filename(const char* base_name, const char* ext, char *out_name, size_t len) {
    if (!mounted) return; 
    
    int index = 0;
    struct stat st;
    
    // Vòng lặp tìm file chưa tồn tại: log_0 -> log_1 -> ... -> log_999
    while (index < 1000) {
        snprintf(out_name, len, "%s/%s_%d%s", MOUNT_POINT, base_name, index, ext);
        if (stat(out_name, &st) != 0) {
            // Stat != 0 nghĩa là file không tồn tại => Dùng tên này
            ESP_LOGW(TAG, "New Session File: %s", out_name);
            return;
        }
        index++;
    }
    // Fallback nếu quá 1000 file
    snprintf(out_name, len, "%s/%s_overflow%s", MOUNT_POINT, base_name, ext);
}

esp_err_t sd_card_append_file(const char *path, const char *data) {
    if (!mounted) {
        ESP_LOGE(TAG, "Card not mounted");
        return ESP_FAIL;
    }

    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file %s", path);
        return ESP_FAIL;
    }
    fprintf(f, "%s", data);
    fclose(f);
    return ESP_OK;
}

esp_err_t sd_card_dump_file_to_console(const char *path) {
    if (!mounted) return ESP_FAIL;

    ESP_LOGW(TAG, "--- START DUMPING FILE CONTENT: %s ---", path);
    
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "File not found to dump");
        return ESP_FAIL;
    }

    char buffer[128];
    while (fgets(buffer, sizeof(buffer), f) != NULL) {
        // Dùng printf để in thô, không kèm timestamp của LOG hệ thống -> dễ copy
        printf("%s", buffer); 
    }
    
    fclose(f);
    ESP_LOGW(TAG, "--- END OF FILE ---");
    return ESP_OK;
}

void sd_log_sensor_data(const char* filename, float temp, float hum) {
    if (!mounted) return;

    char buffer[64];
    // Format yêu cầu: "Nhiệt độ:xxx, độ ẩm: xxx\n"
    snprintf(buffer, sizeof(buffer), "Nhiệt độ:%.2f, độ ẩm: %.2f\n", temp, hum);
    
    esp_err_t ret = sd_card_append_file(filename, buffer);
    if (ret == ESP_OK) {
        // Chỉ in dấu chấm để biết đang chạy, đỡ rác màn hình khi chưa cần xem
        // printf("."); 
        // fflush(stdout);
        ESP_LOGI(TAG, "Saved: %.2f | %.2f", temp, hum);
    } else {
        ESP_LOGE(TAG, "Save Failed!");
    }
}

// --- UTILS ---
void sd_card_print_info(void) {
    if (!mounted) return; 
    FATFS *fs;
    DWORD fre_clust, fre_sect, tot_sect;
    if (f_getfree("0:", &fre_clust, &fs) == FR_OK) {
        tot_sect = (fs->n_fatent - 2) * fs->csize;
        fre_sect = fre_clust * fs->csize;
        ESP_LOGI(TAG, "Total: %lu MB, Free: %lu MB", tot_sect / 2 / 1024, fre_sect / 2 / 1024);
    }
}