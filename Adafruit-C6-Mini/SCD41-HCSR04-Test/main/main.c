#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

/* ── SCD41 config ── */
#define SCD41_ADDR          0x62
#define I2C_SDA_GPIO        19
#define I2C_SCL_GPIO        18
#define I2C_POWER_GPIO      20

/* ── HC-SR04 config ── */
#define TRIG_GPIO           14
#define ECHO_GPIO           15
#define TIMEOUT_US          30000  // ~5 meter max range

static const char *TAG_SCD  = "SCD41";
static const char *TAG_SR04 = "HC-SR04";

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

/* ── SCD41 helpers ── */

static esp_err_t scd41_send_cmd(uint16_t cmd)
{
    uint8_t buf[2] = { cmd >> 8, cmd & 0xFF };
    return i2c_master_transmit(dev_handle, buf, 2, 1000);
}

static esp_err_t scd41_read(uint8_t *data, size_t len)
{
    return i2c_master_receive(dev_handle, data, len, 1000);
}

static esp_err_t scd41_cmd_read(uint16_t cmd, uint8_t *data, size_t len,
                                uint32_t delay_ms)
{
    esp_err_t err = scd41_send_cmd(cmd);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    return scd41_read(data, len);
}

/* ── HC-SR04 helper ── */

static float measure_distance_cm(void)
{
    /* Send 10µs trigger pulse */
    gpio_set_level(TRIG_GPIO, 0);
    ets_delay_us(2);
    gpio_set_level(TRIG_GPIO, 1);
    ets_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);

    /* Wait for echo to go HIGH (start of pulse) */
    int64_t deadline = esp_timer_get_time() + TIMEOUT_US;
    while (gpio_get_level(ECHO_GPIO) == 0) {
        if (esp_timer_get_time() > deadline) return -1.0f;
    }
    int64_t start = esp_timer_get_time();

    /* Wait for echo to go LOW (end of pulse) */
    deadline = start + TIMEOUT_US;
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if (esp_timer_get_time() > deadline) return -1.0f;
    }
    int64_t end = esp_timer_get_time();

    /* distance_cm = pulse_us / 58 */
    return (float)(end - start) / 58.0f;
}

/* ── Tasks ── */

static void scd41_task(void *arg)
{
    /* Start periodic measurement */
    scd41_send_cmd(0x21B1);
    ESP_LOGI(TAG_SCD, "Periodic measurement started. First reading in ~5s...\n");

    uint32_t reading_num = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        uint8_t data[9];
        esp_err_t err = scd41_cmd_read(0xEC05, data, 9, 1);
        if (err != ESP_OK) {
            ESP_LOGW(TAG_SCD, "Read failed, retrying...");
            continue;
        }

        uint16_t co2_raw  = (data[0] << 8) | data[1];
        uint16_t temp_raw = (data[3] << 8) | data[4];
        uint16_t rh_raw   = (data[6] << 8) | data[7];

        float temp_c = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
        float rh     = 100.0f * ((float)rh_raw / 65535.0f);

        reading_num++;
        ESP_LOGI(TAG_SCD, "[Reading #%lu]  CO2: %u ppm  |  Temp: %.1f C  |  RH: %.1f %%",
                 (unsigned long)reading_num, co2_raw, temp_c, rh);
    }
}

static void hcsr04_task(void *arg)
{
    /* Configure trig as output */
    gpio_config_t trig_cfg = {
        .pin_bit_mask = (1ULL << TRIG_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&trig_cfg);

    /* Configure echo as input */
    gpio_config_t echo_cfg = {
        .pin_bit_mask = (1ULL << ECHO_GPIO),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&echo_cfg);

    ESP_LOGI(TAG_SR04, "GPIO configured (Trig=%d, Echo=%d)", TRIG_GPIO, ECHO_GPIO);

    uint32_t reading_num = 0;

    while (1) {
        float dist = measure_distance_cm();
        reading_num++;

        if (dist < 0) {
            ESP_LOGW(TAG_SR04, "[Reading #%lu] TIMEOUT — no echo received",
                     (unsigned long)reading_num);
        } else {
            ESP_LOGI(TAG_SR04, "[Reading #%lu] Distance: %.1f cm (%.1f in)",
                     (unsigned long)reading_num, dist, dist / 2.54f);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ── Main ── */

void app_main(void)
{
    ESP_LOGI(TAG_SCD, "========================================");
    ESP_LOGI(TAG_SCD, "  SCD41 + HC-SR04 Test — ESP-IDF");
    ESP_LOGI(TAG_SCD, "  Target: ESP32-C6 Feather");
    ESP_LOGI(TAG_SCD, "  IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG_SCD, "========================================");

    /* Enable STEMMA QT power — Arduino does this for you, ESP-IDF does not */
    gpio_config_t pwr_cfg = {
        .pin_bit_mask = (1ULL << I2C_POWER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&pwr_cfg);
    gpio_set_level(I2C_POWER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG_SCD, "STEMMA QT power enabled (GPIO %d HIGH)", I2C_POWER_GPIO);

    /* Configure I2C bus */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    /* Add SCD41 device */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD41_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    ESP_LOGI(TAG_SCD, "I2C initialized (SDA=%d, SCL=%d, addr=0x%02X)",
             I2C_SDA_GPIO, I2C_SCL_GPIO, SCD41_ADDR);

    /* Stop any previous measurement */
    scd41_send_cmd(0x3F86);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Read serial number (0x3682 -> 9 bytes: 3x [MSB, LSB, CRC]) */
    uint8_t serial_buf[9];
    esp_err_t err = scd41_cmd_read(0x3682, serial_buf, 9, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SCD, "Failed to read serial number (err=%d). Check wiring!", err);
        return;
    }
    ESP_LOGI(TAG_SCD, "SCD41 found! Serial: %02X%02X-%02X%02X-%02X%02X",
             serial_buf[0], serial_buf[1],
             serial_buf[3], serial_buf[4],
             serial_buf[6], serial_buf[7]);

    /* Launch tasks */
    xTaskCreate(scd41_task,   "scd41",   4096, NULL, 5, NULL);
    xTaskCreate(hcsr04_task,  "hcsr04",  4096, NULL, 5, NULL);
}
