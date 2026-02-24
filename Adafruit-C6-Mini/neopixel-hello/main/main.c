#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"

static const char *TAG = "neopixel";

#define NEOPIXEL_GPIO       9
#define NEOPIXEL_POWER_GPIO 20

static led_strip_handle_t strip;
static volatile bool led_on = true;
static volatile int brightness = 20;  // 0-100 (HSV value)

static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    h %= 360;
    float S = s / 100.0f;
    float V = v / 100.0f;
    float C = V * S;
    float X = C * (1.0f - fabsf(fmodf(h / 60.0f, 2) - 1.0f));
    float m = V - C;

    float rf, gf, bf;
    if (h < 60)       { rf = C; gf = X; bf = 0; }
    else if (h < 120) { rf = X; gf = C; bf = 0; }
    else if (h < 180) { rf = 0; gf = C; bf = X; }
    else if (h < 240) { rf = 0; gf = X; bf = C; }
    else if (h < 300) { rf = X; gf = 0; bf = C; }
    else               { rf = C; gf = 0; bf = X; }

    *r = (uint8_t)((rf + m) * 255);
    *g = (uint8_t)((gf + m) * 255);
    *b = (uint8_t)((bf + m) * 255);
}

// Print uptime every 10 seconds
static void time_task(void *arg)
{
    while (1) {
        int64_t uptime_us = esp_timer_get_time();
        int total_secs = (int)(uptime_us / 1000000);
        int hrs  = total_secs / 3600;
        int mins = (total_secs % 3600) / 60;
        int secs = total_secs % 60;

        ESP_LOGI(TAG, "Uptime: %02d:%02d:%02d  |  LED %s  |  Brightness: %d%%",
                 hrs, mins, secs, led_on ? "ON" : "OFF", brightness);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// Listen for serial input: 'a' = toggle, '1' = dim, '2' = brighten
static void serial_input_task(void *arg)
{
    uint8_t buf[8];
    while (1) {
        int len = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < len; i++) {
            switch (buf[i]) {
            case 'a':
            case 'A':
                led_on = !led_on;
                ESP_LOGI(TAG, "LED %s", led_on ? "ON" : "OFF");
                break;
            case '1':
                if (brightness > 0) {
                    brightness -= 10;
                    if (brightness < 0) brightness = 0;
                }
                ESP_LOGI(TAG, "Brightness: %d%%", brightness);
                break;
            case '2':
                if (brightness < 100) {
                    brightness += 10;
                    if (brightness > 100) brightness = 100;
                }
                ESP_LOGI(TAG, "Brightness: %d%%", brightness);
                break;
            default:
                break;
            }
        }
    }
}

// Rainbow LED animation
static void led_task(void *arg)
{
    uint16_t hue = 0;

    while (1) {
        if (led_on) {
            uint8_t r, g, b;
            hsv_to_rgb(hue, 100, brightness, &r, &g, &b);
            led_strip_set_pixel(strip, 0, r, g, b);
            led_strip_refresh(strip);
            hue = (hue + 2) % 360;
        } else {
            led_strip_clear(strip);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Adafruit Feather ESP32-C6 — NeoPixel + Timer");
    ESP_LOGI(TAG, "Controls: 'a' = toggle LED, '1' = dim, '2' = brighten");

    // Install USB Serial/JTAG driver (ESP32-C6 uses this for console, not UART0)
    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = 256,
        .tx_buffer_size = 256,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    // Enable NeoPixel power (GPIO20 must be HIGH)
    gpio_config_t pwr_conf = {
        .pin_bit_mask = (1ULL << NEOPIXEL_POWER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&pwr_conf));
    ESP_ERROR_CHECK(gpio_set_level(NEOPIXEL_POWER_GPIO, 1));

    // Configure WS2812 LED strip (1 pixel) via RMT
    led_strip_config_t strip_config = {
        .strip_gpio_num = NEOPIXEL_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));
    ESP_ERROR_CHECK(led_strip_clear(strip));

    ESP_LOGI(TAG, "Starting tasks...");
    xTaskCreate(time_task, "time", 2048, NULL, 3, NULL);
    xTaskCreate(serial_input_task, "serial", 2048, NULL, 4, NULL);
    xTaskCreate(led_task, "led", 4096, NULL, 5, NULL);
}
