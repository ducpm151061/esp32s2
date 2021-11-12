#include "config.h"
#include "mqtt.h"

static const char *TAG = "template";

void led_task(void *pvParameters)
{
    led_strip_t strip = {
        .type = LED_TYPE,
        .length = LED_STRIP_LEN,
        .gpio = LED_GPIO,
        .buf = NULL,
#ifdef LED_STRIP_BRIGHTNESS
        .brightness = 255,
#endif
    };
    ESP_ERROR_CHECK(led_strip_init(&strip));
    static rgb_t color;
    while (1)
    {
        color.r = esp_random() % 0xff;
        color.g = esp_random() % 0xff;
        color.b = esp_random() % 0xff;

        ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, color));
        ESP_ERROR_CHECK(led_strip_flush(&strip));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void aht_task(void *pvParameters)
{
    aht_t dev = {0};
    dev.mode = AHT_MODE_NORMAL;
    dev.type = AHT_TYPE;

    ESP_ERROR_CHECK(aht_init_desc(&dev, AHT_I2C_ADDRESS_GND, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(aht_init(&dev));

    bool calibrated;
    ESP_ERROR_CHECK(aht_get_status(&dev, NULL, &calibrated));
    if (calibrated)
        ESP_LOGI(TAG, "Sensor calibrated");
    else
        ESP_LOGW(TAG, "Sensor not calibrated!");

    float temperature, humidity;

    while (1)
    {
        esp_err_t res = aht_get_data(&dev, &temperature, &humidity);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.2f%%", temperature, humidity);
            /* send data */
            mqtt_send(temperature, "temperature");
            mqtt_send(humidity, "humidity");
        }
        else
            ESP_LOGE(TAG, "Error reading data: %d (%s)", res, esp_err_to_name(res));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    mqtt_init();
    led_strip_install();
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(led_task, "led", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    xTaskCreatePinnedToCore(aht_task, "aht10", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
