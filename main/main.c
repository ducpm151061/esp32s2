#include "driver/i2c.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_wifi.h"

#include "cJSON.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "led_strip.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>

#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#define AHT10_ADDRESS_0X38 0x38 // chip I2C address no.1 for AHT10/AHT15/AHT20, address pin connected to GND
#define AHT10_ADDRESS_0X39 0x39 // chip I2C address no.2 for AHT10 only, address pin connected to Vcc

#define I2C_MASTER_SCL_IO 9 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 8 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM                                                                                                 \
    0 /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define AHT10_CMD_INIT 0xE1       // Calibration command for AHT10/AHT15
#define AHT20_CMD_INIT 0xBE       // Calibration command for AHT10/AHT15
#define AHTX0_CMD_SOFTRESET 0xBA  // Soft reset command
#define AHTX0_CMD_MEASURMENT 0xAC // Start measurment command
#define AHTX0_CMD_NORMAL 0xA8     // Normal cycle mode command, no info in datasheet!!!

#define AHTX0_INIT_NORMAL_MODE 0x00 // Enable normal mode
#define AHTX0_INIT_CYCLE_MODE 0x20  // Enable cycle mode
#define AHTX0_INIT_CAL_ENABLE 0x08  // Load factory calibration coeff
#define AHTX0_INIT_CMD_MODE 0x40    // Enable command mode

#define AHTX0_STATUS_CALIBRATED 0x08 // Status bit for calibrated
#define AHTX0_STATUS_BUSY 0x80       // Status bit for busy
#define AHTX0_STATUS_REG 0x71

#define AHTX0_DATA_MEASURMENT                                                                                          \
    0x33                    // No info in datasheet!!! my guess it is DAC resolution, saw someone send 0x00 instead
#define AHTX0_DATA_NOP 0x00 // No info in datasheet!!!

#define AHTX0_MEASURMENT_DELAY 80 // at least 75 milliseconds
#define AHTX0_POWER_ON_DELAY 40   // at least 20..40 milliseconds
#define AHTX0_SOFT_RESET_DELAY 20 // less than 20 milliseconds
#define AHTX0_CMD_DELAY 350       // at least 300 milliseconds, no info in datasheet!!!
#define AHTX0_TIMEOUT 1000        // default timeout
#define AHT10_ERROR 0xFF          // returns 255, if communication error is occurred

#define BROKER_URL "mqtt://demo.thingsboard.io"
#define BROKER_PORT 1883
#define TOPIC_TELEMETRY "v1/devices/me/telemetry"
#define TOPIC_ATTRIBUTES "v1/devices/me/attributes"
#define TOKEN_DEVICE "esp8266demo"
#define EXAMPLE_ESP_WIFI_SSID "AnSoft2.4G"
#define EXAMPLE_ESP_WIFI_PASS "0902246233"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define BLINK_LED_RMT_CHANNEL 0
#define BLINK_GPIO 18
typedef enum
{
    NORMAL_MODE = 0,
    CYCLE_MODE = 1,
    CMD_MODE = 2
} aht_mode_t;
static uint8_t s_led_state = 0;
static esp_mqtt_client_handle_t client;
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "template";

static int s_retry_num = 0;

static led_strip_t *pStrip_a;

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t __attribute__((unused)) i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (AHT10_ADDRESS_0X38 << 1) | I2C_MASTER_READ, 0x1);
    i2c_master_write_byte(cmd, AHTX0_STATUS_REG, 0x1);
    if (size > 1)
    {
        i2c_master_read(cmd, data_rd, size - 1, 0X0);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, 0X1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t __attribute__((unused)) i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT10_ADDRESS_0X38 << 1) | I2C_MASTER_WRITE, 0x1);
    i2c_master_write(cmd, data_wr, size, 0x1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief ath10 initialization
 */
static void aht10_init(uint8_t mode)
{
    /* init aht10 sensor */
    uint8_t init_reg[] = {AHT10_CMD_INIT, AHTX0_INIT_CAL_ENABLE | mode, AHTX0_DATA_NOP};
    ESP_ERROR_CHECK(i2c_master_write_slave(I2C_MASTER_NUM, init_reg, 3));
}

/**
 * @brief ath10 reset
 */
static void aht10_reset()
{
    /* reset aht10 sensor */
    ESP_ERROR_CHECK(i2c_master_write_slave(I2C_MASTER_NUM, (uint8_t *)AHTX0_CMD_SOFTRESET, 1));
}

/**
 * @brief ath10 read data
 */
static void aht10_read(int16_t *temperature, int16_t *humidity)
{
    /* measure aht10 */
    static uint8_t measure_reg[] = {AHTX0_CMD_MEASURMENT, AHTX0_DATA_MEASURMENT, AHTX0_DATA_NOP};
    ESP_ERROR_CHECK(i2c_master_write_slave(I2C_MASTER_NUM, measure_reg, 3));

    /* read aht10 */
    static uint32_t temp = 0, humi = 0;
    static uint8_t data[5];
    ESP_ERROR_CHECK(i2c_master_read_slave(I2C_MASTER_NUM, data, 6));
    // ESP_LOGI(TAG, "AHT10 status: %d", data[0] & 0x68);

    if ((data[0] & 0x68) == 0x08)
    {
        humi = data[1];
        humi = (humi << 8) | data[2];
        humi = (humi << 4) | data[3];
        humi = humi >> 4;
        humi = (humi * 1000) / 1024 / 1024;

        temp = data[3];
        temp = temp & 0x0000000F;
        temp = (temp << 8) | data[4];
        temp = (temp << 8) | data[5];
        temp = (temp * 200) / 1024 / 1024 - 50;
    }
    *temperature = temp;
    *humidity = humi;
    ESP_LOGI(TAG, "AHT10 temperature: %d°C, humidity: %d", temp, humi);
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        msg_id = esp_mqtt_client_subscribe(client, TOPIC_TELEMETRY, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, TOPIC_TELEMETRY, 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, TOPIC_TELEMETRY);
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC_TELEMETRY=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last error string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void send_data(void)
{
    static int16_t temperature, humidity;
    aht10_read(&temperature, &humidity);
    cJSON *data = cJSON_CreateObject();
    cJSON_AddNumberToObject(data, "temperature", temperature);
    cJSON_AddNumberToObject(data, "humidity", humidity);
    char *post_data = cJSON_PrintUnformatted(data);
    esp_mqtt_client_publish(client, TOPIC_TELEMETRY, post_data, 0, 1, 0);
    cJSON_Delete(data);
    free(post_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = BROKER_URL, .port = BROKER_PORT, .username = TOKEN_DEVICE, .password = ""};
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state)
    {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        uint32_t r = esp_random() % 255;
        uint32_t g = esp_random() % 255;
        uint32_t b = esp_random() % 255;
        pStrip_a->set_pixel(pStrip_a, 0, r, g, b);
        /* Refresh the strip to send data */
        pStrip_a->refresh(pStrip_a, 100);
    }
    else
    {
        /* Set all LED off to clear all pixels */
        pStrip_a->clear(pStrip_a, 50);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(BLINK_LED_RMT_CHANNEL, BLINK_GPIO, 1);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = EXAMPLE_ESP_WIFI_SSID,
                .password = EXAMPLE_ESP_WIFI_PASS,
                /* Setting a password implies station will connect to all security modes including WEP/WPA.
                 * However these modes are deprecated and not advisable to be used. Incase your Access point
                 * doesn't support WPA2, these mode can be enabled by commenting below line */
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,

                .pmf_cfg = {.capable = true, .required = false},
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits =
        xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

void app_main(void)
{
    /* init i2c */
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    aht10_init(NORMAL_MODE);
    // Configure Led
    configure_led();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    /* config mqtt */
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    mqtt_app_start();

    while (1)
    {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        send_data();
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
