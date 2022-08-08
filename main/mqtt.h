#include "config.h"

#define BROKER_URL "mqtt://demo.thingsboard.io"
#define BROKER_PORT 1883
#define TOPIC_TELEMETRY "v1/devices/me/telemetry"
#define TOPIC_ATTRIBUTES "v1/devices/me/attributes"
#define TOKEN_DEVICE "esp32s2"
#define ESP_WIFI_SSID "Ansoft"
#define ESP_WIFI_PASS "0902246233"
#define ESP_MAXIMUM_RETRY 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

void mqtt_init(void);
void mqtt_send(float param, const char *name);