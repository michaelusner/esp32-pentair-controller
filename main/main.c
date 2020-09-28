#include <stdlib.h>
#include <string.h>
#include <esp_wifi.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "wifi_manager.h"
#include "http_app.h"

#include "mqtt_client.h"

const esp_mqtt_client_config_t mqtt_cfg = {
    .uri = "mqtt://nas.usner.net:1883",
    .username = "hauser",
    .password = "l3mm3IN",
    .lwt_topic = "homeassistant/pool/active",
    .lwt_msg = "offline",
    .lwt_msg_len = 7,
    .lwt_qos = 0,
    .lwt_retain = true};
esp_mqtt_client_handle_t client;
#define MQTT_LOOP_STACK_SIZE (2048)
#define MQTT_LOOP_PRIO (11)

#define MQTT_HASSIO_STATUS_TOPIC "homeassistant/status"
#define MQTT_POOL_AVAILABLE_TOPIC "homeassistant/pool/available"
#define MQTT_POOL_PUMP1_TOPIC "homeassistant/pool/pump1"
#define MQTT_POOL_PUMP2_TOPIC "homeassistant/pool/pump2"
#define MQTT_POOL_PUMP3_TOPIC "homeassistant/pool/pump3"
#define MQTT_POOL_PUMP4_TOPIC "homeassistant/pool/pump4"
#define MQTT_POOL_TEMPERATURE_TOPIC "homeassistant/pool/pool_temperature"
#define MQTT_AIR_TEMPERATURE_TOPIC "homeassistant/pool/air_temperature"
#define MQTT_SPA_TOPIC "homeassistant/pool/spa"
#define MQTT_CLEANER_TOPIC "homeassistant/pool/cleaner"
#define MQTT_AIR_BLOWER_TOPIC "homeassistant/pool/air_blower"
#define MQTT_SPA_LIGHT_TOPIC "homeassistant/pool/spa_light"
#define MQTT_POOL_LIGHT_TOPIC "homeassistant/pool/pool_light"
#define MQTT_POOL_TOPIC "homeassistant/pool/pool"
#define MQTT_WATER_FEATURE1_TOPIC "homeassistant/pool/water_feature1"
#define MQTT_SPILLWAY_TOPIC "homeassistant/pool/spillway"

#define MQTT_POOL_COMMAND_TOPIC "homeassistant/pool/pool/set"
#define MQTT_SPA_COMMAND_TOPIC "homeassistant/pool/spa/set"
#define MQTT_POOL_LIGHT_COMMAND_TOPIC "homeassistant/pool/pool_light/set"
#define MQTT_SPA_LIGHT_COMMAND_TOPIC "homeassistant/pool/spa_light/set"

#define SPA_STATE(x) (x >> 0) & 1
#define CLEANER_STATE(x) (x >> 1) & 1
#define AIR_BLOWER_STATE(x) (x >> 2) & 1
#define SPA_LIGHT_STATE(x) (x >> 3) & 1
#define POOL_LIGHT_STATE(x) (x >> 4) & 1
#define PUMP1_STATE(x) (x >> 5) & 1
#define WATER_FEATURE1_STATE(x) (x >> 6) & 1
#define SPILLWAY_STATE(x) (x >> 7) & 1

// Note: UART2 default pins IO16, IO17 do not work on ESP32-WROVER module
// because these pins connected to PSRAM
#define ECHO_TEST_TXD (17)
#define ECHO_TEST_RXD (16)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define ECHO_TEST_RTS (4)

// CTS is not used in RS485 Half-Duplex Mode
#define ECHO_TEST_CTS UART_PIN_NO_CHANGE

#define BUF_SIZE (127)
#define BAUD_RATE (9600)

// Read packet timeout
#define PACKET_READ_TICS (100 / portTICK_RATE_MS)
#define READ_RS485_STACK_SIZE (2048)
#define READ_RS485_PRIO (10)
#define ECHO_UART_PORT (UART_NUM_2)

static const char *TAG = "RS485_PENTAIR_CONTROLLER";

const uint8_t packet_header = 0xA5;
const int uart_num = ECHO_UART_PORT;

typedef struct
{
    uint8_t hour;          // 6
    uint8_t minute;        // 7
    uint8_t equip1;        // 8
    uint8_t equip2;        // 9
    uint8_t equip3;        // 10
    uint8_t reserved1;     // 11
    uint8_t reserved2;     // 12
    uint8_t reserved3;     // 13
    uint8_t reserved4;     // 14
    uint8_t uom;           // 15 - 0 == Fahrenheit, 4 == celsius
    uint8_t valve;         // 16
    uint8_t reserved5;     // 17
    uint8_t delay;         // 18 - 64==??; 65-135 (for 50 circuits) is the circuit that is currently delayed.
    uint8_t unknown;       // 19 - Something to do with heat.
    uint8_t pool_temp;     // 20
    uint8_t spa_temp;      // 21
    uint8_t heater_active; // 22 - 0=off.  32=on.  More here?
    uint8_t reserved6;     // 23
    uint8_t air_temp;      // 24
    uint8_t solar_temp;    // 25
    uint8_t reserved7;     // 26
    uint8_t reserved8;     // 27
    uint8_t heater_mode;   // 28
    uint8_t reserved9;     // 29
    uint8_t reserved10;    // 30
    uint8_t reserved11;    // 31
    uint8_t misc2;         // 32 - 0=do not automatically adjust DST, 1=automatically adjust DST
} MAIN_STATUS_PACKET;

typedef struct __attribute__((packed, scalar_storage_order("big-endian")))
{
    uint8_t started;     // 6
    uint8_t feature1;    // 7
    uint8_t drive_state; // 8
    uint16_t watts;      // 9
    uint16_t rpm;        // 11
    uint8_t gpm;         // 12
    uint8_t percent;     // 13
    uint8_t unknown1;    // 14
    uint8_t err;         // 15
    uint8_t unknown2;    //16
    uint8_t timer;       // 17
    uint16_t clk;        // 18
} PUMP_STATUS_PACKET;

typedef enum
{
    CHLORINATOR = 0x02,
    BROADCAST = 0x0f,
    MAIN = 0x10,
    SECONDARY = 0x11,
    REMOTE = 0x20,
    PUMP1 = 0x60,
    PUMP2 = 0x61,
    PUMP3 = 0x62,
    PUMP4 = 0x63
} DEVICE;

typedef enum
{
    SPA = 1,
    CLEANER = 2,
    AIR_BLOWER = 3,
    SPA_LIGHT = 4,
    POOL_LIGHT = 5,
    POOL = 6,
    WATER_FEATURE = 7,
    SPILLWAY = 8,
    AUX = 9
} FEATURE;

// define the Pentair RS485 packet struct
typedef struct
{
    uint8_t leading_byte;
    uint8_t unknown;
    DEVICE dest;
    DEVICE src;
    uint8_t command;
    uint8_t length;
    uint8_t *data;
    uint16_t checksum;
} Packet;

typedef enum
{
    SET_COLOR = 96,
    SET_CIRCUIT = 134
} COMMAND;

char *
getDeviceName(DEVICE device)
{
    switch (device)
    {
    case CHLORINATOR:
        return "Chlorinator";
    case BROADCAST:
        return "Broadcast";
    case MAIN:
        return "Main";
    case SECONDARY:
        return "Secondary";
    case REMOTE:
        return "Remote";
    case PUMP1:
        return "Pump1";
    case PUMP2:
        return "Pump2";
    case PUMP3:
        return "Pump3";
    case PUMP4:
        return "Pump4";
    default:
        return "Unknown";
    }
}

MAIN_STATUS_PACKET *main_status;
PUMP_STATUS_PACKET *pump_status;

static esp_err_t my_get_handler(httpd_req_t *req)
{
    char response[100];
    /* our custom page sits at /helloworld in this example */
    if (strcmp(req->uri, "/helloworld") == 0)
    {
        ESP_LOGI(TAG, "Serving page /helloworld");
        sprintf(response, "<html><body>%02d:%02d</body></html>", main_status->hour, main_status->minute);

        httpd_resp_set_status(req, "200 OK");
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, response, strlen(response));
    }
    else
    {
        /* send a 404 otherwise */
        httpd_resp_send_404(req);
    }

    return ESP_OK;
}

static char *str_state(int state)
{
    return state ? "on" : "off";
}

static void sendCommand(DEVICE from, DEVICE to, COMMAND command, uint8_t feature, uint8_t state)
{
    int res;
    ESP_LOGI(TAG, "Sending command from=%02x to=%02x command=%02x feature=%02x state=%02x", from, to, command, feature, state);
    uint8_t packet[12] = {0x00, 0xFF, 0xA5, 0x1F, to, from, command, 2, feature, state};
    uint16_t checksum = 0;
    for (int i = 2; i < 12; i++)
        checksum += packet[i];
    packet[10] = checksum / 256;
    packet[11] = checksum % 256;
    for (int i = 0; i < 12; i++)
    {
        printf("%02x ", packet[i]);
    }
    res = uart_write_bytes(uart_num, (char *)&packet, 12);
    ESP_LOGI(TAG, "%d", res);
}

static void readRS485()
{
    Packet packet;
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "Start RS485 application test and configure UART.");

    // Configure UART parameters
    uart_param_config(uart_num, &uart_config);

    ESP_LOGI(TAG, "UART set pins, mode and install driver.");
    // Set UART1 pins(TX: IO23, RX: I022, RTS: IO18, CTS: IO19)
    uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Set RS485 half duplex mode
    uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);

    // Allocate buffers for UART
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    char mqtt_data[20];
    char decode_data[10];
    uint16_t checksum;

    ESP_LOGI(TAG, "UART start recieve loop.\r\n");

    while (1)
    {
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, pdMS_TO_TICKS(100));

        if (len > 0)
        {
            //ESP_LOGI(TAG, "Received %u bytes:", len);
            for (int i = 0; i < len; i++)
            {
                if (data[i] == 0xFF && data[i + 1] == 0x00 && data[i + 2] == 0xFF && data[i + 3] == 0xA5)
                {
                    i += 3;
                    memset(&packet, 0, sizeof(packet));
                    packet.leading_byte = data[i];
                    packet.unknown = data[i + 1];
                    packet.dest = data[i + 2];
                    packet.src = data[i + 3];
                    packet.command = data[i + 4];
                    packet.length = data[i + 5];
                    packet.data = &data[i + 6];
                    packet.checksum = (data[i + 6 + packet.length] << 8) + data[i + 7 + packet.length];

                    // calculate the checksum
                    checksum = 0;
                    for (int j = i; j < i + packet.length + 6; j++)
                    {
                        ESP_LOGD(TAG, "%02x ", data[j]);
                        checksum = checksum + data[j];
                    }
                    ESP_LOGD(TAG, "\n");
                    if (checksum != packet.checksum)
                        ESP_LOGD(TAG, "\nCHECKSUM MISMATCH - got %d, expected %d\n", checksum, packet.checksum);
                    else
                    {
                        if (packet.dest != BROADCAST)
                        {
                            ESP_LOGI(TAG, "Dest    : (0x%02x) %s", packet.dest, getDeviceName(packet.dest));
                            ESP_LOGI(TAG, "Source  : (0x%02x) %s", packet.src, getDeviceName(packet.src));
                            ESP_LOGI(TAG, "Command : (0x%02x) %d", packet.command, packet.command);
                            ESP_LOGI(TAG, "\n");
                        }
                        if (packet.src == MAIN && packet.dest == BROADCAST)
                        {
                            main_status = (MAIN_STATUS_PACKET *)packet.data;
                            ESP_LOGD(TAG, "Time:      %d:%d", main_status->hour, main_status->minute);
                            ESP_LOGD(TAG, "Pump1: %s", str_state(PUMP1_STATE(main_status->equip1)));
                            ESP_LOGD(TAG, "Cleaner: %s", str_state(CLEANER_STATE(main_status->equip1)));
                            ESP_LOGD(TAG, "Air Blower: %s", str_state(AIR_BLOWER_STATE(main_status->equip1)));
                            ESP_LOGD(TAG, "Pool temp: %d\n", main_status->pool_temp);
                            ESP_LOGD(TAG, "Spa temp:  %d\n", main_status->spa_temp);
                            ESP_LOGD(TAG, "Air temp:  %d\n", main_status->air_temp);
                            ESP_LOGD(TAG, "Heater:    %d\n", main_status->heater_active);
                            ESP_LOGD(TAG, "Spa: %s", str_state(SPA_STATE(main_status->equip1)));
                            for (int i = 0; i < 8; i++)
                                decode_data[i] = 48 + ((main_status->equip1 >> (7 - i)) & 1);
                            ESP_LOGD(TAG, "%s", decode_data);
                            ESP_LOGD(TAG, "\n");
                            sprintf(mqtt_data, "%d", main_status->pool_temp);
                            esp_mqtt_client_publish(client, MQTT_POOL_TEMPERATURE_TOPIC, mqtt_data, 0, 1, 1);
                            sprintf(mqtt_data, "%d", main_status->air_temp);
                            esp_mqtt_client_publish(client, MQTT_AIR_TEMPERATURE_TOPIC, mqtt_data, 0, 1, 1);
                            esp_mqtt_client_publish(client, MQTT_SPA_TOPIC, str_state(SPA_STATE(main_status->equip1)), 0, 1, 1);
                            esp_mqtt_client_publish(client, MQTT_CLEANER_TOPIC, str_state(CLEANER_STATE(main_status->equip1)), 0, 1, 1);
                            esp_mqtt_client_publish(client, MQTT_AIR_BLOWER_TOPIC, str_state(AIR_BLOWER_STATE(main_status->equip1)), 0, 1, 1);
                            esp_mqtt_client_publish(client, MQTT_SPA_TOPIC, str_state(SPA_STATE(main_status->equip1)), 0, 1, 1);
                            esp_mqtt_client_publish(client, MQTT_POOL_LIGHT_TOPIC, str_state(POOL_LIGHT_STATE(main_status->equip1)), 0, 1, 1);
                            esp_mqtt_client_publish(client, MQTT_SPA_LIGHT_TOPIC, str_state(SPA_LIGHT_STATE(main_status->equip1)), 0, 1, 1);
                            esp_mqtt_client_publish(client, MQTT_POOL_TOPIC, str_state(PUMP1_STATE(main_status->equip1)), 0, 1, 1);
                            esp_mqtt_client_publish(client, MQTT_WATER_FEATURE1_TOPIC, str_state(WATER_FEATURE1_STATE(main_status->equip1)), 0, 1, 1);
                            esp_mqtt_client_publish(client, MQTT_SPILLWAY_TOPIC, str_state(SPILLWAY_STATE(main_status->equip1)), 0, 1, 1);
                        }
                        else if ((packet.src = PUMP1 || packet.src == PUMP2 || packet.src == PUMP3 || packet.src == PUMP4) && packet.dest == MAIN && packet.command == 0x07)
                        {
                            // we have a pump status packet
                            pump_status = (PUMP_STATUS_PACKET *)packet.data;
                            ESP_LOGD(TAG, "Started:     %d\n", pump_status->started);
                            ESP_LOGD(TAG, "Feature 1:   %d\n", pump_status->feature1);
                            ESP_LOGD(TAG, "Drive state: %d\n", pump_status->drive_state);
                            ESP_LOGD(TAG, "Watts:       %d\n", pump_status->watts);
                            ESP_LOGD(TAG, "Watts:       %d\n", (packet.data[3] << 8) + packet.data[4]);
                            ESP_LOGD(TAG, "RPMs:        %d\n", pump_status->rpm);
                            ESP_LOGD(TAG, "GPMs:        %d\n", pump_status->gpm);
                            ESP_LOGD(TAG, "Percent:     %d\n", pump_status->percent);
                            ESP_LOGD(TAG, "Err:         %d\n", pump_status->err);
                            ESP_LOGD(TAG, "Timer:       %d\n", pump_status->timer);
                            ESP_LOGD(TAG, "Clock:       %d\n", pump_status->clk);
                        }
                    }

                    i = i + packet.length;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    uint8_t feature;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        msg_id = esp_mqtt_client_subscribe(client, MQTT_HASSIO_STATUS_TOPIC, 0);
        ESP_LOGI(TAG, "Subscribe to HomeAssistant, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, MQTT_POOL_COMMAND_TOPIC, 0);
        ESP_LOGI(TAG, "Subscribe to pool command topic, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, MQTT_SPA_COMMAND_TOPIC, 0);
        ESP_LOGI(TAG, "Subscribe to spa command topic, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, MQTT_POOL_LIGHT_COMMAND_TOPIC, 0);
        ESP_LOGI(TAG, "Subscribe to pool_light command topic, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, MQTT_SPA_LIGHT_COMMAND_TOPIC, 0);
        ESP_LOGI(TAG, "Subscribe to spa_light command topic, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, MQTT_POOL_AVAILABLE_TOPIC, "online", 0, 0, 1);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        ESP_LOGI(TAG, "TOPIC='%.*s'\r\n", event->topic_len, event->topic);
        ESP_LOGI(TAG, "DATA='%.*s'\r\n", event->data_len, event->data);

        if (strncmp(event->topic, MQTT_POOL_COMMAND_TOPIC, event->topic_len) == 0)
            feature = POOL;
        else if (strncmp(event->topic, MQTT_SPA_COMMAND_TOPIC, event->topic_len) == 0)
            feature = SPA;
        else if (strncmp(event->topic, MQTT_POOL_LIGHT_COMMAND_TOPIC, event->topic_len) == 0)
            feature = POOL_LIGHT;
        else if (strncmp(event->topic, MQTT_SPA_LIGHT_COMMAND_TOPIC, event->topic_len) == 0)
            feature = SPA_LIGHT;
        else
        {
            ESP_LOGI(TAG, "Unknown feature topic %s", event->topic);
            break;
        }
        if (strncmp(event->data, "on", event->data_len) == 0)
        {
            ESP_LOGI(TAG, "Switching feature %02x ON **********************", feature);
            sendCommand(REMOTE, MAIN, SET_CIRCUIT, feature, 1);
        }
        else if (strncmp(event->data, "off", event->data_len) == 0)
        {
            ESP_LOGI(TAG, "Switching feature %02x OFF *********************", feature);
            sendCommand(REMOTE, MAIN, SET_CIRCUIT, feature, 0);
        }

        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqttLoop()
{
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
    // init mqtt
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
    while (1)
    {
        ESP_LOGD(TAG, "MQTT delay");
        vTaskDelay(xDelay);
    }
}

void app_main()
{
    /* start the wifi manager */
    wifi_manager_start();

    /* set custom handler for the http server
     * Now navigate to /helloworld to see the custom page
     * */
    http_app_set_handler_hook(HTTP_GET, &my_get_handler);
    xTaskCreate(readRS485, "readRS485", READ_RS485_STACK_SIZE, NULL, READ_RS485_PRIO, NULL);
    xTaskCreate(mqttLoop, "mqttLoop", MQTT_LOOP_STACK_SIZE, NULL, MQTT_LOOP_PRIO, NULL);
}
