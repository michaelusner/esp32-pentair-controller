#include <stdlib.h>
#include <string.h>
#include <esp_wifi.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "wifi_manager.h"
#include "http_app.h"

#include "mqtt_client.h"

const esp_mqtt_client_config_t mqtt_cfg = {
    .uri = "mqtt://192.168.1.24:1883",
    .username = "hauser",
    .password = "l3mm3IN",
    .lwt_topic = "homeassistant/switch/pool/active",
    .lwt_msg = "offline",
    .lwt_msg_len = 7,
    .lwt_qos = 0,
    .lwt_retain = true};
esp_mqtt_client_handle_t client;

#define COMMAND_BLINK_GPIO 5
#define MQTT_BLINK_GPIO 18
#define HTTP_BLINK_GPIO 19

#define MQTT_LOOP_STACK_SIZE (2048)
#define MQTT_LOOP_PRIO (11)

#define COMMAND(X) X "/set"
#define CONFIG(X) X "/config"
#define STATE(X) X "/state"
#define STATUS(X) X "/status"

#define MQTT_QOS 0
#define MQTT_RETAIN 0
#define MQTT_HA_DISCOVERY_QOS 0
#define MQTT_HA_DISCOVERY_RETAIN 1
#define MQTT_HASSIO_STATUS_TOPIC "homeassistant/status"
#define MQTT_POOL_TEMPERATURE_TOPIC "homeassistant/sensor/pool/pool_temperature"
#define MQTT_AIR_TEMPERATURE_TOPIC "homeassistant/sensor/pool/air_temperature"
#define MQTT_SPA_TOPIC "homeassistant/switch/pool/spa"
#define MQTT_POOL_CLEANER_TOPIC "homeassistant/switch/pool/cleaner"
#define MQTT_AIR_BLOWER_TOPIC "homeassistant/switch/pool/air_blower"
#define MQTT_SOLAR_TEMPERATURE_TOPIC "homeassistant/sensor/pool/solar_temperature"
#define MQTT_SPA_LIGHT_TOPIC "homeassistant/switch/pool/spa_light"
#define MQTT_POOL_LIGHT_TOPIC "homeassistant/switch/pool/pool_light"
#define MQTT_POOL_TOPIC "homeassistant/switch/pool/pool"
#define MQTT_WATER_FEATURE1_TOPIC "homeassistant/switch/pool/water_feature1"
#define MQTT_SPILLWAY_TOPIC "homeassistant/switch/pool/spillway"

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

SemaphoreHandle_t semMainStatus = NULL;

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
    UNKNOWN = 0,
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

char *getDeviceName(DEVICE device)
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
bool mqtt_connected = false;

// Get a string state status from an int
static char *str_state(int state)
{
    return state ? "on" : "off";
}

// The prometheus /metrics handler
static esp_err_t http_get_handler(httpd_req_t *req)
{
    char response[700];
    ESP_LOGI(TAG, "GET %s\n", req->uri);
    gpio_set_level(HTTP_BLINK_GPIO, 1);
    if (strcmp(req->uri, "/metrics") == 0)
    {
        // take the packet semaphore
        xSemaphoreTake(semMainStatus, portMAX_DELAY);
        if (main_status == NULL)
        {
            gpio_set_level(HTTP_BLINK_GPIO, 0);
            httpd_resp_send_500(req);
        }
        else
        {
            ESP_LOGI(TAG, "Serving page /metrics");
            sprintf(response, "# HELP pool_air_temperature (F)\n"
                              "# TYPE pool_air_temperature gauge\n"
                              "pool_air_temperature %d\n"
                              "# HELP pool_water_temperature (F)\n"
                              "# TYPE pool_water_temperature gauge\n"
                              "pool_water_temperature %d\n"
                              "# HELP pool_pool_status\n"
                              "# TYPE pool_pool_status gauge\n"
                              "pool_pool_status %d\n"
                              "# HELP pool_spa_status\n"
                              "# TYPE pool_spa_status gauge\n"
                              "pool_spa_status %d\n"
                              "# HELP pool_cleaner_status\n"
                              "# TYPE pool_cleaner_status gauge\n"
                              "pool_cleaner_status %d\n"
                              "# HELP pool_air_blower_status\n"
                              "# TYPE pool_air_blower_status gauge\n"
                              "pool_air_blower_status %d\n"
                              "# HELP pool_spa_light_status\n"
                              "# TYPE pool_spa_light_status gauge\n"
                              "pool_spa_light_status %d\n"
                              "# HELP pool_light_status\n"
                              "# TYPE pool_light_status gauge\n"
                              "pool_light_status %d\n",
                    main_status->air_temp,
                    main_status->pool_temp,
                    PUMP1_STATE(main_status->equip1),
                    SPA_STATE(main_status->equip1),
                    CLEANER_STATE(main_status->equip1),
                    AIR_BLOWER_STATE(main_status->equip1),
                    SPA_LIGHT_STATE(main_status->equip1),
                    POOL_LIGHT_STATE(main_status->equip1));

            httpd_resp_set_status(req, "200 OK");
            httpd_resp_set_type(req, "text/plain");
            gpio_set_level(HTTP_BLINK_GPIO, 0);
            httpd_resp_send(req, response, strlen(response));
        }
        xSemaphoreGive(semMainStatus);
    }
    else
    {
        gpio_set_level(HTTP_BLINK_GPIO, 0);
        /* send a 404 otherwise */
        httpd_resp_send_404(req);
    }

    return ESP_OK;
}

static void sendCommand(DEVICE from, DEVICE to, COMMAND command, uint8_t feature, uint8_t state)
{
    int res;
    gpio_set_level(COMMAND_BLINK_GPIO, 1);
    ESP_LOGI(TAG, "Sending command from=%02x to=%02x command=%02x feature=%02x state=%02x", from, to, command, feature, state);
    uint8_t cmd_packet[12] = {0x00, 0xFF, 0xA5, 0x1F, to, from, command, 2, feature, state};
    uint16_t checksum = 0;
    for (int i = 2; i < 12; i++)
        checksum += cmd_packet[i];
    cmd_packet[10] = checksum / 256;
    cmd_packet[11] = checksum % 256;
    for (int i = 0; i < 12; i++)
    {
        printf("%02x ", cmd_packet[i]);
    }
    res = uart_write_bytes(uart_num, (char *)&cmd_packet, 12);
    vTaskDelay(50);
    ESP_LOGI(TAG, "%d", res);
    gpio_set_level(COMMAND_BLINK_GPIO, 0);
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
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "Start RS485 application and configure UART.");

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
    uint16_t checksum;
    int start;

    ESP_LOGI(TAG, "UART start recieve loop.\r\n");
    while (1)
    {
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            gpio_set_level(MQTT_BLINK_GPIO, 1);
            ESP_LOGI(TAG, "Received %u bytes:", len);
            for (int i = 0; i < len; i++)
            {
                if (data[i] == 0xFF && data[i + 1] == 0x00 && data[i + 2] == 0xFF && data[i + 3] == 0xA5)
                {
                    ESP_LOGI(TAG, "Found header start at offset %d", i);
                    start = i + 3;
                    i = len; // make sure we're done with the loop
                    memset(&packet, 0, sizeof(packet));
                    packet.leading_byte = data[start];
                    packet.unknown = data[start + 1];
                    packet.dest = data[start + 2];
                    packet.src = data[start + 3];
                    packet.command = data[start + 4];
                    packet.length = data[start + 5];
                    packet.data = &data[start + 6];
                    packet.checksum = (data[start + 6 + packet.length] << 8) + data[start + 7 + packet.length];
                    // calculate the checksum
                    checksum = 0;
                    for (int j = start; j < start + packet.length + 6; j++)
                    {
                        checksum = checksum + data[j];
                    }
                    if (checksum != packet.checksum)
                        ESP_LOGI(TAG, "\nCHECKSUM MISMATCH - got %d, expected %d\n", checksum, packet.checksum);
                    else
                    { // checksum is good - decode the packet
                        ESP_LOGI(TAG, "Source  : (0x%02x) %s", packet.src, getDeviceName(packet.src));
                        ESP_LOGI(TAG, "Dest    : (0x%02x) %s", packet.dest, getDeviceName(packet.dest));
                        ESP_LOGI(TAG, "Command : (0x%02x) %d\n", packet.command, packet.command);

                        if (packet.src == MAIN && packet.dest == BROADCAST && packet.command == 0x02)
                        {
                            int isequal = 0;

                            xSemaphoreTake(semMainStatus, portMAX_DELAY);
                            isequal = memcmp(main_status, packet.data, sizeof(MAIN_STATUS_PACKET));
                            memcpy(main_status, packet.data, sizeof(MAIN_STATUS_PACKET));
                            xSemaphoreGive(semMainStatus);

                            //main_status = (MAIN_STATUS_PACKET *)packet.data;
                            ESP_LOGI(TAG, "Time:        %d:%d", main_status->hour, main_status->minute);
                            ESP_LOGI(TAG, "Pump1:       %s", str_state(PUMP1_STATE(main_status->equip1)));
                            ESP_LOGI(TAG, "Cleaner:     %s", str_state(CLEANER_STATE(main_status->equip1)));
                            ESP_LOGI(TAG, "Air Blower:  %s", str_state(AIR_BLOWER_STATE(main_status->equip1)));
                            ESP_LOGI(TAG, "Pool temp:   %d", main_status->pool_temp);
                            ESP_LOGI(TAG, "Spa temp:    %d", main_status->spa_temp);
                            ESP_LOGI(TAG, "Air temp:    %d", main_status->air_temp);
                            ESP_LOGI(TAG, "Solar temp:  %d", main_status->solar_temp);
                            ESP_LOGI(TAG, "Heater:      %d", main_status->heater_active);
                            ESP_LOGI(TAG, "Heater mode: %d", main_status->heater_mode);
                            ESP_LOGI(TAG, "Unknown:     %d", main_status->unknown);
                            ESP_LOGI(TAG, "Spa:         %s", str_state(SPA_STATE(main_status->equip1)));
                            ESP_LOGI(TAG, "Pool light:  %s", str_state(POOL_LIGHT_STATE(main_status->equip1)));
                            ESP_LOGI(TAG, "Spa light:   %s\n", str_state(SPA_LIGHT_STATE(main_status->equip1)));

                            // send the info to the mqtt broker if changed (the times changes once per minute).
                            if (mqtt_connected && isequal != 0)
                            {
                                sprintf(mqtt_data, "%d", main_status->pool_temp);
                                esp_mqtt_client_publish(client, STATE(MQTT_POOL_TEMPERATURE_TOPIC), mqtt_data, 0, MQTT_QOS, MQTT_RETAIN);
                                sprintf(mqtt_data, "%d", main_status->air_temp);
                                esp_mqtt_client_publish(client, STATE(MQTT_AIR_TEMPERATURE_TOPIC), mqtt_data, 0, MQTT_QOS, MQTT_RETAIN);
                                sprintf(mqtt_data, "%d", main_status->solar_temp);
                                esp_mqtt_client_publish(client, STATE(MQTT_SOLAR_TEMPERATURE_TOPIC), mqtt_data, 0, MQTT_QOS, MQTT_RETAIN);
                                esp_mqtt_client_publish(client, STATE(MQTT_SPA_TOPIC), str_state(SPA_STATE(main_status->equip1)), 0, MQTT_QOS, MQTT_RETAIN);
                                esp_mqtt_client_publish(client, STATE(MQTT_POOL_CLEANER_TOPIC), str_state(CLEANER_STATE(main_status->equip1)), 0, MQTT_QOS, MQTT_RETAIN);
                                esp_mqtt_client_publish(client, STATE(MQTT_AIR_BLOWER_TOPIC), str_state(AIR_BLOWER_STATE(main_status->equip1)), 0, MQTT_QOS, MQTT_RETAIN);
                                esp_mqtt_client_publish(client, STATE(MQTT_SPA_TOPIC), str_state(SPA_STATE(main_status->equip1)), 0, MQTT_QOS, MQTT_RETAIN);
                                esp_mqtt_client_publish(client, STATE(MQTT_POOL_LIGHT_TOPIC), str_state(POOL_LIGHT_STATE(main_status->equip1)), 0, MQTT_QOS, MQTT_RETAIN);
                                esp_mqtt_client_publish(client, STATE(MQTT_SPA_LIGHT_TOPIC), str_state(SPA_LIGHT_STATE(main_status->equip1)), 0, MQTT_QOS, MQTT_RETAIN);
                                esp_mqtt_client_publish(client, STATE(MQTT_POOL_TOPIC), str_state(PUMP1_STATE(main_status->equip1)), 0, MQTT_QOS, MQTT_RETAIN);
                                esp_mqtt_client_publish(client, STATE(MQTT_WATER_FEATURE1_TOPIC), str_state(WATER_FEATURE1_STATE(main_status->equip1)), 0, MQTT_QOS, MQTT_RETAIN);
                                esp_mqtt_client_publish(client, STATE(MQTT_SPILLWAY_TOPIC), str_state(SPILLWAY_STATE(main_status->equip1)), 0, MQTT_QOS, MQTT_RETAIN);
                            }
                        }
                        else if ((packet.src == PUMP1 || packet.src == PUMP2 || packet.src == PUMP3 || packet.src == PUMP4) && packet.dest == MAIN)
                        {
                            // we have a pump status packet
                            // TODO - add this info to mqtt
                            pump_status = (PUMP_STATUS_PACKET *)packet.data;
                            ESP_LOGI(TAG, "Watts:       %d", pump_status->watts);
                            ESP_LOGI(TAG, "RPMs:        %d", pump_status->rpm);
                        }
                        break;
                    }
                    i = i + packet.length;
                }
            }
        }
        gpio_set_level(MQTT_BLINK_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    uint8_t feature;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // configure the state subscription topics
        esp_mqtt_client_subscribe(client, MQTT_HASSIO_STATUS_TOPIC, 0);
        esp_mqtt_client_subscribe(client, COMMAND(MQTT_POOL_TOPIC), 0);
        esp_mqtt_client_subscribe(client, COMMAND(MQTT_SPA_TOPIC), 0);
        esp_mqtt_client_subscribe(client, COMMAND(MQTT_POOL_CLEANER_TOPIC), 0);
        esp_mqtt_client_subscribe(client, COMMAND(MQTT_POOL_LIGHT_TOPIC), 0);
        esp_mqtt_client_subscribe(client, COMMAND(MQTT_SPA_LIGHT_TOPIC), 0);
        esp_mqtt_client_subscribe(client, COMMAND(MQTT_AIR_BLOWER_TOPIC), 0);
        esp_mqtt_client_subscribe(client, COMMAND(MQTT_WATER_FEATURE1_TOPIC), 0);
        esp_mqtt_client_subscribe(client, COMMAND(MQTT_SPILLWAY_TOPIC), 0);

        // configure the discovery topics
        esp_mqtt_client_publish(client, CONFIG(MQTT_POOL_TOPIC), "{\"unique_id\": \"pool_pool\", \"name\": \"Pool: Pool\", \"state_topic\": \"homeassistant/switch/pool/pool/state\", \"command_topic\": \"homeassistant/switch/pool/pool/set\", \"state_on\": \"on\", \"state_off\": \"off\", \"payload_on\": \"on\", \"payload_off\": \"off\"}", 0, MQTT_HA_DISCOVERY_QOS, MQTT_HA_DISCOVERY_RETAIN);
        esp_mqtt_client_publish(client, CONFIG(MQTT_SPA_TOPIC), "{\"unique_id\": \"pool_spa\", \"name\": \"Pool: Spa\", \"state_topic\": \"homeassistant/switch/pool/spa/state\", \"command_topic\": \"homeassistant/switch/pool/spa/set\", \"state_on\": \"on\", \"state_off\": \"off\", \"payload_on\": \"on\", \"payload_off\": \"off\"}", 0, MQTT_HA_DISCOVERY_QOS, MQTT_HA_DISCOVERY_RETAIN);
        esp_mqtt_client_publish(client, CONFIG(MQTT_POOL_LIGHT_TOPIC), "{\"unique_id\": \"pool_pool_light\", \"name\": \"Pool: Pool Light\", \"state_topic\": \"homeassistant/switch/pool/pool_light/state\", \"command_topic\": \"homeassistant/switch/pool/pool_light/set\", \"state_on\": \"on\", \"state_off\": \"off\", \"payload_on\": \"on\", \"payload_off\": \"off\"}", 0, MQTT_HA_DISCOVERY_QOS, MQTT_HA_DISCOVERY_RETAIN);
        esp_mqtt_client_publish(client, CONFIG(MQTT_SPA_LIGHT_TOPIC), "{\"unique_id\": \"pool_spa_light\", \"name\": \"Pool: Spa Light\", \"state_topic\": \"homeassistant/switch/pool/spa_light/state\", \"command_topic\": \"homeassistant/switch/pool/spa_light/set\", \"state_on\": \"on\", \"state_off\": \"off\", \"payload_on\": \"on\", \"payload_off\": \"off\"}", 0, MQTT_HA_DISCOVERY_QOS, MQTT_HA_DISCOVERY_RETAIN);
        esp_mqtt_client_publish(client, CONFIG(MQTT_AIR_BLOWER_TOPIC), "{\"unique_id\": \"pool_spa_air_blower\", \"name\": \"Pool: Air Blower\", \"state_topic\": \"homeassistant/switch/pool/air_blower/state\", \"command_topic\": \"homeassistant/switch/pool/air_blower/set\", \"state_on\": \"on\", \"state_off\": \"off\", \"payload_on\": \"on\", \"payload_off\": \"off\"}", 0, MQTT_HA_DISCOVERY_QOS, MQTT_HA_DISCOVERY_RETAIN);
        esp_mqtt_client_publish(client, CONFIG(MQTT_POOL_CLEANER_TOPIC), "{\"unique_id\": \"pool_cleaner\", \"name\": \"Pool: Pool Cleaner\", \"state_topic\": \"homeassistant/switch/pool/cleaner/state\", \"command_topic\": \"homeassistant/switch/pool/cleaner/set\", \"state_on\": \"on\", \"state_off\": \"off\", \"payload_on\": \"on\", \"payload_off\": \"off\"}", 0, MQTT_HA_DISCOVERY_QOS, MQTT_HA_DISCOVERY_RETAIN);
        esp_mqtt_client_publish(client, CONFIG(MQTT_WATER_FEATURE1_TOPIC), "{\"unique_id\": \"pool_water_feature1\", \"name\": \"Pool: Water Feature 1\", \"state_topic\": \"homeassistant/switch/pool/water_feature1/state\", \"command_topic\": \"homeassistant/switch/pool/water_feature1/set\", \"state_on\": \"on\", \"state_off\": \"off\", \"payload_on\": \"on\", \"payload_off\": \"off\"}", 0, MQTT_HA_DISCOVERY_QOS, MQTT_HA_DISCOVERY_RETAIN);
        esp_mqtt_client_publish(client, CONFIG(MQTT_SPILLWAY_TOPIC), "{\"unique_id\": \"pool_spillway\", \"name\": \"Pool: Spillway\", \"state_topic\": \"homeassistant/switch/pool/spillway/state\", \"command_topic\": \"homeassistant/switch/pool/spillway/set\", \"state_on\": \"on\", \"state_off\": \"off\", \"payload_on\": \"on\", \"payload_off\": \"off\"}", 0, MQTT_HA_DISCOVERY_QOS, MQTT_HA_DISCOVERY_RETAIN);
        esp_mqtt_client_publish(client, CONFIG(MQTT_POOL_TEMPERATURE_TOPIC), "{\"unique_id\": \"pool_pool_temperature\", \"name\": \"Pool: Pool Temperature\", \"device_class\": \"temperature\", \"state_topic\": \"homeassistant/sensor/pool/pool_temperature/state\", \"unit_of_measurement\": \"°F\"}", 0, MQTT_HA_DISCOVERY_QOS, MQTT_HA_DISCOVERY_RETAIN);
        esp_mqtt_client_publish(client, CONFIG(MQTT_AIR_TEMPERATURE_TOPIC), "{\"unique_id\": \"pool_air_temperature\", \"name\": \"Pool: Air Temperature\", \"device_class\": \"temperature\", \"state_topic\": \"homeassistant/sensor/pool/air_temperature/state\", \"unit_of_measurement\": \"°F\"}", 0, MQTT_HA_DISCOVERY_QOS, MQTT_HA_DISCOVERY_RETAIN);
        esp_mqtt_client_publish(client, CONFIG(MQTT_SOLAR_TEMPERATURE_TOPIC),  "{\"unique_id\": \"pool_solar_temperature\", \"name\": \"Pool: Solar Temperature\", \"device_class\": \"temperature\", \"state_topic\": \"homeassistant/sensor/pool/solar_temperature/state\", \"unit_of_measurement\": \"°F\"}", 0, MQTT_HA_DISCOVERY_QOS, MQTT_HA_DISCOVERY_RETAIN);
        mqtt_connected = true;
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = false;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGD(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGD(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        if (!mqtt_connected)
            break;
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        ESP_LOGI(TAG, "TOPIC='%.*s'\r\n", event->topic_len, event->topic);
        ESP_LOGI(TAG, "DATA='%.*s'\r\n", event->data_len, event->data);
        feature = UNKNOWN;
        if (strncmp(event->topic, COMMAND(MQTT_POOL_TOPIC), event->topic_len) == 0)
            feature = POOL;
        else if (strncmp(event->topic, COMMAND(MQTT_SPA_TOPIC), event->topic_len) == 0)
            feature = SPA;
        else if (strncmp(event->topic, COMMAND(MQTT_POOL_LIGHT_TOPIC), event->topic_len) == 0)
            feature = POOL_LIGHT;
        else if (strncmp(event->topic, COMMAND(MQTT_SPA_LIGHT_TOPIC), event->topic_len) == 0)
            feature = SPA_LIGHT;
        else if (strncmp(event->topic, COMMAND(MQTT_POOL_CLEANER_TOPIC), event->topic_len) == 0)
            feature = CLEANER;
        else if (strncmp(event->topic, COMMAND(MQTT_AIR_BLOWER_TOPIC), event->topic_len) == 0)
            feature = AIR_BLOWER;
        else if (strncmp(event->topic, COMMAND(MQTT_WATER_FEATURE1_TOPIC), event->topic_len) == 0)
            feature = WATER_FEATURE;
        else if (strncmp(event->topic, COMMAND(MQTT_SPILLWAY_TOPIC), event->topic_len) == 0)
            feature = SPILLWAY;
        else
        {
            ESP_LOGE(TAG, "Unknown feature topic %s", event->topic);
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
        vTaskDelay(xDelay);
    }
}

void app_main()
{
    main_status = malloc(sizeof(MAIN_STATUS_PACKET));
    semMainStatus = xSemaphoreCreateMutex();
    gpio_reset_pin(MQTT_BLINK_GPIO);
    gpio_set_direction(MQTT_BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(COMMAND_BLINK_GPIO);
    gpio_set_direction(COMMAND_BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(HTTP_BLINK_GPIO);
    gpio_set_direction(HTTP_BLINK_GPIO, GPIO_MODE_OUTPUT);

    /* start the wifi manager */
    wifi_manager_start();

    // set /metrics handler for prometheus stats
    http_app_set_handler_hook(HTTP_GET, &http_get_handler);

    //create the rs485 reader task
    xTaskCreate(readRS485, "readRS485", READ_RS485_STACK_SIZE, NULL, READ_RS485_PRIO, NULL);

    // create the mqtt reader/writer task
    xTaskCreate(mqttLoop, "mqttLoop", MQTT_LOOP_STACK_SIZE, NULL, MQTT_LOOP_PRIO, NULL);
}
