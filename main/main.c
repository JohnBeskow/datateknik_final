#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include <nvs_flash.h> 
#include "esp_netif.h" 
#include "esp_log.h"
#include "esp_wifi.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_mac.h"
#include "lwip/sockets.h" 
#include "lwip/err.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


static const char *TAG = "esp32_John";
static const char *TAGTCPCLI = "TCP_client";


#define MAX_SSID_LENGTH 40
#define MAX_PASSWORD_LENGTH 32
#define MAX_PORT_LENGTH 10
#define MAX_IP_LENGTH 20

//default values
char SSID[MAX_SSID_LENGTH] = "DEFAULT";
char PASSWORD[MAX_PASSWORD_LENGTH] = "DEFAULT";
char PORT[MAX_PORT_LENGTH] = "DEFAULt";
char IP[MAX_IP_LENGTH] = "DEFAULT";


//funktion to test if strncpy doesnt work
//////////////////////////////////////////////////////////////////
void strcpy_safe(char *output, const char *pSrc, size_t output_size)
{
    // Ensure only allowed characters
    size_t copyLength = strlen(pSrc) < (output_size - 1) ? strlen(pSrc) : (output_size - 1);

    // copy string
    memcpy(output, pSrc, copyLength);

    // make sure \0
    output[copyLength] = '\0';
}
/////////////////////////////////////////////////////////////////////

// fan GPIO
#define FAN_GPIO GPIO_NUM_23

// fan on/off
void fan_on()
{
    gpio_set_level(FAN_GPIO, 1);
}

void fan_off()
{
    gpio_set_level(FAN_GPIO, 0);
}

// Configure the Potentiometer
#define TMP_SENSOR_PIN ADC_CHANNEL_3  // Corresponds to GPIO3 (ADC1_CHANNEL_3)

static adc_oneshot_unit_handle_t adc1_handle;

void configure_adc() {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,  // Set 12-bit resolution
    };
    adc_oneshot_config_channel(adc1_handle, TMP_SENSOR_PIN, &config);  // Configure the channel
}


// TCP client task
void tcpclient(void *pvParameters)
{
    char rxbuffer[20];  // Buffer for receiving data
    char txbuffer[20];  // Buffer for sending data

    int tcpport = atoi(PORT);

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(IP); // Server IP
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(tcpport);       // Server port

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0)  // Use < 0 instead of != 0 to check for error
    {
        ESP_LOGE(TAGTCPCLI, "Error when creating socket: errno %d", errno);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAGTCPCLI, "Socket created");

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAGTCPCLI, "Error when connecting to socket: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAGTCPCLI, "Successfully connected");
    ESP_LOGI(TAGTCPCLI, "Connected to server: %s:%d", IP, tcpport);

    while (1)
    {
        int adc_raw;
        adc_oneshot_read(adc1_handle, TMP_SENSOR_PIN, &adc_raw);  // Read raw ADC value
        float temperature = (adc_raw / 3312.0) * 100.0;           // Convert raw ADC value to temperature
        snprintf(txbuffer, sizeof(txbuffer), "%.2f\n", temperature);  // Format temperature as string
        ESP_LOGI(TAGTCPCLI, "Sending temperature: %.2f°C", temperature);

        // Send temperature data to the server
        err = send(sock, txbuffer, strlen(txbuffer), 0);
        if (err < 0)
        {
            ESP_LOGE(TAGTCPCLI, "Error occurred during sending: errno %d", errno);
            break;
        }

        // Wait for server response ('1' or '0')
        int len = recv(sock, rxbuffer, sizeof(rxbuffer) - 1, 0);  // Use recv for TCP
        if (len < 0)
        {
            ESP_LOGE(TAGTCPCLI, "Error occurred during receiving: errno %d", errno);
            break;
        }

        rxbuffer[len] = '\0';  // Null-terminate received data
        ESP_LOGI(TAGTCPCLI, "Server response: %s", rxbuffer);

        // Check the response
        if (rxbuffer[0] == '1')
        {
            int fan = check_fan(); 
            if (fan == 1)
            {
                ESP_LOGI(TAGTCPCLI, "Temperature > 50. Fan is already ON.");
            }
            else
            {
                ESP_LOGI(TAGTCPCLI, "Temperature > 50. Turning fan ON.");
                fan_on();
            }
        }
        else if (rxbuffer[0] == '0')
        {
            ESP_LOGI(TAGTCPCLI, "Temperature <= 50. Turning fan OFF.");
            fan_off();
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
    }

    close(sock);
    vTaskDelete(NULL);
}




static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();  // Start Wi-Fi connection
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();  // Reconnect on disconnect
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

        // Start the TCP client after getting IP
        ESP_LOGI(TAG, "Starting TCP client...");
        xTaskCreate(tcpclient, "tcp_client", 10096, NULL, 5, NULL);
    }
}


// Initialize Wi-Fi
void wifi_init_sta(void) {
    esp_netif_init();
    wifi_event_group = xEventGroupCreate();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = {0},
            .password = {0},
        },
    };
    strncpy((char*)wifi_config.sta.ssid, SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, PASSWORD, sizeof(wifi_config.sta.password));

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Wi-Fi reinitialized with new credentials.");
}


char *TAGBLE = "BLE-Server";
uint8_t ble_addr_type;
void ble_app_advertise(void);

// Function to parse the incoming data
static int parse_received_data(const char *data) {
    char *token;
    char buffer[256]; //buffer for parsed data
    strncpy(buffer, data, sizeof(buffer) - 1);  //save data

    // Tokenize and assign the SSID
    token = strtok(buffer, ",");
    if (token != NULL) {
        strncpy(SSID, token, MAX_SSID_LENGTH - 1);
        SSID[MAX_SSID_LENGTH - 1] = '\0';
    } else {
        ESP_LOGE(TAG, "SSID is missing in the data");
        return -1;
    }

    // Tokenize and assign the Password
    token = strtok(NULL, ",");
    if (token != NULL) {
        strncpy(PASSWORD, token, MAX_PASSWORD_LENGTH - 1);
        PASSWORD[MAX_PASSWORD_LENGTH - 1] = '\0';
    } else {
        ESP_LOGE(TAG, "Password is missing in the data");
        return -1;
    }

    // Tokenize and assign the Port
    token = strtok(NULL, ",");
    if (token != NULL) {
        strncpy(PORT, token, MAX_PORT_LENGTH - 1);
        PORT[MAX_PORT_LENGTH - 1] = '\0';
    } else {
        ESP_LOGE(TAG, "Port is missing in the data");
        return -1;
    }

    // Tokenize and assign the IP Address
    token = strtok(NULL, ",");
    if (token != NULL) {
        strncpy(IP, token, MAX_IP_LENGTH - 1);
        IP[MAX_IP_LENGTH - 1] = '\0';
    } else {
        ESP_LOGE(TAG, "IP is missing in the data");
        return -1;
    }

    ESP_LOGI(TAG, "Parsed SSID: %s", SSID);
    ESP_LOGI(TAG, "Parsed Password: %s", PASSWORD);
    ESP_LOGI(TAG, "Parsed Port: %s", PORT);
    ESP_LOGI(TAG, "Parsed IP: %s", IP);

    return 0;
}

// handler for recieving all data
static int ble_write_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    char received_data[256];

    // Copy the received data to a local buffer 
    if (ctxt->om->om_len >= sizeof(received_data)) {
        ESP_LOGE(TAG, "Received data is too long");
        return -1;
    }
    memcpy(received_data, ctxt->om->om_data, ctxt->om->om_len);
    received_data[ctxt->om->om_len] = '\0'; //make sure \0

    ESP_LOGI(TAG, "Raw data received: %s", received_data);

    // Parse the received data
    if (parse_received_data(received_data) != 0) {
        ESP_LOGE(TAG, "Failed to parse received data");
        return -1;
    }

    // id good, start wifi and tcp
    ESP_LOGI(TAG, "Starting Wi-Fi with received credentials");
    wifi_init_sta();

    xTaskCreate(tcpclient, "tcp_client", 10096, NULL, 5, NULL);

    return 0;
}

// GATT Services Definition 
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(0xF00D),
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = ble_write_handler, 
            },
            {0}
        },
    },
    {0}
};

//event, if connected, disconnected or adv already done
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();    //if failed, advertise
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
        ble_app_advertise();    //if failed, advertise 
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT ADV COMPLETE");
        ble_app_advertise();    //if completed, advertise
        break;
    default:
        break;
    }
    return 0;
}

// advertise config
void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    const char *device_name;

    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

void host_task(void *param)
{
    nimble_port_run();
}


void Ble_task(void *pvParameters)
{
    nimble_port_init();
    ble_svc_gap_device_name_set("BleServer");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));  
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // initiate the adc for the potentiometer
    configure_adc();

    xTaskCreate(Ble_task, "Ble_task", 8192, NULL, 5, NULL);
}
