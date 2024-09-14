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
// #include "esp_gatt_defs.h"
// #include "esp_gatts_api.h"
// #include "esp_gatt_common_api.h"
// #include "esp_bt_defs.h"


static const char *TAG = "esp32_John";
static const char *TAGTCPSERV = "TCP_server";
static const char *TAGTCPCLI = "TCP_client";


#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64
#define MAX_PORT_LENGTH 5
#define MAX_IP_LENGTH 15

//default values
char ssid[MAX_SSID_LENGTH] = "defaultSSID";
char password[MAX_PASSWORD_LENGTH] = "defaultPassword";
char PORT[MAX_PORT_LENGTH] = "80";
char ip[MAX_IP_LENGTH] = "0000.0000.0000.0000";


static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
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
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
    
    ESP_LOGI(TAG, "Wi-Fi reinitialized with new credentials.");
}


// BLE deklarationer
char *TAGBLE = "BLE-Server";
uint8_t ble_addr_type;
void ble_app_advertise(void);

static int ssid_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) 
{
    if (ctxt->om->om_len > sizeof(ssid)) {
        ESP_LOGE(TAG, "SSID too long: %d", ctxt->om->om_len);
        return -1;
    }else if (ctxt->om->om_len == 0) {
        ESP_LOGE(TAG, "SSID too short: %d", ctxt->om->om_len);
        return -1;
    }
    
    ESP_LOGI(TAG, "SSID received: %s", (char*)ctxt->om->om_data);
    strncpy(ssid, (char*)ctxt->om->om_data, sizeof(ssid));

    return 0;
}
static int password_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) 
{
    if (ctxt->om->om_len > sizeof(password)) {
        ESP_LOGE(TAG, "Password too long: %d", ctxt->om->om_len);
        return -1;
    }else if (ctxt->om->om_len == 0) {
        ESP_LOGE(TAG, "Password too short: %d", ctxt->om->om_len);
        return -1;
    }
    printf("Password received: %s\n", (char*)ctxt->om->om_data);
    strncpy(password, (char*)ctxt->om->om_data, sizeof(password));
    wifi_init_sta();
    return 0;
}

static int port_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) 
{
    if (ctxt->om->om_len > sizeof(PORT)) {
        ESP_LOGE(TAG, "Port too long: %d", ctxt->om->om_len);
        return -1;
    }else if (ctxt->om->om_len == 0) {
        ESP_LOGE(TAG, "Port too short: %d", ctxt->om->om_len);
        return -1;
    }
    
    printf("Port received: %s\n", (char*)ctxt->om->om_data);
    strncpy(PORT, (char*)ctxt->om->om_data, sizeof(PORT));

    return 0;
}
static int ip_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) 
{
    if (ctxt->om->om_len > sizeof(ip)) {
        ESP_LOGE(TAG, "IP too long: %d", ctxt->om->om_len);
        return -1;
    }else if (ctxt->om->om_len == 0) {
        ESP_LOGE(TAG, "IP too short: %d", ctxt->om->om_len);
        return -1;
    }
    printf("IP received: %s\n", (char*)ctxt->om->om_data);
    strncpy(ip, (char*)ctxt->om->om_data, sizeof(ip));

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
                .access_cb = ssid_write,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xFEED),
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = password_write,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xF567),
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = port_write,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xDEAD),
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = ip_write,
            },
            {0}
        },
    },
    {0}
};

// BLE-händelser, inklusive återannonsering vid frånkoppling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
        ble_app_advertise();    //if failed, advertise 
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT ADV COMPLETE");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// Annonseringskonfiguration
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



// Fläktkontroll GPIO
#define FAN_GPIO GPIO_NUM_23


// Fläktkontrollfunktioner
void fan_on()
{
    gpio_set_level(FAN_GPIO, 1);
}

void fan_off()
{
    gpio_set_level(FAN_GPIO, 0);
}



static const char *TAG = "adc_example";
#define TMP_SENSOR_PIN ADC_CHANNEL_3  // Corresponds to GPIO3 (ADC1_CHANNEL_3)

static adc_oneshot_unit_handle_t adc1_handle;

void configure_adc() {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,  // Set 12-bit resolution
    };
    adc_oneshot_config_channel(adc1_handle, TMP_SENSOR_PIN, &config);  // Configure the channel
}

void read_temperature(void *pvParameters) {
    int adc_raw;
    while (1) {
        adc_oneshot_read(adc1_handle, TMP_SENSOR_PIN, &adc_raw);  // Read raw ADC value
        float temperature = adc_raw;
        float temp= (temperature/3312*100);
        ESP_LOGI(TAG, "Temperature: %.2f °C", temp);  // Print the raw value, convert as needed
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
    }
}

// TCP-server
void tcpserver(void *pvParameters)
{
    struct sockaddr_storage dest_addr;
    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAGTCPSERV, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0)
    {
        ESP_LOGE(TAGTCPSERV, "Error occurred during listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
    }

    while (1)
    {
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        ESP_LOGI(TAG, "Connection accepted");
        // Hantera anslutning
        close(sock);
    }

    close(listen_sock);
    vTaskDelete(NULL);
}

// TCP-klient
void tcpclient(void *pvParameters)
{
    char rxbuffer[20];

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0)
    {
        ESP_LOGE(TAGTCPCLI, "Error when creating socket: errno %d", errno);
        vTaskDelete(NULL);
    }

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        ESP_LOGE(TAGTCPCLI, "Error when connecting to socket: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
    }

    struct sockaddr_storage source_addr;
    socklen_t socklen = sizeof(source_addr);

    while (1)
    {
        int len = recvfrom(sock, rxbuffer, sizeof(rxbuffer), 0, (struct sockaddr *)&source_addr, &socklen);

        if(len < 0){
            ESP_LOGE(TAGTCPCLI, "Error occurred during receiving: errno %d", errno);
            break;
        }

        if(rxbuffer[0] == '1'){
            fan_on();
        } else if(rxbuffer[0] == '0'){
            fan_off();
        }else{
            ESP_LOGI(TAGTCPCLI, "Received: %s", rxbuffer);
            break;
        }

    }

    close(sock);
    vTaskDelete(NULL);
}



// Huvudfunktionen för ESP32-applikationen
void app_main(void)
{
    // Initiera GPIO för potentiometer
    configure_adc();

    // Initiera NVS-flash
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    // Initiera Nimble för BLE
    nimble_port_init();
    ble_svc_gap_device_name_set("BleServer");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);


    xTaskCreate(read_temperature, "read temperature", 4096, NULL, 5, NULL);
    xTaskCreate(tcpserver, "tcp_server", 4096, NULL, 5, NULL);
    xTaskCreate(tcpclient, "tcp_client", 4096, NULL, 5, NULL);
}
