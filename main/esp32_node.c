#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_sntp.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "coap_client_server.h"

#include "lib_camera.h"

#include "lib_nvs.h"

#include "esp32_node.h"

#include "data_collection.h"

#include "utils.h"

static const char* TAG = "esp32_node";

#define MY_DNS_IP_ADDR 0x08080808  // 8.8.8.8

uint16_t connect_count = 0;
uint8_t retry_connection_count = 0;

char* ssid = NULL;
uint8_t default_bssid[6];
uint8_t old_bssid[6];
uint8_t bssid[6];
char* passwd = NULL;
char* ap_ssid = NULL;
char* ap_passwd = NULL;
uint8_t status_data_transfer_type;
uint8_t is_sensing_active;
int image_size;
int image_format;
uint8_t data_send_period;
int block_size;
int block_seq_len;

esp_netif_t* wifiAP = NULL;
esp_netif_t* wifiSTA = NULL;

uint8_t ap_mac[6];
int ip;

uint8_t is_connected = 0;
uint8_t sending_initial_data = 0;
uint8_t sending_network_discovery = 0;

uint32_t sensing_destination_ip = 0;

uint8_t first_time_connected;

// Function initialization
static esp_err_t wifi_event_handler(void* ctx, system_event_t* event);

void get_global_config_from_nvs();
void wifi_init();
void time_sync_notification_cb(struct timeval* tv);
static void initialize_sntp(void);
static void obtain_time(void);

void app_main(void) {
    ESP_LOGI(TAG, "Getting initial heap information!");
    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);
    heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "Initial free heap = %d", esp_get_free_heap_size());
    first_time_connected = 1;

    esp_read_mac(ap_mac, ESP_MAC_WIFI_SOFTAP);
    ESP_LOGI(TAG, "Reading mac, this softap MAC: " MACSTR, MAC2STR(ap_mac));

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    get_global_config_from_nvs();

    if (SUPPORT_SENSING || is_sensing_active) {
        ESP_ERROR_CHECK(camera_init_default());
    }

    wifi_init();
    while (!is_connected) {
        vTaskDelay(1 * 1000 / portTICK_RATE_MS);
    }

    // obtain_time();

    coap_client_server(NULL);
}

// Function implementation
static esp_err_t wifi_event_handler(void* ctx, system_event_t* event) {
    /* ESP WIFI CONFIG */
    wifi_config_t wifi_config = {0};
    wifi_config_t ap_config = {
        .ap = {
            .channel = 0,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .ssid_hidden = 0,
            .max_connection = CONFIG_LWIP_DHCPS_MAX_STATION_NUM,
            .beacon_interval = 100,
        }};

    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            ESP_LOGI(TAG, "Wifi connecting!");
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            retry_connection_count = 0;
            is_connected = 1;

            esp_netif_ip_info_t got_ip;
            got_ip = event->event_info.got_ip.ip_info;
            ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&got_ip.ip));
            ip = got_ip.ip.addr;

            // Stop dhcps and modify access point IP address
            // ESP_LOGI(TAG, "modifying wifi mode and access point ip address!");
            // strlcpy((char*)ap_config.ap.ssid, ap_ssid, sizeof(ap_config.ap.ssid));
            // if (strlen(ap_passwd) < 8) {
            //     ap_config.ap.authmode = WIFI_AUTH_OPEN;
            // } else {
            //     strlcpy((char*)ap_config.ap.password, ap_passwd, sizeof(ap_config.ap.password));
            // }

            // if (strlen(ssid) > 0) {
            //     strlcpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
            //     strlcpy((char*)wifi_config.sta.password, passwd, sizeof(wifi_config.sta.password));
            //     if (strlen((char*)bssid) > 0) {
            //         wifi_config.sta.bssid_set = true;
            //         memcpy(wifi_config.sta.bssid, bssid, sizeof(wifi_config.sta.bssid));
            //     } else {
            //         wifi_config.sta.bssid_set = false;
            //     }
            //     ESP_LOGI(TAG, "wifi mode modified, wifi mode: APSTA");
            //     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
            //     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
            //     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));
            // } else {
            //     ESP_LOGI(TAG, "wifi mode modified, wifi mode: AP");
            //     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
            //     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));
            // }

            // // DHCPS CONFIG
            // ESP_LOGI(TAG, "Setting access point address and dhcps!");
            // esp_netif_ip_info_t ap_ip_info;
            // ap_ip_info.ip.addr = got_ip.ip.addr;
            // ap_ip_info.gw.addr = got_ip.ip.addr;
            // if ((htonl(got_ip.ip.addr) & 0xFFFFFF) < 255) {
            //     // Node is root node
            //     ap_ip_info.netmask.addr = got_ip.netmask.addr;
            // } else {
            //     ap_ip_info.netmask.addr = htonl((int)htonl(got_ip.netmask.addr) >> 2);
            // }

            // // uint32_t ip_temp;
            // // dhcps_lease_t dhcps_lease;
            // // ip_temp = htonl(ap_ip_info.ip.addr);
            // // ip_temp = ip_temp + (1 << (32 - subnet_from_netmask(htonl(ap_ip_info.netmask.addr)) - 2));
            // // dhcps_lease.start_ip.addr = htonl(ip_temp);
            // // ip_temp = htonl(ap_ip_info.ip.addr);
            // // ip_temp = (ip_temp | ~htonl(ap_ip_info.netmask.addr)) - 1;
            // // dhcps_lease.end_ip.addr = htonl(ip_temp);
            // // dhcps_lease.enable = true;
            // // int lease_time = 10;

            // esp_netif_dhcps_stop(wifiAP);  // stop before setting ip WifiAP
            // esp_netif_set_ip_info(wifiAP, &ap_ip_info);
            // //ESP_ERROR_CHECK(esp_netif_dhcps_option(wifiAP, ESP_NETIF_OP_SET, ESP_NETIF_REQUESTED_IP_ADDRESS, &dhcps_lease, sizeof(dhcps_lease)));
            // //ESP_ERROR_CHECK(esp_netif_dhcps_option(wifiAP, ESP_NETIF_OP_SET, ESP_NETIF_IP_ADDRESS_LEASE_TIME, &lease_time, sizeof(lease_time)));
            // esp_netif_dhcps_start(wifiAP);
            // //ESP_LOGI(TAG, "access point address and dhcps setted! ip:" IPSTR " gw:" IPSTR " netmask:" IPSTR " dhcps-start-ip:" IPSTR " dhcps-end-ip:" IPSTR " lease-time:%ds", IP2STR(&ap_ip_info.ip), IP2STR(&ap_ip_info.gw), IP2STR(&ap_ip_info.netmask), IP2STR(&dhcps_lease.start_ip), IP2STR(&dhcps_lease.end_ip), lease_time);
            // if (!first_time_connected) {
            //     coap_send_initial_data();
            // }
            // first_time_connected = 0;
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            if (!first_time_connected) {
                ESP_LOGI(TAG, "disconnected - restarting");
                esp_restart();
            } else {
                is_connected = 0;
                retry_connection_count++;
                uint8_t bssid_set[6];
                if (retry_connection_count < 20) {
                    ESP_LOGI(TAG, "disconnected - retry to connect to the current AP");
                    if (strlen((char*)bssid) > 0) {
                        memcpy(bssid_set, bssid, sizeof(bssid_set));
                    }
                } else if (retry_connection_count < 40) {
                    ESP_LOGI(TAG, "disconnected - retry to connect to the previous AP");
                    if (strlen((char*)old_bssid) > 0) {
                        memcpy(bssid_set, old_bssid, sizeof(bssid_set));
                    }
                } else {
                    ESP_LOGI(TAG, "disconnected - retry to connect to the default AP");
                    if (strlen((char*)default_bssid) > 0) {
                        memcpy(bssid_set, default_bssid, sizeof(bssid_set));
                    }
                }
                /* ESP WIFI CONFIG */
                strlcpy((char*)ap_config.sta.ssid, ap_ssid, sizeof(ap_config.sta.ssid));
                if (strlen(ap_passwd) < 8) {
                    ap_config.ap.authmode = WIFI_AUTH_OPEN;
                } else {
                    strlcpy((char*)ap_config.sta.password, ap_passwd, sizeof(ap_config.sta.password));
                }

                if (strlen(ssid) > 0) {
                    strlcpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
                    strlcpy((char*)wifi_config.sta.password, passwd, sizeof(wifi_config.sta.password));
                    if (strlen((char*)bssid_set) > 0) {
                        wifi_config.sta.bssid_set = true;
                        memcpy(wifi_config.sta.bssid, bssid_set, sizeof(wifi_config.sta.bssid));
                    }
                    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
                    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
                } else {
                    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
                    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));
                }
                connect_count = 0;

                esp_wifi_connect();
            }
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            connect_count++;
            ESP_LOGI(TAG, "%d. station connected", connect_count);
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            connect_count--;
            ESP_LOGI(TAG, "station disconnected - %d remain", connect_count);
            break;
        default:
            break;
    }
    return ESP_OK;
}

void get_global_config_from_nvs() {
    esp_err_t err;
    size_t size;

    err = get_config_param_str("ssid", &ssid);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ssid = ESP_DEFAULT_SSID;
        ESP_ERROR_CHECK(set_config_param_str("ssid", ssid));
    }

    size = sizeof(default_bssid);
    err = get_config_param_blob("default_bssid", default_bssid, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        uint8_t bssid_set[6] = ESP_DEFAULT_ROUTER_BSSID;
        memcpy(default_bssid, bssid_set, sizeof(default_bssid));
        set_config_param_blob("default_bssid", default_bssid, sizeof(default_bssid));
    }

    size = sizeof(old_bssid);
    err = get_config_param_blob("old_bssid", old_bssid, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        uint8_t bssid_set[6] = ESP_DEFAULT_BSSID;
        memcpy(old_bssid, bssid_set, sizeof(old_bssid));
        set_config_param_blob("old_bssid", old_bssid, sizeof(old_bssid));
    }

    err = get_config_param_str("passwd", &passwd);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        passwd = ESP_DEFAULT_PASSWD;
        ESP_ERROR_CHECK(set_config_param_str("passwd", passwd));
    }

    err = get_config_param_str("ap_ssid", &ap_ssid);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ap_ssid = ESP_DEFAULT_AP_SSID;
        ESP_ERROR_CHECK(set_config_param_str("ap_ssid", ap_ssid));
    }

    err = get_config_param_str("ap_passwd", &ap_passwd);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ap_passwd = ESP_DEFAULT_AP_PASSWD;
        ESP_ERROR_CHECK(set_config_param_str("ap_passwd", ap_passwd));
    }

#ifdef DATA_COLLECTION_MODE
    status_data_transfer_type = DATA_COLLECTION_DEFAULT_STATUS_DATA_METHOD;
    ESP_ERROR_CHECK(set_config_param_unsigned_char("st_ctl_dtype", status_data_transfer_type));
    is_sensing_active = DATA_COLLECTION_DEFAULT_IS_SENSING;
    ESP_ERROR_CHECK(set_config_param_unsigned_char("sensing", is_sensing_active));
    if (strcmp(DATA_COLLECTION_DEFAULT_PARENT_NODE, FLEXIBLE_ADDRESS) != 0) {
        str2mac(DATA_COLLECTION_DEFAULT_PARENT_NODE, bssid);
        ESP_ERROR_CHECK(set_config_param_blob("bssid", bssid, sizeof(bssid)));
    } else {
        str2mac(FLEXIBLE_ADDRESS, bssid);
        ESP_ERROR_CHECK(set_config_param_blob("bssid", bssid, sizeof(bssid)));
    }
    if (DATA_COLLECTION_DEFAULT_IMAGE_SIZE != DYNAMIC_PARAMETER) {
        image_size = DATA_COLLECTION_DEFAULT_IMAGE_SIZE;
        ESP_ERROR_CHECK(set_config_param_int("img_sz", image_size));
    }
    if (DATA_COLLECTION_DEFAULT_IMAGE_FORMAT != DYNAMIC_PARAMETER) {
        image_format = DATA_COLLECTION_DEFAULT_IMAGE_FORMAT;
        ESP_ERROR_CHECK(set_config_param_int("img_fmt", image_format));
    }
    if (DATA_COLLECTION_DEFAULT_DATA_SEND_PERIOD != DYNAMIC_PARAMETER) {
        data_send_period = DATA_COLLECTION_DEFAULT_DATA_SEND_PERIOD;
        ESP_ERROR_CHECK(set_config_param_unsigned_char("send_period", data_send_period));
    }
    if (DATA_COLLECTION_DEFAULT_BLOCK_SIZE != DYNAMIC_PARAMETER) {
        block_size = DATA_COLLECTION_DEFAULT_BLOCK_SIZE;
        ESP_ERROR_CHECK(set_config_param_int("block_size", block_size));
    }
    if (DATA_COLLECTION_DEFAULT_BLOCK_SEQ_LEN != DYNAMIC_PARAMETER) {
        block_seq_len = DATA_COLLECTION_DEFAULT_BLOCK_SEQ_LEN;
        ESP_ERROR_CHECK(set_config_param_int("block_seq_len", block_seq_len));
    }

#else

    size = sizeof(bssid);
    err = get_config_param_blob("bssid", bssid, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        uint8_t bssid_set[6] = ESP_DEFAULT_BSSID;
        memcpy(bssid, bssid_set, sizeof(bssid));
        set_config_param_blob("bssid", bssid, sizeof(bssid));
    }

    err = get_config_param_unsigned_char("st_ctl_dtype", &status_data_transfer_type);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        status_data_transfer_type = ESP_DEFAULT_STATUS_DATA_TRANSFER_TYPE;
        ESP_ERROR_CHECK(set_config_param_unsigned_char("st_ctl_dtype", status_data_transfer_type));
    }

    err = get_config_param_unsigned_char("sensing", &is_sensing_active);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        is_sensing_active = ESP_DEFAULT_IS_SENSING_ACTIVE;
        ESP_ERROR_CHECK(set_config_param_unsigned_char("sensing", is_sensing_active));
    }

    err = get_config_param_int("img_sz", &image_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        image_size = ESP_DEFAULT_IMAGE_SIZE;
        ESP_ERROR_CHECK(set_config_param_int("img_sz", image_size));
    }

    err = get_config_param_int("img_fmt", &image_format);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        image_format = ESP_DEFAULT_IMAGE_FORMAT;
        ESP_ERROR_CHECK(set_config_param_int("img_fmt", image_format));
    }

    err = get_config_param_unsigned_char("send_period", &data_send_period);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        data_send_period = ESP_DEFAULT_DATA_SEND_PERIOD;
        ESP_ERROR_CHECK(set_config_param_unsigned_char("send_period", data_send_period));
    }

    err = get_config_param_int("block_size", &block_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        block_size = ESP_DEFAULT_BLOCK_SIZE;
        ESP_ERROR_CHECK(set_config_param_int("block_size", block_size))
    }

    err = get_config_param_int("block_seq_len", &block_seq_len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        block_seq_len = ESP_DEFAULT_BLOCK_SEQ_LEN;
        ESP_ERROR_CHECK(set_config_param_int("block_seq_len", block_seq_len));
    }

#endif
}

void wifi_init() {
    ip_addr_t dnsserver;

    ESP_LOGI(TAG, "initializing wifi");

    esp_netif_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifiSTA = esp_netif_create_default_wifi_sta();
    wifiAP = esp_netif_create_default_wifi_ap();

    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* ESP WIFI CONFIG */
    wifi_config_t wifi_config = {0};
    wifi_config_t ap_config = {
        .ap = {
            .channel = 0,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .ssid_hidden = 1,
            .max_connection = CONFIG_LWIP_DHCPS_MAX_STATION_NUM,
            .beacon_interval = 100,
        }};

    strlcpy((char*)ap_config.sta.ssid, ap_ssid, sizeof(ap_config.sta.ssid));
    if (strlen(ap_passwd) < 8) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    } else {
        strlcpy((char*)ap_config.sta.password, ap_passwd, sizeof(ap_config.sta.password));
    }

    if (strlen(ssid) > 0) {
        strlcpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
        strlcpy((char*)wifi_config.sta.password, passwd, sizeof(wifi_config.sta.password));
        if (strlen((char*)bssid) > 0) {
            wifi_config.sta.bssid_set = true;
            wifi_config.sta.bssid[0] = bssid[0];
            wifi_config.sta.bssid[1] = bssid[1];
            wifi_config.sta.bssid[2] = bssid[2];
            wifi_config.sta.bssid[3] = bssid[3];
            wifi_config.sta.bssid[4] = bssid[4];
            wifi_config.sta.bssid[5] = bssid[5];
        }
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));
    }

    // Enable DNS (offer) for dhcp server
    dhcps_offer_t dhcps_dns_value = OFFER_DNS;
    dhcps_set_option_info(6, &dhcps_dns_value, sizeof(dhcps_dns_value));

    // Set custom dns server address for dhcp server
    dnsserver.u_addr.ip4.addr = htonl(MY_DNS_IP_ADDR);
    dnsserver.type = IPADDR_TYPE_V4;
    dhcps_dns_setserver(&dnsserver);

    ESP_ERROR_CHECK(esp_wifi_start());
    esp_netif_dhcps_stop(wifiAP);

    if (strlen(ssid) > 0) {
        ESP_LOGI(TAG, "wifi_init_apsta finished.");
        ESP_LOGI(TAG, "connect to ap SSID: %s ", ssid);
    } else {
        ESP_LOGI(TAG, "wifi_init_ap with default finished.");
    }
}

void time_sync_notification_cb(struct timeval* tv) {
    ESP_LOGI(TAG, "Notification of a time synchronization event triggered! Time sync received");
}

static void initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "10.0.0.15");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}

static void obtain_time(void) {
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET) {
        retry++;
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d)", retry);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);
}
