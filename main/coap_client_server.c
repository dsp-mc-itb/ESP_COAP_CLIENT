#include "coap_client_server.h"

#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <sys/param.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

//#include "libcoap.h"
#include "coap3/coap.h"
//#include "coap_dtls.h"
//#include "coap.h"

#include "coap_custom.h"
#include "lib_nvs.h"

#include "esp32_node.h"
#include "data_collection.h"

#include "esp_camera.h"
#include "lib_camera.h"

const static char *TAG = "CoAP_server_client";

#define CONTROLLER_ADDRESS 251658250 /* 10.0.0.15 but in integer */

//uint32_t local_server_ip = 2265032896;  /*192.168.1.135 */
//uint32_t local_server_ip = 2499913920; /*192.168.1.149 */
//uint32_t local_server_ip = 118073536; /* 192.168.9.7*/
//uint32_t local_server_ip = 269068480; /* 192.168.9.16*/

const char *server_uri = "coap://192.168.9.16"; //alamat server coap

int64_t send_duration = 0;

static int identity_response_flag = 0;

static int image_resp_wait = 0;
static int wait_ms;

static coap_string_t payload = {0, NULL}; /* optional payload to send */

typedef unsigned char method_t;

coap_block_mod_t block = {.num = 0, .m = 0, .szx = 6, .part_len = 10};
uint16_t last_block1_tid = 0;

unsigned int last_block_num;
unsigned int total_payload_sent = 0;
unsigned int total_missing_payload = 0;

uint8_t request_tag = 0;

unsigned long count_total_block;
unsigned long count_missing_block;
unsigned long count_retransmission;
unsigned long data_collection_count = 0;

camera_fb_t *image = NULL;

coap_context_t *ctx = NULL;

uint32_t count_monitor_device = 1;
uint32_t free_heap = 0;
uint64_t count_free_heap_sec = 0;
uint64_t count_free_heap = 0;
double average_free_heap_sec = 0;
double average_free_heap = 0;
int8_t rssi = 0;

char *image_path = "image";
char *tp_path = "troug";

coap_session_t *session_status = NULL;
coap_session_t *session_device = NULL;
coap_session_t *session_image = NULL;

coap_session_t *session_throughput_prediction = NULL;

uint8_t nack_flag = 0;

#if DATA_COLLECTION_RETRANSMISSION_TIMEOUT_SEC == 0 && DATA_COLLECTION_RETRANSMISSION_TIMEOUT_FRAC == 0
uint64_t rtt_micros;
#endif

void change_dynamic_parameter(unsigned long queue_number, unsigned int dynamic_value);
static coap_response_t client_response_handler(coap_session_t *session,
                                    const coap_pdu_t *sent, const coap_pdu_t *received,
                                    const coap_mid_t id);
static void client_nack_handler(coap_session_t *session,const coap_pdu_t *sent,const coap_nack_reason_t reason, const coap_mid_t id);
void camera_sensing_client();

void wifi_disconnect();

// static void hnd_espressif_get(coap_context_t *ctx, coap_resource_t *resource, coap_session_t *session, coap_pdu_t *request, coap_binary_t *token, coap_string_t *query, coap_pdu_t *response);
// static void hnd_espressif_put(coap_context_t *ctx, coap_resource_t *resource, coap_session_t *session, coap_pdu_t *request, coap_binary_t *token, coap_string_t *query, coap_pdu_t *response);

int64_t tick_device_monitor = 0;
int64_t tick_status_data = 0;
int64_t tick_send_image = 0;

int64_t tick_get_throughput_prediction = 0;

void prepare_device_monitor_session(coap_session_t **session, int64_t *tick);
void send_device_monitor(coap_session_t *session, int64_t *tick);
void prepare_status_data_session(coap_session_t **session, int64_t *tick);
void send_status_data(coap_session_t *session, int64_t *tick);
void prepare_image_session(coap_context_t *ctx,coap_session_t **session, int64_t *tick);
void send_image(coap_session_t *session, int64_t *tick);
void send_image_continue(coap_session_t *session, int64_t *tick);

void prepare_get_throughput_prediction(coap_context_t *ctx,coap_session_t **session, int64_t *tick);
void get_throughput_prediction(coap_session_t *session, int64_t *tick);

void coap_client_server(void *p) {
    ESP_LOGI(TAG, "CoAP client running!");
    coap_context_t *ctx = NULL;
    request_tag = rand();

    coap_address_t serv_addr;
    coap_resource_t *controller_resource = NULL;

    coap_set_log_level(COAP_LOG_DEFAULT_LEVEL);

    if (!coap_debug_set_packet_loss(COAP_DEBUG_PACKET_LOSS)) {
        coap_log(LOG_NOTICE, "Cannot add debug packet loss!\n");
        ESP_ERROR_CHECK(-1);
    }

    //coap_endpoint_t *ep = NULL;

    ctx = coap_new_context(NULL);
    if (!ctx) {
        coap_log(LOG_NOTICE, "coap_new_context() failed\n");
    }
    coap_context_set_block_mode(ctx,COAP_BLOCK_USE_LIBCOAP);

    /* Prepare the CoAP server socket */
    // coap_address_init(&serv_addr);
    // serv_addr.addr.sin.sin_family = AF_INET;
    // serv_addr.addr.sin.sin_addr.s_addr = INADDR_ANY;
    // serv_addr.addr.sin.sin_port = htons(COAP_DEFAULT_PORT);

    while (!is_connected) {
        vTaskDelay(0.5 * 1000 / portTICK_RATE_MS);
    }

    // ep = coap_new_endpoint(ctx, &serv_addr, COAP_PROTO_UDP);
    // if (!ep) {
    //     coap_log(LOG_NOTICE, "udp: coap_new_endpoint() failed\n");
    // }

    // controller_resource = coap_resource_init(coap_make_str_const(""), 0);
    // if (!controller_resource) {
    //     coap_log(LOG_NOTICE, "coap_resource_init() failed\n");
    // }
    // coap_register_handler(controller_resource, COAP_REQUEST_PUT, hnd_espressif_put);
    // coap_register_handler(controller_resource, COAP_REQUEST_GET, hnd_espressif_get);
    // coap_add_resource(ctx, controller_resource);

    coap_register_response_handler(ctx, client_response_handler);
    coap_register_nack_handler(ctx, client_nack_handler);

    //prepare_status_data_session(&session_status, &tick_status_data);
    //prepare_device_monitor_session(&session_device, &tick_device_monitor);
    prepare_image_session(ctx,&session_image, &tick_send_image);
    prepare_get_throughput_prediction(ctx,&session_throughput_prediction, &tick_get_throughput_prediction);

    while (1) {
        if (is_connected) {
            //send_device_monitor(session_device, &tick_device_monitor);
            if (is_sensing_active) {
                send_image(session_image, &tick_send_image);
                get_throughput_prediction(session_throughput_prediction, &tick_get_throughput_prediction);
            }        
            wait_ms = 60000;
            while (image_resp_wait) {
                int result = coap_io_process(ctx, wait_ms > 500 ? 500 : wait_ms);
                if (result >= 0) {
                    if (result >= wait_ms) {
                        ESP_LOGE(TAG, "No response from server");
                        break;
                    } else {
                        wait_ms -= result;
                    }
                }
            } 
            esp_camera_fb_return(image);
            vTaskDelay(60 * 1000 / portTICK_RATE_MS);
            
        } else {
            coap_log(LOG_NOTICE, "Waiting for wifi connection!\n");
            vTaskDelay(1 * 1000 / portTICK_RATE_MS);
        }
    }
}



void change_dynamic_parameter(unsigned long queue_number, unsigned int dynamic_value) {
    // The order is:
    // 1. Block size
    // 2. Block sequence length
    // 3. Image size
    unsigned long scaled_queue = queue_number / dynamic_value;

    if (DATA_COLLECTION_DEFAULT_BLOCK_SIZE == DYNAMIC_PARAMETER) {
        block_size = scaled_queue % MULTIPLIER_BLOCK_SIZE;
    }
    if (DATA_COLLECTION_DEFAULT_BLOCK_SEQ_LEN == DYNAMIC_PARAMETER) {
        if (DATA_COLLECTION_DEFAULT_BLOCK_SIZE == DYNAMIC_PARAMETER) {
            block_seq_len = ((scaled_queue / MULTIPLIER_BLOCK_SIZE) % MULTIPLIER_BLOCK_PART_LEN) + 1;  // Add 1 for sequence length cuz the minimum value is 1 and maximum value is MULTIPLIER_BLOCK_PART_LEN
        } else {
            block_seq_len = scaled_queue % MULTIPLIER_BLOCK_PART_LEN;
        }
    }
    if (DATA_COLLECTION_DEFAULT_IMAGE_SIZE == DYNAMIC_PARAMETER) {
        int total_multiplier = 1;
        total_multiplier = DATA_COLLECTION_DEFAULT_BLOCK_SIZE == DYNAMIC_PARAMETER ? total_multiplier * MULTIPLIER_BLOCK_SIZE : total_multiplier;
        total_multiplier = DATA_COLLECTION_DEFAULT_BLOCK_SEQ_LEN == DYNAMIC_PARAMETER ? total_multiplier * MULTIPLIER_BLOCK_PART_LEN : total_multiplier;
        image_size = (scaled_queue / total_multiplier) % MULTIPLIER_IMAGE_SIZE;

        sensor_t *s = esp_camera_sensor_get();
        set_config_param_int("img_sz", image_size);
        s->set_framesize(s, image_size);

        printf("total_multiplier: %d\n", total_multiplier);
        printf("image_size: %d\n", image_size);
        printf("frame_size: %d\n", s->status.framesize);
    }
}

static coap_response_t client_response_handler(coap_session_t *session,
                                    const coap_pdu_t *sent, const coap_pdu_t *received,
                                    const coap_mid_t id) {
    coap_pdu_t *pdu = NULL;
    coap_opt_t *control_opt;
    uint8_t control_opt_value;
    coap_opt_t *block_opt;
    coap_opt_iterator_t opt_iter;
    unsigned char buf[4];
    size_t len;
    const unsigned char *databuf = NULL;
    coap_mid_t tid;

    coap_block_missing_t missing_block_opt;

    coap_pdu_code_t rcvd_code = coap_pdu_get_code(received);
    const unsigned char *data = NULL;
    size_t data_len;
    size_t offset;
    size_t total;
  

    if (COAP_RESPONSE_CLASS(rcvd_code) == 2)
   {
      
      if (coap_get_data_large(received, &data_len, &data, &offset, &total))
      {
         if (data_len != total)
         {
            printf("Unexpected partial data received offset %u, length %u\n", offset, data_len);
         }
         printf("Received:\n%.*s\n", (int)data_len, data);
         //resp_wait = 0;
      }
      if (rcvd_code == 0b01000100){
         printf("TRIGGER 2.04");
         image_resp_wait = 0;
      }
      return COAP_RESPONSE_OK;
   }
 return COAP_RESPONSE_OK;
}

static void client_nack_handler(coap_session_t *session,const coap_pdu_t *sent,const coap_nack_reason_t reason, const coap_mid_t id) {
    coap_log(LOG_NOTICE, "NACK trigger received, the reason is %d\n", reason);
    coap_opt_iterator_t opt_iter;

    nack_flag = 1;
    if (coap_check_option(sent, COAP_OPTION_BLOCK_MOD, &opt_iter)) {
        image_resp_wait = 0;
    }
}


void wifi_disconnect() {
    vTaskDelay(3 * 1000 / portTICK_RATE_MS);
    // ESP_ERROR_CHECK(esp_wifi_deauth_sta(0));
    // vTaskDelay(1 * 1000 / portTICK_RATE_MS);
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    // vTaskDelay(1 * 1000 / portTICK_RATE_MS);
    // ESP_ERROR_CHECK(esp_wifi_stop());
    // vTaskDelay(1 * 1000 / portTICK_RATE_MS);
    // esp_restart();
    ESP_LOGI(TAG, "RESTARTING ESP32");
    vTaskDelay(3 * 1000 / portTICK_RATE_MS);
    // esp_restart();
    vTaskDelete(NULL);
}

void prepare_device_monitor_session(coap_session_t **session, int64_t *tick) {
    coap_address_t controller_addr;

    coap_address_init(&controller_addr);
    controller_addr.addr.sin.sin_family = AF_INET;
    controller_addr.addr.sin.sin_port = htonl(COAP_DEFAULT_PORT);
    controller_addr.addr.sin.sin_addr.s_addr = CONTROLLER_ADDRESS;

    *session = coap_new_client_session(ctx, NULL, &controller_addr, COAP_PROTO_UDP);
    if (!*session) {
        coap_log(LOG_NOTICE, "coap_new_client_session() failed\n");
        coap_session_release(*session);
    }

    *tick = esp_timer_get_time();
}

void send_device_monitor(coap_session_t *session, int64_t *tick) {
    free_heap = esp_get_free_heap_size();
    count_free_heap++;
    count_free_heap_sec++;

    average_free_heap = average_free_heap + ((free_heap - average_free_heap) / count_free_heap);
    average_free_heap_sec = average_free_heap_sec + ((free_heap - average_free_heap_sec) / count_free_heap_sec);

    int64_t current_time;

    current_time = esp_timer_get_time();

    if (current_time - *tick > 1 * 1000000) {
        *tick = current_time;

        wifi_ap_record_t ap_record;

        esp_wifi_sta_get_ap_info(&ap_record);
        rssi = ap_record.rssi;

        time_t ntp_time;
        time(&ntp_time);

        coap_address_t controller_addr;
        coap_address_init(&controller_addr);
        controller_addr.addr.sin.sin_family = AF_INET;
        controller_addr.addr.sin.sin_port = htonl(COAP_DEFAULT_PORT);
        controller_addr.addr.sin.sin_addr.s_addr = CONTROLLER_ADDRESS;

        coap_pdu_t *pdu;

        pdu = coap_new_pdu(COAP_MESSAGE_NON,
                             COAP_REQUEST_CODE_PUT, session);
        if (!pdu){
            ESP_LOGE(TAG, "coap_new_pdu() failed");
        }
        uint8_t optval = 21;
        coap_add_option(pdu, COAP_OPTION_CONTROL, 1, &optval);

        uint8_t data[55];
        memcpy(data, ap_mac, 6);
        memcpy(data + 6, &ip, 4);
        memcpy(data + 6 + 4, &ntp_time, 4);
        memcpy(data + 6 + 4 + 4, &current_time, 8);
        memcpy(data + 6 + 4 + 4 + 8, &count_monitor_device, 4);
        memcpy(data + 6 + 4 + 4 + 8 + 4, &count_free_heap, 4);
        memcpy(data + 6 + 4 + 4 + 8 + 4 + 4, &count_free_heap_sec, 4);
        memcpy(data + 6 + 4 + 4 + 8 + 4 + 4 + 4, &free_heap, 4);
        memcpy(data + 6 + 4 + 4 + 8 + 4 + 4 + 4 + 4, &average_free_heap, 8);
        memcpy(data + 6 + 4 + 4 + 8 + 4 + 4 + 4 + 4 + 8, &average_free_heap_sec, 8);
        memcpy(data + 6 + 4 + 4 + 8 + 4 + 4 + 4 + 4 + 8 + 8, &rssi, 1);

        coap_add_data(pdu, 55, data);

        coap_send(session, pdu);

        coap_log(LOG_NOTICE, MACSTR ",%d,%ld,%lld,%d,%lld,%lld,%d,%lf,%lf,%d\n", MAC2STR(ap_mac), ip, ntp_time, current_time, count_monitor_device, count_free_heap, count_free_heap_sec, free_heap, average_free_heap, average_free_heap_sec, rssi);

        average_free_heap_sec = 0;
        count_free_heap_sec = 0;

        count_monitor_device++;
    }
}

void prepare_status_data_session(coap_session_t **session, int64_t *tick) {
    coap_address_t controller_addr;


    coap_address_init(&controller_addr);
    controller_addr.addr.sin.sin_family = AF_INET;
    controller_addr.addr.sin.sin_port = htonl(COAP_DEFAULT_PORT);
    controller_addr.addr.sin.sin_addr.s_addr = CONTROLLER_ADDRESS;

    *session = coap_new_client_session(ctx, NULL, &controller_addr, COAP_PROTO_UDP);
    if (!*session) {
        coap_log(LOG_NOTICE, "coap_new_client_session() failed\n");
        coap_session_release(*session);
    }

    *tick = esp_timer_get_time();
}

void send_status_data(coap_session_t *session, int64_t *tick) {
    if (esp_timer_get_time() - *tick > status_data_transfer_type * 1000000) {
        *tick = esp_timer_get_time();
        uint16_t scan_list_size = 20;
        wifi_ap_record_t ap_info[scan_list_size];
        uint16_t scan_count = 0;
        wifi_scan_config_t scan_config;
        scan_config.ssid = (uint8_t *)ap_ssid;
        scan_config.bssid = NULL;
        scan_config.channel = 0;
        scan_config.show_hidden = 1;
        scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
        scan_config.scan_time.active.min = 0;
        scan_config.scan_time.active.max = 0;

        coap_pdu_t *pdu = NULL;

        uint8_t mac_address[6];
        esp_base_mac_addr_get(mac_address);
        if (mac_address[5] != 0xFF) {
            mac_address[5]++;
        } else {
            mac_address[4]++;
        }
        uint32_t free_heap = esp_get_free_heap_size();
        wifi_ap_record_t ap_record;
        esp_wifi_sta_get_ap_info(&ap_record);
        uint8_t parent_rssi = (uint8_t)ap_record.rssi;

        if (sending_network_discovery) {
            // esp_wifi_scan_start(&scan_config, false);
            coap_log(LOG_NOTICE, "STOP SCAN!\n");
            esp_wifi_scan_stop();
            esp_wifi_scan_get_ap_records(&scan_list_size, ap_info);
            esp_wifi_scan_get_ap_num(&scan_count);
        }

        uint8_t *status_data = (uint8_t *)malloc(sizeof(mac_address) + sizeof(free_heap) + sizeof(parent_rssi) + 7 * scan_count * sizeof(uint8_t));
        uint8_t *option_value = (uint8_t *)malloc(1 * sizeof(uint8_t));
        option_value[0] = 1;

        memcpy(status_data, mac_address, sizeof(mac_address));
        memcpy(status_data + sizeof(mac_address), &free_heap, sizeof(free_heap));
        memcpy(status_data + sizeof(mac_address) + sizeof(free_heap), &parent_rssi, sizeof(parent_rssi));

        coap_log(LOG_NOTICE, "Total APs scanned = %u\n", scan_count);
        for (int i = 0; (i < 20) && (i < scan_count); i++) {
            coap_log(LOG_NOTICE, "SSID \t\t%s\n", ap_info[i].ssid);
            coap_log(LOG_NOTICE, "BSSID \t\t" MACSTR "\n", MAC2STR(ap_info[i].bssid));
            coap_log(LOG_NOTICE, "RSSI \t\t%d\n", ap_info[i].rssi);
            memcpy(status_data + sizeof(mac_address) + sizeof(free_heap) + sizeof(parent_rssi) + (i * 7), &(ap_info[i].bssid), sizeof(ap_info[i].bssid));
            memcpy(status_data + sizeof(mac_address) + sizeof(free_heap) + sizeof(parent_rssi) + (i * 7 + 6), &(ap_info[i].rssi), sizeof(ap_info[i].rssi));
        }

        
        pdu = coap_new_pdu(COAP_MESSAGE_NON,
                             COAP_REQUEST_CODE_PUT, session);
        if (!pdu){
            ESP_LOGE(TAG, "coap_new_pdu() failed");
        }

        coap_add_option(pdu, COAP_OPTION_CONTROL, 1, option_value);

        coap_add_data(pdu, 11 + 7 * scan_count, status_data);

        coap_log(LOG_NOTICE, "SENDING STATUS DATA WITH RSSI ETC TO CONTROLLER!\n");
        coap_send(session, pdu);

        free(status_data);
        free(option_value);
        if (sending_network_discovery) {
            coap_log(LOG_NOTICE, "START SCAN!\n");
            esp_wifi_scan_start(&scan_config, false);
        }
    }
}

static coap_address_t * coap_get_address(coap_uri_t *uri)
{
   static coap_address_t dst_addr;
   char *phostname = NULL;
   struct addrinfo hints;
   struct addrinfo *addrres;
   int error;
   char tmpbuf[INET6_ADDRSTRLEN];

   phostname = (char *)calloc(1, uri->host.length + 1);
   if (phostname == NULL)
   {
      ESP_LOGE(TAG, "calloc failed");
      return NULL;
   }
   memcpy(phostname, uri->host.s, uri->host.length);

   memset((char *)&hints, 0, sizeof(hints));
   hints.ai_socktype = SOCK_DGRAM;
   hints.ai_family = AF_UNSPEC;

   error = getaddrinfo(phostname, NULL, &hints, &addrres);
   if (error != 0)
   {
      ESP_LOGE(TAG, "DNS lookup failed for destination address %s. error: %d", phostname, error);
      free(phostname);
      return NULL;
   }
   if (addrres == NULL)
   {
      ESP_LOGE(TAG, "DNS lookup %s did not return any addresses", phostname);
      free(phostname);
      return NULL;
   }
   free(phostname);
   coap_address_init(&dst_addr);
   switch (addrres->ai_family)
   {
   case AF_INET:
      memcpy(&dst_addr.addr.sin, addrres->ai_addr, sizeof(dst_addr.addr.sin));
      dst_addr.addr.sin.sin_port = htons(uri->port);
      inet_ntop(AF_INET, &dst_addr.addr.sin.sin_addr, tmpbuf, sizeof(tmpbuf));
      ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", tmpbuf);
      break;
   case AF_INET6:
      memcpy(&dst_addr.addr.sin6, addrres->ai_addr, sizeof(dst_addr.addr.sin6));
      dst_addr.addr.sin6.sin6_port = htons(uri->port);
      inet_ntop(AF_INET6, &dst_addr.addr.sin6.sin6_addr, tmpbuf, sizeof(tmpbuf));
      ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", tmpbuf);
      break;
   default:
      ESP_LOGE(TAG, "DNS lookup response failed");
      return NULL;
   }
   freeaddrinfo(addrres);

   return &dst_addr;
}

void prepare_image_session(coap_context_t *ctx, coap_session_t **session, int64_t *tick) {
    coap_address_t *dst_addr;
    static coap_uri_t uri;

    if (coap_split_uri((const uint8_t *)server_uri, strlen(server_uri), &uri) == -1)
    {
      ESP_LOGE(TAG, "CoAP server uri error");
      
    }
    dst_addr = coap_get_address(&uri);
    *session = coap_new_client_session(
        ctx, NULL, dst_addr, COAP_PROTO_UDP);
    if (!*session) {
        coap_log(LOG_NOTICE, "coap_new_client_session() failed\n");
        coap_session_release(*session);
    }
    *tick = esp_timer_get_time();
}

void send_image(coap_session_t *session, int64_t *tick) {
        

        coap_pdu_t *request = NULL;
        payload.length = 0;
        payload.s = NULL;
        total_payload_sent = 0;
        total_missing_payload = 0;

        count_total_block = 0;
        count_missing_block = 0;
        count_retransmission = 0;

        request_tag++;
        request_tag = request_tag == 0 ? request_tag + 1 : request_tag;

        coap_optlist_t *optlist = NULL;

        // if (DYNAMIC_TYPE == DYNAMIC_PER_IMAGE) {
        //     printf("setting dynamic value\n");
        //     change_dynamic_parameter(data_collection_count, DYNAMIC_VALUE);
        // }

        //coap_insert_optlist(&optlist, coap_new_optlist(COAP_OPTION_URI_PATH, 5, (uint8_t *)image_path));

        // Set block and payload
        image = camera_capture();

        if (!image) {
            coap_log(LOG_NOTICE, "Take image failed!\n");
            goto clean_up;
        }

        payload.s = image->buf;
        payload.length = image->len;

        int remaining_payload =
            coap_get_remaining_payload_number(payload.length, -1, block.szx);

        // block.num = 0;
        // block.szx = block_size;
        // block.m =
        //     ((1ull << (block.szx + 4)) < payload.length);

        send_duration = esp_timer_get_time();
        coap_log(LOG_NOTICE, "Start sending image, start tick %lld\n", send_duration);

        // count_total_block++;
                                   
        request = coap_new_pdu(COAP_MESSAGE_CON,
                             COAP_REQUEST_CODE_PUT, session);
        if (!request){
            ESP_LOGE(TAG, "coap_new_pdu() failed");
        }
#if DATA_COLLECTION_RETRANSMISSION_TIMEOUT_SEC == 0 && DATA_COLLECTION_RETRANSMISSION_TIMEOUT_FRAC == 0
        rtt_micros = esp_timer_get_time();
#endif
        coap_log(LOG_NOTICE, "Start sending image 2, start tick %lld\n", send_duration);
        unsigned char buf[4];
        coap_add_option(request, COAP_OPTION_URI_PATH, 5, (uint8_t *)image_path);
        
        coap_add_option(request, COAP_OPTION_SIZE1, coap_encode_var_safe(buf, sizeof(buf), payload.length),
                        buf);
        printf("\nLENG DATA : %d\n\n",payload.length);
        coap_add_data_large_request(session,request, payload.length, payload.s, NULL, NULL);
        coap_log(LOG_NOTICE, "Start sending image 3, start tick %lld\n", send_duration);
        image_resp_wait = 1;
        coap_send(session, request);
        coap_log(LOG_NOTICE, "Start sending image 4, start tick %lld\n", send_duration);
    clean_up:
        
        
        coap_log(LOG_NOTICE, "Start sending image 5, start tick %lld\n", send_duration);
        if (optlist) {
            coap_delete_optlist(optlist);
            optlist = NULL;
        }
        
    
}

void prepare_get_throughput_prediction(coap_context_t *ctx,coap_session_t **session, int64_t *tick) {
    coap_address_t server_addr;
    coap_address_t *dst_addr;
    static coap_uri_t uri;

    if (coap_split_uri((const uint8_t *)server_uri, strlen(server_uri), &uri) == -1)
    {
      ESP_LOGE(TAG, "CoAP server uri error");    
    }
    dst_addr = coap_get_address(&uri);
    *session = coap_new_client_session(ctx, NULL, dst_addr, COAP_PROTO_UDP);
    if (!*session) {
        coap_log(LOG_NOTICE, "coap_new_client_session() failed\n");
        coap_session_release(*session);
    }
    *tick = esp_timer_get_time();
}

void get_throughput_prediction(coap_session_t *session, int64_t *tick) {
    coap_pdu_t *pdu = NULL;

    if (esp_timer_get_time() - *tick > 5000000) {
        *tick = esp_timer_get_time();
       
        pdu = coap_new_pdu(COAP_MESSAGE_CON,COAP_REQUEST_CODE_GET, session);
        if (!pdu){
            ESP_LOGE(TAG, "coap_new_pdu() failed");
        }
        coap_add_option(pdu, COAP_OPTION_URI_PATH, 5, (uint8_t *)tp_path);
        coap_send(session, pdu);

    }
}

