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
#define size_of_list_frame 12
const char *server_uri = "coap://192.168.1.113"; // alamat server coap Raspi dd-wrt
//const char *server_uri = "coap://192.168.9.4"; // alamat server coap TP LINK
static unsigned char _token_data[8];
coap_binary_t base_token = { 0, _token_data };
int64_t send_duration = 0;

int64_t coba_duration = 0;
int64_t coba_duration2 = 0;
int64_t coba_duration3 = 0;

static int identity_response_flag = 0;

static int image_resp_wait = 0;
static int wait_ms;

static coap_string_t payload = {0, NULL}; /* optional payload to send */
int list_of_frame[size_of_list_frame] = {9216,19200,25344,42240,57600,76800,118400,153600,307200,480000,786432,921600};
typedef unsigned char method_t;

coap_block_mod_t block = {.num = 0, .m = 0, .szx = 6, .part_len = 10};
uint16_t last_block1_tid = 0;

unsigned int last_block_num;

unsigned long count_total_block;
unsigned long count_missing_block;
unsigned long count_retransmission;
unsigned long data_collection_count = 0;
int wait_ms_reset = 0;
/* reading is done when this flag is set */
static int ready = 0;
static int single_block_requested = 0;
static int add_nl = 0;
static int doing_getting_block = 0;
unsigned int obs_ms = 0;                /* timeout for current subscription */
int obs_ms_reset = 0;
unsigned int obs_seconds = 30; 
int obs_started = 0;
int doing_observe = 0;
static int is_mcast = 0;
static int frameRate = 5;
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
char *delay_path = "delay";
char *tp_path = "troug";

coap_session_t *session_status = NULL;
coap_session_t *session_device = NULL;
coap_session_t *session_image = NULL;

coap_session_t *session_throughput_prediction = NULL;

uint8_t nack_flag = 0;

void change_dynamic_parameter(char* dynamic_value);
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
int64_t tick_put_delay = 0;

void prepare_device_monitor_session(coap_session_t **session, int64_t *tick);
void send_device_monitor(coap_session_t *session, int64_t *tick);
// void prepare_status_data_session(coap_session_t **session, int64_t *tick);
void send_status_data(coap_session_t *session, int64_t *tick);
void prepare_session(coap_context_t *ctx,coap_session_t **session, int64_t *tick);
void send_image(coap_session_t *session, int64_t *tick);
void send_delay(coap_session_t *session, int64_t *tick);
void send_image_continue(coap_session_t *session, int64_t *tick);

void prepare_get_throughput_prediction(coap_context_t *ctx,coap_session_t **session, int64_t *tick);
void get_throughput_prediction(coap_session_t *session, int64_t *tick);

typedef struct {
  coap_binary_t *token;
    int observe;
} track_token;

track_token *tracked_tokens = NULL;
size_t tracked_tokens_count = 0;

static void
track_new_token(size_t tokenlen, uint8_t *token)
{
  track_token *new_list =  realloc(tracked_tokens,
                      (tracked_tokens_count + 1) * sizeof(tracked_tokens[0]));
  if (!new_list) {
    coap_log(LOG_INFO, "Unable to track new token\n");
    return;
  }
  tracked_tokens = new_list;
  tracked_tokens[tracked_tokens_count].token = coap_new_binary(tokenlen);
  if (!tracked_tokens[tracked_tokens_count].token)
    return;
  memcpy(tracked_tokens[tracked_tokens_count].token->s, token, tokenlen);
  tracked_tokens[tracked_tokens_count].observe = doing_observe;
  tracked_tokens_count++;
}

static int
track_check_token(coap_bin_const_t *token)
{
  size_t i;

  for (i = 0; i < tracked_tokens_count; i++) {
    if (coap_binary_equal(token, tracked_tokens[i].token)) {
      return 1;
    }
  }
  return 0;
}

static void
track_flush_token(coap_bin_const_t *token)
{
  size_t i;

  for (i = 0; i < tracked_tokens_count; i++) {
    if (coap_binary_equal(token, tracked_tokens[i].token)) {
      if (!tracked_tokens[i].observe || !obs_started) {
        /* Only remove if not Observing */
        coap_delete_binary(tracked_tokens[i].token);
        if (tracked_tokens_count-i > 1) {
           memmove (&tracked_tokens[i],
                    &tracked_tokens[i+1],
                   (tracked_tokens_count-i-1) * sizeof (tracked_tokens[0]));
        }
        tracked_tokens_count--;
      }
      break;
    }
  }
}

static int
resolve_address(const coap_str_const_t *server, struct sockaddr *dst) {

  struct addrinfo *res, *ainfo;
  struct addrinfo hints;
  static char addrstr[256];
  int error, len=-1;

  memset(addrstr, 0, sizeof(addrstr));
  if (server->length)
    memcpy(addrstr, server->s, server->length);
  else
    memcpy(addrstr, "localhost", 9);

  memset ((char *)&hints, 0, sizeof(hints));
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_family = AF_UNSPEC;

  error = getaddrinfo(addrstr, NULL, &hints, &res);

  if (error != 0) {
    //fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(error));
    return error;
  }

  for (ainfo = res; ainfo != NULL; ainfo = ainfo->ai_next) {
    switch (ainfo->ai_family) {
    case AF_INET6:
    case AF_INET:
      len = (int)ainfo->ai_addrlen;
      memcpy(dst, ainfo->ai_addr, len);
      goto finish;
    default:
      ;
    }
  }

 finish:
  freeaddrinfo(res);
  return len;
}

static coap_session_t*
open_session(
  coap_context_t *ctx,
  coap_proto_t proto,
  coap_address_t *bind_addr,
  coap_address_t *dst,
  const uint8_t *identity,
  size_t identity_len,
  const uint8_t *key,
  size_t key_len
) {
  coap_session_t *session;

    /* Non-encrypted session */
    session = coap_new_client_session(ctx, bind_addr, dst, proto);
  
  return session;
}

static coap_session_t *
get_session(
  coap_context_t *ctx,
  const char *local_addr,
  const char *local_port,
  coap_proto_t proto,
  coap_address_t *dst,
  const uint8_t *identity,
  size_t identity_len,
  const uint8_t *key,
  size_t key_len
) {
  coap_session_t *session = NULL;

  is_mcast = coap_is_mcast(dst);
  if ( local_addr ) {
    int s;
    struct addrinfo hints;
    struct addrinfo *result = NULL, *rp;

    memset( &hints, 0, sizeof( struct addrinfo ) );
    hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
    hints.ai_socktype = COAP_PROTO_RELIABLE(proto) ? SOCK_STREAM : SOCK_DGRAM; /* Coap uses UDP */
    hints.ai_flags = AI_PASSIVE | AI_NUMERICHOST | AI_NUMERICSERV | AI_ALL;

    s = getaddrinfo( local_addr, local_port, &hints, &result );
    if ( s != 0 ) {
     
      return NULL;
    }

    /* iterate through results until success */
    for ( rp = result; rp != NULL; rp = rp->ai_next ) {
      coap_address_t bind_addr;
      if ( rp->ai_addrlen <= (socklen_t)sizeof( bind_addr.addr ) ) {
        coap_address_init( &bind_addr );
        bind_addr.size = (socklen_t)rp->ai_addrlen;
        memcpy( &bind_addr.addr, rp->ai_addr, rp->ai_addrlen );
        session = open_session(ctx, proto, &bind_addr, dst,
                               identity, identity_len, key, key_len);
        if ( session )
          break;
      }
    }
    freeaddrinfo( result );
  } else if (local_port) {
    coap_address_t bind_addr;

    coap_address_init(&bind_addr);
    bind_addr.size = dst->size;
    bind_addr.addr.sa.sa_family = dst->addr.sa.sa_family;
    /* port is in same place for IPv4 and IPv6 */
    bind_addr.addr.sin.sin_port = ntohs(atoi(local_port));
    session = open_session(ctx, proto, &bind_addr, dst,
                               identity, identity_len, key, key_len);
  } else {
    session = open_session(ctx, proto, NULL, dst,
                               identity, identity_len, key, key_len);
  }
  return session;
}

void coap_client_server(void *p) {
    ESP_LOGI(TAG, "CoAP client running!");
    coap_context_t *ctx = NULL;    
    coap_session_t *session_image = NULL;
    coap_session_t *session_tp = NULL;
    coap_session_t *session_delay = NULL;

    /*INITIATE COAP */
    coap_startup();
    coap_set_log_level(COAP_LOG_DEFAULT_LEVEL);
    if (!coap_debug_set_packet_loss(COAP_DEBUG_PACKET_LOSS)) {
        coap_log(LOG_NOTICE, "Cannot add debug packet loss!\n");
        ESP_ERROR_CHECK(-1);
    }
    /*INITIATE CONTEXT */
    ctx = coap_new_context(NULL);
    if (!ctx) {
        coap_log(LOG_NOTICE, "coap_new_context() failed\n");
    }
    coap_context_set_keepalive(ctx, 0);
    coap_context_set_block_mode(ctx,COAP_BLOCK_USE_LIBCOAP);
    /*INITIATE RESPONSE HANDLER */
    coap_register_response_handler(ctx, client_response_handler);
    coap_register_nack_handler(ctx, client_nack_handler);
    /*ENSURE CONNECT TO AP */
    while (!is_connected) {
        vTaskDelay(0.5 * 1000 / portTICK_RATE_MS);
    }

    //prepare_status_data_session(&session_status, &tick_status_data);
    //prepare_device_monitor_session(&session_device, &tick_device_monitor);
    prepare_session(ctx,&session_image, &tick_send_image); 
    prepare_session(ctx,&session_tp, &tick_get_throughput_prediction);
    prepare_session(ctx,&session_delay, &tick_put_delay);
    //int increment = 0;
    while (1) {
        if (is_connected) {
            //send_device_monitor(session_device, &tick_device_monitor);
            if (is_sensing_active) {
                //increment++;
                send_image(session_image, &tick_send_image);
                //printf("increment : %d\n",increment);
                
                //get_throughput_prediction(session_tp, &tick_get_throughput_prediction);
            }        
            wait_ms = 60000;
            // coba_duration = esp_timer_get_time();
            while (image_resp_wait) {

                int result = coap_io_process(ctx, wait_ms > 400 ? 400 : wait_ms);
               
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
           
            //send_delay(session_delay, &tick_put_delay);
            
            
        } else {
            coap_log(LOG_NOTICE, "Waiting for wifi connection!\n");
            vTaskDelay(1 * 1000 / portTICK_RATE_MS);
        }
    }
}

void change_dynamic_parameter(char* dynamic_value) {
    
        sensor_t *s = esp_camera_sensor_get();
        int tp = atoi(dynamic_value);
        float sizeAllowed = tp/frameRate;
       
        int min_size = 1700;
        int max_size = 30000;
        int kons = 921600 - 9216;
        float beta = min_size - (max_size-min_size)*9216/kons;
        float x = (sizeAllowed - beta)*kons/(max_size-min_size);
        //printf("hasil :%f\n",x);
        int i = 0;
        int output_expected = 0;
        for (i = size_of_list_frame -1; i >=0; i--){
          if (x > list_of_frame[i]) {
            output_expected = i;
            break;
          }
        }
        s->set_framesize(s, output_expected);
        //printf("Change image frame to : %d\n", output_expected);
        //printf("frame_size now: %d\n", s->status.framesize);
    
}

static coap_response_t client_response_handler(coap_session_t *session,
                                    const coap_pdu_t *sent, const coap_pdu_t *received,
                                    const coap_mid_t id) {
    coap_pdu_t *pdu = NULL;
    coap_opt_t *control_opt;
    uint8_t* control_opt_value;
    coap_opt_t *block_opt; //ok
    coap_opt_iterator_t opt_iter; //ook
    unsigned char buf[4];
    size_t len;
    const unsigned char *databuf = NULL;
    coap_mid_t tid;
   
    coap_block_missing_t missing_block_opt;
    
    coap_pdu_code_t rcv_code = coap_pdu_get_code(received);
    coap_pdu_type_t rcv_type = coap_pdu_get_type(received);
    coap_bin_const_t token = coap_pdu_get_token(received);
    coap_string_t *uriPath = coap_get_uri_path(received);
    
    const unsigned char *data = NULL;
    size_t data_len;
    size_t offset;
    size_t total;
 
    if (!track_check_token(&token)) {
     
      /* drop if this was just some message, or send RST in case of notification */
      if (!sent && (rcv_type == COAP_MESSAGE_CON ||
                    rcv_type == COAP_MESSAGE_NON)) {
        /* Cause a CoAP RST to be sent */
        return COAP_RESPONSE_FAIL;
      }
      return COAP_RESPONSE_OK;
    }
    if (rcv_type == COAP_MESSAGE_RST) {
      coap_log(LOG_INFO, "got RST\n");
      return COAP_RESPONSE_OK;
    }
    /* output the received data, if any */

  block_opt = coap_check_option(received, COAP_OPTION_URI_PATH, &opt_iter);

  if (COAP_RESPONSE_CLASS(rcv_code) == 2) {

    if (strncmp((const char*)uriPath->s,"troug",5) == 0){ //handle troug
    
    } else if (strncmp((const char*)uriPath->s,"image",5) == 0){ //handle image
      image_resp_wait = 0;
    } else {
      image_resp_wait = 0;
    }
    
    /* set obs timer if we have successfully subscribed a resource */
    if (doing_observe && !obs_started &&
    coap_check_option(received, COAP_OPTION_OBSERVE, &opt_iter)) {
      coap_log(LOG_DEBUG,
          "observation relationship established, set timeout to %d\n",
          obs_seconds);
      obs_started = 1;
      obs_ms = obs_seconds * 1000;
      obs_ms_reset = 1;
    }

    if (coap_get_data_large(received, &len, &databuf, &offset, &total)) {
      if (strncmp((const char*)uriPath->s,"troug",5) == 0){ //handle troug
      //doing something
      //printf("len asli: %d\n",len);
      char data[8];
      strncpy(data, (char *)databuf, len);
      //printf("Prediction asli : %s\n",data);
      change_dynamic_parameter(data);
      //printf("EXit\n");
      }   
    }

    /* Check if Block2 option is set */
    block_opt = coap_check_option(received, COAP_OPTION_BLOCK2, &opt_iter);
    if (!single_block_requested && block_opt) { /* handle Block2 */

      /* TODO: check if we are looking at the correct block number */
      if (coap_opt_block_num(block_opt) == 0) {
        /* See if observe is set in first response */
        ready = doing_observe ? coap_check_option(received,
                                  COAP_OPTION_OBSERVE, &opt_iter) == NULL : 1;
      }
      if(COAP_OPT_BLOCK_MORE(block_opt)) {
        
        wait_ms = 60 * 1000;
        wait_ms_reset = 1;
        doing_getting_block = 1;
      }
      else {
        doing_getting_block = 0;
        track_flush_token(&token);
      }
      return COAP_RESPONSE_OK;
    }
  } else {      /* no 2.05 */
    /* check if an error was signaled and output payload if so */
    if (COAP_RESPONSE_CLASS(rcv_code) >= 4) {
      fprintf(stderr, "err %d.%02d", COAP_RESPONSE_CLASS(rcv_code),
              rcv_code & 0x1F);
      if (coap_get_data_large(received, &len, &databuf, &offset, &total)) {
        fprintf(stderr, " ");
        while(len--) {
          fprintf(stderr, "%c", isprint(*databuf) ? *databuf : '.');
          databuf++;
        }
      }
      fprintf(stderr, "\n");
      return COAP_RESPONSE_FAIL;
    }

  }
//   if (!is_mcast)
//     track_flush_token(&token);

  /* our job is done, we can exit at any time */
  ready = doing_observe ? coap_check_option(received,
                                  COAP_OPTION_OBSERVE, &opt_iter) == NULL : 1;
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
    // esp_restart();
    ESP_LOGI(TAG, "RESTARTING ESP32");
    vTaskDelay(3 * 1000 / portTICK_RATE_MS);
    // esp_restart();
    vTaskDelete(NULL);
}

void prepare_device_monitor_session(coap_session_t **session, int64_t *tick) {
  
}

void send_device_monitor(coap_session_t *session, int64_t *tick) {
}

// void prepare_status_data_session(coap_session_t **session, int64_t *tick) {
//     coap_address_t controller_addr;

//     coap_address_init(&controller_addr);
//     controller_addr.addr.sin.sin_family = AF_INET;
//     controller_addr.addr.sin.sin_port = htonl(COAP_DEFAULT_PORT);
//     controller_addr.addr.sin.sin_addr.s_addr = CONTROLLER_ADDRESS;

//     *session = coap_new_client_session(ctx, NULL, &controller_addr, COAP_PROTO_UDP);
//     if (!*session) {
//         coap_log(LOG_NOTICE, "coap_new_client_session() failed\n");
//         coap_session_release(*session);
//     }

//     *tick = esp_timer_get_time();
// }

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

// static coap_address_t * coap_get_address(coap_uri_t *uri)
// {
//    static coap_address_t dst_addr;
//    char *phostname = NULL;
//    struct addrinfo hints;
//    struct addrinfo *addrres;
//    int error;
//    char tmpbuf[INET6_ADDRSTRLEN];

//    phostname = (char *)calloc(1, uri->host.length + 1);
//    if (phostname == NULL)
//    {
//       ESP_LOGE(TAG, "calloc failed");
//       return NULL;
//    }
//    memcpy(phostname, uri->host.s, uri->host.length);

//    memset((char *)&hints, 0, sizeof(hints));
//    hints.ai_socktype = SOCK_DGRAM;
//    hints.ai_family = AF_UNSPEC;

//    error = getaddrinfo(phostname, NULL, &hints, &addrres);
//    if (error != 0)
//    {
//       ESP_LOGE(TAG, "DNS lookup failed for destination address %s. error: %d", phostname, error);
//       free(phostname);
//       return NULL;
//    }
//    if (addrres == NULL)
//    {
//       ESP_LOGE(TAG, "DNS lookup %s did not return any addresses", phostname);
//       free(phostname);
//       return NULL;
//    }
//    free(phostname);
//    coap_address_init(&dst_addr);
//    switch (addrres->ai_family)
//    {
//    case AF_INET:
//       memcpy(&dst_addr.addr.sin, addrres->ai_addr, sizeof(dst_addr.addr.sin));
//       dst_addr.addr.sin.sin_port = htons(uri->port);
//       inet_ntop(AF_INET, &dst_addr.addr.sin.sin_addr, tmpbuf, sizeof(tmpbuf));
//       ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", tmpbuf);
//       break;
//    case AF_INET6:
//       memcpy(&dst_addr.addr.sin6, addrres->ai_addr, sizeof(dst_addr.addr.sin6));
//       dst_addr.addr.sin6.sin6_port = htons(uri->port);
//       inet_ntop(AF_INET6, &dst_addr.addr.sin6.sin6_addr, tmpbuf, sizeof(tmpbuf));
//       ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", tmpbuf);
//       break;
//    default:
//       ESP_LOGE(TAG, "DNS lookup response failed");
//       return NULL;
//    }
//    freeaddrinfo(addrres);

//    return &dst_addr;
// }

void prepare_session(coap_context_t *ctx, coap_session_t **session, int64_t *tick) {
    coap_address_t dst;
    static coap_uri_t uri;
    static coap_str_const_t server;
    int res;

    if (coap_split_uri((const uint8_t *)server_uri, strlen(server_uri), &uri) == -1)
      {ESP_LOGE(TAG, "CoAP server uri error"); }

    server = uri.host;
    coap_address_init(&dst);
    res = resolve_address(&server, &dst.addr.sa);
    if (res < 0) {
      fprintf(stderr, "failed to resolve address\n");
      exit(-1);
    }
    dst.size = res;
    dst.addr.sin.sin_port = htons(COAP_DEFAULT_PORT);

    *session = get_session(
      ctx,NULL, "0",COAP_PROTO_UDP,
      &dst,NULL,0,NULL,0
    );

    if ( !session ) {
      coap_log( LOG_EMERG, "cannot create client session\n" );
    }
    coap_session_init_token(*session, base_token.length, base_token.s);
    *tick = esp_timer_get_time();
}

void send_delay(coap_session_t *session, int64_t *tick) {
        
    coap_pdu_t *request = NULL;
    uint8_t token[8];
    size_t tokenlen; 
  
    coap_optlist_t *optlist = NULL;
    double throughput = 0;

    if (!(request = coap_new_pdu(COAP_MESSAGE_NON,COAP_REQUEST_CODE_PUT, session))) {  
        ESP_LOGE(TAG, "coap_new_pdu() failed");
        goto clean_up;
    }                        
    
    coap_session_new_token(session, &tokenlen, token);
    track_new_token(tokenlen, token);
    if (!coap_add_token(request, tokenlen, token)) {
        coap_log(LOG_DEBUG, "cannot add token to request\n");
        goto clean_up;
    }
    coap_add_option(request, COAP_OPTION_URI_PATH, 5, (uint8_t *)delay_path);
    double d = (double)send_duration/(1000*1000);
    throughput = image->len / d;
    
    coap_add_data_large_request(session,request, sizeof(throughput),(uint8_t *)&throughput , NULL, NULL);
    coap_send(session, request);
   
clean_up:
    // coap_log(LOG_NOTICE, "Clean Up %lld\n", send_duration);
    if (optlist) {
        coap_delete_optlist(optlist);
        optlist = NULL;
    }
      
}

void send_image(coap_session_t *session, int64_t *tick) {
        
    coap_pdu_t *request = NULL;
    uint8_t token[8];
    size_t tokenlen; 
    payload.length = 0;
    payload.s = NULL; 

    coap_optlist_t *optlist = NULL;
    coba_duration = esp_timer_get_time();
    image = camera_capture();
    coba_duration = esp_timer_get_time() - coba_duration;

    if (!image) {
        coap_log(LOG_NOTICE, "Take image failed!\n");
        goto clean_up;
    }

    payload.s = image->buf;
    payload.length = image->len;
   
    coap_log(LOG_NOTICE, "Take image success\n");
    //send_duration = esp_timer_get_time();
    //coap_log(LOG_NOTICE, "Start sending image, start tick %lld\n", send_duration);

    if (!(request = coap_new_pdu(COAP_MESSAGE_CON,COAP_REQUEST_CODE_PUT, session))) {  
        ESP_LOGE(TAG, "coap_new_pdu() failed");
    }                        
    
    coap_session_new_token(session, &tokenlen, token);
    track_new_token(tokenlen, token);
    if (!coap_add_token(request, tokenlen, token)) {
        coap_log(LOG_DEBUG, "cannot add token to request\n");
    }

    coap_add_option(request, COAP_OPTION_URI_PATH, 5, (uint8_t *)image_path);
    coap_add_data_large_request(session,request, payload.length, payload.s, NULL, NULL);
    
    image_resp_wait = 1;
    coap_send(session, request);
   
clean_up:
    coap_log(LOG_NOTICE, "Clean Up %lld\n", send_duration);
    if (optlist) {
        coap_delete_optlist(optlist);
        optlist = NULL;
    }
      
}

void get_throughput_prediction(coap_session_t *session, int64_t *tick) {
    
    uint8_t token[8];
    size_t tokenlen; 
    coap_pdu_t *pdu = NULL;
    
    if (esp_timer_get_time() - *tick > 5000000) {
        *tick = esp_timer_get_time();
       
        pdu = coap_new_pdu(COAP_MESSAGE_CON,COAP_REQUEST_CODE_GET, session);
        if (!pdu){
            ESP_LOGE(TAG, "coap_new_pdu() failed");
        }

        coap_session_new_token(session, &tokenlen, token);
        track_new_token(tokenlen, token);
        if (!coap_add_token(pdu, tokenlen, token)) {
            coap_log(LOG_DEBUG, "cannot add token to request\n");
        }
        coap_add_option(pdu, COAP_OPTION_URI_PATH, 5, (uint8_t *)tp_path);
       
        image_resp_wait = 1;
        coap_send(session, pdu);

    }
}

