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

//#include "coap_dtls.h" deprecated
// #include "coap.h" deprecated
#include "coap3/coap.h"

#include "coap_custom.h"
#include "lib_nvs.h"

#include "esp32_node.h"
#include "data_collection.h"

#include "esp_camera.h"
#include "lib_camera.h"

const static char *TAG = "CoAP_server_client";

#define CONTROLLER_ADDRESS 251658250 /* 10.0.0.15 but in integer */

uint32_t local_server_ip = 0;

int64_t send_duration = 0;

static int identity_response_flag = 0;

static int image_resp_wait = 0;

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

coap_session_t *session_status = NULL;
coap_session_t *session_device = NULL;
coap_session_t *session_image = NULL;

uint8_t nack_flag = 0;

#if DATA_COLLECTION_RETRANSMISSION_TIMEOUT_SEC == 0 && DATA_COLLECTION_RETRANSMISSION_TIMEOUT_FRAC == 0
uint64_t rtt_micros;
#endif

static coap_pdu_t *coap_new_request(coap_context_t *ctx,
                                    coap_session_t *session, method_t m,
                                    coap_optlist_t **options, uint8_t type,
                                    unsigned char *data, size_t length);
void change_dynamic_parameter(unsigned long queue_number, unsigned int dynamic_value);
static void client_response_handler(coap_context_t *ctx, coap_session_t *session, coap_pdu_t *sent, coap_pdu_t *received, const coap_tid_t id);
static void client_nack_handler(coap_context_t *ctx, coap_session_t *session, coap_pdu_t *sent, coap_nack_reason_t reason, const coap_tid_t id);
void camera_sensing_client();
void coap_send_initial_data();
void wifi_disconnect();
static void coap_check_execute_broadcast(coap_pdu_t *request);
static void hnd_espressif_get(coap_context_t *ctx, coap_resource_t *resource, coap_session_t *session, coap_pdu_t *request, coap_binary_t *token, coap_string_t *query, coap_pdu_t *response);
static void hnd_espressif_put(coap_context_t *ctx, coap_resource_t *resource, coap_session_t *session, coap_pdu_t *request, coap_binary_t *token, coap_string_t *query, coap_pdu_t *response);

int64_t tick_device_monitor = 0;
int64_t tick_status_data = 0;
int64_t tick_send_image = 0;

void prepare_device_monitor_session(coap_session_t **session, int64_t *tick);
void send_device_monitor(coap_session_t *session, int64_t *tick);
void prepare_status_data_session(coap_session_t **session, int64_t *tick);
void send_status_data(coap_session_t *session, int64_t *tick);
void prepare_image_session(coap_session_t **session, int64_t *tick);
void send_image(coap_session_t *session, int64_t *tick);
void send_image_continue(coap_session_t *session, int64_t *tick);

void coap_client_server(void *p) {
    ESP_LOGI(TAG, "CoAP client running!");

    request_tag = rand();

    coap_address_t serv_addr;
    coap_resource_t *controller_resource = NULL;

    coap_set_log_level(COAP_LOG_DEFAULT_LEVEL);

    if (!coap_debug_set_packet_loss(COAP_DEBUG_PACKET_LOSS)) {
        coap_log(LOG_NOTICE, "Cannot add debug packet loss!\n");
        ESP_ERROR_CHECK(-1);
    }

    coap_endpoint_t *ep = NULL;

    ctx = coap_new_context(NULL);
    if (!ctx) {
        coap_log(LOG_NOTICE, "coap_new_context() failed\n");
    }

    /* Prepare the CoAP server socket */
    coap_address_init(&serv_addr);
    serv_addr.addr.sin.sin_family = AF_INET;
    serv_addr.addr.sin.sin_addr.s_addr = INADDR_ANY;
    serv_addr.addr.sin.sin_port = htons(COAP_DEFAULT_PORT);

    while (!is_connected) {
        vTaskDelay(0.5 * 1000 / portTICK_RATE_MS);
    }

    ep = coap_new_endpoint(ctx, &serv_addr, COAP_PROTO_UDP);
    if (!ep) {
        coap_log(LOG_NOTICE, "udp: coap_new_endpoint() failed\n");
    }

    controller_resource = coap_resource_init(coap_make_str_const(""), 0);
    if (!controller_resource) {
        coap_log(LOG_NOTICE, "coap_resource_init() failed\n");
    }
    coap_register_handler(controller_resource, COAP_REQUEST_PUT, hnd_espressif_put);
    coap_register_handler(controller_resource, COAP_REQUEST_GET, hnd_espressif_get);
    coap_add_resource(ctx, controller_resource);

    coap_register_response_handler(ctx, client_response_handler);
    coap_register_nack_handler(ctx, client_nack_handler);

    coap_log(LOG_NOTICE, "Sending initial data!\n");
    coap_send_initial_data();

    prepare_status_data_session(&session_status, &tick_status_data);
    prepare_device_monitor_session(&session_device, &tick_device_monitor);
    prepare_image_session(&session_image, &tick_send_image);

    while (1) {
        if (is_connected) {
            send_device_monitor(session_device, &tick_device_monitor);
            if (is_sensing_active) {
                while (local_server_ip == 0) {
                    coap_log(LOG_NOTICE, "local server ip is zero %d\n", local_server_ip);
                    // sending_initial_data = 1;
                    coap_send_initial_data();
                    vTaskDelay(1000 / portTICK_RATE_MS);
                }
                send_image(session_image, &tick_send_image);
            }
            if (status_data_transfer_type) {
                send_status_data(session_status, &tick_status_data);
            }

            coap_run_once(ctx, 10);
        } else {
            coap_log(LOG_NOTICE, "Waiting for wifi connection!\n");
            vTaskDelay(1 * 1000 / portTICK_RATE_MS);
        }
    }
}

static coap_pdu_t *coap_new_request(coap_context_t *ctx,
                                    coap_session_t *session, method_t m,
                                    coap_optlist_t **options, uint8_t type,
                                    unsigned char *data, size_t length) {
    coap_pdu_t *pdu;
    (void)ctx;

    if (!(pdu = coap_new_pdu(session))) {
        return NULL;
    }

    pdu->type = type;
    pdu->tid = coap_new_message_id(session);
    pdu->code = m;

    if (options) coap_add_optlist_pdu(pdu, options);

    if (length) {
        coap_add_data(pdu, length, data);
    }

    return pdu;
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

static void client_response_handler(coap_context_t *ctx, coap_session_t *session,
                                    coap_pdu_t *sent, coap_pdu_t *received,
                                    const coap_tid_t id) {
    coap_pdu_t *pdu = NULL;
    coap_opt_t *control_opt;
    uint8_t control_opt_value;
    coap_opt_t *block_opt;
    coap_opt_iterator_t opt_iter;
    unsigned char buf[4];
    size_t len;
    unsigned char *databuf;
    coap_tid_t tid;

    coap_block_missing_t missing_block_opt;

    if (COAP_RESPONSE_CLASS(received->code) == 2 ||
        COAP_RESPONSE_CLASS(received->code) == 4) {
        control_opt = coap_check_option(received, COAP_OPTION_CONTROL, &opt_iter);
        if (control_opt) {
            control_opt_value = *coap_opt_value(control_opt);
            switch (control_opt_value) {
                case 0:
                    coap_log(LOG_NOTICE, "Control opt identity response received!\n");
                    identity_response_flag = 1;
                    size_t data_len;
                    uint8_t *data;
                    coap_get_data(received, &data_len, &data);
                    if (data_len > 0) {
                        memcpy(&local_server_ip, data, data_len);
                        coap_log(LOG_NOTICE, "local ip received, local_ip=%d\n", local_server_ip);
                    }
                    break;
                default:
                    coap_log(LOG_NOTICE, "Control opt number %d response received!\n", control_opt_value);
                    break;
            }
        }

        /* Need to see if blocked response */
        block_opt = coap_check_option(received, COAP_OPTION_BLOCK1, &opt_iter);
        if (block_opt) {
            unsigned int szx = COAP_OPT_BLOCK_SZX(block_opt);
            unsigned int num = coap_opt_block_num(block_opt);
            unsigned int m = COAP_OPT_BLOCK_MORE(block_opt);
            coap_log(LOG_NOTICE, "Received block option! block size is %u, block nr. %u, more %d\n", szx, num, m);

            count_retransmission += session->last_retransmission_cnt;
            session->last_retransmission_cnt = 0;
            if (szx != block.szx) {
                unsigned int bytes_sent = ((block.num + 1) << (block.szx + 4));
                if (bytes_sent % (1 << (szx + 4)) == 0) {
                    /* Recompute the block number of the previous packet given
           * the new block size */
                    num = block.num = (bytes_sent >> (szx + 4)) - 1;
                    block.szx = szx;
                    coap_log(LOG_NOTICE,
                             "new Block1 size is %u, block number %u completed\n",
                             (1 << (block.szx + 4)), block.num);
                } else {
                    coap_log(LOG_NOTICE,
                             "ignoring request to increase Block1 size, "
                             "next block is not aligned on requested block "
                             "size boundary. "
                             "(%u x %u mod %u = %u != 0)\n",
                             block.num + 1, (1 << (block.szx + 4)), (1 << (szx + 4)),
                             bytes_sent % (1 << (szx + 4)));
                }
            }

            if (last_block1_tid == received->tid) {
                coap_log(LOG_NOTICE, "Duplicated block ack, ignoring\n");
                /*
         * Duplicate BLOCK1 ACK
         *
         * RFCs not clear here, but on a lossy connection, there could
         * be multiple BLOCK1 ACKs, causing the client to retransmit the
         * same block multiple times.
         *
         * Once a block has been ACKd, there is no need to retransmit
         * it.
         */
                return;
            }
            last_block1_tid = received->tid;

            // TODO: handle packet loss7

            block.num = num;
            block.m = m;

            if (payload.length <= (block.num + 1) * (1 << (block.szx + 4))) {
                coap_log(LOG_NOTICE, "Maximum block option received, updating log data to controller\n");
                // no more miss and last block was received
                send_duration = esp_timer_get_time() - send_duration;
                data_collection_count++;

                coap_log(LOG_EMERG, "img %lu sent\n", data_collection_count);

                time_t ntp_time;
                time(&ntp_time);

                coap_session_t *controller_session;

                coap_address_t controller_addr;
                coap_address_init(&controller_addr);
                controller_addr.addr.sin.sin_family = AF_INET;
                controller_addr.addr.sin.sin_port = htonl(COAP_DEFAULT_PORT);
                controller_addr.addr.sin.sin_addr.s_addr = CONTROLLER_ADDRESS;

                controller_session = coap_new_client_session(
                    ctx, NULL, &controller_addr, COAP_PROTO_UDP);
                if (!controller_session) {
                    coap_log(LOG_NOTICE, "coap_new_client_session() failed\n");
                }

                pdu = coap_new_request(
                    ctx, controller_session, COAP_REQUEST_PUT, NULL, COAP_MESSAGE_NON, NULL,
                    0); /* first, create bare PDU w/o any option  */
                uint8_t data[61];
                memcpy(data, ap_mac, 6);
                memcpy(data + 6, &ip, 4);
                memcpy(data + 6 + 4, &ntp_time, 4);
                uint64_t device_timestamp = esp_timer_get_time();
                memcpy(data + 6 + 4 + 4, &device_timestamp, 8);
                memcpy(data + 6 + 4 + 4 + 8, &data_collection_count, 4);
                memcpy(data + 6 + 4 + 4 + 8 + 4, &send_duration, 8);
                memcpy(data + 6 + 4 + 4 + 8 + 4 + 8, &payload.length, 4);  //
                memcpy(data + 6 + 4 + 4 + 8 + 4 + 8 + 4, &total_payload_sent, 4);
                memcpy(data + 6 + 4 + 4 + 8 + 4 + 8 + 4 + 4, &total_missing_payload, 4);
                memcpy(data + 6 + 4 + 4 + 8 + 4 + 8 + 4 + 4 + 4, &count_total_block, 4);
                memcpy(data + 6 + 4 + 4 + 8 + 4 + 8 + 4 + 4 + 4 + 4, &count_missing_block, 4);
                memcpy(data + 6 + 4 + 4 + 8 + 4 + 8 + 4 + 4 + 4 + 4 + 4, &count_retransmission, 4);
                memcpy(data + 6 + 4 + 4 + 8 + 4 + 8 + 4 + 4 + 4 + 4 + 4 + 4, (uint8_t *)&block_size, 1);
                memcpy(data + 6 + 4 + 4 + 8 + 4 + 8 + 4 + 4 + 4 + 4 + 4 + 4 + 1, (uint8_t *)&block_seq_len, 1);
                memcpy(data + 6 + 4 + 4 + 8 + 4 + 8 + 4 + 4 + 4 + 4 + 4 + 4 + 1 + 1, (uint8_t *)&image_size, 1);

                coap_log(LOG_NOTICE, MACSTR ",%d,%ld,%lld,%ld,%lld,%d,%d,%d,%ld,%ld,%ld,%d,%d,%d\n", MAC2STR(ap_mac), ip, ntp_time, device_timestamp, data_collection_count, send_duration, payload.length, total_payload_sent, total_missing_payload, count_total_block, count_missing_block, count_retransmission, block_size, block_seq_len, image_size);

                uint8_t optval = 20;
                coap_add_option(pdu, COAP_OPTION_CONTROL, 1, &optval);

                coap_add_data(pdu, 61, data);

                coap_send(controller_session, pdu);

                coap_session_release(controller_session);

                image_resp_wait = 0;
                tick_send_image = esp_timer_get_time();
                return;
            }

            unsigned int remaining_payload =
                coap_get_remaining_payload_number(payload.length, block.num,
                                                  block.szx);
            coap_log(LOG_NOTICE, "Remaining payload %d\n", remaining_payload);

            pdu = coap_new_request(
                ctx, session, COAP_REQUEST_PUT, NULL, COAP_MESSAGE_CON, NULL,
                0); /* first, create bare PDU w/o any option  */

            if (pdu) {
                count_total_block++;

                block.num = block.num + 1;
                block.m = ((block.num + 1) * (1 << (block.szx + 4)) < payload.length);
#if DATA_COLLECTION_RETRANSMISSION_TIMEOUT_SEC == 0 && DATA_COLLECTION_RETRANSMISSION_TIMEOUT_FRAC == 0
                rtt_micros = esp_timer_get_time() - rtt_micros;
                coap_fixed_point_t fixed_point;
                fixed_point.integer_part = rtt_micros / 100000;
                fixed_point.fractional_part = (rtt_micros % 100000) / 1000;
                coap_session_set_ack_timeout(session, fixed_point);
#endif

                coap_add_option(pdu, COAP_OPTION_URI_PATH, 5, (uint8_t *)image_path);

                coap_add_option(
                    pdu, COAP_OPTION_BLOCK1,
                    coap_encode_var_safe(buf, sizeof(buf),
                                         (block.num << 4) | (block.m << 3) | block.szx),
                    buf);

                coap_add_block(pdu, payload.length, payload.s, block.num, block.szx);

                total_payload_sent += pdu->used_size + 46;
                last_block_num = block.num;
                tid = coap_send(session, pdu);
                if (tid == COAP_INVALID_TID) {
                    coap_log(LOG_NOTICE,
                             "client_response_handler: error sending new request\n");
                }
            }
        } else {
            if (coap_get_data(received, &len, &databuf)) {
                coap_log(LOG_NOTICE, "Received: %.*s\n", (int)len, databuf);
            }
        }
    }
}

static void client_nack_handler(coap_context_t *ctx, coap_session_t *session, coap_pdu_t *sent, coap_nack_reason_t reason, const coap_tid_t id) {
    coap_log(LOG_NOTICE, "NACK trigger received, the reason is %d\n", reason);
    coap_opt_iterator_t opt_iter;

    nack_flag = 1;
    if (coap_check_option(sent, COAP_OPTION_BLOCK_MOD, &opt_iter)) {
        image_resp_wait = 0;
    }
}

void coap_send_initial_data() {
    coap_pdu_t *pdu = NULL;
    coap_session_t *session = NULL;
    coap_address_t controller_addr;
    uint8_t *identity_data = (uint8_t *)malloc(15 * sizeof(uint8_t));
    uint8_t *option_value = (uint8_t *)malloc(1 * sizeof(uint8_t));
    option_value[0] = 0;

    identity_response_flag = 0;
    uint8_t mac_address[6];
    esp_base_mac_addr_get(mac_address);
    if (mac_address[5] != 0xFF) {
        mac_address[5]++;
    } else {
        mac_address[4]++;
    }
    wifi_ap_record_t ap_record;
    esp_wifi_sta_get_ap_info(&ap_record);
    memcpy(identity_data, mac_address, sizeof(mac_address));

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(wifiSTA, &ip_info);
    uint32_t ip_addr = ip_info.ip.addr;

    memcpy(identity_data + 6, &ip_addr, sizeof(ip_addr));

    identity_data[10] = status_data_transfer_type;
    identity_data[11] = is_sensing_active;
    identity_data[12] = (uint8_t)image_size;
    identity_data[13] = (uint8_t)image_format;
    identity_data[14] = data_send_period;

    coap_address_init(&controller_addr);
    controller_addr.addr.sin.sin_family = AF_INET;
    controller_addr.addr.sin.sin_port = htonl(COAP_DEFAULT_PORT);
    controller_addr.addr.sin.sin_addr.s_addr = CONTROLLER_ADDRESS;

    session = coap_new_client_session(ctx, NULL, &controller_addr, COAP_PROTO_UDP);
    if (!session) {
        coap_log(LOG_NOTICE, "coap_new_client_session() failed\n");
        return;
    }

    pdu = coap_new_request(ctx, session, COAP_REQUEST_PUT, NULL, COAP_MESSAGE_CON, NULL, 0);

    coap_add_option(pdu, COAP_OPTION_CONTROL, 1, option_value);

    coap_add_data(pdu, 15, identity_data);

    coap_send(session, pdu);

    while (!identity_response_flag && !nack_flag) {
        coap_run_once(ctx, 10);
        if (nack_flag) {
            nack_flag = 0;
            break;
        }
    }
    coap_session_release(session);
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

static void coap_check_execute_broadcast(coap_pdu_t *request) {
    coap_context_t *ctx;
    size_t size;
    uint8_t *data;

    if (!coap_get_broadcast_option(request)) {
        coap_log(LOG_NOTICE, "Not broadcast packet, exiting!\n");
    } else {
        coap_get_data(request, &size, &data);
        coap_log(LOG_NOTICE, "Get broadcast option\n");
        // SENDING BROADCAST
        coap_address_t dst_addr;

        coap_session_t *broadcast_session = NULL;

        char tmpbuf[INET6_ADDRSTRLEN];

        coap_address_init(&dst_addr);

        dst_addr.addr.sin.sin_family = AF_INET;
        dst_addr.addr.sin.sin_port = htons(5683);
        esp_netif_ip_info_t ap_ip_info;
        esp_netif_get_ip_info(wifiAP, &ap_ip_info);
        inet_ntop(AF_INET, &ap_ip_info.ip.addr, tmpbuf, sizeof(tmpbuf));
        inet_ntop(AF_INET, &ap_ip_info.gw.addr, tmpbuf, sizeof(tmpbuf));
        inet_ntop(AF_INET, &ap_ip_info.netmask.addr, tmpbuf, sizeof(tmpbuf));

        ip4_addr_t temp;
        temp.addr = ap_ip_info.ip.addr | ~(ap_ip_info.netmask.addr);
        // dst_addr.addr.sin.sin_addr = temp.addr;
        memcpy(&dst_addr.addr.sin.sin_addr, &temp.addr, sizeof(dst_addr.addr.sin.sin_addr));
        inet_ntop(AF_INET, &dst_addr.addr.sin.sin_addr, tmpbuf, sizeof(tmpbuf));
        coap_log(LOG_NOTICE, "Broadcast dest addr has been set, IP=%s\n", tmpbuf);

        ctx = coap_new_context(NULL);

        broadcast_session = coap_new_client_session(
            ctx, NULL, &dst_addr, COAP_PROTO_UDP);
        if (!broadcast_session) {
            coap_log(LOG_NOTICE, "coap_new_client_session() failed\n");
            return;
        }

        coap_opt_iterator_t opt_iter;
        coap_optlist_t *optlist = NULL;
        coap_opt_t *opt;

        coap_log(LOG_NOTICE, "Preparing coap option\n");

        opt = coap_check_option(request, COAP_OPTION_CONTROL, &opt_iter);
        if (opt) {
            coap_insert_optlist(&optlist, coap_new_optlist(COAP_OPTION_CONTROL, coap_opt_length(opt), coap_opt_value(opt)));
        }

        opt = coap_check_option(request, COAP_OPTION_BROADCAST, &opt_iter);
        uint8_t *broadcast_opt_value = coap_opt_value(opt);

        if (opt && (*broadcast_opt_value > 0)) {
            (*broadcast_opt_value)--;
            coap_insert_optlist(&optlist, coap_new_optlist(COAP_OPTION_BROADCAST, 1, broadcast_opt_value));
            coap_log(LOG_NOTICE, "Broadcast option list added, depth = %d\n", *broadcast_opt_value);
        } else {
            coap_log(LOG_NOTICE, "Broadcast option list not added, depth already %d\n", *broadcast_opt_value);
        }

        coap_pdu_t *broadcast_request = coap_new_pdu(broadcast_session);
        if (broadcast_request == NULL) {
            coap_log(LOG_NOTICE, "Make broadcast PDU failed!\n");
        }
        broadcast_request->type = COAP_MESSAGE_NON;
        broadcast_request->tid = coap_new_message_id(broadcast_session);
        broadcast_request->code = COAP_REQUEST_PUT;

        coap_add_optlist_pdu(broadcast_request, &optlist);
        coap_add_data(broadcast_request, size, (uint8_t *)data);

        if (coap_send(broadcast_session, broadcast_request) == COAP_INVALID_TID) {
            coap_log(LOG_NOTICE, "Send broadcast failed! Invalid TID!\n");
        }

        coap_log(LOG_NOTICE, "Broadcast sent\n");

        if (broadcast_session) {
            coap_session_release(broadcast_session);
        }

        if (ctx) {
            coap_free_context(ctx);
        }

        coap_log(LOG_NOTICE, "Broadcast session released\n");
    }
}

static void hnd_espressif_get(coap_context_t *ctx,
                              coap_resource_t *resource,
                              coap_session_t *session,
                              coap_pdu_t *request,
                              coap_binary_t *token,
                              coap_string_t *query,
                              coap_pdu_t *response) {
    coap_opt_iterator_t opt_iter;
    uint8_t opt_response;
    uint8_t *data_response;
    coap_opt_t *opt;
    opt = coap_check_option(request, COAP_OPTION_CONTROL, &opt_iter);
    uint8_t opt_value = *coap_opt_value(opt);
    wifi_ap_record_t ap_record;
    if (coap_opt_length(opt) > 1) {
        coap_log(LOG_NOTICE, "Invalid control opt length!\n");
        response->code = COAP_RESPONSE_CODE(404);
    } else {
        switch (opt_value) {
            case 2: {
                coap_log(LOG_NOTICE, "Receiving free heap status control request!\n");
                response->code = COAP_RESPONSE_CODE(205);
                opt_response = 2;
                coap_add_option(response,
                                COAP_OPTION_CONTROL,
                                sizeof(opt_response), &opt_response);
                uint32_t free_heap = esp_get_free_heap_size();
                data_response = (uint8_t *)malloc(sizeof(free_heap));
                memcpy(data_response, &free_heap, sizeof(free_heap));
                coap_add_data(response, sizeof(free_heap), data_response);
                break;
            }
            case 3: {
                coap_log(LOG_NOTICE, "Receiving parent rssi control request!\n");
                if (esp_wifi_sta_get_ap_info(&ap_record) == ESP_OK) {
                    response->code = COAP_RESPONSE_CODE(205);
                    opt_response = 3;
                    coap_add_option(response,
                                    COAP_OPTION_CONTROL,
                                    sizeof(opt_response), &opt_response);
                    data_response = (uint8_t *)malloc(sizeof(ap_record.rssi));
                    *data_response = ap_record.rssi;
                    coap_add_data(response, sizeof(ap_record.rssi), data_response);
                } else {
                    response->code = COAP_RESPONSE_CODE(404);
                }
                break;
            }
            case 4: {
                coap_log(LOG_NOTICE, "Receiving network discovery control request!\n");

                coap_log(LOG_NOTICE, "Config scan!\n");
                wifi_scan_config_t scan_config;
                scan_config.ssid = (uint8_t *)ap_ssid;
                scan_config.bssid = NULL;
                scan_config.channel = 0;
                scan_config.show_hidden = 1;
                scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
                scan_config.scan_time.active.min = 0;
                scan_config.scan_time.active.max = 0;

                coap_log(LOG_NOTICE, "AP SSID %s\n", ap_ssid);

                coap_log(LOG_NOTICE, "Start scan!\n");
                if (esp_wifi_scan_start(&scan_config, true) == ESP_OK) {
                    response->code = COAP_RESPONSE_CODE(205);
                    opt_response = 4;

                    coap_log(LOG_NOTICE, "Add option!\n");
                    coap_add_option(response,
                                    COAP_OPTION_CONTROL,
                                    sizeof(opt_response), &opt_response);
                    uint16_t scan_list_size = 20;
                    wifi_ap_record_t ap_info[scan_list_size];
                    uint16_t scan_count = 0;

                    coap_log(LOG_NOTICE, "Get AP!\n");
                    esp_wifi_scan_get_ap_records(&scan_list_size, ap_info);
                    esp_wifi_scan_get_ap_num(&scan_count);

                    data_response = (uint8_t *)malloc(7 * scan_count * sizeof(uint8_t));

                    coap_log(LOG_NOTICE, "Total APs scanned = %u\n", scan_count);
                    for (int i = 0; (i < 20) && (i < scan_count); i++) {
                        coap_log(LOG_NOTICE, "SSID \t\t%s\n", ap_info[i].ssid);
                        coap_log(LOG_NOTICE, "BSSID \t\t" MACSTR "\n", MAC2STR(ap_info[i].bssid));
                        coap_log(LOG_NOTICE, "RSSI \t\t%d\n", ap_info[i].rssi);
                        memcpy(data_response + (i * 7), &(ap_info[i].bssid), sizeof(ap_info[i].bssid));
                        memcpy(data_response + (i * 7 + 6), &(ap_info[i].rssi), sizeof(ap_info[i].rssi));
                    }
                    coap_log(LOG_NOTICE, "Add data!\n");
                    coap_add_data(response, scan_count * 7, data_response);
                } else {
                    response->code = COAP_RESPONSE_CODE(404);
                }
                break;
            }
            default: {
                coap_log(LOG_NOTICE, "Invalid control opt value!\n");
                response->code = COAP_RESPONSE_CODE(404);
            }
        }
    }
}

static void hnd_espressif_put(coap_context_t *ctx,
                              coap_resource_t *resource,
                              coap_session_t *session,
                              coap_pdu_t *request,
                              coap_binary_t *token,
                              coap_string_t *query,
                              coap_pdu_t *response) {
    coap_opt_iterator_t opt_iter;
    coap_opt_t *opt;
    size_t size;
    uint8_t opt_value;
    size_t opt_length;
    unsigned char *data;
    opt = coap_check_option(request, COAP_OPTION_CONTROL, &opt_iter);
    opt_value = *coap_opt_value(opt);
    opt_length = coap_opt_length(opt);

    // coap_add_option(response, COAP_OPTION_CONTROL, opt_length, opt_value);
    // coap_add_data(response, size, data);

    response->code = COAP_RESPONSE_CODE(201);

    if (opt_length > 1) {
        coap_log(LOG_NOTICE, "Invalid control opt length!\n");
        response->code = COAP_RESPONSE_CODE(404);
    } else {
        switch (opt_value) {
            case 5: {
                coap_log(LOG_NOTICE, "Receiving send status type control put!\n");
                (void)coap_get_data(request, &size, &data);
                if (size != 1) {
                    coap_log(LOG_NOTICE, "Invalid size length %d for control option number %d\n", size, opt_value);
                    response->code = COAP_RESPONSE_CODE(400);
                } else {
                    status_data_transfer_type = *data;
                    set_config_param_unsigned_char("st_ctl_dtype", status_data_transfer_type);

                    response->code = COAP_RESPONSE_CODE(204);
                }
                break;
            }
            case 6: {
                coap_log(LOG_NOTICE, "Receiving sensing/not control put!\n");
                /* coap_get_data() sets size to 0 on error */
                (void)coap_get_data(request, &size, &data);

                if (SUPPORT_SENSING == 0) {
                    response->code = COAP_RESPONSE_CODE(401);
                    coap_log(LOG_NOTICE, "sensing not supported!\n");
                } else if (size != 1) {
                    response->code = COAP_RESPONSE_CODE(400);
                    coap_log(LOG_NOTICE, "payload format wrong!\n");
                } else {
                    is_sensing_active = data[0];
                    set_config_param_unsigned_char("sensing", is_sensing_active);
                    // xTaskCreate(wifi_disconnect, "wifi_dc", 1 * 1024, NULL, 5, NULL);
                    response->code = COAP_RESPONSE_CODE(204);
                }
                break;
            }
            case 7: {
                /* Total data of this option is 1 bytes, including:
                image_size = 1 bytes */
                coap_log(LOG_NOTICE, "Receiving image size control put!\n");

                coap_get_data(request, &size, &data);
                if (SUPPORT_SENSING == 0) {
                    response->code = COAP_RESPONSE_CODE(401);
                    coap_log(LOG_NOTICE, "sensing not supported!\n");
                } else if (size != 1) {
                    coap_log(LOG_NOTICE, "Data size %d for control type number %d wrong!\n", size, opt_value);
                } else {
                    coap_log(LOG_NOTICE, "Get camera sensor!\n");
                    sensor_t *s = esp_camera_sensor_get();
                    coap_log(LOG_NOTICE, "Get image size value!\n");
                    // memcpy(&image_size, data, sizeof(image_size));
                    image_size = data[0];
                    set_config_param_int("img_sz", image_size);
                    coap_log(LOG_NOTICE, "Set value!\n");
                    s->set_framesize(s, image_size);
                    xTaskCreate(wifi_disconnect, "wifi_dc", 1 * 1024, NULL, 5, NULL);
                }
                break;
            }
            case 8: {
                /* Total data of this option is 1 bytes, including:
                data_send_period = 1 byte */
                coap_log(LOG_NOTICE, "Receiving image format control put!\n");

                coap_get_data(request, &size, &data);
                if (SUPPORT_SENSING == 0) {
                    response->code = COAP_RESPONSE_CODE(401);
                    coap_log(LOG_NOTICE, "sensing not supported!\n");
                } else if (size != 1) {
                    coap_log(LOG_NOTICE, "Data size %d for control type number %d wrong!\n", size, opt_value);
                } else {
                    sensor_t *s = esp_camera_sensor_get();
                    // memcpy(&image_format, data, sizeof(image_format));
                    image_format = data[0];
                    set_config_param_int("img_fmt", image_format);
                    s->set_pixformat(s, image_format);
                    xTaskCreate(wifi_disconnect, "wifi_dc", 1 * 1024, NULL, 5, NULL);
                }
                break;
            }
            case 9: {
                /* Total data of this option is 1 bytes, including:
                data_send_period = 1 byte */
                coap_log(LOG_NOTICE, "Receiving sending frequency control put!\n");

                coap_get_data(request, &size, &data);
                if (size != 1) {
                    coap_log(LOG_NOTICE, "Data size %d for control type number %d wrong!\n", size, opt_value);
                } else {
                    memcpy(&data_send_period, data, sizeof(data_send_period));
                    set_config_param_unsigned_char("send_period", data_send_period);
                }
                break;
            }
            case 10: {
                /* total 4 bytes that is local machine ip address */
                coap_log(LOG_NOTICE, "Receiving local machine destination control put!\n");
                coap_get_data(request, &size, &data);
                if (!SUPPORT_SENSING) {
                    coap_log(LOG_NOTICE, "Sensing not supported!\n");
                } else if (size != 3) {
                    coap_log(LOG_NOTICE, "Data size %d for control type number %d wrong!\n", size, opt_value);
                } else {
                    memcpy(&sensing_destination_ip, data, sizeof(sensing_destination_ip));
                }
                break;
            }
            case 11: {
                /* The data included in this control opt is 6 byte which is the mac address */
                coap_log(LOG_NOTICE, "Receiving network configure control put!\n");

                /* coap_get_data() sets size to 0 on error */
                (void)coap_get_data(request, &size, &data);

                if (size != 6) {
                    coap_log(LOG_NOTICE, "Invalid pdu size! It's not 6, its %d\n", size);
                } else {
                    // Set new bssid
                    memcpy(old_bssid, bssid, sizeof(bssid));
                    memcpy(bssid, data, sizeof(bssid));
                    set_config_param_blob("old_bssid", old_bssid, sizeof(old_bssid));
                    set_config_param_blob("bssid", bssid, sizeof(bssid));
                    xTaskCreate(wifi_disconnect, "wifi_dc", 1 * 1024, NULL, 5, NULL);
                }
                break;
            }
            case 18: {
                coap_log(LOG_NOTICE, "Receiving block size control put!\n");

                if (size != 4) {
                    coap_log(LOG_NOTICE, "Invalid pdu size! It's not 4, its %d\n", size);
                } else {
                    // Set new bssid
                    memcpy(&block_size, data, sizeof(block_size));
                    set_config_param_int("block_size", block_size);
                }

                break;
            }
            case 19: {
                coap_log(LOG_NOTICE, "Receiving block seq len control put!\n");

                if (size != 4) {
                    coap_log(LOG_NOTICE, "Invalid pdu size! It's not 4, its %d\n", size);
                } else {
                    // Set new bssid
                    memcpy(&block_seq_len, data, sizeof(block_seq_len));
                    set_config_param_int("block_seq_len", block_seq_len);
                }

                break;
            }
            default: {
                coap_log(LOG_NOTICE, "Invalid control opt value!\n");
                response->code = COAP_RESPONSE_CODE(404);
            }
        }
    }
    coap_check_execute_broadcast(request);
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

        pdu = coap_new_request(
            ctx, session, COAP_REQUEST_PUT, NULL, COAP_MESSAGE_NON, NULL,
            0); /* first, create bare PDU w/o any option  */

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

        pdu = coap_new_request(ctx, session, COAP_REQUEST_PUT, NULL, COAP_MESSAGE_NON, NULL, 0);

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

void prepare_image_session(coap_session_t **session, int64_t *tick) {
    coap_address_t dst_addr;
    coap_address_init(&dst_addr);
    dst_addr.addr.sin.sin_family = AF_INET;
    dst_addr.addr.sin.sin_port = htons(COAP_DEFAULT_PORT);
    dst_addr.addr.sin.sin_addr.s_addr = local_server_ip;

    *session = coap_new_client_session(
        ctx, NULL, &dst_addr, COAP_PROTO_UDP);
    if (!*session) {
        coap_log(LOG_NOTICE, "coap_new_client_session() failed\n");
        coap_session_release(*session);
    }
    *tick = esp_timer_get_time();
}

void send_image(coap_session_t *session, int64_t *tick) {
    if (!image_resp_wait && esp_timer_get_time() - *tick > 0.1 * data_send_period * 1000000) {
        image_resp_wait = 1;

        coap_address_t dst_addr;
        coap_address_init(&dst_addr);
        dst_addr.addr.sin.sin_family = AF_INET;
        dst_addr.addr.sin.sin_port = htons(COAP_DEFAULT_PORT);
        dst_addr.addr.sin.sin_addr.s_addr = local_server_ip;

        if (DATA_COLLECTION_RETRANSMISSION_TIMEOUT_SEC == 0 && DATA_COLLECTION_RETRANSMISSION_TIMEOUT_FRAC == 0) {
            coap_fixed_point_t fixed_point;
            fixed_point.integer_part = 1;
            fixed_point.fractional_part = 0;
            coap_session_set_ack_timeout(session, fixed_point);
        } else {
            coap_fixed_point_t fixed_point;
            fixed_point.integer_part = DATA_COLLECTION_RETRANSMISSION_TIMEOUT_SEC;
            fixed_point.fractional_part = DATA_COLLECTION_RETRANSMISSION_TIMEOUT_FRAC;
            coap_session_set_ack_timeout(session, fixed_point);
        }

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

        if (DYNAMIC_TYPE == DYNAMIC_PER_IMAGE) {
            printf("setting dynamic value\n");
            change_dynamic_parameter(data_collection_count, DYNAMIC_VALUE);
        }

        coap_insert_optlist(&optlist, coap_new_optlist(COAP_OPTION_URI_PATH, 5, (uint8_t *)image_path));

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

        block.num = 0;
        block.szx = block_size;
        block.m =
            ((1ull << (block.szx + 4)) < payload.length);

        send_duration = esp_timer_get_time();
        coap_log(LOG_NOTICE, "Start sending image, start tick %lld\n", send_duration);

        count_total_block++;
        request = coap_new_request(ctx, session, COAP_REQUEST_PUT, &optlist,
                                   COAP_MESSAGE_CON, NULL, 0);
#if DATA_COLLECTION_RETRANSMISSION_TIMEOUT_SEC == 0 && DATA_COLLECTION_RETRANSMISSION_TIMEOUT_FRAC == 0
        rtt_micros = esp_timer_get_time();
#endif

        unsigned char buf[4];

        coap_add_option(request, COAP_OPTION_BLOCK1, coap_encode_var_safe(buf, sizeof(buf), (block.num << 4 | block.m << 3 | block.szx)), buf);

        coap_add_option(request, COAP_OPTION_SIZE1, coap_encode_var_safe(buf, sizeof(buf), payload.length),
                        buf);

        coap_add_block(request, payload.length, payload.s, block.num, block.szx);

        coap_log(LOG_NOTICE,
                 "Sending block 1, size %d, msgtype %d, num %d, m %d,"
                 "szx %d\n",
                 request->used_size, request->type, block.num, block.m, block.szx);
        total_payload_sent += request->used_size + 46;
        last_block_num = block.num;

        coap_send(session, request);

    clean_up:
        esp_camera_fb_return(image);
        if (optlist) {
            coap_delete_optlist(optlist);
            optlist = NULL;
        }
    }
}