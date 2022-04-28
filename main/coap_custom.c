#include "coap_custom.h"
#include "esp_log.h"

// #include "encode.h"
const static char* TAG = "coap_custom.h";

unsigned int coap_opt_block_mod_num(const coap_opt_t* block_opt) {
    unsigned int num = 0;
    uint16_t len;

    len = coap_opt_length(block_opt);

    if (len == 0) {
        return 0;
    }

    if (len == 1) {
        return 0;
    }

    if (len > 1) {
        num = coap_decode_var_bytes(coap_opt_value(block_opt),
                                    coap_opt_length(block_opt));
    }

    num >>= 8;

    return num;
}

int coap_add_block_mod(coap_pdu_t* pdu, unsigned int len, const uint8_t* data,
                       unsigned int block_num, unsigned char block_szx) {
    unsigned int start;
    start = block_num << (block_szx + 4);

    if (len <= start) return 0;

    return coap_add_data(pdu, min(len - start, (1U << (block_szx + 4))),
                         data + start);
}

int coap_get_missing_block_mod(coap_pdu_t* pdu, uint16_t type,
                               coap_block_missing_t* missing_block,
                               uint32_t* last_block_mod_sent) {
    coap_opt_iterator_t opt_iter;
    coap_opt_t* option;

    assert(missing_block);
    memset(missing_block, 0, sizeof(coap_block_missing_t));

    if (pdu && (option = coap_check_option(pdu, type, &opt_iter)) != NULL) {
        missing_block->missing_block_len = COAP_OPT_BLOCK_MISSING_TOTAL(option);

        uint16_t missing_block_list = (uint16_t)COAP_OPT_BLOCK_MISSING_LIST(option);
        // ESP_LOGI(TAG, "Missing block list %02X", missing_block_list);

        uint32_t missing_block_index = 0;

        for (int i = 0; i < 16; i++) {
            if (missing_block_list >> i & 0x01) {
                missing_block->missing_block[missing_block_index] =
                    last_block_mod_sent[i];
                // printf("%d,", i);
                // ESP_LOGI(TAG, "Missing block num is %d",
                //          missing_block->missing_block[missing_block_index]);
                missing_block_index++;
            }
        }
        // printf("\n");

        return 1;
    }

    return 0;
}

unsigned int coap_get_remaining_payload_number(unsigned int payload_length,
                                               unsigned int block_num,
                                               unsigned int block_szx) {
    if (payload_length > ((block_num + 1) << (block_szx + 4)))
        // ((length - sended) / size) + last
        return ((payload_length - ((block_num + 1) << (block_szx + 4))) /
                (1 << (block_szx + 4))) +
               (payload_length % (1 << (block_szx + 4)) > 0);
    else
        return 0;
}

uint8_t coap_get_broadcast_option(coap_pdu_t* pdu) {
    coap_opt_iterator_t opt_iter;
    coap_opt_t* option;
    if (pdu && (option = coap_check_option(pdu, COAP_OPTION_BROADCAST, &opt_iter)) != NULL) {
        if (coap_opt_length(option) > 1) {
            ESP_LOGE(TAG, "Invalid broadcast option length!");
            return 0;
        } else if (coap_opt_length(option) == 1) {
            return 1;
        } else {
            ESP_LOGE(TAG, "No length broadcast option!");
            return 0;
        }
    } else {
        return 0;
    }
}