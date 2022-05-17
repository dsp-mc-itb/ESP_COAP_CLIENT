#ifndef _COAP_CUSTOM_H_
#define _COAP_CUSTOM_H_

#if 1
/* Needed until coap_dtls.h becomes a part of libcoap proper */
//#include "libcoap.h"
//#include "coap_dtls.h"
#include "freertos/FreeRTOS.h"
#endif

// #include "coap.h"
#include "coap3/coap.h"

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#define ACK_TIMEOUT 1 * 1000

#define COAP_OPTION_BLOCK_MOD 65000
#define COAP_OPTION_SIZE_MOD 65001
#define COAP_OPTION_MISSING_BLOCK_MOD 65002
#define COAP_OPTION_REQUEST_TAG 65003
#define COAP_OPTION_BROADCAST 65004
#define COAP_OPTION_CONTROL 65005

#define COAP_OPT_BLOCK_MOD_LAST(opt)                                         \
  (coap_opt_length(opt) ? (coap_opt_value(opt) + (coap_opt_length(opt) - 1)) \
                        : 0)

/** Returns the value of the More-bit of a Block option @p opt. */
#define COAP_OPT_BLOCK_MOD_MORE(opt) \
  (coap_opt_length(opt) ? (*COAP_OPT_BLOCK_MOD_LAST(opt) & 0x80) >> 7 : 0)

/** Returns the value of the SZX-field of a Block option @p opt. */
#define COAP_OPT_BLOCK_MOD_SZX(opt) \
  (coap_opt_length(opt) ? (*COAP_OPT_BLOCK_MOD_LAST(opt) & 0x70) >> 4 : 0)

/** Returns the value of the SZX-field of a Block option @p opt. */
#define COAP_OPT_BLOCK_MOD_PART_LEN(opt) \
  (coap_opt_length(opt) ? (*COAP_OPT_BLOCK_MOD_LAST(opt) & 0x0F) : 0)

#define COAP_OPT_BLOCK_MISSING_TOTAL(opt)                                     \
  (coap_opt_length(opt) ? *(coap_opt_value(opt) + (coap_opt_length(opt) - 3)) \
                        : 0)

#define COAP_OPT_BLOCK_MISSING_LIST(opt)                             \
  (coap_opt_length(opt)                                              \
       ? (*(coap_opt_value(opt) + (coap_opt_length(opt) - 2)) << 8 | \
          *(coap_opt_value(opt) + (coap_opt_length(opt) - 1)))       \
       : 0)

typedef struct {
  unsigned int num;     /**< block number */
  unsigned int m : 1;   /**< 1 if more blocks follow, 0 otherwise */
  unsigned int szx : 3; /**< block size */
  unsigned int part_len : 10;
} coap_block_mod_t;

typedef struct {
  unsigned int missing_block[16];
  unsigned int missing_block_len;
} coap_block_missing_t;

typedef struct {
  unsigned int num[16];
  unsigned int part_len;
} coap_block_part_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Returns the value of field @c num in the given block option @p block_opt.
 */
unsigned int coap_opt_block_mod_num(const coap_opt_t* block_opt);

/**
 * Adds the @p block_num block of size 1 << (@p block_szx + 4) from source @p
 * data to @p pdu.
 *
 * @param pdu       The message to add the block.
 * @param len       The length of @p data.
 * @param data      The source data to fill the block with.
 * @param block_num The actual block number.
 * @param block_szx Encoded size of block @p block_number.
 *
 * @return          @c 1 on success, @c 0 otherwise.
 */
int coap_add_block_mod(coap_pdu_t* pdu, unsigned int len, const uint8_t* data,
                       unsigned int block_num, unsigned char block_szx);

int coap_get_missing_block_mod(coap_pdu_t* pdu, uint16_t type,
                               coap_block_missing_t* missing_block,
                               uint32_t* last_block_mod_sent);

unsigned int coap_get_remaining_payload_number(unsigned int payload_length,
                                               unsigned int block_num,
                                               unsigned int block_szx);

uint8_t coap_get_broadcast_option(coap_pdu_t* pdu);

#ifdef __cplusplus
}
#endif

#endif