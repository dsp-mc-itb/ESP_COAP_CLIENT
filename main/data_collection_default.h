// NOTES
// NOTES: Put this code into data_collection.h and edit there
// NOTES

#include "sensor.h"

#define SUPPORT_SENSING 1

#define COAP_LOG_DEFAULT_LEVEL 0

// Only for: DATA_COLLECTION_DEFAULT_BLOCK_SIZE, DATA_COLLECTION_DEFAULT_BLOCK_SEQ_LEN, and DATA_COLLECTION_DEFAULT_IMAGE_SIZE
#define DYNAMIC_PARAMETER -1

// Only for: DATA_COLLECTION_DEFAULT_PARENT_NODE
#define FLEXIBLE_ADDRESS "00:00:00:00:00:00"

// TOGGLE DATA COLLECTION MODE
#define DATA_COLLECTION_MODE

#ifdef DATA_COLLECTION_MODE

typedef enum dynamic_parameter_type {
    NON_DYNAMIC,
    DYNAMIC_PER_IMAGE,
    DYNAMIC_PER_BLOCK
} dynamic_parameter_type;

#define SCENARIO_ID "N-036"
#define DATA_COLLECTION_DEFAULT_STATUS_DATA_METHOD 0
#define DATA_COLLECTION_DEFAULT_IS_SENSING 1
// #define DATA_COLLECTION_DEFAULT_PARENT_NODE "3C:84:6A:D2:AC:44"
#define DATA_COLLECTION_DEFAULT_PARENT_NODE FLEXIBLE_ADDRESS
#define DATA_COLLECTION_DEFAULT_BLOCK_SIZE 6
#define DATA_COLLECTION_DEFAULT_BLOCK_SEQ_LEN -1
#define DATA_COLLECTION_DEFAULT_IMAGE_SIZE FRAMESIZE_HD
// #define DATA_COLLECTION_DEFAULT_BLOCK_SIZE DYNAMIC_PARAMETER
// #define DATA_COLLECTION_DEFAULT_BLOCK_SEQ_LEN DYNAMIC_PARAMETER
// #define DATA_COLLECTION_DEFAULT_IMAGE_SIZE DYNAMIC_PARAMETER
#define DATA_COLLECTION_DEFAULT_IMAGE_FORMAT PIXFORMAT_JPEG
#define DATA_COLLECTION_DEFAULT_DATA_SEND_PERIOD 5
#define DATA_COLLECTION_RETRANSMISSION_TIMEOUT_SEC 0
#define DATA_COLLECTION_RETRANSMISSION_TIMEOUT_FRAC 0

#define DYNAMIC_TYPE NON_DYNAMIC
#define DYNAMIC_VALUE 1

#define MULTIPLIER_BLOCK_SIZE 7
#define MULTIPLIER_BLOCK_PART_LEN 15
#define MULTIPLIER_IMAGE_SIZE 9 // until framesize VGA

// Example for retransmission timeout:
// DATA_COLLECTION_RETRANSMISSION_TIMEOUT_SEC = 1
// DATA_COLLECTION_RETRANSMISSION_TIMEOUT_FRAC = 500
// Then the value of ack_timeout is 1.5 seconds except if both value is 0 then the ack_timeout is RTT

#endif