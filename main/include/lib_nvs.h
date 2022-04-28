#ifndef _LIB_NVS_H_
#define _LIB_NVS_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t get_config_param_unsigned_char(char* name, unsigned char* param);
esp_err_t get_config_param_char(char* name, signed char* param);
esp_err_t get_config_param_int(char* name, int* param);
esp_err_t get_config_param_str(char* name, char** param);
esp_err_t get_config_param_blob(char* name, void* param, size_t* length);

esp_err_t set_config_param_unsigned_char(char* name, unsigned char param);
esp_err_t set_config_param_char(char* name, signed char param);
esp_err_t set_config_param_int(char* name, int param);
esp_err_t set_config_param_str(char* name, char* param);
esp_err_t set_config_param_blob(char* name, void* param, size_t length);


#ifdef __cplusplus
}
#endif

#endif