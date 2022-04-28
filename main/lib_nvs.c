#include "lib_nvs.h"
#include "esp32_node.h"

#include "esp_log.h"
#include "nvs.h"

const char* TAG = "lib_nvs";

esp_err_t get_config_param_unsigned_char(char* name, unsigned char* param) {
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        if ((err = nvs_get_u8(nvs, name, param)) == ESP_OK) {
            ESP_LOGI(TAG, "%s %d", name, *param);
        } else {
            return err;
        }
        nvs_close(nvs);
    } else {
        return err;
    }
    return ESP_OK;
}

esp_err_t get_config_param_char(char* name, signed char* param) {
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        if ((err = nvs_get_i8(nvs, name, param)) == ESP_OK) {
            ESP_LOGI(TAG, "%s %d", name, *param);
        } else {
            return err;
        }
        nvs_close(nvs);
    } else {
        return err;
    }
    return ESP_OK;
}

esp_err_t get_config_param_int(char* name, int* param) {
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        if ((err = nvs_get_i32(nvs, name, param)) == ESP_OK) {
            ESP_LOGI(TAG, "%s %d", name, *param);
        } else {
            return err;
        }
        nvs_close(nvs);
    } else {
        return err;
    }
    return ESP_OK;
}

esp_err_t get_config_param_str(char* name, char** param) {
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        size_t len;
        if ((err = nvs_get_str(nvs, name, NULL, &len)) == ESP_OK) {
            *param = (char*)malloc(len);
            err = nvs_get_str(nvs, name, *param, &len);
            ESP_LOGI(TAG, "%s %s", name, *param);
        } else {
            return err;
        }
        nvs_close(nvs);
    } else {
        return err;
    }
    return ESP_OK;
}

esp_err_t get_config_param_blob(char* name, void* param, size_t* length) {
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        if ((err = nvs_get_blob(nvs, name, param, length)) == ESP_OK) {
            ESP_LOGI(TAG, "%s", name);
        } else {
            return err;
        }
        nvs_close(nvs);
    } else {
        return err;
    }
    return ESP_OK;
}

esp_err_t set_config_param_unsigned_char(char* name, unsigned char param) {
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        if ((err = nvs_set_u8(nvs, name, param)) == ESP_OK) {
            ESP_LOGI(TAG, "%s %d", name, param);
        } else {
            return err;
        }
        nvs_close(nvs);
    } else {
        return err;
    }
    return ESP_OK;
}

esp_err_t set_config_param_char(char* name, signed char param) {
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        if ((err = nvs_set_i8(nvs, name, param)) == ESP_OK) {
            ESP_LOGI(TAG, "%s %d", name, param);
        } else {
            return err;
        }
        nvs_close(nvs);
    } else {
        return err;
    }
    return ESP_OK;
}

esp_err_t set_config_param_int(char* name, int param) {
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        if ((err = nvs_set_i32(nvs, name, param)) == ESP_OK) {
            ESP_LOGI(TAG, "%s %d", name, param);
        } else {
            return err;
        }
        nvs_close(nvs);
    } else {
        return err;
    }
    return ESP_OK;
}

esp_err_t set_config_param_str(char* name, char* param) {
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        size_t len;
        if ((err = nvs_set_str(nvs, name, param)) == ESP_OK) {
            ESP_LOGI(TAG, "%s %s", name, param);
        } else {
            return err;
        }
        nvs_close(nvs);
    } else {
        return err;
    }
    return ESP_OK;
}

esp_err_t set_config_param_blob(char* name, void* param, size_t length) {
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        size_t len;
        if ((err = nvs_set_blob(nvs, name, param, length)) == ESP_OK) {
            ESP_LOGI(TAG, "%s", name);
        } else {
            return err;
        }
        nvs_close(nvs);
    } else {
        return err;
    }
    return ESP_OK;
}