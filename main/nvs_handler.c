#include "nvs.h"
#include "nvs_flash.h"

void read_nvs(uint16_t* prop, uint16_t* integral, uint16_t* deriv) {
    nvs_handle nvs_h;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READONLY, &nvs_h));
    ESP_ERROR_CHECK(nvs_get_u16(nvs_h, "Kp", prop));
    ESP_ERROR_CHECK(nvs_get_u16(nvs_h, "Ki", integral));
    ESP_ERROR_CHECK(nvs_get_u16(nvs_h, "Kd", deriv));
    nvs_close(nvs_h);
}

void set_nvs(uint16_t prop, uint16_t integral, uint16_t deriv) {
    nvs_handle nvs_h;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_h));
    ESP_ERROR_CHECK(nvs_set_u16(nvs_h, "Kp", prop));
    ESP_ERROR_CHECK(nvs_set_u16(nvs_h, "Ki", integral));
    ESP_ERROR_CHECK(nvs_set_u16(nvs_h, "Kd", deriv));

    ESP_ERROR_CHECK(nvs_commit(nvs_h));
    nvs_close(nvs_h);
}

void reset_nvs() {
    set_nvs(0, 0, 0);
}