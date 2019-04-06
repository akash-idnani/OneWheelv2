#ifndef ONEWHEEL_BLUETOOTH_H
#define ONEWHEEL_BLUETOOTH_H

#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "One Wheel"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX. 
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

enum {
    IDX_ANGLE_SVC,
    IDX_CHAR_ANGLES,
    IDX_CHAR_VAL_ANGLES,

    HRS_IDX_NB,
};

void init_ble();
void send_euler(long* euler);

#endif