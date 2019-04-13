#include "mpu_reader.h"
#include "motiondriver_defs.h"

#include "driver/i2c.h"

#include <inv_mpu_dmp_motion_driver.h>
#include <inv_mpu.h>
#include <mlmath.h>
#include <ml_math_func.h>

static struct platform_data_s {
    signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};

static esp_err_t init_i2c_master() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = 22;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &conf);
    return i2c_driver_install(I2C_NUM_0, conf.mode,
                              I2C_BUFFER_SIZE,
                              I2C_BUFFER_SIZE, 0);
}

static void get_euler_from_quat(long *quat, long *data) {
    long t1, t2, t3;
    long q00, q01, q02, q03, q11, q12, q13, q22, q23, q33;
    float values[3];

    q00 = inv_q29_mult(quat[0], quat[0]);
    q01 = inv_q29_mult(quat[0], quat[1]);
    q02 = inv_q29_mult(quat[0], quat[2]);
    q03 = inv_q29_mult(quat[0], quat[3]);
    q11 = inv_q29_mult(quat[1], quat[1]);
    q12 = inv_q29_mult(quat[1], quat[2]);
    q13 = inv_q29_mult(quat[1], quat[3]);
    q22 = inv_q29_mult(quat[2], quat[2]);
    q23 = inv_q29_mult(quat[2], quat[3]);
    q33 = inv_q29_mult(quat[3], quat[3]);

    /* X component of the Ybody axis in World frame */
    t1 = q12 - q03;

    /* Y component of the Ybody axis in World frame */
    t2 = q22 + q00 - (1L << 30);
    values[2] = -atan2f((float) t1, (float) t2) * 180.f / (float) M_PI;

    /* Z component of the Ybody axis in World frame */
    t3 = q23 + q01;
    values[0] =
        atan2f((float) t3,
                sqrtf((float) t1 * t1 +
                      (float) t2 * t2)) * 180.f / (float) M_PI;
    /* Z component of the Zbody axis in World frame */
    t2 = q33 + q00 - (1L << 30);
    if (t2 < 0) {
        if (values[0] >= 0)
            values[0] = 180.f - values[0];
        else
            values[0] = -180.f - values[0];
    }

    /* X component of the Xbody axis in World frame */
    t1 = q11 + q00 - (1L << 30);
    /* Y component of the Xbody axis in World frame */
    t2 = q12 + q03;
    /* Z component of the Xbody axis in World frame */
    t3 = q13 - q02;

    values[1] =
        (atan2f((float)(q33 + q00 - (1L << 30)), (float)(q13 - q02)) *
          180.f / (float) M_PI - 90);
    if (values[1] >= 90)
        values[1] = 180 - values[1];

    if (values[1] < -90)
        values[1] = -180 - values[1];
    data[0] = (long)(values[0] * 65536.f);
    data[1] = (long)(values[1] * 65536.f);
    data[2] = (long)(values[2] * 65536.f);
}

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, 
        unsigned char length, unsigned char const *data) {

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1, I2C_ACK_EN);
    i2c_master_write_byte(cmd, reg_addr, I2C_ACK_EN);

    unsigned char to_write[length];
    for (int i = 0; i < length ; i++) {
        to_write[i] = data[i];
    }

    if (length > 0) i2c_master_write(cmd, to_write, length, I2C_ACK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 500 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr,
        unsigned char length, unsigned char *data) {
    if (length == 0) return ESP_OK;

    esp_err_t ret;

    i2c_write(slave_addr, reg_addr, 0, NULL);
    
    unsigned char read_addr = (slave_addr << 1) | 1;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    ret = i2c_master_write_byte(cmd, read_addr, I2C_ACK_EN);

    if (length > 1) {
        ret = i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    ret = i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    ret = i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void init_mpu() {
    init_i2c_master();

    inv_error_t result;

    struct int_param_s int_param;
    result = mpu_init(&int_param);
    if (result) {
        printf("Could not initialize gyro.\n");
    }

    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));   
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1); 
}

void euler_reader(void* pvParameters) {
    void (*callback) (long*) = pvParameters;

    init_mpu();

    short gyro[3], accel_short[3], sensors;
    unsigned char more;
    long quat[4], euler[3];
    unsigned long sensor_timestamp;

    while (1) {

        dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
        get_euler_from_quat(quat, euler);
        (*callback)(euler);
        delay_ms(5);
    }
}