#include <cstdint>

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "Temp_Sensor.h"

// I2C Config
#define I2C_MASTER_SCL_IO 8     // GPIO for I2C SCL
#define I2C_MASTER_SDA_IO 10    // GPIO for I2C SDA
#define I2C_MASTER_NUM I2C_NUM_0    // I2C port num
#define I2C_MASTER_FREQ_HZ 100000   // I2C master clock freq

void Temp_Sensor::init() {
    i2c_config_t conf;
    conf.clk_flags = 0;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

float Temp_Sensor::read_temp_c() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CMD_MEASURE >> 8, true);
    i2c_master_write_byte(cmd, CMD_MEASURE & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

    if (err != ESP_OK) {
        return 65535;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);

    uint8_t data[2] = {0, };
    i2c_cmd_link_delete(cmd);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        return 65535;
    }

    uint16_t temp_raw = (data[0] << 8) | data[1];
    return(175.0 * ((float)temp_raw / 65536.0) - 45.0);
}

float Temp_Sensor::read_temp_f() {
    return ((read_temp_c() * 9.0/5.0) + 32.0);
}

float Temp_Sensor::read_humidity() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CMD_MEASURE >> 8, true);
    i2c_master_write_byte(cmd, CMD_MEASURE & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

    if (err != ESP_OK) {
        return 65535;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);

    uint8_t data[6] = {0, };
    i2c_cmd_link_delete(cmd);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        return 65535;
    }

    uint16_t hum_raw = (data[3] << 8) | data[4];
    return (100.0 * ((float)hum_raw / 65536.0));
}
