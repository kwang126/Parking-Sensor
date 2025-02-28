#include <stdio.h>
#include <string>
#include <sstream>
#include <iomanip>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "soc/clk_tree_defs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "DFRobot_LCD.h"

// I2C Configuration
#define I2C_MASTER_SCL_IO 8        // GPIO for I2C SCL
#define I2C_MASTER_SDA_IO 10        // GPIO for I2C SDA
#define I2C_MASTER_NUM I2C_NUM_0    // I2C port number
#define I2C_MASTER_FREQ_HZ 100000   // I2C master clock frequency
#define CMD_MEASURE 0x7CA2  // Measure Command

// Sensor I2C Address
#define SENSOR_I2C_ADDRESS 0x70

// Distance Sensor
#define TRIGGER_PIN 4
#define ECHO_PIN 5

// Thresholds
#define THRES_OK 99
#define THRES_STOP 55

extern "C" {

void i2c_master_init() {
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


esp_err_t i2c_master_read_sensor(uint16_t mes_cmd, uint8_t *data, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, mes_cmd >> 8, true);
    i2c_master_write_byte(cmd, mes_cmd & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main() {
    i2c_master_init();
    DFRobot_LCD disp = DFRobot_LCD(16, 2);
    disp.init();
    disp.setRGB(0, 255, 0);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_PIN) | (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    gpio_set_level((gpio_num_t)TRIGGER_PIN, 0);

    io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000,
    };
    gptimer_new_timer(&timer_config, &gptimer);
    gptimer_enable(gptimer);

    int prev_state = -1; // 0 = CONTINUE, 1 = OK, 2 = STOP 

    while (1) {
        gptimer_set_raw_count(gptimer, 0);

        gpio_set_level((gpio_num_t)TRIGGER_PIN, 0);
        esp_rom_delay_us(4);
        gpio_set_level((gpio_num_t)TRIGGER_PIN, 1);
        esp_rom_delay_us(10);
        gpio_set_level((gpio_num_t)TRIGGER_PIN, 0);
        
        gptimer_start(gptimer);

        while (gpio_get_level((gpio_num_t)ECHO_PIN) == 0) {
        }

        uint64_t start = 0;
        gptimer_get_raw_count(gptimer, &start);
        uint64_t end = 0;

        while (gpio_get_level((gpio_num_t)ECHO_PIN) == 1) {
            gptimer_get_raw_count(gptimer, &end);
        }
        
        gptimer_stop(gptimer);

        double temp = 0;
        uint8_t sensor_data[2] = {0,};
        esp_err_t err = i2c_master_read_sensor(CMD_MEASURE, sensor_data, 2);
        
        if (err == ESP_OK) {
            uint16_t temp_raw = (sensor_data[0] << 8) | sensor_data[1];

            temp = 175.0 * ((double)temp_raw / 65536.0) - 45.0;
        } else {
            printf("Failed to read sensor data\n");
        }

        double c = 331.3 + (0.6 * temp);
        uint64_t pulse_duration = end - start;
        double distance = ((double)pulse_duration / 1e4) * c / 2;
        double feet = distance / 30.48;
        printf("Distance: %.1f cm at %.1fC\n", distance, temp);

        if (distance > THRES_OK && prev_state != 0) {
            disp.setRGB(0, 255, 0);
            disp.setCursor(0, 0);
            disp.printstr("                ");
            disp.setCursor(4, 0);
            disp.printstr("CONTINUE");
            prev_state = 0;
            printf("CONTINUE\n");
        } else if (distance <= THRES_OK && distance > THRES_STOP && prev_state != 1) {
            disp.setRGB(255, 255, 0);
            disp.setCursor(0, 0);
            disp.printstr("                ");
            disp.setCursor(7, 0);
            disp.printstr("OK");
            prev_state = 1;
            printf("OK\n");
        } else if (distance <= THRES_STOP && prev_state != 2) {
            disp.setRGB(255, 0, 0);
            disp.setCursor(0, 0);
            disp.printstr("                ");
            disp.setCursor(6, 0);
            disp.printstr("STOP");
            prev_state = 2;
            printf("STOP\n");
        }

        disp.setCursor(0, 1);
        disp.printstr("                ");
        disp.setCursor(5, 1);
        std::stringstream temp_stream;
        temp_stream << std::fixed << std::setprecision(1) << feet;
        std::string feet_string = temp_stream.str() + " FT";
        disp.printstr(feet_string.c_str());

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
}
