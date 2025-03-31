#include <stdio.h>
#include <time.h>
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
#include "esp_sleep.h"
#include "DFRobot_LCD.h"
#include "Temp_Sensor.h"

// Distance Sensor
#define TRIGGER_PIN 4
#define ECHO_PIN 5

// Motion Sensor
#define MOTION_PIN 3

// Thresholds
#define THRES_OK 99
#define THRES_STOP 55
#define TIME_TO_SLEEP 10

extern "C" {

void distance_task(void *pvParameter) {
    UBaseType_t uxHighWaterMark;
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printf("%d words leftover in distance_task\n", (int)uxHighWaterMark);

    Temp_Sensor sensor;
    sensor.init();
    DFRobot_LCD disp = DFRobot_LCD(16, 2);
    disp.init();
    disp.setRGB(0, 255, 0);

    // Configure trigger pin as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_PIN) | (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)TRIGGER_PIN, 0);

    // Configure echo pin as input
    io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);

    // Configure motion sensor pin as wakeup source
    gpio_hold_dis((gpio_num_t)MOTION_PIN);
    gpio_deep_sleep_hold_dis();
    esp_deep_sleep_enable_gpio_wakeup(1ULL << MOTION_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);

    // Configuring timer to use with USS readouts
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000,
    };
    gptimer_new_timer(&timer_config, &gptimer);
    gptimer_enable(gptimer);

    // Timer that determines when to go into deep sleep
    gptimer_handle_t sleep_timer = NULL;
    gptimer_new_timer(&timer_config, &sleep_timer);
    gptimer_enable(sleep_timer);
    gptimer_set_raw_count(sleep_timer, 0);
    gptimer_start(sleep_timer);

    int prev_state = -1; // 0 = CONTINUE, 1 = STOP, 2 = BACK UP

    // Variables used to determine change in distance and time for deep sleep
    int curr_distance = -1;
    int prev_distance = -1;
    uint64_t sleep_timer_count = 0;

    while (1) {
        // Set timer to 0
        gptimer_set_raw_count(gptimer, 0);

        // Trigger USS to send out an ultrasonic wave
        gpio_set_level((gpio_num_t)TRIGGER_PIN, 0);
        esp_rom_delay_us(4);
        gpio_set_level((gpio_num_t)TRIGGER_PIN, 1);
        esp_rom_delay_us(10);
        gpio_set_level((gpio_num_t)TRIGGER_PIN, 0);
        
        gptimer_start(gptimer);

        // Getting start time
        while (gpio_get_level((gpio_num_t)ECHO_PIN) == 0) {
        }

        uint64_t start = 0;
        gptimer_get_raw_count(gptimer, &start);
        uint64_t end = 0;

        // Getting receive time
        while (gpio_get_level((gpio_num_t)ECHO_PIN) == 1) {
            gptimer_get_raw_count(gptimer, &end);
        }

        gptimer_stop(gptimer);

        // Calculate distance in feet, taking temperature into account
        double temp = (double)sensor.read_temp_c();
        double c = 331.3 + (0.6 * temp);
        uint64_t pulse_duration = end - start;
        double distance = ((double)pulse_duration / 1e4) * c / 2;
        double feet = distance / 30.48;

        curr_distance = (int)feet;
        gptimer_get_raw_count(sleep_timer, &sleep_timer_count);
        double time_since_change = sleep_timer_count / 1e6;

        // If distance changes, reset sleep timer
        // If distance hasn't changed for set amount of time, go into deep sleep
        if (prev_distance != curr_distance) {
            prev_distance = curr_distance;
            gptimer_set_raw_count(sleep_timer, 0);
        } else {
            if (time_since_change >= (double)TIME_TO_SLEEP) {
                disp.noDisplay();
                disp.setRGB(0, 0, 0);
                gpio_hold_en((gpio_num_t)MOTION_PIN);
                gpio_deep_sleep_hold_en();
                esp_deep_sleep_start();
            }
        }

        // Determine by distance if we need to continue, stop, or back up
        if (distance > THRES_OK && prev_state != 0) {
            disp.setRGB(0, 255, 0);
            disp.noBlinkLED();
            disp.setCursor(0, 0);
            disp.printstr("                ");
            disp.setCursor(4, 0);
            disp.printstr("CONTINUE");
            prev_state = 0;
        } else if (distance <= THRES_OK && distance > THRES_STOP && prev_state != 1) {
            disp.setRGB(255, 0, 0);
            disp.noBlinkLED();
            disp.setCursor(0, 0);
            disp.printstr("                ");
            disp.setCursor(6, 0);
            disp.printstr("STOP");
            prev_state = 1;
        } else if (distance <= THRES_STOP && prev_state != 2) {
            disp.setRGB(255,90, 0);
            disp.blinkLED();
            disp.setCursor(0, 0);
            disp.printstr("                ");
            disp.setCursor(4, 0);
            disp.printstr("BACK UP");
            prev_state = 2;
        }

        // Always update displayed distance in feet
        disp.setCursor(0, 1);
        disp.printstr("                ");
        disp.setCursor(5, 1);
        std::stringstream temp_stream;
        temp_stream << std::fixed << std::setprecision(1) << feet;
        std::string feet_string = temp_stream.str() + " FT";
        disp.printstr(feet_string.c_str());

        vTaskDelay(200 / portTICK_PERIOD_MS);

        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        printf("%d words leftover in distance_task\n", (int)uxHighWaterMark);
    }
}

void app_main() {
    xTaskCreate(&distance_task, "distance_task", 3000, NULL, 5, NULL);
}
}
