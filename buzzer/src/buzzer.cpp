/**
 ******************************************************************************
 * @file        : buzzer.c
 * @brief       : Buzzer driver
 * @author      : Jacques Supcik <jacques@supcik.net>
 * @date        : 11 March 2024
 ******************************************************************************
 * @copyright   : Copyright (c) 2024 Jacques Supcik
 * @attention   : SPDX-License-Identifier: MIT
 ******************************************************************************
 * @details
 * Buzzer driver for ESP32. The driver uses the LEDC peripheral to generate
 * the sound. The driver is non-blocking and uses a FreeRTOS task to generate
 * the sound at the requested frequency and duration.
 ******************************************************************************
 */

#include "buzzer.hpp"

#include <inttypes.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char* kTag = "buzzer";

static const uint32_t kQueueSize = 10;
static const uint32_t kStackSize = 2048;

void Buzzer::Stop() {
    BaseType_t err = ledc_set_duty(speed_mode_, channel_, 0);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to set duty cycle to zero");
    }
    err = ledc_update_duty(speed_mode_, channel_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to update duty cycle");
    }
    err = ledc_stop(speed_mode_, channel_, idle_level_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to stop buzzer");
    }
}

void Buzzer::Start(uint32_t frequency) {
    uint32_t duty = (1 << timer_bit_) / 2;

    esp_err_t err = ledc_set_duty(speed_mode_, channel_, duty);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to set duty cycle");
        return;
    }
    err = ledc_update_duty(speed_mode_, channel_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to update duty cycle");
        return;
    }
    err = ledc_set_freq(speed_mode_, timer_num_, frequency);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to set frequency");
        return;
    }
}

void Buzzer::Task() {
    message_t message;
    TickType_t waitTime = portMAX_DELAY;

    while (1) {
        BaseType_t res = xQueueReceive(queue_handle_, &message, waitTime);
        if (res != pdPASS) {
            ESP_LOGD(kTag, "No new message, Stop beeping");
            Stop();
            waitTime = portMAX_DELAY;
        } else {
            ESP_LOGD(kTag,
                     "Beep at %" PRIu32 " Hz for %" PRIu32 " ms",
                     message.frequency,
                     message.duration_ms);
            Start(message.frequency);
            waitTime = pdMS_TO_TICKS(message.duration_ms);
        }
    }
}

Buzzer::Buzzer(const gpio_num_t gpio_num,
               ledc_mode_t speed_mode,
               ledc_timer_bit_t timer_bit,
               ledc_timer_t timer_num,
               ledc_channel_t channel,
               uint32_t idle_level)
    : speed_mode_(speed_mode),
      timer_bit_(timer_bit),
      timer_num_(timer_num),
      channel_(channel),
      idle_level_(idle_level) {
    // Configure Timer
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = speed_mode_;
    ledc_timer.timer_num = timer_num_;
    ledc_timer.duty_resolution = timer_bit_;
    ledc_timer.freq_hz = 4000;  // Can be any value
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure Channel
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode = speed_mode_;
    ledc_channel.channel = channel_;
    ledc_channel.timer_sel = timer_num_;
    ledc_channel.gpio_num = gpio_num;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_ERROR_CHECK(ledc_stop(speed_mode_, channel_, idle_level_));

    queue_handle_ = xQueueCreate(kQueueSize, sizeof(message_t));
    BaseType_t res = xTaskCreate(
        TaskForwarder, "buzzer_task", kStackSize, this, uxTaskPriorityGet(nullptr), &task_handle_);

    assert(res == pdPASS);
}

Buzzer::~Buzzer() {
    vQueueDelete(queue_handle_);
    vTaskDelete(task_handle_);
}

BaseType_t Buzzer::Beep(uint32_t frequency, uint32_t duration_ms) {
    message_t message = {
        .frequency = frequency,
        .duration_ms = duration_ms,
    };
    return xQueueSend(queue_handle_, &message, 0);
}
