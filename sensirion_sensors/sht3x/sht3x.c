/*
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2024 Cédric Toulotte <https://github.com/Pl0uf-tlt>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file sht3x.c
 *
 * ESP-IDF driver for Sensirion SHT3x digital temperature and humidity sensor
 *
 * Copyright (c) 2017 Gunar Schorcht <https://github.com/gschorcht>\n
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2024 Cédric Toulotte <https://github.com/Pl0uf-tlt>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <common_sensirion_sensor.h>
#include "sht3x.h"


static uint8_t SHT3X_READ_STATUS_CMD[2] =               {0xF3, 0x2D};
static uint8_t SHT3X_CLEAR_STATUS_CMD[2] =         {0x30, 0x41};
static uint8_t SHT3X_RESET_CMD[2] =                {0x30, 0xA2};
static uint8_t SHT3X_FETCH_DATA_CMD[2] =           {0xE0, 0x00};
static uint8_t SHT3X_STOP_PERIODIC_MEAS_CMD[2] =   {0x30, 0x93};
static uint8_t SHT3X_HEATER_ON_CMD[2] =            {0x30, 0x6D};
static uint8_t SHT3X_HEATER_OFF_CMD[2] =           {0x30, 0x66};
static uint8_t SHT3X_MEASURE_CMD[6][3][2] = {
        {{0x24, 0x00}, {0x24, 0x0b}, {0x24, 0x16}}, // [SINGLE_SHOT][H,M,L] without clock stretching
        {{0x20, 0x32}, {0x20, 0x24}, {0x20, 0x2f}}, // [PERIODIC_05][H,M,L]
        {{0x21, 0x30}, {0x21, 0x26}, {0x21, 0x2d}}, // [PERIODIC_1 ][H,M,L]
        {{0x22, 0x36}, {0x22, 0x20}, {0x22, 0x2b}}, // [PERIODIC_2 ][H,M,L]
        {{0x23, 0x34}, {0x23, 0x22}, {0x23, 0x29}}, // [PERIODIC_4 ][H,M,L]
        {{0x27, 0x37}, {0x27, 0x21}, {0x27, 0x2a}}  // [PERIODIC_10][H,M,L]
};
static uint8_t SHT3X_MEASURE_CMD_DELAY[3] = {15, 6, 4};

esp_err_t sht3x_init_desc(sht3x_t *dev, i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *sht_handle)
{
    CHECK_ARG(dev && sht_handle && bus_handle);

    dev->bus_mst_handle = bus_handle;
    dev->sht_dev_handle = sht_handle;
    dev->humidity = 0;
    dev->temperature = 0;

    return ESP_OK;
}

esp_err_t sht3x_init(sht3x_t *dev)
{
    CHECK_ARG(dev);

    dev->mode = SHT3X_SINGLE_SHOT;
    return execute_command(dev->sht_dev_handle, SHT3X_CLEAR_STATUS_CMD, 2, 50, NULL, 0);
}

esp_err_t sht3x_start_measurement_single_shot(sht3x_t *dev, sht3x_repeat_t repeat)
{
    CHECK_ARG(dev);

    dev->mode = SHT3X_SINGLE_SHOT;
    dev->repeatability = repeat;
    
    CHECK(execute_command(dev->sht_dev_handle, SHT3X_MEASURE_CMD[SHT3X_SINGLE_SHOT][repeat], 2, 20 + SHT3X_MEASURE_CMD_DELAY[repeat], NULL, 0));
    
    CHECK(sht3x_fetch_data(dev));
    
    return ESP_OK;
}

esp_err_t sht3x_start_periodic_measurement(sht3x_t *dev, sht3x_mode_t mode, sht3x_repeat_t repeat)
{
    CHECK_ARG(dev && repeat && mode);

    dev->mode = mode;
    dev->repeatability = repeat;

    CHECK(execute_command(dev->sht_dev_handle, SHT3X_MEASURE_CMD[mode][repeat], 2, 20 +  SHT3X_MEASURE_CMD_DELAY[repeat], NULL, 0));

    dev->measurement_started = true;
    
    return ESP_OK;
}

esp_err_t sht3x_fetch_data(sht3x_t *dev)
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sht_dev_handle, SHT3X_FETCH_DATA_CMD, 2, 30,  dev->raw_data, sizeof(sht3x_raw_data_t)));

    return ESP_OK;
}

esp_err_t sht3x_set_heater(sht3x_t *dev, bool enable)
{
    CHECK_ARG(dev);

    return execute_command(dev->sht_dev_handle, enable ? SHT3X_HEATER_ON_CMD : SHT3X_HEATER_OFF_CMD, 2, 30, NULL, 0);
}

esp_err_t sht3x_compute_values(sht3x_raw_data_t raw_data, float *temperature, float *humidity)
{
    CHECK_ARG(raw_data);

    *temperature = ((((raw_data[0] * 256.0) + raw_data[1]) * 175) / 65535.0) - 45;

    *humidity = ((((raw_data[3] * 256.0) + raw_data[4]) * 100) / 65535.0);
    
    return ESP_OK;
}

esp_err_t sht3x_measure(sht3x_t *dev)
{
    CHECK_ARG(dev);

    CHECK(sht3x_start_measurement_single_shot(dev, SHT3X_HIGH));

    CHECK(sht3x_compute_values(dev->raw_data, &dev->temperature, &dev->humidity));

    return ESP_OK;
}

esp_err_t sht3x_stop_periodic_measurement(sht3x_t *dev)
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sht_dev_handle, SHT3X_STOP_PERIODIC_MEAS_CMD, 2, 30, NULL, 0));

    dev->mode = SHT3X_SINGLE_SHOT;
    dev->measurement_started = false;

    return ESP_OK;
}

esp_err_t sht3x_get_results(sht3x_t *dev)
{
    CHECK_ARG(dev);

    CHECK(sht3x_fetch_data(dev));

    CHECK(sht3x_compute_values(dev->raw_data, &dev->temperature, &dev->humidity));

    return ESP_OK;
}

esp_err_t sht3x_reset(sht3x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sht_dev_handle, SHT3X_RESET_CMD, 2, 50, NULL, 0));

    dev->measurement_started = false;
    dev->mode = SHT3X_SINGLE_SHOT;
    dev->repeatability = SHT3X_HIGH;

    return ESP_OK;
}

esp_err_t sht3x_read_status(sht3x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sht_dev_handle, SHT3X_READ_STATUS_CMD, 2, 50, dev->status, sizeof(sht3x_status_t)));

    return ESP_OK;
}


