/*
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
 * @file sgp30.c
 *
 * ESP-IDF driver for Sensirion sgp30 air quality sensor 
 *
 * Copyright (c) 2024 Cédric Toulotte <https://github.com/Pl0uf-tlt>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "common_sensirion_sensor.h"
#include "sgp30.h"

static uint8_t SGP30_IAQ_INIT[2] =                    { 0x20, 0x03 };
static uint8_t SGP30_MEASURE_IAQ[2] =                 { 0x20, 0x08 };
static uint8_t SGP30_GET_IAQ_BASELINE[2] =            { 0x20, 0x15 };
static uint8_t SGP30_SET_IAQ_BASELINE[2] =            { 0x20, 0x1E };
static uint8_t SGP30_SET_ABSOLUTE_HUMIDITY[2] =       { 0x20, 0x61 };
static uint8_t SGP30_MEASURE_TEST[2] =                { 0x20, 0x32 };
static uint8_t SGP30_GET_FEATURE_SET[2] =             { 0x20, 0x2F };
static uint8_t SGP30_MEASURE_RAW[2] =                 { 0x20, 0x50 };
static uint8_t SGP30_GET_TVOC_INCEPTIVE_BASELINE[2] = { 0x20, 0xB3 };
static uint8_t SGP30_SET_TVOC_INCEPTIVE_BASELINE[2] = { 0x20, 0x77 };
static uint8_t SGP30_GET_SERIAL_ID[2] =               { 0x36, 0x82 };

esp_err_t sgp30_measure(sgp30_t *dev)
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sgp30_dev_handle, SGP30_MEASURE_IAQ, 2, 20, dev->data, sizeof(sgp30_data_t)));
    CHECK(sgp30_compute_values(dev->data, &(dev->tvoc), &(dev->co2)));

    return ESP_OK;
}

esp_err_t sgp30_init_desc(sgp30_t *dev, i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *sgp30_handle)
{
    CHECK_ARG(dev && sgp30_handle && bus_handle);

    dev->bus_mst_handle = bus_handle;
    dev->sgp30_dev_handle = sgp30_handle;

    return ESP_OK;
}

esp_err_t sgp30_init(sgp30_t *dev)
{
    CHECK_ARG(dev);
    
    CHECK(execute_command(dev->sgp30_dev_handle, SGP30_IAQ_INIT, 2, 10, NULL , 0));
    CHECK(sgp30_get_serial_id(dev, dev->serial_id));
    CHECK(sgp30_get_feature_set(dev, dev->feature_set));
    ESP_LOGI("sgp30", "Serial Number: %02x %02x %02x", dev->serial_id[0],
                                dev->serial_id[1], dev->serial_id[2]);

    ESP_LOGI("sgp30", "Feature set version: %02x %02x %02x", dev->feature_set[0],
                                dev->feature_set[1], dev->feature_set[2]);

    return ESP_OK;
}

esp_err_t sgp30_compute_values(sgp30_data_t data, uint16_t *tvoc, uint16_t *co2)
{
    CHECK_ARG(data && (tvoc || co2));

    if (co2)
        *co2 = ((data[0] * 256.0) + data[1]);

    if (tvoc)
        *tvoc = ((data[3] * 256.0) + data[4]);

    return ESP_OK;
}

esp_err_t sgp30_get_iaq_baseline(sgp30_t *dev)
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sgp30_dev_handle, SGP30_GET_IAQ_BASELINE, 2, 10, dev->iaq_baseline, sizeof(sgp30_iaq_baseline_t)));

    return ESP_OK;
}

esp_err_t sgp30_set_IAQ_baseline(sgp30_t *dev, sgp30_iaq_baseline_t iaq_bl) 
{
    CHECK_ARG(dev && iaq_bl);
    
    uint8_t command[8];

    command[0] = SGP30_SET_IAQ_BASELINE[0];
    command[1] = SGP30_SET_IAQ_BASELINE[1];

    command[2] = iaq_bl[4];
    command[3] = iaq_bl[3];
    command[4] = iaq_bl[5];

    command[5] = iaq_bl[1];
    command[6] = iaq_bl[0];
    command[7] = iaq_bl[2];

    CHECK(execute_command(dev->sgp30_dev_handle, command, 8, 10, NULL, 0));

    return ESP_OK;
}

esp_err_t sgp30_set_humidity(sgp30_t *dev, uint32_t absolute_humidity) 
{
    CHECK_ARG(dev && absolute_humidity);
    
    if (absolute_humidity > 256000) {
        ESP_LOGW("sgp30", "Absolute humidity value (%ld) is too high!", absolute_humidity);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t command[5];
    uint16_t ah_scaled = (uint16_t)(((uint64_t)absolute_humidity * 256 * 16777) >> 24);

    command[0] = SGP30_SET_ABSOLUTE_HUMIDITY[0];
    command[1] = SGP30_SET_ABSOLUTE_HUMIDITY[1];

    command[2] = ah_scaled >> 8;
    command[3] = ah_scaled & 0xFF;
    command[4] = crc8(command + 2, 2);

    CHECK(execute_command(dev->sgp30_dev_handle, command, 5, 10, NULL, 0));

    return ESP_OK;
}

esp_err_t sgp30_measure_test(sgp30_t *dev) 
{
    CHECK_ARG(dev);

    uint8_t test_result[3];

    uint8_t success[3];
    success[0] = 0xD4;
    success[1] = 0x00;
    success[2] = crc8(success, 2);
    
    CHECK(execute_command(dev->sgp30_dev_handle, SGP30_MEASURE_TEST, 2, 220, test_result, 3));
    
    if ((test_result[0] != success[0]) && (test_result[1] != success[1]) && (test_result[2] != success[2])) {
        ESP_LOGE("sgp30", "Test result negative");
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t sgp30_get_feature_set(sgp30_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sgp30_dev_handle, SGP30_GET_FEATURE_SET, 2, 10, dev->feature_set, sizeof(sgp30_feature_set_t)));

    return ESP_OK;
}

esp_err_t sgp30_measure_raw(sgp30_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sgp30_dev_handle, SGP30_MEASURE_RAW, 2, 25, dev->raw_data, sizeof(sgp30_raw_data_t)));

    return ESP_OK;
}

esp_err_t sgp30_get_tvoc_inceptive_baseline(sgp30_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sgp30_dev_handle, SGP30_GET_TVOC_INCEPTIVE_BASELINE, 2, 10, dev->tvoc_baseline, sizeof(sgp30_tvoc_baseline_t)));
    
    return ESP_OK;
}

esp_err_t sgp30_set_tvoc_inceptive_baseline(sgp30_t *dev, sgp30_tvoc_baseline_t tvoc_baseline) 
{
    CHECK_ARG(dev && tvoc_baseline);

    uint8_t command[5];

    command[0] = SGP30_SET_TVOC_INCEPTIVE_BASELINE[0];
    command[1] = SGP30_SET_TVOC_INCEPTIVE_BASELINE[1];

    command[2] = tvoc_baseline[0];
    command[3] = tvoc_baseline[1];
    command[4] = tvoc_baseline[2];

    CHECK(execute_command(dev->sgp30_dev_handle, command, 5, 10, NULL, 0));

    return ESP_OK;
}

esp_err_t sgp30_get_serial_id(sgp30_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sgp30_dev_handle, SGP30_GET_SERIAL_ID, 2, 10, dev->serial_id, sizeof(sgp30_serial_id_t)));
    
    return ESP_OK;
}








