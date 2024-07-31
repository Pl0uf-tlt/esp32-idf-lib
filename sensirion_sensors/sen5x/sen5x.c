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
 * @file sen5x.c
 *
 * ESP-IDF driver for Sensirion sen5x air quality sensor
 *
 * Copyright (c) 2024 Cédric Toulotte <https://github.com/Pl0uf-tlt>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "common_sensirion_sensor.h"
#include "sen5x.h"


static uint8_t SEN5X_START_MEASUREMENT[2] =                              { 0x00, 0x21 };
static uint8_t SEN5X_STOP_MEASUREMENT[2] =                               { 0x00, 0x37 };
static uint8_t SEN5X_READ_DATA_READY_FLAG[2] =                           { 0x02, 0x02 };
static uint8_t SEN5X_READ_MEASURED_VALUES[2] =                           { 0x03, 0xC4 };
static uint8_t SEN5X_READ_WRITE_TEMPERATURE_COMPENSATION_PARAMETERS[2] = { 0x60, 0xB2 };
static uint8_t SEN5X_READ_WRITE_WARM_START_PARAMETER[2] =                { 0x60, 0xC6 };
static uint8_t SEN5X_START_FAN_CLEANING[2] =                             { 0x56, 0x07 };
static uint8_t SEN5X_READ_WRITE_AUTO_CLEANING_INTERVAL[2] =              { 0x80, 0x04 };
static uint8_t SEN5X_READ_PRODUCT_NAME[2] =                              { 0xD0, 0x14 };
static uint8_t SEN5X_READ_SERIAL_NUMBER[2] =                             { 0xD0, 0x33 };
static uint8_t SEN5X_READ_FIRMWARE_VERSION[2] =                          { 0xD1, 0x00 };
static uint8_t SEN5X_READ_DEVICE_STATUS[2] =                             { 0xD2, 0x06 };
static uint8_t SEN5X_CLEAR_DEVICE_STATUS[2] =                            { 0xD2, 0x10 };
static uint8_t SEN5X_DEVICE_RESET[2] =                                   { 0xD3, 0x04 };

esp_err_t sen5x_init_desc(sen5x_t *dev, i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *sen5x_handle)
{
    CHECK_ARG(dev && sen5x_handle && bus_handle);

    dev->bus_mst_handle = bus_handle;
    dev->sen5x_dev_handle = sen5x_handle;

    return ESP_OK;
}

esp_err_t sen5x_init(sen5x_t *dev)
{
    CHECK_ARG(dev);
    
    CHECK(sen5x_read_serial_number(dev, &(dev->serial_number)));
    CHECK(sen5x_read_firmware_version(dev, &(dev->firmware_version)));
    CHECK(sen5x_read_product_name(dev, &(dev->product_name)));

    ESP_LOGI("sen5x", "Serial number: %s", dev->serial_number);
    ESP_LOGI("sen5x", "Product name: %s", dev->product_name);
    ESP_LOGI("sen5x", "Firmware version: %02x %02x", dev->firmware_version[0], dev->firmware_version[1]);

    CHECK(sen5x_start_measurement(dev));
    return ESP_OK;
}

esp_err_t sen5x_start_measurement(sen5x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sen5x_dev_handle, SEN5X_START_MEASUREMENT, 2, 50, NULL, 0));

    return ESP_OK;
}

esp_err_t sen5x_stop_measurement(sen5x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sen5x_dev_handle, SEN5X_STOP_MEASUREMENT, 2, 200, NULL, 0));

    return ESP_OK;
}

esp_err_t sen5x_read_data_ready_flag(sen5x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sen5x_dev_handle, SEN5X_READ_DATA_READY_FLAG, 2, 20, &(dev->data_ready_flag), sizeof(sen5x_data_ready_flag_t)));

    return ESP_OK;
}

esp_err_t sen5x_read_measured_values(sen5x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sen5x_dev_handle, SEN5X_READ_MEASURED_VALUES, 2, 20, &(dev->measured_values), sizeof(sen5x_measured_values_t)));

    dev->pm1p0 = ((double)(dev->measured_values[0]*256 + dev->measured_values[1]))/10;
    dev->pm2p5 = ((double)(dev->measured_values[3]*256 + dev->measured_values[4]))/10;
    dev->pm4p0 = ((double)(dev->measured_values[6]*256 + dev->measured_values[7]))/10;
    dev->pm10p0 = ((double)(dev->measured_values[9]*256 + dev->measured_values[10]))/10;

    return ESP_OK;
}

esp_err_t sen5x_write_temperature_compensation_parameters(sen5x_t *dev, sen5x_temperature_compensation_parameters_t *temperature_compensation_parameters) 
{
    CHECK_ARG(dev && temperature_compensation_parameters);
    uint8_t command_len = 2 + sizeof(sen5x_temperature_compensation_parameters_t);
    uint8_t command[command_len];
    command[0] = SEN5X_READ_WRITE_TEMPERATURE_COMPENSATION_PARAMETERS[0];
    command[1] = SEN5X_READ_WRITE_TEMPERATURE_COMPENSATION_PARAMETERS[1];
    
    for (uint8_t i = 2; i < command_len; i++) 
    {
        command[i] = *temperature_compensation_parameters[i - 2];
    }

    CHECK(execute_command(dev->sen5x_dev_handle, command, command_len, 20, NULL, 0));

    return ESP_OK;
}

esp_err_t sen5x_write_warm_start_parameter(sen5x_t *dev, sen5x_warm_start_parameter_t *warm_start_parameter) 
{
    CHECK_ARG(dev && warm_start_parameter);
    uint8_t command_len = 2 + sizeof(sen5x_warm_start_parameter_t);
    uint8_t command[command_len];
    command[0] = SEN5X_READ_WRITE_WARM_START_PARAMETER[0];
    command[1] = SEN5X_READ_WRITE_WARM_START_PARAMETER[1];
    
    for (uint8_t i = 2; i < command_len; i++) 
    {
        command[i] = *warm_start_parameter[i - 2];
    }

    CHECK(execute_command(dev->sen5x_dev_handle, command, command_len, 20, NULL, 0));

    return ESP_OK;
}

esp_err_t sen5x_start_fan_cleaning(sen5x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sen5x_dev_handle, SEN5X_START_FAN_CLEANING, 2, 20, NULL, 0));

    return ESP_OK;
}

esp_err_t sen5x_write_auto_cleaning_interval(sen5x_t *dev, sen5x_fan_auto_cleaning_interval_t *fan_auto_cleaning_interval) 
{
    CHECK_ARG(dev && fan_auto_cleaning_interval);
    uint8_t command_len = 2 + sizeof(sen5x_fan_auto_cleaning_interval_t);
    uint8_t command[command_len];
    command[0] = SEN5X_READ_WRITE_AUTO_CLEANING_INTERVAL[0];
    command[1] = SEN5X_READ_WRITE_AUTO_CLEANING_INTERVAL[1];
    
    for (uint8_t i = 2; i < command_len; i++) 
    {
        command[i] = *fan_auto_cleaning_interval[i - 2];
    }

    CHECK(execute_command(dev->sen5x_dev_handle, command, command_len, 20, NULL, 0));

    return ESP_OK;
}

esp_err_t sen5x_read_product_name(sen5x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sen5x_dev_handle, SEN5X_READ_PRODUCT_NAME, 2, 20, &(dev->product_name), sizeof(sen5x_product_name_t)));

    return ESP_OK;
}

esp_err_t sen5x_read_serial_number(sen5x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sen5x_dev_handle, SEN5X_READ_SERIAL_NUMBER, 2, 20, &(dev->serial_number), sizeof(sen5x_serial_number_t)));

    return ESP_OK;
}

esp_err_t sen5x_read_firmware_version(sen5x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sen5x_dev_handle, SEN5X_READ_FIRMWARE_VERSION, 2, 20, &(dev->firmware_version) , sizeof(sen5x_firmware_version_t)));

    return ESP_OK;
}

esp_err_t sen5x_read_device_status(sen5x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sen5x_dev_handle, SEN5X_READ_DEVICE_STATUS, 2, 20, &(dev->device_status), sizeof(sen5x_device_status_t)));

    return ESP_OK;
}

esp_err_t sen5x_clear_device_status(sen5x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sen5x_dev_handle, SEN5X_CLEAR_DEVICE_STATUS, 2, 20, NULL, 0));

    return ESP_OK;
}

esp_err_t sen5x_device_reset(sen5x_t *dev) 
{
    CHECK_ARG(dev);

    CHECK(execute_command(dev->sen5x_dev_handle, SEN5X_DEVICE_RESET, 2, 100, NULL, 0));

    return ESP_OK;
}









