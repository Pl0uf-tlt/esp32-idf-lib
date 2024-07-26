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
 * @file sens5x.h
 * @defgroup sen5x sens5x
 * 
 *
 * ESP-IDF driver for Sensirion sen5x air quality sensor
 *
 * Copyright (c) 2024 Cédric Toulotte <https://github.com/Pl0uf-tlt>
 *
 * BSD Licensed as described in the file LICENSE
 */

#ifndef __SEN5X_H__
#define __SEN5X_H__

#include <stdbool.h>
#include <esp_err.h> 
#include <driver/i2c_master.h>

#define SEN5X_I2C_ADDR 0x69

#define SEN5X_DATA_READY_FLAG_SIZE 3
#define SEN5X_MEASURED_VALUES_SIZE 24
#define SEN5X_TEMPERATURE_COMPENSATION_PARAMETERS_SIZE 9
#define SEN5X_WARM_START_PARAMETER_SIZE 3
#define SEN5X_FAN_AUTO_CLEANING_INTERVAL_SIZE 6
#define SEN5X_PRODUCT_NAME_SIZE 48
#define SEN5X_SERIAL_NUMBER_SIZE 48
#define SEN5X_FIRMWARE_VERSION_SIZE 3
#define SEN5X_DEVICE_STATUS_SIZE 6

typedef uint8_t sen5x_data_ready_flag_t[SEN5X_DATA_READY_FLAG_SIZE];
typedef uint8_t sen5x_measured_values_t[SEN5X_MEASURED_VALUES_SIZE];
typedef uint8_t sen5x_temperature_compensation_parameters_t[SEN5X_TEMPERATURE_COMPENSATION_PARAMETERS_SIZE];
typedef uint8_t sen5x_warm_start_parameter_t[SEN5X_WARM_START_PARAMETER_SIZE];
typedef uint8_t sen5x_fan_auto_cleaning_interval_t[SEN5X_FAN_AUTO_CLEANING_INTERVAL_SIZE];
typedef uint8_t sen5x_product_name_t[SEN5X_PRODUCT_NAME_SIZE];
typedef uint8_t sen5x_serial_number_t[SEN5X_SERIAL_NUMBER_SIZE];
typedef uint8_t sen5x_firmware_version_t[SEN5X_FIRMWARE_VERSION_SIZE];
typedef uint8_t sen5x_device_status_t[SEN5X_DEVICE_STATUS_SIZE];

/**
 * Device descriptor
 */
typedef struct
{
    i2c_master_bus_handle_t *bus_mst_handle; //!< master bus handle 
    i2c_master_dev_handle_t *sen5x_dev_handle; //!< sen5x's i2c device handle

    sen5x_data_ready_flag_t data_ready_flag;
    sen5x_measured_values_t measured_values;
    sen5x_temperature_compensation_parameters_t temperature_compensation_parameters;
    sen5x_warm_start_parameter_t warm_start_parameter;
    sen5x_fan_auto_cleaning_interval_t fan_auto_cleaning_interval;
    sen5x_product_name_t product_name;
    sen5x_serial_number_t serial_number;
    sen5x_firmware_version_t firmware_version;
    sen5x_device_status_t device_status;
    double pm1p0;
    double pm2p5;
    double pm4p0;
    double pm10p0;
} sen5x_t;

esp_err_t sen5x_init_desc(sen5x_t *dev, i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *sen5x_handle);

esp_err_t sen5x_init(sen5x_t *dev);

esp_err_t sen5x_start_measurement(sen5x_t *dev);

esp_err_t sen5x_stop_measurement(sen5x_t *dev);

esp_err_t sen5x_read_data_ready_flag(sen5x_t *dev, sen5x_data_ready_flag_t *data_ready_flag);

esp_err_t sen5x_read_measured_values(sen5x_t *dev, sen5x_measured_values_t *measured_values);

esp_err_t sen5x_read_write_temperature_compensation_parameters(sen5x_t *dev, sen5x_temperature_compensation_parameters_t *temperature_compensation_parameters);

esp_err_t sen5x_read_write_warm_start_parameter(sen5x_t *dev, sen5x_warm_start_parameter_t *warm_start_parameter);

esp_err_t sen5x_start_fan_cleaning(sen5x_t *dev);

esp_err_t sen5x_read_write_auto_cleaning_interval(sen5x_t *dev, sen5x_fan_auto_cleaning_interval_t *fan_auto_cleaning_interval);

esp_err_t sen5x_read_product_name(sen5x_t *dev, sen5x_product_name_t *product_name);

esp_err_t sen5x_read_serial_number(sen5x_t *dev, sen5x_serial_number_t *serial_number);

esp_err_t sen5x_read_firmware_version(sen5x_t *dev, sen5x_firmware_version_t *firmware_version);

esp_err_t sen5x_read_device_status(sen5x_t *dev, sen5x_device_status_t *device_status);

esp_err_t sen5x_clear_device_status(sen5x_t *dev);

esp_err_t sen5x_device_reset(sen5x_t *dev);


#endif /* __SEN5X_H__ */
