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
#define SEN5X_MEASURED_PM_VALUES_SIZE 30

typedef uint8_t sen5x_data_ready_flag_t[SEN5X_DATA_READY_FLAG_SIZE];
typedef uint8_t sen5x_measured_values_t[SEN5X_MEASURED_VALUES_SIZE];
typedef uint8_t sen5x_temperature_compensation_parameters_t[SEN5X_TEMPERATURE_COMPENSATION_PARAMETERS_SIZE];
typedef uint8_t sen5x_warm_start_parameter_t[SEN5X_WARM_START_PARAMETER_SIZE];
typedef uint8_t sen5x_fan_auto_cleaning_interval_t[SEN5X_FAN_AUTO_CLEANING_INTERVAL_SIZE];
typedef uint8_t sen5x_product_name_t[SEN5X_PRODUCT_NAME_SIZE];
typedef uint8_t sen5x_serial_number_t[SEN5X_SERIAL_NUMBER_SIZE];
typedef uint8_t sen5x_firmware_version_t[SEN5X_FIRMWARE_VERSION_SIZE];
typedef uint8_t sen5x_device_status_t[SEN5X_DEVICE_STATUS_SIZE];
typedef uint8_t sen5x_measured_pm_values_t[SEN5X_MEASURED_PM_VALUES_SIZE];

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
    sen5x_measured_pm_values_t measured_pm_values;
    double pm1p0;
    double pm2p5;
    double pm4p0;
    double pm10p0;
    double pm1p0_nbr;
    double pm2p5_nbr;
    double pm4p0_nbr;
    double pm10p0_nbr;
    double pm0p5_nbr;
    double typical_particul_size;
} sen5x_t;

esp_err_t sen5x_read_pm_values(sen5x_t *dev);

/**
 * @brief Initializes the sen5x device descriptor 
 * 
 * @param dev        A pointer to the sen5x device descriptor 
 * @param bus_handle A pointer to the i2c master bus handle
 * @param sen5x_handle A pointer to the sen5x's i2c device handle
 * 
 * @return ESP_OK on success - ESP_ERR_INVALID_ARG if sen5x_handle or bus_handle doesn't exist
 */
esp_err_t sen5x_init_desc(sen5x_t *dev, i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *sen5x_handle);

/**
 * @brief Initializes the state of the sen5x sensor
 * 
 * @param dev A pointer to the sen5x device descriptor 
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALID_ARG if dev doesn't exist
 */
esp_err_t sen5x_init(sen5x_t *dev);

/**
 * @brief Start the measurement 
 * 
 * 
 * @note Thread safe
 *
 * @param dev    A pointer to the sen5x device descriptor 
 * 
 * @return       ESP_OK on success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_INVALID_ARG: dev doesn't exist - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
 */
esp_err_t sen5x_start_measurement(sen5x_t *dev);

/**
 * @brief Stops the measurements
 *
 * The function stops the measurements 
 *
 * @note Thread safe
 * 
 * @param dev A pointer to the sen5x device descriptor 
 * 
 * @return    ESP_OK on success
 */
esp_err_t sen5x_stop_measurement(sen5x_t *dev);

/**
 * @brief Read the data ready bytes of the sen5x
 *
 * @note Thread safe
 * 
 * @param dev A pointer to the sen5x device descriptor
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALID_ARG if dev doesn't exist
 */
esp_err_t sen5x_read_data_ready_flag(sen5x_t *dev);

/**
 * @brief Read measurement results from sensor as raw data
 *
 * The function read measurement results from the sensor, checks the CRC
 * checksum and stores them in a byte array in the device descriptor as 
 * indicated bellow as unsigned byte integers  
 *
 *      data[0] = Mass Concentration PM1.0 [µg/m³] MSB
 *      data[1] = Mass Concentration PM1.0 [µg/m³] LSB
 *      data[2] = CRC for the previous two bytes
 *      data[3] = Mass Concentration PM2.5 [µg/m³] MSB
 *      data[4] = Mass Concentration PM2.5 [µg/m³] LSB
 *      data[5] = CRC for the previous two bytes
 *      data[6] = Mass Concentration PM4.0 [µg/m³] MSB
 *      data[7] = Mass Concentration PM4.0 [µg/m³] LSB
 *      data[8] = Temperature CRC
 *      data[9] = Mass Concentration PM10.0 [µg/m³] MSB
 *      data[10] = Mass Concentration PM10.0 [µg/m³] LSB
 *      data[11] = CRC for the previous two bytes
 *      data[12] = Compensated Ambient Humidity [%RH] MSB
 *      data[13] = Compensated Ambient Humidity [%RH] LSB
 *      data[14] = CRC for the previous two bytes
 *      data[15] = Compensated Ambient Temperature [°C] MSB
 *      data[16] = Compensated Ambient Temperature [°C] LSB
 *      data[17] = CRC for the previous two bytes
 *      data[18] = VOC Index MSB
 *      data[19] = VOC Index LSB
 *      data[20] = CRC for the previous two bytes
 *      data[21] = NOx Index MSB
 *      data[22] = NOx Index LSB
 *      data[23] = CRC for the previous two bytes
 * In case that there are no new data that can be read, the function fails.
 * In RHT/Gas-Only Measurement Mode, the PM output is 0xFFFF. If any value is unknown, 0xFFFF is returned. 
 *
 * @note Thread safe
 * 
 * @param dev      A pointer to the sen5x device descriptor
 *
 * @return         ESP_OK on success - ESP_ERR_INVALID_CRC if the crc failed 
 */
esp_err_t sen5x_read_measured_values(sen5x_t *dev);

/**
 * @brief Sets the temperature compensation algorithme in case the sensor is integrated in a device that changes the way heat is distributed.
 * 
 * @note Thread safe
 * 
 * @param dev A pointer to the sen5x device descriptor 
 * @param temperature_compensation_parameters An array of 2 bytes unsigned integers (and there CRC) that represents the temperature compensation parameters. 
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALIG_ARG if dev and/or temperature_compensation_parameters don't exist - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
 */
esp_err_t sen5x_write_temperature_compensation_parameters(sen5x_t *dev, sen5x_temperature_compensation_parameters_t *temperature_compensation_parameters);

/**
 * @brief Sets the warm start parameters in case the sensor starting warm from previous measurements
 * 
 * @note Thread safe
 * 
 * @param dev A pointer to the sen5x device descriptor 
 * @param warm_start_parameter An array of 16 bits unsigned integers that represents the temperature
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALIG_ARG if dev and/or warm_start_parameter don't exist - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
 */
esp_err_t sen5x_write_warm_start_parameter(sen5x_t *dev, sen5x_warm_start_parameter_t *warm_start_parameter);

/**
 * @brief Starts the fan cleaning procedure
 * 
 * @note Thread safe
 * 
 * @param dev    A pointer to the sen5x device descriptor 
 * 
 * @return       ESP_OK on success - ESP_ERR_INVALID_ARG if dev doesn't exist
 */
esp_err_t sen5x_start_fan_cleaning(sen5x_t *dev);

/**
 * @brief Sets the fan auto cleaning time interval in seconds. Default is 604’800 seconds (i.e., 168 hours or 1 week) with a tolerance of 3%. 
 * 
 * @note Thread safe
 * 
 * @param dev A pointer to the sen5x device descriptor 
 * @param fan_auto_cleaning_interval A 32 bits unsigned integer in a 6 bytes array (2 times 2 bytes for the time and 2 bytes of CRC for bytes 0 and 1, 3 and 4)
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALIG_ARG if dev and/or fan_auto_cleaning_interval don't exist - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
 */
esp_err_t sen5x_write_auto_cleaning_interval(sen5x_t *dev, sen5x_fan_auto_cleaning_interval_t *fan_auto_cleaning_interval);

/**
 * @brief Read the product name, either SEN50 or SEN54 or SEN55 and stores it in an array in the device descriptor
 *
 * @note Thread safe
 * 
 * @param dev A pointer to the sen5x device descriptor
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALID_ARG if dev doesn't exist
 */
esp_err_t sen5x_read_product_name(sen5x_t *dev);

/**
 * @brief Read the serial number of the sensor and stores it in the device descriptor
 *
 * @note Thread safe
 * 
 * @param dev A pointer to the sen5x device descriptor
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALID_ARG if dev doesn't exist
 */
esp_err_t sen5x_read_serial_number(sen5x_t *dev);

/**
 * @brief Read the firmware version of the sensor and stores it in the device descriptor
 *
 * @note Thread safe
 * 
 * @param dev A pointer to the sen5x device descriptor
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALID_ARG if dev doesn't exist
 */
esp_err_t sen5x_read_firmware_version(sen5x_t *dev);

/**
 * @brief Read the device status bytes of the sensor and stores it in the device descriptor
 *
 * @note Thread safe
 * 
 * @param dev A pointer to the sen5x device descriptor
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALID_ARG if dev doesn't exist
 */
esp_err_t sen5x_read_device_status(sen5x_t *dev);

/**
 * @brief Clears the device status bytes of the sensor
 *
 * @note Thread safe
 * 
 * @param dev A pointer to the sen5x device descriptor
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALID_ARG if dev doesn't exist
 */
esp_err_t sen5x_clear_device_status(sen5x_t *dev);

/**
 * @brief Reset the sensor 
 *
 * @note Thread safe
 * 
 * @param dev A pointer to the sen5x device descriptor
 * 
 * @return    ESP_OK on success - ESP_ERR_INVALID_ARG if dev doesn't exist
 */
esp_err_t sen5x_device_reset(sen5x_t *dev);


#endif /* __SEN5X_H__ */
