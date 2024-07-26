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
 * @file common_sensirion_sensor.h
 * @defgroup sensirion_sensors sensirion_sensors
 * 
 *
 * ESP-IDF driver for Sensirion's sensors, it works att least with sht3x, sgp30 and sen5x sensors
 *
 * Copyright (c) 2024 Cédric Toulotte <https://github.com/Pl0uf-tlt>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __COMMON_SENSIRION_SENSOR_H__
#define __COMMON_SENSIRION_SENSOR_H__

#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define G_POLYNOM 0x31

/**
 * @brief Calculate the Cyclic Redundancy Check for an array of bytes
 * 
 * @param data An array of bytes of lenght \p len
 * @param len The lenght of \p data
 * 
 * @return A byte wich is the crc8 value for the given bytes
 */
uint8_t crc8(uint8_t data[], int len); 

/**
 * @brief Write a command to a sensor with I2C and, if needed, wait for the response and read the response. It checks the crc of received bytes
 * 
 * @param dev A pointer to the device handler of the sensor
 * @param cmd A byte array that contains the data to write, it means both the command and the parameters when there are some
 * @param command_len An unsigned 8 bits integer. The lenght of the data to write in bytes
 * @param delay The delay to wait after having written the command and the parameters for the sensor to process the command and eventually send somme data
 * @param read_data A pointer to a byte array of lenght \p read_len that will receive the data sent by the sensor in response to the command, if any. NULL if the sensor doesn't send data in response to the command
 * @param read_len The lenght of \p read_data
 * 
 * @return ESP_OK if success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
 */
esp_err_t execute_command(i2c_master_dev_handle_t *dev, uint8_t cmd[], uint8_t command_len, uint16_t delay, uint8_t *read_data, uint8_t read_len); 

#endif /* __COMMON_SENSIRION_SENSOR_H__ */
