# Common driver for sensirion's sensors

The driver is to be used with an ESP32 and ESP-IDF 

## About this driver

This driver uses the native i2c_master library. It has been made to work with some sensirion's sensors. It works at least with sht3x, sgp30 and sen5x. 

This driver has only been tested on **ESP32-H2** with **esp-idf v5.2.2**. I can't affirm that it will neither work or won't work with other version of esp-idf or with other soc.

I made this driver because the three aforementioned sensors work in the same way. There are commands that one can send to the sensor to have it do specifics things. There is a delay for the sensor to execute the command. Data sent by the sensor are sent in bytes and each pair of bytes are protected by a Cyclic Redundancy Check. The CRC for the 3 sensors are identical. Therefore it made sense to factorize the two functions common to all three sensors. 

## Cyclic Redundancy Check

The three aforempentioned sensors use the same CRC :

|Property        | Value                        |
|:---------------|:-----------------------------|
|Name            | CRC-8                        |
|Protected Data  | read and/or write data       |
|Width           | 8 bit                        |
|Polynomial      | 0x31 (x^8 + x^5 + x^4 + 1)   |
|Initialization  | 0xFF                         |
|Reflect Input   | false                        |
|Reflect Output  | false                        |
|Final XOR       | 0x00                         |
|Example         | CRC(0xBEEF) = 0x9            |

Here is the implementation I used : 

```C
uint8_t crc8(uint8_t data[], int len)
{
    // initialization value
    uint8_t crc = 0xff;

    // iterate over all bytes
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int i = 0; i < 8; i++)
        {
            bool xor = crc & 0x80;
            crc = crc << 1;
            crc = xor ? crc ^ G_POLYNOM : crc;
        }
    }
    return crc;
}
```

## Execute a command

In order for the sensor to understand the command it needs to be sent byte after byte, the most significants bits first and for the parameters a CRC byte is sent for each pair of bytes sent. Every data sent by the sensor is structured in the same way. Furthermore there is a delay specific for each command. The command can be sent with or without parameters and can call for data or not. The result is a function that looks like this : 
```C
esp_err_t execute_command(i2c_master_dev_handle_t *dev, uint8_t cmd[], uint8_t command_len, uint16_t delay, uint8_t *read_data, uint8_t read_len);
```