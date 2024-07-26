cmake_minimum_required(VERSION 3.5)

idf_component_register(SRCS common_sensirion_sensor.c
                    INCLUDE_DIRS .
                    REQUIRES "driver")
