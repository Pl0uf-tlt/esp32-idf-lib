cmake_minimum_required(VERSION 3.5)

idf_component_register(SRCS sgp30.c
                    INCLUDE_DIRS .
                    REQUIRES "common_sensirion_sensor")
