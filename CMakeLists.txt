cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if(CONFIG_NODE_ROLE_SENDER)
    project(esp32s3_mpu_sender)
else()
    project(esp32s3_arm_receiver)
endif()