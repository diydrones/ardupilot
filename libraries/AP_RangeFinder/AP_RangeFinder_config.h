#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_MSP/AP_MSP_config.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <AP_CANManager/AP_CANManager_config.h>
#include <AP_MSP/AP_MSP_config.h>
#include <GCS_MAVLink/GCS_config.h>

#ifndef AP_RANGEFINDER_ENABLED
#define AP_RANGEFINDER_ENABLED 1
#endif

#ifndef AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#define AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED AP_RANGEFINDER_ENABLED
#endif

#ifndef AP_RANGEFINDER_BACKEND_CAN_ENABLED
#define AP_RANGEFINDER_BACKEND_CAN_ENABLED AP_RANGEFINDER_ENABLED && HAL_MAX_CAN_PROTOCOL_DRIVERS
#endif

#ifndef AP_RANGEFINDER_ANALOG_ENABLED
#define AP_RANGEFINDER_ANALOG_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_BBB_PRU_ENABLED
#define AP_RANGEFINDER_BBB_PRU_ENABLED (AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI)
#endif

#ifndef AP_RANGEFINDER_BENEWAKE_ENABLED
#define AP_RANGEFINDER_BENEWAKE_ENABLED AP_RANGEFINDER_ENABLED
#endif

#ifndef AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
#define AP_RANGEFINDER_BENEWAKE_CAN_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#ifndef AP_RANGEFINDER_BENEWAKE_TF02_ENABLED
#define AP_RANGEFINDER_BENEWAKE_TF02_ENABLED (AP_RANGEFINDER_BENEWAKE_ENABLED && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#ifndef AP_RANGEFINDER_BENEWAKE_TF03_ENABLED
#define AP_RANGEFINDER_BENEWAKE_TF03_ENABLED (AP_RANGEFINDER_BENEWAKE_ENABLED && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#ifndef AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED
#define AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED (AP_RANGEFINDER_BENEWAKE_ENABLED && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#ifndef AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED
#define AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_BLPING_ENABLED
#define AP_RANGEFINDER_BLPING_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_BEBOP_ENABLED
#define AP_RANGEFINDER_BEBOP_ENABLED \
    AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED &&                           \
    (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP ||       \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) &&      \
    defined(HAVE_LIBIIO)
#endif

#ifndef AP_RANGEFINDER_DRONECAN_ENABLED
#define AP_RANGEFINDER_DRONECAN_ENABLED (HAL_ENABLE_DRONECAN_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#ifndef AP_RANGEFINDER_GYUS42V2_ENABLED
#define AP_RANGEFINDER_GYUS42V2_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_HC_SR04_ENABLED
#define AP_RANGEFINDER_HC_SR04_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_LANBAO_ENABLED
#define AP_RANGEFINDER_LANBAO_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_LEDDARONE_ENABLED
#define AP_RANGEFINDER_LEDDARONE_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_LEDDARVU8_ENABLED
#define AP_RANGEFINDER_LEDDARVU8_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_LWI2C_ENABLED
#define AP_RANGEFINDER_LWI2C_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED
#define AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_LUA_ENABLED
#define AP_RANGEFINDER_LUA_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && AP_SCRIPTING_ENABLED
#endif

#ifndef AP_RANGEFINDER_MAVLINK_ENABLED
#define AP_RANGEFINDER_MAVLINK_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_GCS_ENABLED
#endif

#ifndef AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED
#define AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_MAXSONARI2CXL_ENABLED
#define AP_RANGEFINDER_MAXSONARI2CXL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef HAL_MSP_RANGEFINDER_ENABLED
#define HAL_MSP_RANGEFINDER_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_MSP_ENABLED
#endif

#ifndef AP_RANGEFINDER_NMEA_ENABLED
#define AP_RANGEFINDER_NMEA_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_NOOPLOOP_ENABLED
#define AP_RANGEFINDER_NOOPLOOP_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

#ifndef AP_RANGEFINDER_NRA24_CAN_ENABLED
#define AP_RANGEFINDER_NRA24_CAN_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#ifndef AP_RANGEFINDER_PWM_ENABLED
#define AP_RANGEFINDER_PWM_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#ifndef AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED
#define AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_RDS02UF_ENABLED
#define AP_RANGEFINDER_RDS02UF_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

#ifndef AP_RANGEFINDER_SIM_ENABLED
#define AP_RANGEFINDER_SIM_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#ifndef HAL_MSP_RANGEFINDER_ENABLED
#define HAL_MSP_RANGEFINDER_ENABLED (AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_MSP_ENABLED)
#endif

#ifndef AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED
#define AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_TOFSENSEP_CAN_ENABLED
#define AP_RANGEFINDER_TOFSENSEP_CAN_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && AP_RANGEFINDER_BACKEND_CAN_ENABLED
#endif

#ifndef AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED
#define AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

#ifndef AP_RANGEFINDER_TRI2C_ENABLED
#define AP_RANGEFINDER_TRI2C_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_USD1_CAN_ENABLED
#define AP_RANGEFINDER_USD1_CAN_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && AP_RANGEFINDER_BACKEND_CAN_ENABLED
#endif

#ifndef AP_RANGEFINDER_USD1_SERIAL_ENABLED
#define AP_RANGEFINDER_USD1_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_VL53L0X_ENABLED
#define AP_RANGEFINDER_VL53L0X_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_VL53L1X_ENABLED
#define AP_RANGEFINDER_VL53L1X_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_WASP_ENABLED
#define AP_RANGEFINDER_WASP_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_RANGEFINDER_JRE_SERIAL_ENABLED
#define AP_RANGEFINDER_JRE_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

#ifndef AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED
#define AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif
