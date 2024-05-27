#ifndef SCD4X_H
#define SCD4X_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>


#define SCD4X_I2C_ADDR 0x62

// Basic commands Section 3.5
#define SCD4X_CMD_START_PERIODIC_MEASUREMENT            0x21B1
#define SCD4X_CMD_READ_MEASUREMENT                      0xEC05
#define SCD4X_CMD_STOP_PERIODIC_MEASUREMENT             0x3F86

// On-chip output signal compensation Section 3.6
#define SCD4X_CMD_SET_TEMPERATURE_OFFSET                0x241D
#define SCD4X_CMD_GET_TEMPERATURE_OFFSET                0x2318
#define SCD4X_CMD_SET_SENSOR_ALTITUDE                   0x2427
#define SCD4X_CMD_GET_SENSOR_ALTITUDE                   0x2322
#define SCD4X_CMD_SET_AMBIENT_PRESSURE                  0xE000
#define SCD4X_CMD_GET_AMBIENT_PRESSURE                  0xE000

// Field calibration Section 3.7
#define SCD4X_CMD_SET_FORCED_RECALIBRATION              0x362F
#define SCD4X_CMD_SET_AUTO_SELF_CALIBRATION             0x2416
#define SCD4X_CMD_GET_AUTO_SELF_CALIBRATION             0x2313

// Low power periodic measurement mode Section 3.8
#define SCD4X_CMD_START_LOW_POWER_PERIODIC_MEASUREMENT  0x21AC
#define SCD4X_CMD_GET_DATA_READY                        0xE4B8

// Advanced features Section 3.9
#define SCD4X_CMD_PERSISTENT_SETTINGS                   0x3615
#define SCD4X_CMD_GET_SERIAL_NUMBER                     0x3682
#define SCD4X_CMD_PERFORM_SELF_TEST                     0x3639
#define SCD4X_CMD_PERFORM_FACTORY_RESET                 0x3632
#define SCD4X_CMD_REINIT                                0x3646

// Single shot measurement mode (SCD41 only) Section 3.10
#define SCD4X_CMD_PERFORM_SINGLE_MEASUREMENT            0x219D
#define SCD4X_CMD_PERFORM_SINGLE_MEASUREMENT_RHT        0x2196
#define SCD4X_CMD_POWER_DOWN                            0x36E0
#define SCD4X_CMD_WAKE_UP                               0x36F6
#define SCD4X_CMD_SET_AUTO_SELF_CAL_INIT_PERIOD         0x2445
#define SCD4X_CMD_GET_AUTO_SELF_CAL_INIT_PERIOD         0x2340
#define SCD4X_CMD_SET_AUTO_SELF_CAL_STD_PERIOD          0x244E
#define SCD4X_CMD_GET_AUTO_SELF_CAL_STD_PERIOD          0x234B

#define SCD4X_MAX_READ_LENGTH 9

struct scd4x_sample {
    uint16_t co2;
    uint16_t temperature;
    uint16_t humidity;
} __packed __aligned(2);

struct scd4x_config {
    const struct i2c_dt_spec i2c;    
};

struct scd4x_data {  
    struct scd4x_sample sample;
};


#endif // SCD4X_H