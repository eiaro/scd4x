#ifndef SCD4X_H
#define SCD4X_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#define SCD4X_OK 0
#define SCD4X_ERR_BASE -100
#define SCD4X_ERR_I2C_INIT (SCD4X_ERR_BASE - 1)
#define SCD4X_ERR_I2C_READ (SCD4X_ERR_BASE - 2)
#define SCD4X_ERR_I2C_WRITE (SCD4X_ERR_BASE - 3)
#define SCD4X_ERR_CRC (SCD4X_ERR_BASE - 4)

#define CRC_ERROR 1
#define I2C_BUS_ERROR 2
#define I2C_NACK_ERROR 3
#define BYTE_NUM_ERROR 4

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF
#define CRC8_LEN 1

#define SENSIRION_COMMAND_SIZE 2
#define SENSIRION_WORD_SIZE 2
#define SENSIRION_NUM_WORDS(x) (sizeof(x) / SENSIRION_WORD_SIZE)
#define SENSIRION_MAX_BUFFER_WORDS 32

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

enum scd4x_sensor_type {
    SCD4X_SENSOR_TYPE_AUTO = 0x00,
    SCD4X_SENSOR_TYPE_SCD40 = 0x01,
    SCD4X_SENSOR_TYPE_SCD41 = 0x02,
};

struct scd4x_sample {
    uint16_t co2;
    uint16_t temperature;
    uint16_t humidity;
} __packed __aligned(2);

struct scd4x_config {
    const struct i2c_dt_spec i2c;   
    enum scd4x_sensor_type sensor_type; 
};

struct scd4x_data {  
    struct scd4x_sample sample;
    uint16_t serial_number[3];
};

// ****************************************************************************
// Function prototypes
// ****************************************************************************
/*
static uint8_t scd4x_compute_crc(uint16_t value);

static int scd4x_send_cmd(const struct device *dev, uint16_t cmd);
static int scd4x_read_register(const struct device *dev, uint16_t reg, uint16_t *data, size_t data_len);

static int scd4x_get_serial_number(const struct device *dev);
static int scd4x_wake_up(const struct device *dev);
static int scd4x_stop_periodic_measurement(const struct device *dev);
static int scd4x_reinit(const struct device *dev);
static int scd4x_start_periodic_measurement(const struct device *dev);
static int scd4x_start_low_power_periodic_measurement(const struct device *dev);
static int scd4x_get_data_ready_status(const struct device *dev);
static int scd4x_read_measurement(const struct device *dev);
*/
#endif // SCD4X_H