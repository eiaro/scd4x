#define DT_DRV_COMPAT sensirion_scd4x

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>

#include "scd4x.h"

LOG_MODULE_REGISTER(SCD4X, CONFIG_SENSOR_LOG_LEVEL);

static void scd4x_temperature_from_raw(struct sensor_value *val, uint16_t raw)
{
    sensor_value_from_double(val, (175.0 * raw / 65535) - 45.0);    
}

static void scd4x_humidity_from_raw(struct sensor_value *val, uint16_t raw)
{
    sensor_value_from_double(val, (100.0 * raw / 65535));
}

static void scd4x_co2_from_raw(struct sensor_value *val, uint16_t raw)
{
    sensor_value_from_double(val, raw);
}

static uint8_t scd4x_compute_crc(uint16_t value)
{
    uint8_t buf[2];

    sys_put_be16(value, buf);
    return crc8(buf, 2, 0x31, 0xff, false);
}

static int scd4x_write_cmd(const struct device *dev, uint16_t cmd)
{
    const struct scd4x_config *cfg = dev->config;
    uint8_t buf[2];
    int ret;

    sys_put_be16(cmd, buf);
    ret = i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to write command 0x%04x (%d)", cmd, ret);
    }

    return ret;
}

static int scd4x_read_words(const struct device *dev, uint16_t cmd, uint16_t *data, uint16_t num_words, uint16_t delay_ms)
{
    const struct scd4x_config *cfg = dev->config;

    uint8_t buf[num_words * (2+1)];
    uint16_t temp16;
    int ret;

    ret = scd4x_write_cmd(dev, cmd);
    if (ret < 0) {
        LOG_ERR("Failed to write command 0x%04x (%d)", cmd, ret);
        return -EIO;
    }

    k_sleep(K_MSEC(delay_ms));
    
    ret = i2c_read_dt(&cfg->i2c, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to read data (%d)", ret);
        return ret;
    }

    for (int i = 0; i < sizeof(buf); i += (2+1)) {
        temp16 = sys_get_be16(&buf[i]);
        if (scd4x_compute_crc(temp16) != buf[i+2]) {
            LOG_ERR("CRC mismatch");
            return -EIO;
        }

        data[i/(2+1)] = temp16;
    }    

    return ret;
}

static int scd4x_init(const struct device *dev)
{
    const struct scd4x_config *cfg = dev->config;
    uint16_t data[3];
    int ret;

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus device is not ready");
        return -ENODEV;
    }

    ret = scd4x_read_words(dev, SCD4X_CMD_GET_SERIAL_NUMBER, data, 3, 10);
    if (ret < 0) {
        LOG_ERR("Failed to read serial number (%d)", ret);
        return ret;
    }

    LOG_INF("Serial number: %04x%04x%04x", data[0], data[1], data[2]);

    return 0;
}

/**
 * @brief Fetches a sample from the SCD4x sensor.
 *
 * This function is used to fetch a sample from the SCD4x sensor. It takes
 * a pointer to the device structure and the desired sensor channel as
 * parameters. The function returns an integer value indicating the success
 * or failure of the sample fetch operation.
 *
 * @param dev Pointer to the device structure.
 * @param chan The desired sensor channel.
 * @return 0 if successful, negative error code otherwise.
 */
static int scd4x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    const struct scd4x_config *cfg = dev->config;
    struct scd4x_data *data = dev->data;
    //uint16_t data[3];    

    if (scd4x_write_cmd(dev, SCD4X_CMD_START_PERIODIC_MEASUREMENT) < 0) {
        LOG_ERR("Failed to start periodic measurement");
        return -EIO;
    }

    if (scd4x_read_words(dev, SCD4X_CMD_READ_MEASUREMENT, (uint16_t)&data->sample, 3, 1) < 0) {
        LOG_ERR("Failed to read measurement");
        return -EIO;
    } else {
        //data->sample.co2 = data[1];
        //data->sample.temperature = data[2];
        //data->sample.humidity = data[0];
    }

    if (scd4x_write_cmd(dev, SCD4X_CMD_STOP_PERIODIC_MEASUREMENT) < 0) {
        LOG_ERR("Failed to stop periodic measurement");
        return -EIO;
    }    

    return 0;
}

/**
 * @brief Get the sensor value for a specific channel.
 *
 * This function retrieves the sensor value for the specified channel from the SCD4x sensor.
 *
 * @param dev Pointer to the device structure.
 * @param chan The sensor channel to retrieve the value from.
 * @param val Pointer to the sensor value structure to store the retrieved value.
 *
 * @return 0 if successful, negative errno code on failure.
 */
static int scd4x_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    const struct scd4x_config *cfg = dev->config;
    struct scd4x_data *data = dev->data;
    
    int ret;

    switch (chan) {
        case SENSOR_CHAN_CO2:
            scd4x_co2_from_raw(val, data->sample.co2);
            break;

        case SENSOR_CHAN_AMBIENT_TEMP:
            scd4x_temperature_from_raw(val, data->sample.temperature);
            break;

        case SENSOR_CHAN_HUMIDITY:
            scd4x_humidity_from_raw(val, data->sample.humidity);
            break;

        default:
            return -ENOTSUP;
    }

    return 0;
}


/**
 * @brief Sensor driver API for the Sensirion SCD4x sensor.
 *
 * This structure defines the driver API for the Sensirion SCD4x sensor.
 * It provides a set of functions that can be used to interact with the sensor.
 */
static const struct sensor_driver_api scd4x_driver_api = {
    .sample_fetch = scd4x_sample_fetch,
    .channel_get = scd4x_channel_get,
};

#define SCD4X_CONFIG(inst)                                          \
{                                                                   \
    .i2c = I2C_DT_SPEC_INST_GET(inst)                               \
}

#define SCD4X_DEFINE(inst) \
    static struct scd4x_data scd4x_data_##inst; \
    static struct scd4x_config scd4x_config_##inst = SCD4X_CONFIG(inst); \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, \
        scd4x_init, \
        NULL, \
        &scd4x_data_##inst, \
        &scd4x_config_##inst, \
        POST_KERNEL, \
        CONFIG_SENSOR_INIT_PRIORITY, \
        &scd4x_driver_api \
    );

DT_INST_FOREACH_STATUS_OKAY(SCD4X_DEFINE)
