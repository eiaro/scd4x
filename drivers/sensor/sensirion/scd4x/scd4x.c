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
    return crc8(buf, 2, CRC8_POLYNOMIAL, CRC8_INIT, false);
}

static int scd4x_send_cmd(const struct device *dev, uint16_t cmd)
{
    LOG_DBG("Sending command 0x%04x", cmd);
    const struct scd4x_config *cfg = dev->config;
    uint8_t buf[2] = {cmd >> 8, cmd & 0xff};
    int ret;

    ret = i2c_write_dt(&cfg->i2c, buf, 2);
    if (ret < 0)
    {
        LOG_ERR("Failed to write command 0x%04x (%d)", cmd, ret);
    }

    return ret;
}

static int scd4x_read_register(const struct device *dev, uint16_t reg, uint16_t *data, size_t data_len)
{
    LOG_DBG("Reading register 0x%04x", reg);
    const struct scd4x_config *cfg = dev->config;
    uint8_t buf[data_len * (SENSIRION_WORD_SIZE  + CRC8_LEN)];
    uint8_t cmd[2] = {reg >> 8, reg & 0xff};
    int ret;

    ret = i2c_write_read_dt(&cfg->i2c, cmd, 2, buf, sizeof(buf));
    if (ret != 0)
    {
        LOG_ERR("Failed to read register 0x%04x (%d)", reg, ret);
        return SCD4X_ERR_I2C_READ;
    }
    
    for (int i = 0, cnt = 0; i < sizeof(buf); i+=(SENSIRION_WORD_SIZE + CRC8_LEN))
    {
        uint16_t value;
        uint8_t crc;

        value = sys_get_be16(&buf[i]);
        crc = buf[i + SENSIRION_WORD_SIZE];

        if (crc != scd4x_compute_crc(value))
        {
            LOG_ERR("CRC mismatch for register 0x%04x", reg);
            return SCD4X_ERR_CRC;
        }

        data[cnt++] = value;
    }

    return SCD4X_OK;
}

static int scd4x_get_serial_number(const struct device *dev)
{
    LOG_DBG("Getting serial number");    
    struct scd4x_data *data = dev->data;
    int ret;

    ret = scd4x_read_register(dev, SCD4X_CMD_GET_SERIAL_NUMBER, data->serial_number, 3);
    if (ret != SCD4X_OK)
    {
        LOG_ERR("Failed to read serial number");
    }

    return SCD4X_OK;
}

static int scd4x_wake_up(const struct device *dev)
{
    LOG_DBG("Waking up sensor");
    (void)scd4x_send_cmd(dev, SCD4X_CMD_WAKE_UP);
    k_sleep(K_MSEC(20));

    return SCD4X_OK;
}

static int scd4x_stop_periodic_measurement(const struct device *dev)
{
    LOG_DBG("Stopping periodic measurement");
    (void)scd4x_send_cmd(dev, SCD4X_CMD_STOP_PERIODIC_MEASUREMENT);
    k_sleep(K_MSEC(500));

    return SCD4X_OK;
}

static int scd4x_reinit(const struct device *dev)
{
    LOG_DBG("Reinitializing sensor");
    (void)scd4x_send_cmd(dev, SCD4X_CMD_REINIT);
    k_sleep(K_MSEC(30));

    return SCD4X_OK;
}

static int scd4x_start_periodic_measurement(const struct device *dev)
{
    LOG_DBG("Starting periodic measurement");
    (void)scd4x_send_cmd(dev, SCD4X_CMD_START_PERIODIC_MEASUREMENT);
    k_sleep(K_MSEC(100)); 

    return SCD4X_OK;
}

static int scd4x_get_data_ready_status(const struct device *dev)
{
    LOG_DBG("Getting data ready status");
    uint16_t data;
    int ret;

    ret = scd4x_read_register(dev, SCD4X_CMD_GET_DATA_READY, &data, 1);
    if (ret != SCD4X_OK)
    {
        LOG_ERR("Failed to get data ready status");
    }

    if ((data & 0x7ff) != 0)
    {
        return 1;
    } else {
        return 0;
    }
}

static int scd4x_read_measurement(const struct device *dev)
{
    LOG_DBG("Reading measurement");
    const struct scd4x_config *cfg = dev->config;
    struct scd4x_data *data = dev->data;
    uint16_t buf[3];
    int ret;

    ret = scd4x_read_register(dev, SCD4X_CMD_READ_MEASUREMENT, buf, 3);
    if (ret != SCD4X_OK)
    {
        LOG_ERR("Failed to read measurement");        
    } else {
        data->sample.co2 = buf[0];
        data->sample.temperature = buf[1];
        data->sample.humidity = buf[2];
    }

    return SCD4X_OK;
}

static int scd4x_init(const struct device *dev)
{
    LOG_DBG("Initializing sensor %s (%p) for sensor API", dev->name, dev);
    struct scd4x_config *cfg = dev->config;
    struct scd4x_data *data = dev->data;
    int ret;

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus device is not ready");
        return -ENODEV;
    }

    ret = scd4x_wake_up(dev);                       // for SCD41 only
    if (ret != SCD4X_OK)
    {
        // The wakeup failed and if nothing else is wrong, this indicate a SCD41 sensor.
        if (cfg->sensor_type == SCD4X_SENSOR_TYPE_AUTO)
            cfg->sensor_type = SCD4X_SENSOR_TYPE_SCD41;
    }
    (void)scd4x_stop_periodic_measurement(dev);     // incase sensor is in periodic measurement mode
    (void)scd4x_reinit(dev);                        // lastly we reinit the sensor

    ret = scd4x_get_serial_number(dev);

    LOG_INF("Serial number: %04x%04x%04x", data->serial_number[0], data->serial_number[1], data->serial_number[2]);

    return 0;
}


static int scd4x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    const struct scd4x_config *cfg = dev->config;
    // struct scd4x_data *data = dev->data;
    int ret = 0;
    uint16_t data[3];

    scd4x_start_periodic_measurement(dev);

    k_sleep(K_MSEC(4000));

    while (!scd4x_get_data_ready_status(dev))
    {
        LOG_INF("Data not ready");
        k_sleep(K_MSEC(500));
    }

    scd4x_read_measurement(dev);
            
    scd4x_stop_periodic_measurement(dev);
    
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

    switch (chan)
    {
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

#define SCD4X_CONFIG(inst)                \
    {                                     \
        .i2c = I2C_DT_SPEC_INST_GET(inst) \
    }

#define SCD4X_DEFINE(inst)                                               \
    static struct scd4x_data scd4x_data_##inst;                          \
    static struct scd4x_config scd4x_config_##inst = SCD4X_CONFIG(inst); \
    SENSOR_DEVICE_DT_INST_DEFINE(inst,                                   \
                                 scd4x_init,                             \
                                 NULL,                                   \
                                 &scd4x_data_##inst,                     \
                                 &scd4x_config_##inst,                   \
                                 POST_KERNEL,                            \
                                 CONFIG_SENSOR_INIT_PRIORITY,            \
                                 &scd4x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCD4X_DEFINE)
