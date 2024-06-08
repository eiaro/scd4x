#ifndef DRIVER_SENSOR_SCD4X_H
#define DRIVER_SENSOR_SCD4X_H

#define IT_WORKS 1

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

#endif /* DRIVER_SENSOR_SCD4X_H */