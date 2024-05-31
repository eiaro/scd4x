# Zephyr driver for Sensirion SCD4X CO<sub>2</sub> sensor

This is a driver for the Sensirion SCD4X sensor implemented as a driver module for Zephyr RTOS.

Released as my first out of tree driver module, so use with caution.

## Usage 

### Module installation

Add the following to your `west.yml` manifest:
```yaml
- name: scd4x
  url: https://github.com/eiaro/scd4x
  revision: main
```

### Driver configuration

Make a new or add to your `.overlay` like this example:
```dts
&i2c0 {
    status = "okay";
    clock-frequency = <I2C_BITRATE_STANDARD>;
    scd4x@62 {
        compatible = "sensirion,scd4x";
        reg = <0x62>;
        status = "okay";
    };
};
```

When the driver is in the devicetree, the defaults should kick in and Kconfig should modified. Relevant strings to look for is:
```ini
CONFIG_I2C=y
CONFIG_SENSOR=y
CONFIG_SCD4X=y
```

### C code

And an example to show the sensor in action.

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <sensor/sensirion/scd4x/scd4x.h>
#include <app_version.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

const struct device *scd4x_dev = DEVICE_DT_GET_ANY(sensirion_scd4x);

int main(void)
{	
	int ret;
	__ASSERT(scd4x_dev, "Could not get SCD4X device");

	if (!device_is_ready(scd4x_dev)) {
		LOG_ERR("SCD4X device is not ready");
		return -ENODEV;
	}	

	while (1) {
		ret = sensor_sample_fetch(scd4x_dev);
		if (ret) {
			LOG_ERR("Failed to fetch sample from SCD4X: %d", ret);
			return ret;
		} else{
			struct sensor_value co2, temperature, humidity;

			ret = sensor_channel_get(scd4x_dev, SENSOR_CHAN_CO2, &co2);
			if (ret) {
				LOG_ERR("Failed to get CO2 sample: %d", ret);
				return ret;
			}

			ret = sensor_channel_get(scd4x_dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
			if (ret) {
				LOG_ERR("Failed to get temperature sample: %d", ret);
				return ret;
			}

			ret = sensor_channel_get(scd4x_dev, SENSOR_CHAN_HUMIDITY, &humidity);
			if (ret) {
				LOG_ERR("Failed to get humidity sample: %d", ret);
				return ret;
			}

			LOG_INF("CO2: %d ppm, Temperature: %d.%06d C, Humidity: %d.%06d %%",
				co2.val1, temperature.val1, temperature.val2, humidity.val1, humidity.val2);
		}

		k_sleep(K_MSEC(5000));
	}

	return 0;
}
```
## Implentation

| Command                                        | Implemented? |
|------------------------------------------------|--------------|
| start_periodic_measurement                     | yes          |
| read_measurement                               | yes          |
| stop_periodic_measurement                      | yes          |
| set_temperature_offset                         | no           |
| get_temperature_offset                         | no           |
| set_sensor_altitude                            | no           |
| get_sensor_altitude                            | no           |
| set_ambient_pressure                           | no           |
| get_ambient_pressure                           | no           |
| perform_forced_recalibration                   | no           |
| set_automatic_self_calibration_enabled         | no           |
| get_automatic_self_calibration_enabled         | no           |
| start_low_power_periodic_measurement           | no           |
| get_stata_ready_status                         | no           | 
| persist_settings                               | no           |
| get_serial_number                              | yes          |
| perform_self_test                              | no           |
| perform_factory_reset                          | no           |
| reinit                                         | yes          |
| measure_single_shot                            | no           |
| measure_single_shot_rht_only                   | no           |
| power_down                                     | no           |
| wake_up                                        | yes          |
| set_automatic_self_calibration_initial_period  | no           |
| get_automatic_self_calibration_initial_period  | no           |
| set_automatic_self_calibration_standard_period | no           |
| get_automatic_self_calibration_standard_period | no           |