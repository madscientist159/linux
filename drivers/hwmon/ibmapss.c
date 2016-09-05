/*
 * Copyright (C) 2016 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#define NR_APSS_CHANNELS 16

struct ibmapss_sensor {
	const char *name;
	u32 channel;
	u32 scale;
};

struct ibmapss {
	struct mutex lock;
	struct i2c_client *client;
	struct ibmapss_sensor *sensors;
};

struct ibmapss_sysfs {
	struct sensor_device_attribute *sdattrs;
	struct attribute **attrs;
	struct attribute_group group;
	const struct attribute_group *groups[2];
};

struct ibmapss_drvdata {
	size_t n_sensors;
	struct ibmapss apss;
	struct device *hwmon_dev;
	struct ibmapss_sysfs sysfs;
};

static inline u8 ch_to_addr(u8 ch)
{
	return 0x14 + 2 * ch;
}

static ssize_t get_input(struct device *dev, struct device_attribute *da,
		char *buf)
{
	struct ibmapss_drvdata *data = dev_get_drvdata(dev);
	struct ibmapss_sensor *sensor;
	struct i2c_client *client = data->apss.client;
	size_t idx = to_sensor_dev_attr(da)->index;
	u8 addr;
	ssize_t ret = 0;
	u16 val = 0;
	int i;

	WARN_ON(idx >= data->n_sensors);
	sensor = &data->apss.sensors[idx];
	addr = ch_to_addr(sensor->channel);

	mutex_lock(&data->apss.lock);
	for (i = 0; i < 2; i++) {
		ret = i2c_smbus_read_byte_data(client, (addr + i));
		if (ret < 0)
			goto err;
		val |= (ret << (i * 8));
	}
	mutex_unlock(&data->apss.lock);

	return sprintf(buf, "%u\n", (val / sensor->scale));

err:
	return ret;
}

static int ibmapss_parse_one_of_sensor(struct device *dev,
		struct ibmapss_sensor *sensor, struct device_node *of_sensor)
{
	int ret;

	sensor->name = devm_kstrdup(dev, of_sensor->name, GFP_KERNEL);
	if (!sensor->name)
		return -ENOMEM;

	ret = of_property_read_u32(of_sensor, "channel", &sensor->channel);
	if (ret < 0) {
		dev_warn(dev, "Failed to read channel value: %d", ret);
		return ret;
	}

	ret = of_property_read_u32(of_sensor, "scale", &sensor->scale);
	if (ret < 0) {
		dev_warn(dev, "Failed to read scale value: %d", ret);
		return ret;
	}

	return 0;
}

static int ibmapss_parse_all_of_sensors(struct device *dev,
		struct ibmapss_drvdata *data)
{
	struct device_node *of_sensor = NULL;
	struct ibmapss_sensor defined[NR_APSS_CHANNELS];

	while ((of_sensor = of_get_next_child(dev->of_node, of_sensor))) {
		int ret;

		ret = ibmapss_parse_one_of_sensor(dev,
				&defined[data->n_sensors], of_sensor);
		if (ret < 0)
			return ret;

		data->n_sensors++;
	}

	data->apss.sensors = devm_kmemdup(dev, defined,
			data->n_sensors * sizeof(*defined),
			GFP_KERNEL);
	if (!data->apss.sensors)
		return -ENOMEM;

	return 0;
}

static int ibmapss_init_attrs(struct device *dev, struct ibmapss_drvdata *data)
{
	struct ibmapss_sysfs *sysfs = &data->sysfs;
	int i;
	size_t size;

	size = data->n_sensors * sizeof(*sysfs->sdattrs);
	sysfs->sdattrs = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!sysfs->sdattrs)
		return -ENOMEM;

	size = data->n_sensors * sizeof(*sysfs->attrs) + 1;
	sysfs->attrs = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!sysfs->attrs)
		return -ENOMEM;

	for (i = 0; i < data->n_sensors; i++) {
		struct sensor_device_attribute *sdattr = &sysfs->sdattrs[i];
		const char *name = data->apss.sensors[i].name;

		sysfs_attr_init(&sdattr->dev_attr.attr);
		sdattr->dev_attr.attr.name = name;
		sdattr->dev_attr.attr.mode = S_IRUGO;
		sdattr->dev_attr.show = get_input;
		sdattr->dev_attr.store = NULL;
		sdattr->index = i;

		sysfs->attrs[i] = &sysfs->sdattrs[i].dev_attr.attr;
	}

	sysfs->group.attrs = sysfs->attrs;
	sysfs->groups[0] = &sysfs->group;
	sysfs->groups[1] = NULL;

	return 0;
}

static int ibmapss_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ibmapss_drvdata *data;

	dev_info(&client->dev, "%s called", __func__);

	data = devm_kzalloc(&client->dev, sizeof(struct ibmapss_drvdata),
			GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ibmapss_parse_all_of_sensors(&client->dev, data);
	ibmapss_init_attrs(&client->dev, data);

	i2c_set_clientdata(client, data);
	data->apss.client = client;
	mutex_init(&data->apss.lock);

	data->hwmon_dev = devm_hwmon_device_register_with_groups(&client->dev,
			client->name, data, data->sysfs.groups);

	if (IS_ERR(data->hwmon_dev))
		return PTR_ERR(data->hwmon_dev);

	return 0;
}

static int ibmapss_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id ibmapss_dt_match[] = {
	{ .compatible = "ibm,apss", },
	{ },
};
MODULE_DEVICE_TABLE(of, ibmapss_dt_match);

static const struct i2c_device_id ibmapss_id[] = {
	{ "ibmapss", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ibmapss_id);

static struct i2c_driver ibmapss_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.of_match_table = of_match_ptr(ibmapss_dt_match),
		.name = "ibmapss",
	},
	.probe = ibmapss_probe,
	.remove = ibmapss_remove,
	.id_table = ibmapss_id,
};

module_i2c_driver(ibmapss_driver);

MODULE_AUTHOR("Andrew Jeffery");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IBM APSS driver");
