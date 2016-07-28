/*
 * IBM PowerNV platform sensors for power (via AMESTER)
 * Copyright (C) 2016 IBM
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/ipmi.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/device.h>

#define IBMAMESTER_NAME "ibm-amester"

/* Defined in POWER8 Processor Registers Specification */
/* See definition in:
 * https://github.com/open-power/docs/blob/master/occ/OCC_OpenPwr_FW_Interfaces.pdf
 */

struct __be_amester_response {
	u8     dead;
	__be32 timestamp;
	__be32 updates;
	__be64 accumulated_value;
	__be16 value;
	__be16 value_min;
	__be16 value_max;
	__be16 status;
} __packed;

struct amester_response {
	u32 timestamp;
	u32 updates;
	u64 accumulated_value;
	u16 value;
	u16 value_min;
	u16 value_max;
	u16 status;
};

enum amester_power_prop {
	power_sys,
	power_fan,
	power_gpu,
	power_mem,
	power_chip0,
	power_chip1,
	power_disk,
	power_vcs,
	power_vdd,
};

/**
 * @update_interval: Minimum timer interval for sampling In jiffies
 */
struct sensor_attr_data {
	enum amester_power_prop prop;
	u32 attr_id;
	bool valid;
	unsigned long last_updated;
	struct amester_response response;
	struct device_attribute dev_attr;
};

struct sensor_group {
	char *name;
	struct sensor_attr_data *sattr;
	struct attribute_group group;
};

struct amester_ipmi_data {
	struct completion read_complete;

	struct kernel_ipmi_msg tx_message;
	long tx_msgid;

	void *rx_msg_data;
	unsigned short rx_msg_len;
	unsigned char rx_result;
	int rx_recv_type;
};

struct amester_comms {
	struct ipmi_user_hndl ipmi_hndlrs;
	struct ipmi_addr address;
	ipmi_user_t user;
	int interface;
	struct amester_ipmi_data data;
};

/* data private to each client */
struct amester_drv_data {
	struct amester_comms *comms;
	struct device *hwmon_dev;
	struct mutex update_lock;
	unsigned long update_interval;
	struct sensor_group sensor_group;
};

#define POWER_LABEL(x) [x] = #x
const char *amester_power_labels[] = {
	POWER_LABEL(power_sys),
	POWER_LABEL(power_fan),
	POWER_LABEL(power_gpu),
	POWER_LABEL(power_mem),
	POWER_LABEL(power_chip0),
	POWER_LABEL(power_chip1),
	POWER_LABEL(power_disk),
	POWER_LABEL(power_vcs),
	POWER_LABEL(power_vdd),
};

#define OCC_NETFN 0x3a
#define OCC_CMD 0x0d

static char amester_ipmi_power_cmds[][7] = {
	[power_sys] =   { 0x01, 0x41, 0x3c, 0x00, 0x07, 0x00, 0x16 },
	[power_fan] =   { 0x01, 0x41, 0x3c, 0x00, 0x07, 0x00, 0x17 },
	[power_gpu] =   { 0x01, 0x41, 0x3c, 0x00, 0x07, 0x00, 0x1a },
	[power_mem] =   { 0x01, 0x41, 0x3c, 0x00, 0x07, 0x00, 0x36 },
	[power_chip0] = { 0x01, 0x41, 0x3c, 0x00, 0x07, 0x00, 0x32 },
	[power_chip1] = { 0x02, 0x41, 0x3c, 0x00, 0x07, 0x00, 0x32 },
	[power_disk] =  { 0x01, 0x41, 0x3c, 0x00, 0x07, 0x00, 0x19 },
	[power_vcs] =   { 0x01, 0x41, 0x3c, 0x00, 0x07, 0x00, 0x35 },
	[power_vdd] =   { 0x01, 0x41, 0x3c, 0x00, 0x07, 0x00, 0x33 },
};

#define IPMI_TIMEOUT		(30 * HZ)

static void amester_msg_handler(struct ipmi_recv_msg *msg, void *user)
{
	struct amester_ipmi_data *data = user;
	unsigned char resp = IPMI_UNKNOWN_ERR_COMPLETION_CODE;

	if (msg->msgid != data->tx_msgid) {
		pr_err("Mismatch between request and response msgids: %02lx : %02lx",
				msg->msgid, data->tx_msgid);
		data->rx_result = resp;
		goto done;
	}

	data->rx_recv_type = msg->recv_type;
	if (msg->msg.data_len > 0) {
		resp = msg->msg.data[0];
		data->rx_msg_len = min((unsigned short)(msg->msg.data_len - 1),
				data->rx_msg_len);
		memcpy(data->rx_msg_data, msg->msg.data + 1, data->rx_msg_len);
	} else {
		data->rx_msg_len = 0;
	}

	data->rx_result = resp;
done:
	ipmi_free_recv_msg(msg);
	complete_all(&data->read_complete);
}

static u8 amester_send_cmd(struct amester_comms *comms,
			   enum amester_power_prop prop, void *resp, size_t n)
{
	struct amester_ipmi_data *data;
	int err;

	if (!comms)
		return -EINVAL;

	data = &comms->data;

	init_completion(&data->read_complete);

	data->tx_msgid++;
	data->tx_message.netfn = OCC_NETFN;
	data->tx_message.cmd = OCC_CMD;
	data->tx_message.data = amester_ipmi_power_cmds[prop];
	data->tx_message.data_len = sizeof(amester_ipmi_power_cmds[prop]);

	data->rx_msg_data = resp;
	data->rx_msg_len = n;
	data->tx_msgid++;

	err = ipmi_request_settime(comms->user, &comms->address,
			data->tx_msgid, &data->tx_message, data,
			0, 0, 0);

	if (err)
		return -EIO;

	err = wait_for_completion_timeout(&data->read_complete, IPMI_TIMEOUT);
	if (!err)
		return -ETIMEDOUT;

	return 0;
}

static int amester_get(struct amester_comms *comms,
		       enum amester_power_prop sensor,
		       struct amester_response *cooked)
{
	int ret;
	struct __be_amester_response raw;

	memset(&raw, 0, sizeof(raw));

	ret = amester_send_cmd(comms, sensor, &raw, sizeof(raw));
	if (ret)
		return -EINVAL;

	cooked->timestamp = be32_to_cpu(raw.timestamp);
	cooked->updates = be32_to_cpu(raw.updates);
	cooked->accumulated_value = be64_to_cpu(raw.accumulated_value);
	cooked->value = be16_to_cpu(raw.value);
	cooked->value_min = be16_to_cpu(raw.value_min);
	cooked->value_max = be16_to_cpu(raw.value_max);
	cooked->status = be16_to_cpu(raw.status);

	return 0;
}

static int amester_update_device(struct amester_drv_data *data,
				 struct sensor_attr_data *sattr)
{
	int ret = 0;

	if (time_after(jiffies, sattr->last_updated + data->update_interval)
	    || !sattr->valid) {
		ret = amester_get(data->comms, sattr->prop, &sattr->response);
		sattr->valid = (ret ? 0 : 1);
		sattr->last_updated = jiffies;
	}

	return ret;
}

static ssize_t show_input(struct device *hwmon_dev,
			  struct device_attribute *attr, char *buf)
{
	struct device *dev = hwmon_dev->parent;
	struct amester_drv_data *data = dev_get_drvdata(dev);
	struct sensor_attr_data *sattr = container_of(attr,
					struct sensor_attr_data, dev_attr);
	ssize_t ret = 0;

	mutex_lock(&data->update_lock);
	ret = amester_update_device(data, sattr);

	if (ret) {
		dev_warn(dev, "Failed to update sensor %d: %zd\n",
				sattr->prop, ret);
	} else {
		ret = snprintf(buf, PAGE_SIZE - 1, "%d\n",
				sattr->response.value);
	}

	mutex_unlock(&data->update_lock);
	return ret;
}

static ssize_t show_update_interval(struct device *hwmon_dev,
				struct device_attribute *attr, char *buf)
{
	struct device *dev = hwmon_dev->parent;
	struct amester_drv_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%u\n",
		jiffies_to_msecs(data->update_interval));
}

static ssize_t set_update_interval(struct device *hwmon_dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct device *dev = hwmon_dev->parent;
	struct amester_drv_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;

	data->update_interval = msecs_to_jiffies(val);
	return count;
}
static DEVICE_ATTR(update_interval, S_IWUSR | S_IRUGO,
		show_update_interval, set_update_interval);

static ssize_t show_name(struct device *hwmon_dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE - 1, "%s\n", IBMAMESTER_NAME);
}
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static void amester_destroy_sensor_group(struct device *hwmon_dev,
					struct sensor_group *sensor_group)
{
	if (sensor_group->group.attrs)
		devm_kfree(hwmon_dev, sensor_group->group.attrs);

	if (sensor_group->sattr)
		devm_kfree(hwmon_dev, sensor_group->sattr);

	sensor_group->group.attrs = NULL;
	sensor_group->sattr = NULL;
}

static void amester_remove_hwmon_attrs(struct device *hwmon_dev)
{
	struct amester_drv_data *data = dev_get_drvdata(hwmon_dev->parent);

	if (!hwmon_dev)
		return;

	device_remove_file(hwmon_dev, &dev_attr_update_interval);
	device_remove_file(hwmon_dev, &dev_attr_name);

	sysfs_remove_group(&hwmon_dev->kobj, &data->sensor_group.group);

	amester_destroy_sensor_group(hwmon_dev, &data->sensor_group);
}

static int amester_create_sensor_group(struct device *hwmon_dev)
{
	struct amester_drv_data *data = dev_get_drvdata(hwmon_dev->parent);
	struct sensor_group *group = &data->sensor_group;
	struct sensor_attr_data *sdata;
	int sensor_num = ARRAY_SIZE(amester_ipmi_power_cmds);
	int ret;
	int i;

	group->group.attrs = devm_kzalloc(hwmon_dev,
			sizeof(struct attribute *) * sensor_num + 1,
			GFP_KERNEL);
	if (!group->group.attrs) {
		ret = -ENOMEM;
		goto err;
	}

	group->sattr = devm_kzalloc(hwmon_dev,
			sizeof(struct sensor_attr_data) *
			sensor_num, GFP_KERNEL);
	if (!group->sattr) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < sensor_num; i++) {
		sdata = &group->sattr[i];

		sdata->prop = i;

		sysfs_attr_init(&sdata->dev_attr.attr);
		sdata->dev_attr.attr.name = amester_power_labels[i];
		sdata->dev_attr.attr.mode = S_IRUGO;
		sdata->dev_attr.show = show_input;

		group->group.attrs[i] = &sdata->dev_attr.attr;
	}

	ret = sysfs_create_group(&hwmon_dev->kobj, &group->group);
	if (ret)
		goto err;

	return ret;
err:
	amester_destroy_sensor_group(hwmon_dev, group);
	return ret;
}

static int amester_create_hwmon_attrs(struct device *hwmon_dev,
		struct amester_drv_data *drv_data)
{
	struct sensor_group *group = &drv_data->sensor_group;
	int ret;

	ret = device_create_file(hwmon_dev, &dev_attr_name);
	if (ret)
		goto error;

	ret = device_create_file(hwmon_dev, &dev_attr_update_interval);
	if (ret)
		goto error;

	group->name = "power";
	ret = amester_create_sensor_group(hwmon_dev);
	if (ret)
		goto error;

	return 0;
error:
	pr_err("ERROR: cannot create hwmon attributes\n");
	amester_remove_hwmon_attrs(drv_data->hwmon_dev);
	return ret;
}

static int amester_create_sysfs_attr(struct device *dev)
{
	struct amester_drv_data *data = dev_get_drvdata(dev);
	int err;

	if (!data)
		return -EINVAL;

	data->hwmon_dev = hwmon_device_register(dev);
	if (IS_ERR(data->hwmon_dev))
		return PTR_ERR(data->hwmon_dev);

	err = amester_create_hwmon_attrs(data->hwmon_dev, data);
	if (err) {
		hwmon_device_unregister(data->hwmon_dev);
		return err;
	}

	data->hwmon_dev->parent = dev;

	return 0;
}

struct amester_comms comms_data = {
	.ipmi_hndlrs = {
		.ipmi_recv_hndl = amester_msg_handler,
	},
};

static int amester_init_comms(struct amester_comms *comms)
{
	int err;

	comms->address.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
	comms->address.channel = IPMI_BMC_CHANNEL;
	comms->address.data[0] = 0;
	comms->interface = 0 /* needs to be configurable */;

	err = ipmi_create_user(comms->interface, &comms->ipmi_hndlrs,
			&comms->data, &comms->user);
	if (err)
		return err;

	err = ipmi_validate_addr(&comms->address, sizeof(comms->address));
	if (err)
		return err;

	return 0;
}

static int amester_destroy_comms(struct amester_comms *comms)
{
	return ipmi_destroy_user(comms->user);
}

static int amester_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct amester_drv_data *drvdata;
	int ret = 0;

	drvdata = devm_kzalloc(dev, sizeof(struct amester_drv_data),
			GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->comms = &comms_data;

	ret = amester_init_comms(drvdata->comms);
	if (ret)
		return ret;

	dev_set_drvdata(dev, drvdata);
	mutex_init(&drvdata->update_lock);
	drvdata->update_interval = HZ;

	amester_create_sysfs_attr(dev);

	return ret;
}

static int amester_remove(struct platform_device *pdev)
{
	struct amester_drv_data *data = dev_get_drvdata(&pdev->dev);
	int err;

	if (!data->hwmon_dev)
		return 0;

	amester_remove_hwmon_attrs(data->hwmon_dev);
	hwmon_device_unregister(data->hwmon_dev);

	err = amester_destroy_comms(data->comms);
	if (err)
		dev_warn(&pdev->dev, "Failed to destroy IPMI user: %d\n", err);

	return 0;

}

static struct platform_driver amester_driver = {
	.probe = amester_probe,
	.remove = amester_remove,
	.driver = {
		.name = KBUILD_MODNAME,
	},
};

static struct platform_device *amester_pdev;

static int __init amester_init(void)
{
	int err;

	err = platform_driver_register(&amester_driver);
	if (err)
		goto exit;

	amester_pdev = platform_device_alloc("amester", 0);
	if (!amester_pdev) {
		err = -ENOMEM;
		goto exit_driver_unregister;
	}

	err = platform_device_add(amester_pdev);
	if (err)
		goto exit_device_put;

	return 0;

exit_device_put:
	platform_device_put(amester_pdev);
exit_driver_unregister:
	platform_driver_unregister(&amester_driver);
exit:
	return err;
}

static void __exit amester_exit(void)
{
	platform_device_unregister(amester_pdev);
	platform_driver_unregister(&amester_driver);
}

module_init(amester_init);
module_exit(amester_exit);

MODULE_AUTHOR("Andrew Jeffery <andrew@aj.id.au>");
MODULE_DESCRIPTION("POWER8 OCC hwmon driver");
MODULE_LICENSE("GPL");
