#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timekeeping.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/gpio/driver.h>
#include <linux/delay.h>
#include <linux/sched.h>

/**
 * | S0 | S1 | Output Freq Scaling
 * ---------------------------------
 * |  L |  L | Power down
 * |  L |  H | 2%
 * |  H |  L | 20%
 * |  H |  H | 100%
*/

/**
 * | S2 | S3 | PHOTODIODE TYPE
 * --------------------------------
 * |  L |  L | RED
 * |  L |  H | BLUE
 * |  H |  L | Clear (no filter)
 * |  H |  H | GREEN
 */

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mr Marshy");
MODULE_DESCRIPTION("A Kernel Module for Reading RGB Values from TCS3200 Colour Sensor");


struct TCS3200 {
	int s0Pin;
	int s1Pin;
	int s2Pin;
	int s3Pin;
	int pulseOutPin;
	struct gpio_desc *s0Desc;
	struct gpio_desc *s1Desc;
	struct gpio_desc *s2Desc;
	struct gpio_desc *s3Desc;
	struct gpio_desc *pulseOutDesc;
	struct timeval time_of_first_edge;
	struct timeval time_of_second_edge;
	volatile int edge_triggered;
	volatile int pulse_received;
	struct mutex measurement_mutex;
	wait_queue_head_t wait_for_egde;
	unsigned long timeout;
	struct list_head list;
};

static LIST_HEAD(tcs3200_devices);
static DEFINE_MUTEX(devices_mutex);

static struct TCS3200 *createTCS3200(int s0Pin, int s1Pin, int s2Pin, int s3Pin, int pulsePin, unsigned long timeout){
	
	struct TCS3200 *new;
	int err;
	
	new = kmalloc(sizeof(*new), GFP_KERNEL);
	if(!new){
		return ERR_PTR(-ENOMEM);
	}

	new->s0Pin = s0Pin;
	new->s1Pin = s1Pin;
	new->s2Pin = s2Pin;
	new->s3Pin = s3Pin;
	new->pulseOutPin = pulsePin;

	new->s0Desc = gpio_to_desc(s0Pin);
	if(!new->s0Desc){
		kfree(new);
		return ERR_PTR(-EINVAL);
	}
	new->s1Desc = gpio_to_desc(s1Pin);
	if(!new->s1Desc){
		kfree(new);
		return ERR_PTR(-EINVAL);
	}
	new->s2Desc = gpio_to_desc(s2Pin);
	if(!new->s2Desc){
		kfree(new);
		return ERR_PTR(-EINVAL);
	}
	new->s3Desc = gpio_to_desc(s3Pin);
	if(!new->s3Desc){
		kfree(new);
		return ERR_PTR(-EINVAL);
	}
	new->pulseOutDesc = gpio_to_desc(pulsePin);
	if(!new->pulseOutDesc){
		kfree(new);
		return ERR_PTR(-EINVAL);
	}

	err = gpiod_direction_input(new->pulseOutDesc);
	if(err < 0){
		kfree(new);
		return ERR_PTR(err);
	}
	err = gpiod_direction_output(new->s0Desc, 0);
	if(err < 0){
		kfree(new);
		return ERR_PTR(err);
	}
	err = gpiod_direction_output(new->s1Desc, 0);
	if(err < 0){
		kfree(new);
		return ERR_PTR(err);
	}
	err = gpiod_direction_output(new->s2Desc, 0);
	if(err < 0){
		kfree(new);
		return ERR_PTR(err);
	}
	err = gpiod_direction_output(new->s3Desc, 0);
	if(err < 0){
		kfree(new);
		return ERR_PTR(err);
	}

	/* Output Freq Scaling 20% */
	gpiod_set_value(new->s0Desc, 1);
	gpiod_set_value(new->s1Desc, 0);

	/* Red Filter */
	gpiod_set_value(new->s2Desc, 0);
	gpiod_set_value(new->s3Desc, 0);

	mutex_init(&new->measurement_mutex);
	init_waitqueue_head(&new->wait_for_egde);

	new->timeout = timeout;

	list_add_tail(&new->list, &tcs3200_devices);

	return new;
}


static irqreturn_t edge_detected_irq(int irq, void *data){

	struct TCS3200 *device = (struct TCS3200 *)data;
	int val;
	struct timeval irq_tv;

	do_gettimeofday(&irq_tv);

	if(device->pulse_received){
		return IRQ_HANDLED;
	}

	val = gpiod_get_value(device->pulseOutDesc);
	if(val == 1){
		device->time_of_first_edge = irq_tv;
		device->edge_triggered = 1;
	}
	else if((val == 0) && (device->edge_triggered == 1)){
		device->time_of_second_edge = irq_tv;
		device->pulse_received = 1;
		device->edge_triggered = 0;
		wake_up_interruptible(&device->wait_for_egde);
	}
	else{
		return IRQ_HANDLED;
	}

	return IRQ_HANDLED;

}

/**
 * @brief devices_mutex must be held by caller, so nobody
 * deletes the device before it is locked.
 */
static int do_measurement(struct TCS3200 *device, unsigned long long *usecs_elapsed){
	long timeout;
	int irq;
	int ret;

	if(!mutex_trylock(&device->measurement_mutex)){
		mutex_unlock(&devices_mutex);
		return -EBUSY;
	}

	mutex_unlock(&devices_mutex);

	irq = gpiod_to_irq(device->pulseOutDesc);
	if(irq < 0){
		printk("Unable to set irq number on out pin\n");
		return -EIO;
	}

	device->edge_triggered = 0;
	device->pulse_received = 0;

	ret = request_any_context_irq(irq, edge_detected_irq,
			IRQF_SHARED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"tcs3200", device);

	if(ret < 0){
		goto out_mutex;
	}

	ret = gpiochip_lock_as_irq(gpiod_to_chip(device->pulseOutDesc),
				device->pulseOutPin);

	if(ret < 0){
		goto out_irq;
	}

	timeout = wait_event_interruptible_timeout(device->wait_for_egde,
					device->pulse_received, device->timeout);

	if(timeout == 0){
		ret = -ETIMEDOUT;
	}
	else if(timeout < 0){
		ret = timeout;
	}
	else{
		*usecs_elapsed = 
		(device->time_of_second_edge.tv_sec - device->time_of_first_edge.tv_sec) * 1000000 +
		(device->time_of_second_edge.tv_usec - device->time_of_first_edge.tv_usec);
	
		ret = 0;
	}

out_irq:
	free_irq(irq, device);

out_mutex:
	mutex_unlock(&device->measurement_mutex);

	return ret;
}


static ssize_t sysfs_do_measurement(struct device *dev, struct device_attribute *attr, char *buf){

	struct TCS3200 *sensor = dev_get_drvdata(dev);
	unsigned long long usecs_elapsed = -1;
	int status;
	
	mutex_lock(&devices_mutex);
	status = do_measurement(sensor, &usecs_elapsed);

	if(status < 0){
		return status;
	}

	return sprintf(buf, "%lld\n", usecs_elapsed);
}


DEVICE_ATTR(measure, 0444, sysfs_do_measurement, NULL);

static struct attribute *sensor_attrs[] = {
	&dev_attr_measure.attr,
	NULL,
};

static const struct attribute_group sensor_group = {
	.attrs  = sensor_attrs
};

static const struct attribute_group *sensor_groups[] = {
	&sensor_group,
	NULL
};

static ssize_t sysfs_configure_store(struct class *class,
				struct class_attribute *attr,
				const char *buf, size_t len);


static ssize_t sysfs_colour_filter_store(struct class *class,
				struct class_attribute *attr,
				const char *buf, size_t len);

static struct class_attribute tcs3200_class_attrs[] = {
	__ATTR(configure, 0200, NULL, sysfs_configure_store),
	__ATTR(filter_type, 0200, NULL, sysfs_colour_filter_store),
	__ATTR_NULL,
};

static struct class tcs3200_class = {
	.name = "colour-sensor",
	.owner = THIS_MODULE,
	.class_attrs = tcs3200_class_attrs
};

static struct TCS3200 *find_sensor(int s0, int s1, int s2, int s3, int pulse){
	struct TCS3200 *sensor;

	list_for_each_entry(sensor, &tcs3200_devices, list){
		if((sensor->s0Pin == s0) &&
			(sensor->s1Pin == s1) &&
			(sensor->s2Pin == s2) &&
			(sensor->s3Pin == s3) &&
			(sensor->pulseOutPin = pulse)){
				
			return sensor;
		}
	}

	return NULL;
}

static int match_device(struct device *dev, const void *data){
	return dev_get_drvdata(dev) == data;
}

static int remove_sensor(struct TCS3200 *rip_sensor){
	/* Must be called with devices_mutex held */
	struct device *dev;

	dev = class_find_device(&tcs3200_class, NULL, rip_sensor, match_device);
	if(!dev){
		return -ENODEV;
	}
	
	/* wait until measurement is finished */
	mutex_lock(&rip_sensor->measurement_mutex);

	
	list_del(&rip_sensor->list);
	kfree(rip_sensor);

	device_unregister(dev);
	put_device(dev);

	return 0;
}

static ssize_t sysfs_configure_store(struct class *class,
				struct class_attribute *attr,
				const char *buf, size_t len){


	int add = buf[0] != '-';
	const char *s = buf;
	int s0, s1, s2, s3, pulse, timeout;
	struct TCS3200 *new_sensor, *rip_sensor;
	int err;

	if(buf[0] == '-' || buf[0] == '+'){
		s++;
	}
	if(add){
		if(sscanf(s, "%d %d %d %d %d %d", &s0, &s1, &s2, &s3, &pulse, &timeout) != 6){
			return -EINVAL;
		}

		mutex_lock(&devices_mutex);
		if(find_sensor(s0, s1, s2, s3, pulse)){
			mutex_unlock(&devices_mutex);
			return -EEXIST;
		}

		new_sensor = createTCS3200(s0, s1, s2, s3, pulse, timeout);
		mutex_unlock(&devices_mutex);
		if(IS_ERR(new_sensor)){
			return PTR_ERR(new_sensor);
		}

		device_create_with_groups(class, NULL, MKDEV(0, 0), new_sensor,
							sensor_groups, "pulsewidth_%d_%d_%d_%d_%d", s0, s1, s2, s3, pulse);
	}
	else{

		if(sscanf(s, "%d %d %d %d %d", &s0, &s1, &s2, &s3, &pulse) != 5){
			return -EINVAL;
		}

		mutex_lock(&devices_mutex);
		rip_sensor = find_sensor(s0, s1, s2, s3, pulse);
		if(!rip_sensor){
			mutex_unlock(&devices_mutex);
			return -ENODEV;
		}

		err = remove_sensor(rip_sensor);
		mutex_unlock(&devices_mutex);
		if(err < 0){
			return err;
		}
	}

	return len;
}

static ssize_t sysfs_colour_filter_store(struct class *class,
				struct class_attribute *attr,
				const char *buf, size_t len){


	const char *s = buf;
	int s0, s1, s2, s3, pulse;
	int s2_new_val;
	int s3_new_val;
	struct TCS3200 *_sensor;

	if(sscanf(s, "%d %d %d %d %d %d %d", 
				&s0, &s1, &s2, &s3, &pulse, &s2_new_val, &s3_new_val) != 7){
		return -EINVAL;
	}

	mutex_lock(&devices_mutex);
	_sensor = find_sensor(s0, s1, s2, s3, pulse);

	if(!_sensor){
		mutex_unlock(&devices_mutex);
		return -ENODEV;
	}

	/* Apply new colour filter */
	gpiod_set_value(_sensor->s2Desc, s2_new_val);
	gpiod_set_value(_sensor->s3Desc, s3_new_val);

	mutex_unlock(&devices_mutex);

	
	return len;
}

static int __init init_tcs3200(void){
	return class_register(&tcs3200_class);
}

static void exit_tcs3200(void){
	struct TCS3200 *rip_sensor, *tmp;

	mutex_lock(&devices_mutex);
	list_for_each_entry_safe(rip_sensor, tmp, &tcs3200_devices, list){
		remove_sensor(rip_sensor); /* ignore errors */
	}

	mutex_unlock(&devices_mutex);

	class_unregister(&tcs3200_class);
}

module_init(init_tcs3200);
module_exit(exit_tcs3200);

