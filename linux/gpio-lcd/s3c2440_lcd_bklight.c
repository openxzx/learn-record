#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <mach/regs-gpio.h>
#include <mach/hardware.h>

#define DEVICE_NAME	"lcd_bklight_ctl"
#define LCD_BKLIGHT_MAJOR 231

static unsigned long lcd_bklight_ctl_pin = S3C2410_GPF1;
static unsigned int lcd_bklight_cfg_direction = S3C2410_GPF1_OUTP;

struct class *lcd_bklight_class;

static int s3c2440_lcd_bklight_ioctl(
	struct inode *inode,
	struct file *file,
	unsigned int cmd)
{
	switch(cmd) {
	case 0:
	case 1:
		s3c2410_gpio_setpin(lcd_bklight_ctl_pin, !cmd);
		return 0;
	default:
		return -EINVAL;
	}
}

static struct file_operations s3c2440_lcd_bklight_fops = {
	.owner	=	THIS_MODULE,
	.ioctl	=	s3c2440_lcd_bklight_ioctl,
};

static int __init s3c2440_lcd_bklight_init(void)
{
	int ret;

	ret = register_chrdev(LCD_BKLIGHT_MAJOR, DEVICE_NAME, &s3c2440_lcd_bklight_fops);
	if (ret < 0)
		return ret;

	//devfs_mk_cdev(MKDEV(LCD_BKLIGHT_MAJOR, 0), S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP, DEVICE_NAME);  (2.6.12 publish)

	/* Auto create device inode file starting */
	lcd_bklight_class = class_create(THIS_MODULE, "lcd_bklight_class");
	if(IS_ERR(lcd_bklight_class)) {
		printk("Err: failed in creating class./n");
		return -1;
	}

        /* auto create device inode file endding */
        device_create(lcd_bklight_class, NULL, MKDEV(LCD_BKLIGHT_MAJOR, 0), NULL, "lcd_bklight");

	s3c2410_gpio_cfgpin(lcd_bklight_ctl_pin, lcd_bklight_cfg_direction);
	s3c2410_gpio_setpin(lcd_bklight_ctl_pin, 0);

	return 0;
}

static void __exit s3c2440_lcd_bklight_exit(void)
{
	//devfs_remove(DEVICE_NAME);  (2.6.12 publish)

	/* delete device inode file */
	device_destroy(lcd_bklight_class, MKDEV(LCD_BKLIGHT_MAJOR, 0));
	class_destroy(lcd_bklight_class);
	unregister_chrdev(LCD_BKLIGHT_MAJOR, DEVICE_NAME);
}

module_init(s3c2440_lcd_bklight_init);
module_exit(s3c2440_lcd_bklight_exit);
MODULE_LICENSE("GPL");
