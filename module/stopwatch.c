#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <linux/version.h>

#define DEV_DRIVER_MAJOR 242
#define DEV_DRIVER_MINOR 0
#define DEV_DRIVER_NAME "stopwatch"

#define IOM_FND_ADDRESS 0x08000004	// address of fnd

// Global variable for fpga_modules
static int dev_driver_usage = 0;
static unsigned char *iom_fpga_fnd_addr;

// Define functions
int dev_driver_open(struct inode *, struct file *);
int dev_driver_release(struct inode *, struct file *);
ssize_t dev_driver_write(struct file *, const char *, size_t, loff_t *);

// Define file_operations structure
static struct file_operations dev_driver_fops =
{ 
	.owner 			= THIS_MODULE,
	.open 			= dev_driver_open, 
	.write 			= dev_driver_write,
	.release 		= dev_driver_release
};

struct struct_mydata {
	struct timer_list timer;
	int count;
};
struct struct_mydata mydata;	// struct var for timer

int dev_driver_release(struct inode *minode, struct file *mfile) {
	printk("dev_driver_release\n");
	dev_driver_usage = 0;
	return 0;
}

int dev_driver_open(struct inode *minode, struct file *mfile) {
	printk("dev_driver_open\n");
	if (dev_driver_usage != 0) {
		return -EBUSY;
	}
	dev_driver_usage = 1;
	return 0;
}

static void kernel_timer_blink(unsigned long timeout) {
	//int i;
	struct struct_mydata *p_data = (struct struct_mydata*)timeout;
	// Variables for FND module
	unsigned short fnd_value_short = 0;

	// Check for terminating timer
	p_data->count++;
	if( p_data->count > 15 ) {
		// Init the fnd module
		fnd_value_short = 0;
		outw(fnd_value_short, (unsigned int)iom_fpga_fnd_addr);
		return;
	}
	printk("Executed kernel_timer_count %d\n", p_data->count);

	// Write to fnd device
	outw(fnd_value_short, (unsigned int)iom_fpga_fnd_addr);
	
	// re-register timer
	mydata.timer.expires=get_jiffies_64()+(1*HZ);
	mydata.timer.data = (unsigned long)&mydata;
	mydata.timer.function	= kernel_timer_blink;

	add_timer(&mydata.timer);
}

ssize_t dev_driver_write(struct file *inode, const char *gdata, size_t length, loff_t *off_what) {
	//int i;

	printk("dev_driver_write\n");

	// Start of Setting timer - don't insert anything in it.
	mydata.count = 0;

	del_timer_sync(&mydata.timer);

	mydata.timer.expires=get_jiffies_64()+(1*HZ);
	mydata.timer.data = (unsigned long)&mydata;
	mydata.timer.function	= kernel_timer_blink;

	add_timer(&mydata.timer);
	// End of setting timer

	return 1;
}

int __init dev_driver_init(void)
{
	int result;
	printk("dev_driver_init\n");

	result = register_chrdev(DEV_DRIVER_MAJOR, DEV_DRIVER_NAME, &dev_driver_fops);
	if(result <0) {
		printk( "error %d\n",result);
		return result;
	}
    printk( "dev_file : /dev/%s , major : %d\n", DEV_DRIVER_NAME, DEV_DRIVER_MAJOR);

	// Mapping fpga_fnd physical mem to kernel
	iom_fpga_fnd_addr = ioremap(IOM_FND_ADDRESS, 0x4);

	// Initialize timer
	init_timer(&(mydata.timer));

	printk("init module\n");
	return 0;
}

void __exit dev_driver_exit(void)
{
	printk("dev_driver_exit\n");
	dev_driver_usage = 0;
	del_timer_sync(&mydata.timer);

	// Unmappingg fpga_fnd physical mem from kernel
	iounmap(iom_fpga_fnd_addr);

	unregister_chrdev(DEV_DRIVER_MAJOR, DEV_DRIVER_NAME);
}

module_init( dev_driver_init);
module_exit( dev_driver_exit);

MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("author");
