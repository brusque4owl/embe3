#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <mach/gpio.h>
#include <linux/platform_device.h>
#include <asm/gpio.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include <linux/cdev.h>

#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#define DEV_DRIVER_MAJOR 242
#define DEV_DRIVER_MINOR 0
#define DEV_DRIVER_NAME "/dev/stopwatch"

#define IOM_FND_ADDRESS 0x08000004	// address of fnd

// Global variable for fpga_modules
static int dev_driver_usage = 0;
static unsigned char *iom_fpga_fnd_addr;
static int inter_major = DEV_DRIVER_MAJOR, inter_minor = 0;
static int result;
static dev_t inter_dev;
static struct cdev inter_cdev;
bool check_for_3sec=false;	// vol- key falling : true
bool flag_start=false, flag_pause=false, flag_reset=false;

irqreturn_t inter_handler_home(int irq, void* dev_id, struct pt_regs *reg);
irqreturn_t inter_handler_back(int irq, void* dev_id, struct pt_regs *reg);
irqreturn_t inter_handler_volup(int irq, void* dev_id, struct pt_regs *reg);
irqreturn_t inter_handler_voldown(int irq, void* dev_id, struct pt_regs *reg);
static void kernel_timer_exit(unsigned long timeout);
static void kernel_timer_blink(unsigned long timeout);

int interruptCount = 0;

wait_queue_head_t wq_write;
DECLARE_WAIT_QUEUE_HEAD(wq_write);

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
struct struct_mydata stopwatch;	// struct var for stopwatch timer
struct struct_mydata exit_stop;	// struct var for exit timer

// struct for showing stopwatch value
struct fnd_time{
	char sec[2];
	char min[2];
};
static struct fnd_time fnd_time={0,0,0,0};

// Interrupt handlers for 4 keys
// START KEY
irqreturn_t inter_handler_home(int irq, void* dev_id, struct pt_regs *reg){
	printk(KERN_ALERT "HOME KEY = %x\n", gpio_get_value(IMX_GPIO_NR(1, 11)));
	// Prevent start key while stopwatch is going on
	if(flag_start==true && 
		flag_pause==false && flag_reset==false)
		return IRQ_HANDLED;

	// pause와 reset flag 풀어주기
	if(flag_pause==true)
		flag_pause = false;
	if(flag_reset==true)
		flag_reset = false;

	printk("start = %d\n", flag_start);
	printk("pause = %d\n", flag_pause);
	printk("reset = %d\n", flag_reset);


	// Start of Setting timer - don't insert anything in it.
	stopwatch.count = 0;

	// 다시 home key누르면 기존 timer 제거한 뒤 timer 재등록
	del_timer_sync(&stopwatch.timer);

	stopwatch.timer.expires=get_jiffies_64()+(1*HZ);
	stopwatch.timer.data = (unsigned long)&stopwatch;
	stopwatch.timer.function	= kernel_timer_blink;

	add_timer(&stopwatch.timer);
	// End of setting timer

	flag_start = true;
	return IRQ_HANDLED;
}
// PAUSE KEY
irqreturn_t inter_handler_back(int irq, void* dev_id, struct pt_regs *reg){
	printk(KERN_ALERT "BACK KEY = %x\n", gpio_get_value(IMX_GPIO_NR(1, 12)));

	// reset된 상태에서는 pause눌러도 반응 없게하기
	if(flag_reset==true) return IRQ_HANDLED;

	flag_pause = !flag_pause;

	printk("start = %d\n", flag_start);
	printk("pause = %d\n", flag_pause);
	printk("reset = %d\n", flag_reset);	
	
	if(flag_pause==false){
		del_timer_sync(&stopwatch.timer);

		stopwatch.timer.expires=get_jiffies_64()+(1*HZ);
		stopwatch.timer.data = (unsigned long)&stopwatch;
		stopwatch.timer.function	= kernel_timer_blink;

		add_timer(&stopwatch.timer);
	}
	return IRQ_HANDLED;
}

// RESET KEY
irqreturn_t inter_handler_volup(int irq, void* dev_id, struct pt_regs *reg){
	printk(KERN_ALERT "VOLUP = %x\n", gpio_get_value(IMX_GPIO_NR(2, 15)));
	flag_reset = true;	// It may be turned to false in the timer

	printk("start = %d\n", flag_start);
	printk("pause = %d\n", flag_pause);
	printk("reset = %d\n", flag_reset);


	fnd_time.sec[0]=fnd_time.sec[1]=0;
	fnd_time.min[0]=fnd_time.min[1]=0;

	// Init the fnd module
	outw(0, (unsigned int)iom_fpga_fnd_addr);

	return IRQ_HANDLED;
}

// EXIT KEY
irqreturn_t inter_handler_voldown(int irq, void* dev_id, struct pt_regs *reg){
	printk(KERN_ALERT "VOLDOWN = %x\n", gpio_get_value(IMX_GPIO_NR(5, 14)));
	// Start of Setting exit timer - don't insert anything in it.
	check_for_3sec=!check_for_3sec;
	exit_stop.count = 1;

	del_timer_sync(&exit_stop.timer);

	if(check_for_3sec==true){
		exit_stop.timer.expires=get_jiffies_64()+(1*HZ);
		exit_stop.timer.data = (unsigned long)&exit_stop;
		exit_stop.timer.function	= kernel_timer_exit;

		add_timer(&exit_stop.timer);
	}
	// End of setting timer

	interruptCount=0;
	return IRQ_HANDLED;
}


int dev_driver_release(struct inode *minode, struct file *mfile) {
	printk("dev_driver_release\n");
	dev_driver_usage = 0;
	free_irq(gpio_to_irq(IMX_GPIO_NR(1, 11)), NULL);
	free_irq(gpio_to_irq(IMX_GPIO_NR(1, 12)), NULL);
	free_irq(gpio_to_irq(IMX_GPIO_NR(2, 15)), NULL);
	free_irq(gpio_to_irq(IMX_GPIO_NR(5, 14)), NULL);

	return 0;
}

int dev_driver_open(struct inode *minode, struct file *mfile) {
	int ret;
	int irq;

	printk("dev_driver_open\n");

	if (dev_driver_usage != 0) {
		return -EBUSY;
	}
	dev_driver_usage = 1;

	// HOME KEY
	gpio_direction_input(IMX_GPIO_NR(1,11));
	irq = gpio_to_irq(IMX_GPIO_NR(1,11));
	printk(KERN_ALERT "IRQ Number : %d\n", irq);
	ret = request_irq(irq, inter_handler_home, IRQF_TRIGGER_FALLING, "home", 0);

	// BACK KEY
	gpio_direction_input(IMX_GPIO_NR(1,12));
	irq = gpio_to_irq(IMX_GPIO_NR(1,12));
	printk(KERN_ALERT "IRQ Number : %d\n", irq);
	ret = request_irq(irq, inter_handler_back, IRQF_TRIGGER_FALLING, "back", 0);

	// VOLUP KEY
	gpio_direction_input(IMX_GPIO_NR(2,15));
	irq = gpio_to_irq(IMX_GPIO_NR(2,15));
	printk(KERN_ALERT "IRQ Number : %d\n", irq);
	ret = request_irq(irq, inter_handler_volup, IRQF_TRIGGER_FALLING, "volup", 0);

	// VOLDOWN KEY
	gpio_direction_input(IMX_GPIO_NR(5,14));
	irq = gpio_to_irq(IMX_GPIO_NR(5,14));
	printk(KERN_ALERT "IRQ Number : %d\n", irq);
	ret = request_irq(irq, inter_handler_voldown, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "voldown", 0);

	return 0;
}

static void kernel_timer_blink(unsigned long timeout) {
	//int i;
	struct struct_mydata *p_data = (struct struct_mydata*)timeout;
	// Variables for FND module
	unsigned short fnd_value_short = 0;

	p_data->count++;
	printk("stopwatch_count %d\n", p_data->count);

	// Check for terminating timer
	if(flag_pause==true){
		// turn to false when pause or start button is pushed.
		return;
	}
	if(flag_reset==true){
		// Init the fnd module
		fnd_value_short = 0;
		outw(fnd_value_short, (unsigned int)iom_fpga_fnd_addr);
		// Do not re-register timer
		return;
	}

	// Write to fnd device
	fnd_time.sec[1]++;
	if(fnd_time.sec[1]>=10){
		fnd_time.sec[1]=0;
		fnd_time.sec[0]++;
		if(fnd_time.sec[0]>=6){
			fnd_time.sec[0]=0;
			fnd_time.min[1]++;
			if(fnd_time.min[1]>=10){
				fnd_time.min[1]=0;
				fnd_time.min[0]++;
				if(fnd_time.min[0]>=6){
					fnd_time.min[0]=0;
				}
			}
		}
	}
	fnd_value_short = fnd_time.min[0] << 12 | fnd_time.min[1] << 8 | fnd_time.sec[0] << 4 | fnd_time.sec[1];
	outw(fnd_value_short, (unsigned int)iom_fpga_fnd_addr);
	
	// re-register timer
	stopwatch.timer.expires=get_jiffies_64()+(1*HZ);
	stopwatch.timer.data = (unsigned long)&stopwatch;
	stopwatch.timer.function	= kernel_timer_blink;

	add_timer(&stopwatch.timer);
}

static void kernel_timer_exit(unsigned long timeout) {
	//int i;
	struct struct_mydata *p_data = (struct struct_mydata*)timeout;
	// Variables for FND module
	unsigned short fnd_value_short = 0;

	// Check for terminating timer
	p_data->count++;
	if( p_data->count > 3 ) {
		// Init the fnd module
		fnd_value_short = 0;
		outw(fnd_value_short, (unsigned int)iom_fpga_fnd_addr);

		// Init exit, pause, reset condition
		check_for_3sec=false;
		flag_start = false;
		flag_pause = false;
		flag_reset = false;
		
		// Init fnd_time
		fnd_time.sec[0] = 0;
		fnd_time.sec[1] = 0;
		fnd_time.min[0] = 0;
		fnd_time.min[1] = 0;

		// Terminate stopwatch timer
		del_timer_sync(&stopwatch.timer);

		// Wake up the process
		__wake_up(&wq_write, 1, 1, NULL);
		printk("wake up\n");
		return;
	}
	printk("exit_count %d\n", p_data->count);
	// re-register timer
	exit_stop.timer.expires=get_jiffies_64()+(1*HZ);
	exit_stop.timer.data = (unsigned long)&exit_stop;
	exit_stop.timer.function	= kernel_timer_exit;

	add_timer(&exit_stop.timer);
}

ssize_t dev_driver_write(struct file *inode, const char *gdata, size_t length, loff_t *off_what) {
	//int i;

	// Start of interrupt - make the process sleep on
	if(interruptCount==0){
		printk("sleep on\n");
		interruptible_sleep_on(&wq_write);
	}
	// End of interrupt

	printk("dev_driver_write\n");
	return 0;
}

static int inter_register_cdev(void)
{
	int error;
	if(inter_major) {
		inter_dev = MKDEV(inter_major, inter_minor);
		error = register_chrdev_region(inter_dev,1,"stopwatch");
	}else{
		error = alloc_chrdev_region(&inter_dev, inter_minor, 1, "stopwatch");
		inter_major = MAJOR(inter_dev);
	}
	if(error<0){
		printk(KERN_WARNING "inter: can't get major %d\n", inter_major);
		return result;
	}
	printk(KERN_ALERT "major number = %d\n", inter_major);
	cdev_init(&inter_cdev, &dev_driver_fops);
	inter_cdev.owner = THIS_MODULE;
	inter_cdev.ops = &dev_driver_fops;
	error = cdev_add(&inter_cdev, inter_dev, 1);
	if(error)
	{
		printk(KERN_NOTICE "inter Register Error %d\n", error);
	}
	return 0;
}
int __init dev_driver_init(void)
{
	int result;
	printk("stopwatch_init\n");
	if((result = inter_register_cdev()) < 0)
		return result;
	printk(KERN_ALERT "Init Module Success \n");
	printk(KERN_ALERT "Device : %s, Major Num : %d \n", DEV_DRIVER_NAME, DEV_DRIVER_MAJOR);
	// Mapping fpga_fnd physical mem to kernel
	iom_fpga_fnd_addr = ioremap(IOM_FND_ADDRESS, 0x4);

	// Initialize stopwatch & exit timer
	init_timer(&(stopwatch.timer));
	init_timer(&(exit_stop.timer));

	return 0;
}

void __exit dev_driver_exit(void)
{
	printk("dev_driver_exit\n");
	dev_driver_usage = 0;
	del_timer_sync(&stopwatch.timer);
	del_timer_sync(&exit_stop.timer);

	// Unmappingg fpga_fnd physical mem from kernel
	iounmap(iom_fpga_fnd_addr);

	//unregister_chrdev(DEV_DRIVER_MAJOR, DEV_DRIVER_NAME);
	cdev_del(&inter_cdev);
	unregister_chrdev_region(inter_dev, 1);
	printk(KERN_ALERT "Remove Module Success \n");
}

module_init( dev_driver_init);
module_exit( dev_driver_exit);

MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("author");
