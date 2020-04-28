#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include "gpio/gpiolib.h"
#include <linux/irqdomain.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>



#define DRIVER_NAME "v_uart"
#define DEV_NAME		"vuart"
#define val_cnt 	8//bit counter
#define byte_cnt	8//bytes counter

#define test_fifo 0

//#define V_UART_MAJOR	234

int major;
static 	unsigned long baudrate = 4800;
static int count;
static int vuart_gpio_irq_state=0;
//static bool rx_timer_start_flag = false;
//static bool timer_start_flag = false;
//static int flag_irq_from_enable = 0; 

#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
struct mxc_gpio_pad_wakeup {
	u32 pin_id;
	u32 type;
	u32 line;
};
#endif

#if 0
struct mxc_gpio_port {
	struct list_head node;
	struct clk *clk;
	void __iomem *base;
	int irq;
	int irq_high;
	struct irq_domain *domain;
	struct gpio_chip gc;
	struct device *dev;
	u32 both_edges;
	int saved_reg[6];
	int suspend_saved_reg[6];
	bool gpio_ranges;
#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
	u32 pad_wakeup_num;
	struct mxc_gpio_pad_wakeup pad_wakeup[32];
#endif
};
#endif

static struct class *vuart_class;

//static struct gpio_chip *rxirq_gpiochip;
//static struct gpio_desc *rxirq_desc;
//static struct mxc_gpio_port *rxirq_port;
//static struct irq_data *rxirq_data;
//static struct irq_domain *rxirq_domain;
//static struct irq_chip		*rx_irqchip;
//static int rxirq_hwirq;


static unsigned long nsecs;
static 	u8 val_count[val_cnt];

static int test_val = 1;

//static struct kfifo *pkfifo;
//char data_buf[byte_cnt];
static char byte_buf;

static DECLARE_KFIFO(test, unsigned char, 32);
/* lock for procfs read access */
static DEFINE_MUTEX(read_lock);

/* lock for procfs write access */
static DEFINE_MUTEX(write_lock);





//static struct workqueue_struct *vuart_wq;
//static struct work_struct vuart_work;


static const int expected_result[32] = {
	 3,  4,  5,  6,  7,  8,  9,  0,
	 1, 20, 21, 22, 23, 24, 25, 26,
	27, 28, 29, 30, 31, 32, 33, 34,
	35, 36, 37, 38, 39, 40, 41, 42,
};

#if test_fifo
static int __init testfunc(void)
{
	unsigned char	buf[6];
	unsigned char	i, j;
	unsigned int	ret;

	printk(KERN_INFO "byte stream fifo test start\n");

	/* put string into the fifo */
	kfifo_in(&test, "hello", 5);

	/* put values into the fifo */
	for (i = 0; i != 10; i++)
		kfifo_put(&test, i);

	/* show the number of used elements */
	printk(KERN_INFO "fifo len: %u\n", kfifo_len(&test));

	/* get max of 5 bytes from the fifo */
	i = kfifo_out(&test, buf, 5);
	printk(KERN_INFO "buf: %.*s\n", i, buf);

	/* get max of 2 elements from the fifo */
	ret = kfifo_out(&test, buf, 2);
	printk(KERN_INFO "ret: %d\n", ret);
	/* and put it back to the end of the fifo */
	ret = kfifo_in(&test, buf, ret);
	printk(KERN_INFO "ret: %d\n", ret);

	/* skip first element of the fifo */
	printk(KERN_INFO "skip 1st element\n");
	kfifo_skip(&test);

	/* put values into the fifo until is full */
	for (i = 20; kfifo_put(&test, i); i++)
		;

	printk(KERN_INFO "queue len: %u\n", kfifo_len(&test));

	/* show the first value without removing from the fifo */
	if (kfifo_peek(&test, &i))
		printk(KERN_INFO "%d\n", i);

	/* check the correctness of all values in the fifo */
	j = 0;
	while (kfifo_get(&test, &i)) {
		printk(KERN_INFO "item = %d\n", i);
		if (i != expected_result[j++]) {
			printk(KERN_WARNING "value mismatch: test failed\n");
			return -EIO;
		}
	}
	if (j != ARRAY_SIZE(expected_result)) {
		printk(KERN_WARNING "size mismatch: test failed\n");
		return -EIO;
	}
	printk(KERN_INFO "test passed\n");

	return 0;
}
#endif

struct vuart_driver {
	int rx_irq;
	int rx_gpio;
	int tx_gpio;
	int vrxirq;
};

struct vuart_driver *vuart;


static struct hrtimer hr_timer;
static struct hrtimer hr_timer_rx;


void timer_start(unsigned long baudrate)
{
	//unsigned long nsecs;
	nsecs = (unsigned long)((1000000000/baudrate));
	//printk("1.nsecs = %ld\n",nsecs);
	nsecs = ktime_set(0,nsecs);
	//printk("nsecs = %ld\n",nsecs);
	//if(!timer_start_flag)
		hrtimer_start(&hr_timer,nsecs,HRTIMER_MODE_REL);
	//	timer_start_flag = true;
	//}
}
 


void rx_timer_start(unsigned long baudrate)
{
	//unsigned long nsecs;
	nsecs = (unsigned long)((1000000000/baudrate));
	//printk("1.nsecs = %ld\n",nsecs);
	nsecs = ktime_set(0,nsecs);
	//printk("nsecs = %ld\n",nsecs);
	//if(!rx_timer_start_flag)
	//{
		hrtimer_start(&hr_timer_rx,nsecs,HRTIMER_MODE_REL);
		//rx_timer_start_flag = true;
	//}
}

void timer_stop(void)
{
	unsigned char ret = 0;
	//if(rx_timer_start_flag)
	//{
		ret = hrtimer_cancel(&hr_timer);
		//timer_start_flag = false;
	//}
}

 
void rx_timer_stop(void)
{
	unsigned char ret = 0;
	//if(rx_timer_start_flag)
	//{
		ret = hrtimer_cancel( &hr_timer_rx ); 
		//if (ret)   
        //printk("hr_timer_rx was still in use...\n"); 
	//	rx_timer_start_flag = false;
	//}
     
}

static enum hrtimer_restart timer_func(struct hrtimer *hrtimer)
{
#if 1
	int val;
	val = gpio_get_value(vuart->rx_gpio);
	test_val = !test_val;
	gpio_set_value(vuart->tx_gpio,test_val);
	//printk("%s : %d",__func__,val);
	if(val == 0){
		goto normal_start;
	}else{
		goto error_start;
	}

normal_start:
	rx_timer_start(baudrate);
	return HRTIMER_NORESTART;

error_start:
	vuart_gpio_irq_state=0;
	//vuart_enable_irq();
	return HRTIMER_NORESTART;
#endif
}

static enum hrtimer_restart rx_vibrator_timer_func(struct hrtimer *hrtimer)
{
	int i=0;
	int val = 2;
	int temp_val;
	test_val = !test_val;
	val = gpio_get_value(vuart->rx_gpio);
	gpio_set_value(vuart->tx_gpio,test_val);
	if(count < val_cnt )
	{
		val_count[count] = val;
		count++;
		goto restart;
	}
	else
	{
		goto norestart;
	}

restart:
	hrtimer_forward(hrtimer,hrtimer->base->get_time(),nsecs);
	return HRTIMER_RESTART;

norestart:
	#if 1
	for(i=0; i < val_cnt; i++){
		byte_buf += (val_count[i]<<i);
		//printk("%d :val = 0x%2x",i,temp);
	}
	#endif
	vuart_gpio_irq_state=0;

	if(mutex_lock_interruptible(&write_lock))
		return HRTIMER_NORESTART;

	temp_val = kfifo_put(&test,byte_buf);

	mutex_unlock(&write_lock);
	return HRTIMER_NORESTART;
}

static irqreturn_t vuart_rx_irqfunc(int irq, void *dev_id)
{
	//irqd_set_trigger_type(rxirq_data,IRQ_TYPE_NONE);
	//disable_irq_nosync(vuart->rx_irq);
	if(vuart_gpio_irq_state==0)
	{
		vuart_gpio_irq_state=1;
		count = 0;
		memset(val_count,0,sizeof(val_count));
		byte_buf = 0x00;
		test_val=!test_val;
		gpio_set_value(vuart->tx_gpio,test_val);
		timer_start(baudrate*2);
	}


	//printk("rx_timer_start state = %x\n",hr_timer_rx.state);
	//printk("timer_start state = %x\n",hr_timer.state);
	
	return IRQ_HANDLED;
}

static int vuart_open(struct inode *inode, struct file *filp)
{
	int retval;
#if 1
	hrtimer_init(&hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hr_timer.function = timer_func;
	
	hrtimer_init(&hr_timer_rx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hr_timer_rx.function = rx_vibrator_timer_func;
	
	//pkfifo = malloc(sizeof(struct kfifo));
	INIT_KFIFO(test);

#if test_fifo
	if (testfunc() < 0) {
		printk("fifo error\n");
	}
#endif

	retval = request_irq(vuart->rx_irq, vuart_rx_irqfunc, IRQF_ONESHOT, "irq_uartrx", vuart);
	
	if (retval < 0) {
		printk("Error requesting IRQ : %d\n",retval);
		return -1;
	}
#endif


	return 0;
}

static ssize_t vuart_read(struct file *filp,char __user *buf, size_t count,loff_t *f_pos)
{
#if 1
	int i = 0;
	ssize_t used_val = 0;
	int temp_val;
	char rcv_data;
	if(mutex_lock_interruptible(&read_lock)){
		goto out;
	}
	used_val = kfifo_len(&test);
	//printk("used_val = %lu\n",used_val);
	if(used_val == 0){
		goto out;
	}
	for(;i<used_val;i++){
		temp_val = kfifo_get(&test,&rcv_data);
		temp_val = copy_to_user(buf+i,&rcv_data,sizeof(rcv_data));
		//printk("rcv_data = %x\n",rcv_data);
	}

out:
	mutex_unlock(&read_lock);
	return used_val;
#endif

#if 0
	printk("starting vuart_read\n");
	//char rcv_data;
	//printk("in = %d out = %d\n",pkfifo->in,pkfifo->out);
	kfifo_out(pkfifo,buf,sizeof(buf));
	printk("rcv buf = %lx\n",buf);
	//memcpy(buf,rcv_data,sizeof(rcv_data));
	//printk("pkfifo->out %d pkfifo-in %d %2x\n",pkfifo->out,pkfifo->in,rcv_data);
	return 0;
#endif
}

static int led_close(struct inode *inode, struct file *file)
{
	timer_stop();
	rx_timer_stop();
	free_irq(vuart->rx_irq,vuart);
	kfifo_free(&test);
	return 0;
}




static const struct of_device_id of_vuart_match[] = {
	{ .compatible = "vuart", },
	{},
};
MODULE_DEVICE_TABLE(of, of_vuart_match);


// device operation
static struct file_operations vuart_fops = {
    .owner = THIS_MODULE,
    .open = vuart_open,
    .release = led_close,
    .read = vuart_read,
};


static int vuart_probe(struct platform_device *pdev)
{
	int retval;
	//struct device_node *np = pdev->dev.of_node;
	//vuart = dev_get_platdata(&pdev->dev);
	vuart = devm_kzalloc(&pdev->dev,sizeof(*vuart) , GFP_KERNEL);

	vuart->rx_gpio = of_get_named_gpio(pdev->dev.of_node, "rx-gpio", 0);
	printk(DEV_NAME":rx_gpio = %d\n",vuart->rx_gpio);
	if(!vuart->rx_gpio){
		printk("get rxgpio failed\n");
		return vuart->rx_gpio;
	}
	retval = devm_gpio_request_one(&pdev->dev,vuart->rx_gpio,GPIOF_IN,"vuart_rxgpio");
	//retval = devm_gpio_request_one(&pdev->dev,vuart->rx_gpio,GPIOF_OUT_INIT_HIGH,"vuart_rxgpio");
	if (retval < 0){
		printk("set rx_gpio failed\n");
		return retval;
	}
	
	vuart->tx_gpio = of_get_named_gpio(pdev->dev.of_node, "tx-gpio", 0);
	printk(DEV_NAME":tx_gpio = %d\n",vuart->tx_gpio);
	if(!vuart->tx_gpio){
		printk("get txgpio failed\n");
		return vuart->tx_gpio;
	}
	retval = devm_gpio_request_one(&pdev->dev,vuart->tx_gpio,GPIOF_OUT_INIT_HIGH,"vuart_txgpio");
	//retval = devm_gpio_request_one(&pdev->dev,vuart->rx_gpio,GPIOF_OUT_INIT_HIGH,"vuart_rxgpio");
	if (retval < 0){
		printk("set tx_gpio failed\n");
		return retval;
	}

#if 0
	//retval = gpio_to_desc((vuart->rx_gpio);
	rxirq_desc = gpio_to_desc(vuart->rx_gpio);
	vuart->rx_irq = gpiod_to_irq(rxirq_desc);
	//gpio_set_irq_type();
	
	printk(DEV_NAME":rx_irq = %d\n",vuart->rx_irq);
	if(!vuart->rx_irq){
		printk("get rx_irq failed\n");
		return -1;
	}

	rxirq_hwirq = rxirq_desc - &rxirq_desc->gdev->descs[0];
	printk("rxirq_hwirq = %d\n",rxirq_hwirq);
	rxirq_gpiochip = rxirq_desc->gdev->chip;
	rxirq_port = gpiochip_get_data(rxirq_gpiochip);
	//vuart->vrxirq =  irq_find_mapping(&rxirq_port->domain, rxirq_hwirq);
	rxirq_data = irq_get_irq_data(vuart->rx_irq);
	//vuart->rx_irq = irq_find_mapping(rxirq_port->domain,rxirq_hwirq);
	//rxirq_data = radix_tree_lookup(&rxirq_port->domain->revmap_tree, rxirq_hwirq);
	irqd_set_trigger_type(rxirq_data,IRQ_TYPE_EDGE_FALLING);

	#endif

#if 1
	vuart->rx_irq = platform_get_irq(pdev, 0);

	printk(DEV_NAME":rx_irq = %d\n",vuart->rx_irq);
	if(!vuart->rx_irq){
		printk("get rx_irq failed\n");
		return -1;
	}
#endif

	major = register_chrdev(0,DEV_NAME,&vuart_fops);/*主设备号，当用户设置为0时，内核会动态分配一个设备号。*/
	if(major<0){
		printk(DEV_NAME"get major fail\n");
		return major;
	}
	vuart_class = class_create(THIS_MODULE,DEV_NAME);/*创建设备信息，执行后会出现 /sys/class/DEV_NAME */
	if(IS_ERR(vuart_class)){
		printk("fail in vuart class\n");
		return -1;
	}

#if 0
	hrtimer_init(&hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hr_timer.function = timer_func;
	
	hrtimer_init(&hr_timer_rx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hr_timer_rx.function = rx_vibrator_timer_func;
	
	//pkfifo = malloc(sizeof(struct kfifo));
	INIT_KFIFO(test);

	//if (testfunc() < 0) {
	//	printk("fifo error\n");
	//}

	retval = request_irq(vuart->rx_irq, vuart_rx_irqfunc, IRQF_ONESHOT, "irq_uartrx", vuart);
	
	if (retval < 0) {
		printk("Error requesting IRQ : %d\n",retval);
		return -1;
	}
#endif
	

	device_create(vuart_class,NULL,MKDEV(major, 0),NULL,DEV_NAME);
	printk(DEV_NAME":initialized\n");

	return 0;
}

static int vuart_remove(struct platform_device *pdev)
{
	unregister_chrdev(major,DEV_NAME);

	devm_kfree(&pdev->dev,vuart);
	
	//删除节点及信息
	device_destroy(vuart_class,MKDEV(major, 0));
	class_destroy(vuart_class);
	
	//timer_stop();
	//rx_timer_stop();
	//free_irq(vuart->rx_irq,vuart);
	
	return 0;
}

struct platform_driver vuart_drv = {
	.probe		= vuart_probe,
	.remove		= vuart_remove,
	.driver		= {
		.name	= DEV_NAME,
		.of_match_table = of_vuart_match,
	}
};


static int v_uart_init(void)
{
	platform_driver_register(&vuart_drv);
	return 0;
}


static void v_uart_exit(void)
{
	platform_driver_unregister(&vuart_drv);
}


module_init(v_uart_init);
module_exit(v_uart_exit);

MODULE_AUTHOR("Raphael Assenat <raph@8d.com>, Trent Piepho <tpiepho@freescale.com>");
MODULE_DESCRIPTION("virtual uart driver");
MODULE_LICENSE("GPL v2");
