//
//
// 2013.06.06 add this FPGA driver for SMIMS FPGA board MaCube
// Modify arch/arm/mach-tegra/board-coribri_t30.c
// 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
 
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/delay.h>

#include <linux/sched.h>
#include <linux/interrupt.h>

#include <linux/gpio.h>

//#define	_DEBUG
#define USE_DMA
#define USE_USER_SPACE_MMAP

#define PREFIX "-->smimsfpga<--: "
#define AHB_ADDRESS 0x40030000

#define AHB_ADDRESS_W 0x40030000
#define AHB_ADDRESS_R 0x40032000

//	/arch/arm/mach-tegra/gpio-names.h
#define TEGRA_GPIO_PJ0      72
#define TEGRA_GPIO_PJ2      74

/* Global variables of the driver */
static unsigned short * rAHB_addr; 

static unsigned long * rSNOR_CONFIG_0_addr;
static unsigned long * rSNOR_DMA_CFG_0_addr;  
static unsigned long * rSNOR_NOR_ADDR_PTR_0_addr;  
static unsigned long * rSNOR_AHB_ADDR_PTR_0_addr;  

#define	DMA_MEMORY_SIZE	0x8000



#define	DEV_IOCTLID	0xD0
#define IOCTL_SETTING_TIMEOUT				_IOW(DEV_IOCTLID, 10, int)

#define	IOCTL_BLOCK_WRITE_TO_FPGA			_IOW(DEV_IOCTLID, 11, int)
#define	IOCTL_BLOCK_READ_FROM_FPGA			_IOR(DEV_IOCTLID, 12, int)
#define	IOCTL_BLOCK_WRITE_TO_FPGA_WAIT_READY	_IOR(DEV_IOCTLID, 13, int)
#define	IOCTL_BLOCK_READ_FROM_FPGA_WAIT_READY	_IOW(DEV_IOCTLID, 14, int)

#define IOCTL_WRITE_TO_FPGA					_IOW(DEV_IOCTLID, 15, int)
#define IOCTL_READ_FROM_FPGA				_IOW(DEV_IOCTLID, 16, int)
#define IOCTL_WRITE_TO_FPGA_WAIT_READY     	_IOW(DEV_IOCTLID, 17, int)
#define IOCTL_READ_FROM_FPGA_WAIT_READY     _IOW(DEV_IOCTLID, 18, int)


//static DEFINE_MUTEX(smims_fpga_read_mutex);
//static DEFINE_MUTEX(smims_fpga_write_mutex);

//int interrupt_read;
//int interrupt_write;

int ShowREG(void);
void PrintREG(char * REGName, unsigned long Addr);
int REGSetting(void);
void PtintfDMA_Channel3_REG(void);
void SetREGValue(char * REGName, unsigned long Addr, unsigned long value);
void SetREGBit(char * REGName, unsigned long Addr, int bitpos, int bitvalue);
void SetAddressValue(unsigned long * rREG_addr, unsigned long value);
void SetAddressBitValue(unsigned long * rREG_addr, int bitpos, int bitvalue);

void Send_Data_To_FPGA(void);
void Get_Data_From_FPGA(void);

//	20121219
void Send_Data_To_FPGA_With_Address_Count(int iAddr, int iCount);
void Get_Data_From_FPGA_With_Address_Count(int iAddr, int iCount);
int TimeOutInSencond = 0;
//	=========================

/* Major number */
int SMIMSFPGA_DEV_MAJOR = 252;
#define	BUFFER_SIZE	DMA_MEMORY_SIZE*2

//irqreturn_t Interrupt_ISR1(int irq, void * dev_id);
//irqreturn_t Interrupt_ISR2(int irq, void * dev_id);

static int SMIMS_FPGA_open(struct inode *inode, struct file *filp);
static int SMIMS_FPGA_close(struct inode *inode, struct file *filp);
static long SMIMS_FPGA_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static int SMIMS_FPGA_probe(struct platform_device *pdev);
static int SMIMS_FPGA_remove(struct platform_device *pdev);

static int __init smimsfpga_init(void);
static void __exit smimsfpga_exit(void);

typedef struct MaCube_smimsfpga {
	struct cdev 		chrdev;
	unsigned int		irq;
	void __iomem		*regs;
	struct device		*dev;
	struct resource		*ioarea;
}MaCube_smimsfpga_t;

static MaCube_smimsfpga_t * glfpga;
struct class * glMacubeclass;

static struct platform_driver MaCube_SMIMS_FPGA_driver = {
	.probe		= SMIMS_FPGA_probe,
	.remove		= SMIMS_FPGA_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "smimsfpga",
	},
};

static struct file_operations MaCube_SMIMS_FPGA_fops = {
	.owner	= THIS_MODULE,
	.open		= SMIMS_FPGA_open,
	.release	= SMIMS_FPGA_close,
	.unlocked_ioctl	= SMIMS_FPGA_ioctl,
};

/*
irqreturn_t Interrupt_ISR1(int irq, void * dev_id)
{
#ifdef	_DEBUG
	printk(KERN_INFO PREFIX "Interrupt_ISR1(), irq = %d\n", irq);
#endif
	mutex_lock(&smims_fpga_read_mutex);
	interrupt_read++;
	mutex_unlock(&smims_fpga_read_mutex);

	return (IRQ_HANDLED);
}

irqreturn_t Interrupt_ISR2(int irq, void * dev_id)
{
#ifdef	_DEBUG
	printk(KERN_INFO PREFIX "Interrupt_ISR2(), irq = %d\n", irq);
#endif

	mutex_lock(&smims_fpga_write_mutex);
	interrupt_write++;
	mutex_unlock(&smims_fpga_write_mutex);

	return (IRQ_HANDLED);
}
*/

static int SMIMS_FPGA_open(struct inode *inode, struct file *filp)
{
#ifdef	_DEBUG
	printk(KERN_INFO PREFIX "SMIMS_FPGA_open() USE_DMA\n");
#endif

	filp->private_data = glfpga;

	return 0;
}

static int SMIMS_FPGA_close(struct inode *inode, struct file *filp)
{
#ifdef	_DEBUG
	printk(KERN_INFO PREFIX "SMIMS_FPGA_close()\n");
#endif
	
	filp->private_data = NULL;
	return 0;
}

static long SMIMS_FPGA_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long timeout;
	bool bTimeOut;
	
#ifdef	_DEBUG
printk("[smimsfpga]SMIMS_FPGA_ioctl(cmd=%x)\n",cmd);			
#endif

	//	20121218
	unsigned long AddressCount;
	int iAddress;
	int iCount;
	int iStatus;

	//	==============

	switch(cmd)
	{
	case IOCTL_SETTING_TIMEOUT:
		get_user(TimeOutInSencond, (int *)arg);
		timeout = jiffies + (TimeOutInSencond * HZ);
	break;
	//	20121218
	case IOCTL_WRITE_TO_FPGA:
		get_user(AddressCount, (unsigned long *)arg);
		iAddress = AddressCount >> 16;
		iCount = AddressCount & 0xFFFF;
		Send_Data_To_FPGA_With_Address_Count(iAddress, iCount);
	break;
	case IOCTL_READ_FROM_FPGA:
		get_user(AddressCount, (unsigned long *)arg);
		iAddress = AddressCount >> 16;
		iCount = AddressCount & 0xFFFF;
		Get_Data_From_FPGA_With_Address_Count(iAddress, iCount);
	break;


	case IOCTL_WRITE_TO_FPGA_WAIT_READY:

		timeout = jiffies + (TimeOutInSencond * HZ);
		bTimeOut = false;
		while(1)
		{
			iStatus = gpio_get_value(TEGRA_GPIO_PJ2);
			if(iStatus == 1)
				break;
			if ( time_after(jiffies, timeout) )
			{
				bTimeOut = true;
#ifdef	_DEBUG
	printk(KERN_INFO PREFIX "IOCTL_WRITE_TO_FPGA_WAIT_READY: Timeout\n");
#endif
				break;
			}
			udelay(1);	
		}
		if ( bTimeOut )
		{
			//put_user(1, (int *)arg);//Time out, return 1
			return 1;
		}
		else
		{
			get_user(AddressCount, (unsigned long *)arg);
			iAddress = AddressCount >> 16;
			iCount = AddressCount & 0xFFFF;
			Send_Data_To_FPGA_With_Address_Count(iAddress, iCount);
			//put_user(0, (int *)arg);//not time out, return 0
		}

	break;
	case IOCTL_READ_FROM_FPGA_WAIT_READY:

		timeout = jiffies + (TimeOutInSencond * HZ);
		bTimeOut = false;
		while(1)
		{
			iStatus = gpio_get_value(TEGRA_GPIO_PJ0);
			if(iStatus == 1)
				break;
			if ( time_after(jiffies, timeout) )
			{
				bTimeOut = true;
#ifdef	_DEBUG
	printk(KERN_INFO PREFIX "IOCTL_READ_FROM_FPGA_WAIT_READY: Timeout\n");
#endif
				break;
			}
			udelay(1);	
		}
		if ( bTimeOut )
		{
			//put_user(1, (int *)arg);//Time out, return 1
			return 1;
		}
		else
		{
			get_user(AddressCount, (unsigned long *)arg);
			iAddress = AddressCount >> 16;
			iCount = AddressCount & 0xFFFF;
			Get_Data_From_FPGA_With_Address_Count(iAddress, iCount);
			//put_user(0, (int *)arg);//not time out, return 0
		}
	break;
	case IOCTL_BLOCK_WRITE_TO_FPGA:
		Send_Data_To_FPGA();

	break;
	case IOCTL_BLOCK_READ_FROM_FPGA:
		Get_Data_From_FPGA();
		//put_user(DPSRAM_Ineterrupt_cnt, (int *)arg);
	break;
	case IOCTL_BLOCK_WRITE_TO_FPGA_WAIT_READY:
//		iStatus = gpio_get_value(TEGRA_GPIO_PJ2);
//printk("Write Ready status === %d\n", iStatus);
		bTimeOut = false;
		timeout = jiffies + (TimeOutInSencond * HZ);

		while(1)
		{
			iStatus = gpio_get_value(TEGRA_GPIO_PJ2);
			if(iStatus == 1)
				break;
			if ( time_after(jiffies, timeout) )
			{
				bTimeOut = true;
#ifdef	_DEBUG
	printk(KERN_INFO PREFIX "IOCTL_WRITE_TO_FPGA_WAIT_READY: Timeout\n");
#endif
				break;
			}
			udelay(1);	
		}

		if ( bTimeOut )
		{
			//put_user(1, (int *)arg);//Time out, return 1
			return 1;
		}
		else
		{
			Send_Data_To_FPGA();
			//put_user(0, (int *)arg);//not time out, return 0
		}

	break;
	case IOCTL_BLOCK_READ_FROM_FPGA_WAIT_READY:
//		iStatus = gpio_get_value(TEGRA_GPIO_PJ0);
//printk("Read Ready status === %d\n", iStatus);
		bTimeOut = false;
		timeout = jiffies + (TimeOutInSencond * HZ);

		while(1)
		{
			iStatus = gpio_get_value(TEGRA_GPIO_PJ0);
			if(iStatus == 1)
				break;
			if ( time_after(jiffies, timeout) )
			{
				bTimeOut = true;
#ifdef	_DEBUG
	printk(KERN_INFO PREFIX "IOCTL_READ_FROM_FPGA_WAIT_READY: Timeout\n");
#endif
				break;
			}
			udelay(1);	
		}

		if ( bTimeOut )
		{
			//put_user(1, (int *)arg);//Time out, return 1
			return 1;
		}
		else
		{
			Get_Data_From_FPGA();
			//put_user(0, (int *)arg);//not time out, return 0
		}
	break;
	default:
		return -EINVAL;
	}

	return 0;
}


/* SMIMS_FPGA_probe
 *
 * called by the bus driver when a suitable device is found
*/

static int SMIMS_FPGA_probe(struct platform_device *pdev)
{
	int ret;
	MaCube_smimsfpga_t *fpga;
	struct resource *res;
//	struct resource *irqres1 = NULL;
//	struct resource *irqres2 = NULL;
	dev_t dev_id;
	int alloc_ret = 0;
	int major, minor;
	struct device * device = NULL;

	ret = 0;

	printk("[smimsfpga]SMIMS_FPGA_probe() called\n");

	fpga = kzalloc(sizeof(struct MaCube_smimsfpga), GFP_KERNEL);
	if (!fpga) 
	{
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	fpga->dev = &pdev->dev;

	/* map the registers */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) 
	{
		dev_err(&pdev->dev, "cannot find IO resource\n");
		ret = -ENOENT;
		goto err;
	}
	printk("[smimsfpga]SMIMS_FPGA_probe() platform_get_resourc ok.\n");

	fpga->ioarea = request_mem_region(res->start, (res->end-res->start)+1,
					 pdev->name);

	if (fpga->ioarea == NULL) 
	{
		dev_err(&pdev->dev, "cannot request IO\n");
		ret = -ENXIO;
		goto err;
	}
	printk("[smimsfpga]SMIMS_FPGA_probe() request_mem_regio ok.\n");

	fpga->regs = ioremap(res->start, (res->end-res->start)+1);

	if (fpga->regs == NULL) 
	{
		dev_err(&pdev->dev, "cannot map IO\n");
		ret = -ENXIO;
		goto err_ioarea;
	}
	printk("[smimsfpga]SMIMS_FPGA_probe() ioremap ok.\n");

	/* find the IRQ for this unit (note, this relies on the init call to
	 * ensure no current IRQs pending
	 */
/*
	fpga->irq = ret = platform_get_irq(pdev, 0);
	if (ret <= 0) 
	{
		dev_err(&pdev->dev, "cannot find IRQ\n");
		goto err_iomap;
	}

	irqres1 = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "smimsfpga_irq1");
	irqres2 = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "smimsfpga_irq2");

	ret = request_irq(irqres1->start, Interrupt_ISR1, IRQF_DISABLED | IRQF_TRIGGER_RISING,
			  "smimsfpga_irq1", NULL);
	if (ret != 0) 
	{
		dev_err(&pdev->dev, "cannot claim IRQ[%s] %d\n", "smimsfpga_irq1", fpga->irq);
		goto err_iomap;
	}
	ret = request_irq(irqres2->start, Interrupt_ISR2, IRQF_DISABLED | IRQF_TRIGGER_RISING,
			  "smimsfpga_irq2", NULL);
	if (ret != 0) 
	{
		dev_err(&pdev->dev, "cannot claim IRQ[%s] %d\n", "smimsfpga_irq2", fpga->irq);
		goto err_iomap;
	}
*/
	//===============================
	//
	// create character device
	//
	dev_id = MKDEV(SMIMSFPGA_DEV_MAJOR, 0);

	printk("[smimsfpga]SMIMS_FPGA_probe() before dynamic allocate mjaor=%d, minor=%d\n",MAJOR(dev_id), MINOR(dev_id));

	alloc_ret = alloc_chrdev_region(&dev_id, 0, 1, "smimsfpga");

	printk("[smimsfpga]SMIMS_FPGA_probe() after dynamic allocate mjaor=%d, minor=%d, alloc_ret=%d\n",MAJOR(dev_id), MINOR(dev_id), alloc_ret);
	
	if ( alloc_ret )
		goto err_iomap;

	major = MAJOR(dev_id);
	minor = MINOR(dev_id);

	dev_id = MKDEV(major, minor);	

	pdev->dev.devt = dev_id;

	cdev_init(&fpga->chrdev, &MaCube_SMIMS_FPGA_fops);
	fpga->chrdev.owner = THIS_MODULE;

	ret = cdev_add(&fpga->chrdev, dev_id, 1);
	if (ret)
	{
		dev_err(&pdev->dev, "fail to register driver for %s\n", dev_name(&pdev->dev));
		goto err_cdev_add;
	}
	printk("[smimsfpga]SMIMS_FPGA_probe() cdev_add ok for %s.\n", dev_name(&pdev->dev));

	//
	// create node "/dev/smimsfpga"
	//
	glMacubeclass = class_create(THIS_MODULE, "smimsfpga");
	if (IS_ERR(glMacubeclass))
	{
		dev_err(&pdev->dev, "fail to class_create for %s\n", dev_name(&pdev->dev));
		goto err_cdev_add;
	}
	device = device_create(glMacubeclass, NULL, dev_id, NULL, "smimsfpga");
	if (IS_ERR(device))
	{
		dev_err(&pdev->dev, "fail to device_create for %s\n", dev_name(&pdev->dev));
		goto err_cdev_add;
	}
	//===============================
	


	platform_set_drvdata(pdev, fpga);

	glfpga = fpga;

	return 0;	

err_cdev_add:
	cdev_del(&fpga->chrdev);	

err_iomap:
	iounmap(fpga->regs);

err_ioarea:
	release_resource(fpga->ioarea);
	kfree(fpga->ioarea);
		
err:
	kfree(fpga);
	return ret;
}

/* SMIMS_FPGA_remove
 *
 * called when device is removed from the bus
*/

static int SMIMS_FPGA_remove(struct platform_device *pdev)
{
	MaCube_smimsfpga_t *fpga = platform_get_drvdata(pdev);

	cdev_del(&fpga->chrdev);
	free_irq(fpga->irq, NULL);
	kfree(fpga);
	
	platform_set_drvdata(pdev, NULL);
	pdev->dev.devt = 0;

	return 0;
}


static int __init smimsfpga_init(void)
{
	//int i;	
	int ret;

	ret = platform_driver_register(&MaCube_SMIMS_FPGA_driver);
	if (ret == 0) 
		printk(KERN_INFO PREFIX "[smimsfpga]platform driver register ok!\n");
	else
		printk(KERN_INFO PREFIX "[smimsfpga]platform driver register failed!\n");


	rAHB_addr = (unsigned short *) ioremap(AHB_ADDRESS, DMA_MEMORY_SIZE);
	if ( !rAHB_addr )
	{
		printk(KERN_INFO PREFIX "[smimsfpga]ioremap rAHB_addr error\n");
		return 1;
	}
	REGSetting();

	rSNOR_CONFIG_0_addr = (unsigned long *) ioremap(0x70009000, 8);
	rSNOR_DMA_CFG_0_addr = (unsigned long *) ioremap(0x70009020, 8);
	rSNOR_NOR_ADDR_PTR_0_addr = (unsigned long *) ioremap(0x70009008, 8);
	rSNOR_AHB_ADDR_PTR_0_addr = (unsigned long *) ioremap(0x7000900C, 8);

//	interrupt_read = 0;
//	interrupt_write = 0;


//#ifdef	_DEBUG
	ShowREG();
//#endif

	return ret;
}

static void __exit smimsfpga_exit(void)
{
	platform_driver_unregister(&MaCube_SMIMS_FPGA_driver);
}

int ShowREG(void)
{
	PrintREG("SNOR_CONFIG_0", 0x70009000);
	PrintREG("SNOR_STA_0", 0x70009004);
	PrintREG("SNOR_NOR_ADDR_PRT_0", 0x70009008);
	PrintREG("SNOR_AHB_ADDR_PRT_0", 0x7000900C);
	PrintREG("SNOR_TIMING0_0", 0x70009010);
	PrintREG("SNOR_TIMING1_0", 0x70009014);
	PrintREG("SNOR_MIO_CFG_0", 0x70009018);
	PrintREG("SNOR_MIO_TIMING0_0", 0x7000901C);
	PrintREG("SNOR_DMA_CFG_0", 0x70009020);
	PrintREG("SNOR_CD_MUX_CFG_0", 0x70009024);
	PrintREG("CLK_RST_CONTROLLER_CLK_SOURCE_NOR_0", 0x600061D0);

	PrintREG("APB_MISC_PP_TRISTATE_REG_A_0", 0x70000014);	
	PrintREG("APB_MISC_PP_TRISTATE_REG_B_0", 0x70000018);	
	
	PrintREG("PINMUX_AUX_GMI_IORDY_0", 0x700031c4);
	PrintREG("PINMUX_AUX_GMI_WAIT_0", 0x700031c8);

	//GPIO3 Port-I
	PrintREG("GPIO_CNF_2 Port-I", 0x6000D100);	
	PrintREG("GPIO_OE_2 Port-I", 0x6000D110);
	PrintREG("GPIO_OUT_2 Port-I", 0x6000D120);
	PrintREG("GPIO_IN_2 Port-I", 0x6000D130);
	PrintREG("GPIO_IN_STA_2 Port-I", 0x6000D140);
	PrintREG("GPIO_IN_ENB_2 Port-I", 0x6000D150);
	PrintREG("GPIO_IN_LVL_2 Port-I", 0x6000D160);
	PrintREG("GPIO_IN_CLR_2 Port-I", 0x6000D170);

	//GPIO3 Port-J
	PrintREG("GPIO_CNF_2 Port-J", 0x6000D104);
	PrintREG("GPIO_OE_2 Port-J", 0x6000D114);
	PrintREG("GPIO_OUT_2 Port-J", 0x6000D124);
	PrintREG("GPIO_IN_2 Port-J", 0x6000D134);
	PrintREG("GPIO_INT_STA_2 Port-J", 0x6000D144);
	PrintREG("GPIO_INT_ENB_2 Port-J", 0x6000D154);
	PrintREG("GPIO_INT_LVL_2 Port-J", 0x6000D164);
	PrintREG("GPIO_INT_CLR_2 Port-J", 0x6000D174);
	return 0;
}	

void PrintREG(char * REGName, unsigned long Addr)
{
	unsigned long * rREG_addr;

	rREG_addr = (unsigned long *) ioremap(Addr,8);
	if ( !rREG_addr )
	{
		printk(KERN_INFO PREFIX "[smimsfpga]ioremap rREG_addr error\n");
		return;
	}
	printk(KERN_INFO PREFIX "[%s]=0x%08x\n", REGName, (unsigned int)*rREG_addr);

	//Release map
	iounmap(rREG_addr);
}
int REGSetting(void)
{

	//Set MGI bus clock source rate
	//	pllP_out0(408MHz)
	//	pllM_out0(<=400MHz)
	//	pllC_out0(<=600MHz)
	//Set CLK_RST_CONTROLLER_CLK_SOURCE_NOR_0 register, set to 	PLLP_OUT0 [31:30]=00
	//								PLLC_OUT0 [31:30]=01
	//								PLLM_OUT0 [31:30]=10
	//						and SNOR_CLK_DIVISION to N = (5+1)
	SetREGValue("CLK_RST_CONTROLLER_CLK_SOURCE_NOR_0", 0x600061D0, 0x00000005);
	
	//Set SNOR timing register, set all to 1 clock, the minimum wave form
	SetREGValue("SNOR_TIMING0_0", 0x70009010, 0x00000000);

	//Set SNOR timing register, set all to 1 clock, the minimum wave form
	SetREGValue("SNOR_TIMING1_0", 0x70009014, 0x00000000);

	//Set WR_WIDTH to 2, [23:16]
	SetREGBit("SNOR_TIMING1_0", 0x70009014, 16, 0);
	SetREGBit("SNOR_TIMING1_0", 0x70009014, 17, 1);
	//SetREGBit("SNOR_TIMING1_0", 0x70009014, 18, 1);
	//SetREGBit("SNOR_TIMING1_0", 0x70009014, 19, 1);

	//Set OE_WIDTH to 4, [15:8]
	SetREGBit("SNOR_TIMING1_0", 0x70009014, 8, 0);
	SetREGBit("SNOR_TIMING1_0", 0x70009014, 9, 0);
	SetREGBit("SNOR_TIMING1_0", 0x70009014,10, 1);
	//SetREGBit("SNOR_TIMING1_0", 0x70009014,11, 1);

	//set DEVICE_MODE to ASYNC
	SetREGBit("SNOR_CONFIG_0", 0x70009000, 0, 0);
	SetREGBit("SNOR_CONFIG_0", 0x70009000, 1, 0);

	//set SNOR_SEL to CS2
	SetREGBit("SNOR_CONFIG_0", 0x70009000, 5, 1);
	//set MST_ENB to  Mater DMA mode
	SetREGBit("SNOR_CONFIG_0", 0x70009000, 7, 1);
	//set NMUX_ASYNC_IO TO ENABLE
	SetREGBit("SNOR_CONFIG_0", 0x70009000, 14, 1);

	//set RDY_ACTIVE to 0:With Data 1:One Cycle Before Data
	SetREGBit("SNOR_CONFIG_0", 0x70009000, 24, 0);
	
	//set MUXMODE_GMI to AD_NONMUX
	SetREGBit("SNOR_CONFIG_0", 0x70009000, 28, 0);
	//set NOR_DEVICE to SNOR
	SetREGBit("SNOR_CONFIG_0", 0x70009000, 29, 0);
	//set WORDWIDE_GMI to NOR16BIT
	SetREGBit("SNOR_CONFIG_0", 0x70009000, 30, 0);

	//set SNOR_NOR_ADDR_PTR_0 
	SetREGValue("SNOR_NOR_ADDR_PTR_0 ", 0x70009008, 0x48000000);

	//set SNOR_AHB_ADDR_PTR_0 to IRAM-D
	SetREGValue("SNOR_AHB_ADDR_PTR_0 ", 0x7000900C, AHB_ADDRESS);

	//SetREGValue("SNOR_MIO_TIMING0_0", 0x7000901C, 0x00000000);


	return 0;
}
void SetREGValue(char * REGName, unsigned long Addr, unsigned long value)
{
	unsigned long * rREG_addr;

	rREG_addr = (unsigned long *) ioremap(Addr,8);
	if ( !rREG_addr )
	{
		printk(KERN_INFO PREFIX "[smimsfpga]ioremap rREG_addr error\n");
		return;
	}
	*rREG_addr = value;

	//Release map
	iounmap(rREG_addr);
}
void SetREGBit(char * REGName, unsigned long Addr, int bitpos, int bitvalue)
{
	unsigned long * rREG_addr;
	unsigned long REGValue;

	rREG_addr = (unsigned long *) ioremap(Addr,8);
	if ( !rREG_addr )
	{
		printk(KERN_INFO PREFIX "[smimsfpga]ioremap rREG_addr error\n");
		return;
	}
	REGValue = *rREG_addr;

	if ( bitvalue == 0 )
		REGValue &= ~(1<<bitpos);
	else
		REGValue |= (1<<bitpos);

	*rREG_addr = REGValue;

	//Release map
	iounmap(rREG_addr);
}

void SetAddressValue(unsigned long * rREG_addr, unsigned long value)
{
	*rREG_addr = value;
}

void SetAddressBitValue(unsigned long * rREG_addr, int bitpos, int bitvalue)
{
	unsigned long REGValue;
	REGValue = *rREG_addr;

	if ( bitvalue == 0 )
		REGValue &= ~(1<<bitpos);
	else
		REGValue |= (1<<bitpos);

	*rREG_addr = REGValue;
}

void Send_Data_To_FPGA(void)
{
	unsigned long ulRegDMA_CFG;

	//ulRegDMA_CFG = 0x24001FFC;//AHB2NOR, BURST_SIZE = 1 WORD, WORD_COUNT = 4096
	ulRegDMA_CFG = 0x34001FFC;	//[29]:1(AHB2NOR)
					//[28]:1(Interrupt enable on DMA transfer completion)
					//[26:24]:4(BURST_SIZE = 1 WORD) 
					//[23]:0(Once DMA transfer)
					//[15:2]:1ffc(WORD_COUNT = 4096)

	//printk(KERN_INFO PREFIX "Send_Data_To_FPGA()::ulRegDMA_CFG = 0x%08x",ulRegDMA_CFG);

	SetAddressValue(rSNOR_DMA_CFG_0_addr, ulRegDMA_CFG);
	//set SNOR_NOR_ADDR_PTR_0 
	SetAddressValue(rSNOR_NOR_ADDR_PTR_0_addr, 0x48000000);
	//set SNOR_AHB_ADDR_PTR_0 to IRAM-D
	SetAddressValue(rSNOR_AHB_ADDR_PTR_0_addr, AHB_ADDRESS_W);
	//set GO_NOR to start
	SetAddressBitValue(rSNOR_CONFIG_0_addr, 31, 1);
	//set DMA_GO to start
	SetAddressBitValue(rSNOR_DMA_CFG_0_addr, 31, 1);

	//wait DMA finish
	while(1)
	{
		//if ( *rSNOR_DMA_CFG_0_addr & 0x40000000 )
		//	udelay(1);	
		//else
		//	break;

		if ( *rSNOR_DMA_CFG_0_addr & 0x08000000 ) // [27]:1(Is DMA Done)
			break;
		else
			udelay(1);	
	}

	// clear DMA done
	SetAddressValue(rSNOR_DMA_CFG_0_addr, 0x08000000);

}

void Send_Data_To_FPGA_With_Address_Count(int iAddr, int iCount)
{
	unsigned long ulRegDMA_CFG;
	
//	ulRegDMA_CFG = 0x24001FFC;//AHB2NOR, BURST_SIZE = 1 WORD, WORD_COUNT = 4096
	ulRegDMA_CFG = 0x24000000 + ((iCount-1) << 2);
	 
//	printk(KERN_INFO PREFIX "%s::ulRegDMA_CFG = 0x%08x\n", 
//					__FUNCTION__, ulRegDMA_CFG);
	
	SetAddressValue(rSNOR_DMA_CFG_0_addr, ulRegDMA_CFG);
	//set SNOR_NOR_ADDR_PTR_0 
	SetAddressValue(rSNOR_NOR_ADDR_PTR_0_addr, 0x48000000 + iAddr*2);
	//set SNOR_AHB_ADDR_PTR_0 to IRAM-D
	SetAddressValue(rSNOR_AHB_ADDR_PTR_0_addr, AHB_ADDRESS_W);
	//set GO_NOR to start
	SetAddressBitValue(rSNOR_CONFIG_0_addr, 31, 1);
	//set DMA_GO to start
	SetAddressBitValue(rSNOR_DMA_CFG_0_addr, 31, 1);

	//wait DMA finish
	while(1)
	{
		if ( *rSNOR_DMA_CFG_0_addr & 0x40000000 )
			udelay(1);  
		else
			break;
	}
	
}

void Get_Data_From_FPGA(void)
{
	unsigned long ulRegDMA_CFG;

	//ulRegDMA_CFG = 0x04001FFC;//NOR2AHB, BURST_SIZE = 1 WORD, WORD_COUNT = 4096
	ulRegDMA_CFG = 0x14001FFC;	//[29]:0(NOR2AHB)
					//[28]:1(Interrupt enable on DMA transfer completion)
					//[26:24]:4(BURST_SIZE = 1 WORD) 
					//[23]:0(Once DMA transfer)
					//[15:2]:1ffc(WORD_COUNT = 4096)

	SetAddressValue(rSNOR_DMA_CFG_0_addr, ulRegDMA_CFG);
	//set SNOR_NOR_ADDR_PTR_0 
	SetAddressValue(rSNOR_NOR_ADDR_PTR_0_addr, 0x48000000);
	//set SNOR_AHB_ADDR_PTR_0 to IRAM-D
	SetAddressValue(rSNOR_AHB_ADDR_PTR_0_addr, AHB_ADDRESS_R);
	//set GO_NOR to start
	SetAddressBitValue(rSNOR_CONFIG_0_addr, 31, 1);
	//set DMA_GO to start
	SetAddressBitValue(rSNOR_DMA_CFG_0_addr, 31, 1);

	//wait DMA finish
	while(1)
	{
		//if ( *rSNOR_DMA_CFG_0_addr & 0x40000000 )
		//	udelay(1);	
		//else
		//	break;

		if ( *rSNOR_DMA_CFG_0_addr & 0x08000000 ) // [27]:1(Is DMA Done)
			break;	
		else
			udelay(1);
	}
	// clear DMA done
	SetAddressValue(rSNOR_DMA_CFG_0_addr, 0x08000000);
}

void Get_Data_From_FPGA_With_Address_Count(int iAddr, int iCount)
{
	unsigned long ulRegDMA_CFG;

//	ulRegDMA_CFG = 0x04001FFC;//NOR2AHB, BURST_SIZE = 1 WORD, WORD_COUNT = 4096

	ulRegDMA_CFG = 0x04000000 + ((iCount-1) << 2);
	     
//	printk(KERN_INFO PREFIX "%s::ulRegDMA_CFG = 0x%08x\n", 
//					__FUNCTION__, ulRegDMA_CFG);

	SetAddressValue(rSNOR_DMA_CFG_0_addr, ulRegDMA_CFG);
	//set SNOR_NOR_ADDR_PTR_0 
	SetAddressValue(rSNOR_NOR_ADDR_PTR_0_addr, 0x48000000 + iAddr * 2);
	//set SNOR_AHB_ADDR_PTR_0 to IRAM-D
	SetAddressValue(rSNOR_AHB_ADDR_PTR_0_addr, AHB_ADDRESS_R);
	//set GO_NOR to start
	SetAddressBitValue(rSNOR_CONFIG_0_addr, 31, 1);
	//set DMA_GO to start
	SetAddressBitValue(rSNOR_DMA_CFG_0_addr, 31, 1);

	//wait DMA finish
	while(1)
	{
		if ( *rSNOR_DMA_CFG_0_addr & 0x40000000 )
			udelay(1);  
		else
			break;
	}
}
module_init(smimsfpga_init);
module_exit(smimsfpga_exit);

MODULE_AUTHOR("ikki chung <ikki@smims.com>");
MODULE_DESCRIPTION("SMIMS FPGA IF Platform Driver");
MODULE_LICENSE("GPL");





