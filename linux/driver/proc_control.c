/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/64
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Linux Driver - ReconOS - Proc control
 *
 *   project:      ReconOS
 *   author:       Christoph Rüthing, University of Paderborn
 *                 Lennart Clausing, University of Paderborn
 * 
 *   description:  Driver for the proc control of the ReconOS system. It
 *                 allows to control the dirfferent components, reset the
 *                 entire system and read out some information like the
 *                 number of OSIFs, ...
 * 
 *  log:
 *                  -- changed register offsets back to 32bit, using high regs instead
 *                  -- use iowrite64
 *
 * ======================================================================
 */

#include "proc_control.h"

#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/io.h>
#include <linux/uaccess.h>
#include <asm/pgtable.h>


/* == General definitions ============================================== */

/*
 * Register definitions as offset from base address
 *
 *32 num_hwts_reg        - number of hardware threads instanciated
 *64 pgd_addr_reg        - physical address of page global directory
 *64 page_fault_addr_reg - addres whicch caused a page fault
 *32 tlb_hits_reg        - number of tlb hits occured since reset
 *32 tlb_misses_reg      - number of tlb misses occured since reset
 *32 sys_reset_reg       - write to initiate system reset
 *32 hwt_reset_reg       - write to set reset of hardware threads
 *32 hwt_signal_reg     - write to set signal of hardware threads
 */
#define NUM_HWTS_REG           0x00

#define PGD_ADDR_REG           0x08
#define PGD_ADDR_REG_HI        0x08
#define PGD_ADDR_REG_LO        0x0C

#define PAGE_FAULT_ADDR_REG    0x10
#define PAGE_FAULT_ADDR_REG_HI 0x10
#define PAGE_FAULT_ADDR_REG_LO 0x14

#define TLB_HITS_REG           0x18
#define TLB_MISSES_REG         0x1C
#define SYS_RESET_REG          0x20
#define HWT_RESET_REG(hwt)     (0x24 + HWT_REG_OFFSET(hwt) * 4)
#define HWT_SIGNAL_REG(hwt)    (HWT_RESET_REG(hwt) + DYNAMIC_REG_COUNT * 4) //*4

/*
 * Struct representing the proc control device
 *
 *   name            - name to identify the device driver
 *   base_addr       - base addr (should always be BASE_ADDR)
 *   mem_size        - memory size (should always be MEM_SIZE)
 *   irq             - irq number (should always be IRQ)
 *
 *   mem             - pointer to the io memory
 *   wait            - wait queue for interrupts
 *   page_fault      - indication if page fault occured
 *   page_fault_addr - address which caused the page fault
 *   hwt_resets      - array representing the reset signals
 *   hwt_signals     - array representing the signals
 *
 *   mdev            - misc device data structure
 *
 *   lock            - spinlock for synchronization
 */
struct proc_control_dev {
	char name[25];
	uint64_t base_addr;        //64
	int mem_size;
	int irq;

	void __iomem *mem;
	wait_queue_head_t wait;
	int page_fault;
	uint64_t page_fault_addr;  //64
	uint32_t *hwt_resets;      //32
	uint32_t *hwt_signals;     //32

	struct miscdevice mdev;

	spinlock_t lock;
};

static struct proc_control_dev proc_control;


/* == Low level functions ============================================== */

/*
 * Reads a register
 *
 *   dev - pointer to the proc control struct
 *   reg - offset from base address to read
 *
 *   @returns read data
 */
static inline uint32_t read_reg32(struct proc_control_dev *dev, unsigned int reg) {
        printk(KERN_INFO "[reconos-proc-control] read_reg32 @offset %#.16lx\n", reg);
	return ioread32(dev->mem + reg);
}

static inline uint64_t read_reg64(struct proc_control_dev *dev, unsigned int reg) {
        printk(KERN_INFO "[reconos-proc-control] read_reg64 @offset %#.16lx\n", reg);
        return ( (((uint64_t)ioread32(dev->mem + reg)) << 32) + (uint64_t) ioread32(dev->mem + reg + 4) );
		//this approach reads 2x 32bit from proc_control, but in the wrong order (low|high)!
		//return ioread64(dev->mem + reg); 
}

/*
 * Writes a register
 *
 *   dev  - pointer to the proc control struct
 *   reg  - offset from base address to write
 *   data - data to write
 */
static inline void write_reg32(struct proc_control_dev *dev, unsigned int reg, uint32_t data) {
        printk(KERN_INFO "[reconos-proc-control] write_reg32 @offset %#.16lx\n", reg);
	iowrite32(data, dev->mem + reg);
}

static inline void write_reg64(struct proc_control_dev *dev, unsigned int reg, uint64_t data) {
        printk(KERN_INFO "[reconos-proc-control] write_reg64 @offset %#.16lx\n", reg);     
        iowrite32((uint32_t) ((data >> 32) & 0xFFFFFFFF), dev->mem + reg);
        iowrite32((uint32_t) (data & 0xFFFFFFFF), dev->mem + reg + 4);
		//this approach is possibly incompatible with the current proc_control implementation
		//iowrite64(data, dev->mem + reg);
}

/*
 * Flushing of the systems caches
 *
 *   no parameters
 */
#if defined(RECONOS_ARCH_zynq64)
static void flush_cache(void) {
}
#elif defined(RECONOS_ARCH_microblaze)
static void flush_cache(void) {
	int i;
	int baseaddr, bytesize, linelen;

	baseaddr = 0x20000000;   // C_DCACHE_BASEADDR
	bytesize = 64 * 1024;    // C_DCACHE_BYTE_SIZE
	linelen = 4 * 4;         // C_DCACHE_LINE_LEN * 4

	for (i = 0; i < bytesize; i += linelen) {
		asm volatile ("wdc.flush %0, %1;" :: "d" (baseaddr), "d" (i));
	}
}
#endif


/* == File operations ================================================== */

/*
 * Function called when opening the device
 *
 *    @see kernel documentation
 */
static int proc_control_open(struct inode *inode, struct file *filp) {
	filp->private_data = &proc_control;

	return 0;
}

/*
 * Function called when issuing an ioctl
 *
 * @see kernel documentation
 */
static long proc_control_ioctl(struct file *filp, unsigned int cmd,
                               unsigned long arg) {
	struct proc_control_dev *dev;
	uint32_t data32;
    uint64_t data64;
	int i, hwt;
	unsigned long ret;

	dev = (struct proc_control_dev *)filp->private_data;

	switch (cmd) {
		case RECONOS_PROC_CONTROL_GET_NUM_HWTS:
			data32 = NUM_HWTS;
			ret = copy_to_user((int *)arg, &data32, sizeof(uint32_t));
			break;

		case RECONOS_PROC_CONTROL_GET_TLB_HITS:
			data32 = read_reg32(dev, TLB_HITS_REG);
			ret = copy_to_user((int *)arg, &data32, sizeof(int));
			break;

		case RECONOS_PROC_CONTROL_GET_TLB_MISSES:
			data32 = read_reg32(dev, TLB_MISSES_REG);
			ret = copy_to_user((int *)arg, &data32, sizeof(int));
			break;

		case RECONOS_PROC_CONTROL_GET_FAULT_ADDR:
			dev->page_fault = 0;
			enable_irq(dev->irq);

			do {
				if (wait_event_interruptible(dev->wait, dev->page_fault) < 0) {
					printk(KERN_DEBUG "[reconos-proc-control] "
					                    "interrupted, aborting ...\n");
					disable_irq(dev->irq);
					dev->page_fault = 0;
					return -1;
				}
			} while (!dev->page_fault);

			disable_irq(dev->irq);
			ret = copy_to_user((uint64_t *)arg, &dev->page_fault_addr, sizeof(uint64_t));
			break;

		case RECONOS_PROC_CONTROL_CLEAR_PAGE_FAULT:
			dev->page_fault = 0;
			write_reg64(dev, PAGE_FAULT_ADDR_REG, 0);
			break;

		case RECONOS_PROC_CONTROL_SET_PGD_ADDR:
			data64 = (uint64_t) virt_to_phys(current->mm->pgd);
			printk(KERN_INFO "[reconos-proc-control] "
						"set_pgd_address: %#.16lx\n", data64);
                        printk(KERN_INFO "[reconos-proc-control] "
                                                "(mem hardware base: %#.16lx)\n", (dev->mem));
			write_reg64(dev, PGD_ADDR_REG, data64);
			break;

		case RECONOS_PROC_CONTROL_SYS_RESET:
			spin_lock(&dev->lock);

			for (i = 0; i < DYNAMIC_REG_COUNT; i++) {
				dev->hwt_resets[i] = 0xFFFFFFFF;
				dev->hwt_signals[i] = 0x00000000;
			}
			write_reg32(dev, SYS_RESET_REG, 0);

			spin_unlock(&dev->lock);
			break;

		case RECONOS_PROC_CONTROL_SET_HWT_RESET:
			ret = copy_from_user(&hwt, (int *)arg, sizeof(int));

			spin_lock(&dev->lock);

			if (hwt >= 0 && hwt < NUM_HWTS) {
				dev->hwt_resets[hwt / 32] |= 0x1 << hwt % 32;
				data32 = dev->hwt_resets[hwt / 32];

				write_reg32(dev, HWT_RESET_REG(hwt), data32);
			}

			spin_unlock(&dev->lock);
			break;

		case RECONOS_PROC_CONTROL_CLEAR_HWT_RESET:
			ret = copy_from_user(&hwt, (int *) arg, sizeof(int));

			spin_lock(&dev->lock);

			if (hwt >= 0 && hwt < NUM_HWTS) {
				dev->hwt_resets[hwt / 32] &= ~(0x1 << hwt % 32);
				data32 = dev->hwt_resets[hwt / 32];

				write_reg32(dev, HWT_RESET_REG(hwt), data32);
			}

			spin_unlock(&dev->lock);
			break;

		case RECONOS_PROC_CONTROL_SET_HWT_SIGNAL:
			ret = copy_from_user(&hwt, (int *)arg, sizeof(int));

			spin_lock(&dev->lock);

			if (hwt >= 0 && hwt < NUM_HWTS) {
				dev->hwt_signals[hwt / 32] |= 0x1 << hwt % 32;
				data32 = dev->hwt_signals[hwt / 32];

				write_reg32(dev, HWT_SIGNAL_REG(hwt), data32);
			}

			spin_unlock(&dev->lock);
			break;

		case RECONOS_PROC_CONTROL_CLEAR_HWT_SIGNAL:
			ret = copy_from_user(&hwt, (int *) arg, sizeof(int));

			spin_lock(&dev->lock);

			if (hwt >= 0 && hwt < NUM_HWTS) {
				dev->hwt_signals[hwt / 32] &= ~(0x1 << hwt % 32);
				data32 = dev->hwt_signals[hwt / 32];

				write_reg32(dev, HWT_SIGNAL_REG(hwt), data32);
			}

			spin_unlock(&dev->lock);
			break;

		case RECONOS_PROC_CONTROL_CACHE_FLUSH:
			flush_cache();
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

/*
 * Struct for file operations to register driver
 *
 *    @see kernel documentation
 */
struct file_operations proc_control_fops = {
	.owner          = THIS_MODULE,
	.open           = proc_control_open,
	.unlocked_ioctl = proc_control_ioctl,
};


/* == Interrupt handling =============================================== */

/*
 * Interrupt handler for handling page faults
 *
 *   @see kernel documentation
 */
static irqreturn_t interrupt(int irq, void *data) {
	struct proc_control_dev *dev = data;

        printk(KERN_INFO "[reconos-proc-control] "
	                   "INTERRUPT: IRQ %d, CALLER: \n", irq, (void *)_RET_IP_);
        printk(KERN_INFO "[reconos-proc-control] "
	                   "proc_control_dev data: base: %#.16lx size: %#x irq: %d page_fault_addr: %#.16lx  \n", dev->base_addr, dev->mem_size, dev->irq, dev->page_fault_addr);
        printk(KERN_INFO "[reconos-proc-control] "
	                   "PAGE_FAULT_ADDR_REG: %#.16lx \n", read_reg64(dev, PAGE_FAULT_ADDR_REG));
	printk(KERN_INFO "[reconos-proc-control] "
	                   "page fault occured\n");

	dev->page_fault_addr = read_reg64(dev, PAGE_FAULT_ADDR_REG);
	dev->page_fault = 1;
	wake_up_interruptible(&dev->wait);

	return IRQ_HANDLED;
}


/* == Init and exit functions ========================================== */

/*
 * Struct for device tree matching
 */
static struct of_device_id proc_control_of_match[] =
{
    { .compatible = "upb,reconos-control-3.1" },
    { .compatible = "upb,reconos-proc-control-3.1" },
    { .compatible = "xlnx,reconos-proc-control-1.0" }, //tmp fix /lc
    {}
};


/*
 * @see header
 */
int proc_control_num_hwts_static(void) {
	struct device_node *node = NULL;
	struct resource res;
	uint64_t base_addr;
	int mem_size;
	void __iomem *mem;
	int num_hwts;

	// get matching of node
	node = of_find_matching_node(NULL, proc_control_of_match);
	if (!node)
	{
		printk(KERN_ERR "[reconos-proc-control-STATIC] "
		                  "device tree node not found\n");
		goto of_failed;
	}
	printk(KERN_INFO "[reconos-proc-control-STATIC] "
	                   "found device %s\n", node->name);

	// getting address from device tree
	if (of_address_to_resource(node, 0, &res))
	{
		printk(KERN_ERR "[reconos-proc-control-STATIC] "
	                      "address could not be determined\n");
		goto req_failed;
	}
	base_addr = res.start;
	mem_size = res.end - res.start + 1;
	printk(KERN_INFO "[reconos-proc-control-STATIC] "
	                   "found memory at %#.16lx with size %#x\n",
	                   base_addr, mem_size);

	// allocation io memory to read proc control registers
	if (!request_mem_region(base_addr, mem_size, "reconos-proc-control")) {
		printk(KERN_WARNING "[reconos-proc-control-STATIC] "
		                      "memory region busy\n");
		goto req_failed;
	}

	mem = ioremap(base_addr, mem_size);
	if (!mem) {
		printk(KERN_WARNING "[reconos-proc-control-STATIC] "
		                      "ioremap failed\n");
		goto map_failed;
	}

	num_hwts = ioread32(mem + NUM_HWTS_REG);
        
        printk(KERN_INFO "[reconos-proc-control-STATIC] "
	                   "found HWTs = %d from mem(%#x + %#x) \n",
	                   num_hwts, mem, NUM_HWTS_REG);

	// free io memory mapping
	iounmap(mem);
	release_mem_region(base_addr, mem_size);

	return num_hwts;

map_failed:
	release_mem_region(base_addr, mem_size);

req_failed:
of_failed:
	return -1;
}

/*
 * @see header
 */
int proc_control_init(void) {
	struct proc_control_dev *dev = NULL;
	struct device_node *node = NULL;
	struct resource res;
	int i;

	printk(KERN_INFO "[reconos-proc-control] "
	                   "initializing 64bit driver ...\n");

        printk(KERN_INFO "[reconos-proc-control] "
                    "ARM64, little-endian ...\n");
        
	dev = &proc_control;

	// get matching of node
	node = of_find_matching_node(NULL, proc_control_of_match);
	if (!node)
	{
		printk(KERN_ERR "[reconos-proc-control] "
		                  "device tree node not found\n");
		goto of_failed;
	}
	printk(KERN_INFO "[reconos-proc-control] "
	                   "found device %s\n", node->name);

	// set some general information of proc control
	strncpy(dev->name, "reconos-proc-control", 25);
	dev->page_fault = 0;

	// allocating reset-register
	dev->hwt_resets = kcalloc(DYNAMIC_REG_COUNT, sizeof(uint32_t), GFP_KERNEL);
	if (!dev->hwt_resets) {
		printk(KERN_WARNING "[reconos-proc-control] "
		                      "cannot allocate proc control memory\n");
		goto hwt_resets_failed;
	}

	for (i = 0; i < DYNAMIC_REG_COUNT; i++) {
		dev->hwt_resets[i] = 0xFFFFFFFF;
	}

	// allocating signal-register
	dev->hwt_signals = kcalloc(DYNAMIC_REG_COUNT, sizeof(uint32_t), GFP_KERNEL);
	if (!dev->hwt_signals) {
		printk(KERN_WARNING "[reconos-proc-control] "
		                      "cannot allocate proc control memory\n");
		goto hwt_signals_failed;
	}

	// getting address from device tree
	if (of_address_to_resource(node, 0, &res))
	{
		printk(KERN_ERR "[reconos-proc-control] "
	                      "address could not be determined\n");
		goto req_failed;
	}
	dev->base_addr = res.start;
	dev->mem_size = res.end - res.start + 1;
	printk(KERN_INFO "[reconos-proc-control] "
	                   "found memory at %#016x with size %#x\n",
	                   dev->base_addr, dev->mem_size);

	// allocation io memory to read proc control registers
	if (!request_mem_region(dev->base_addr, dev->mem_size, dev->name)) {
		printk(KERN_WARNING "[reconos-proc-control] "
		                      "memory region busy\n");
		goto req_failed;
	}

	dev->mem = ioremap(dev->base_addr, dev->mem_size);
	if (!dev->mem) {
		printk(KERN_WARNING "[reconos-proc-control] "
		                      "ioremap failed\n");
		goto map_failed;
	}

	// reset entire system
	write_reg32(dev, SYS_RESET_REG, 1);

	// getting interrupt number from device tree
	dev->irq = irq_of_parse_and_map(node, 0);
	if (!dev->irq)
	{
		printk(KERN_ERR "[reconos-proc-control] "
		                  "irq could not be determined\n");
		goto irq_failed;
	}
	printk(KERN_INFO "[reconos-proc-control] "
	                   "found interrupt %d\n", dev->irq);

	// requesting interrupt
	if (request_irq(dev->irq, interrupt, 0, "reconos-proc-control", dev)) {
		printk(KERN_WARNING "[reconos-proc-control] "
		                      "can't get irq\n");
		goto irq_failed;
	}

	disable_irq(dev->irq);

	// initialize spinlock
	spin_lock_init(&dev->lock);

	// initializing misc-device structure
	dev->mdev.minor = MISC_DYNAMIC_MINOR;
	dev->mdev.fops = &proc_control_fops;
	dev->mdev.name = dev->name;

	if (misc_register(&dev->mdev) < 0) {
		printk(KERN_WARNING "[reconos-proc-control] "
		                      "error while registering misc-device\n");
		goto reg_failed;
	}

	// initialize remaining struct parts
	init_waitqueue_head(&dev->wait);

	printk(KERN_INFO "[reconos-proc-control] "
	                   "driver initialized successfully\n");


	return 0;

reg_failed:
	free_irq(dev->irq, dev);
	misc_deregister(&dev->mdev);

irq_failed:
	iounmap(dev->mem);

map_failed:
	release_mem_region(dev->base_addr, dev->mem_size);

req_failed:
	kfree(dev->hwt_signals);

hwt_signals_failed:
	kfree(dev->hwt_resets);

hwt_resets_failed:
of_failed:
	return -1;
}

/*
 * @see header
 */
int proc_control_exit(void) {
	struct proc_control_dev *dev;
	
	printk(KERN_INFO "[reconos-proc-control] "
	                   "removing driver ...\n");

	dev = &proc_control;

	misc_deregister(&dev->mdev);

	kfree(dev->hwt_resets);

	free_irq(dev->irq, dev);

	iounmap(dev->mem);
	release_mem_region(dev->base_addr, dev->mem_size);

	return 0;
}
