/*
 * dump_pmem.c - initialize and dump pmem from aess
 */

#include <linux/module.h> /* Needed by all modules */
#include <linux/printk.h> /* Needed for pr_info() */

#include <asm/io.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#define OMAP_AESS_REVISION			0x00
#define OMAP_AESS_MCU_IRQSTATUS_RAW		0x24
#define OMAP_AESS_MCU_IRQSTATUS			0x28
#define OMAP_AESS_MCU_IRQENABLE_SET		0x3C
#define OMAP_AESS_MCU_IRQENABLE_CLR		0x40
#define OMAP_AESS_DSP_IRQSTATUS_RAW		0x4C
#define OMAP_AESS_DMAENABLE_SET			0x60
#define OMAP_AESS_DMAENABLE_CLR			0x64
#define OMAP_AESS_EVENT_GENERATOR_COUNTER	0x68
#define OMAP_AESS_EVENT_GENERATOR_START		0x6C
#define OMAP_AESS_EVENT_SOURCE_SELECTION	0x70
#define OMAP_AESS_AUDIO_ENGINE_SCHEDULER	0x74
#define OMAP_AESS_AUTO_GATING_ENABLE		0x7C
#define OMAP_AESS_DMASTATUS_RAW			0x84

#define OMAP_ABE_IO_RESOURCES	5
#define OMAP_ABE_DMEM 0
#define OMAP_ABE_CMEM 1
#define OMAP_ABE_SMEM 2
#define OMAP_ABE_PMEM 3
#define OMAP_ABE_AESS 4

static const char *abe_memory_bank[5] = {
	"dmem",
	"cmem",
	"smem",
	"pmem",
	"mpu"
};

struct omap_abe_aess {
	struct device *dev;
	struct clk *clk;
	void __iomem *io_base[5];
	struct timer_list timer;
};

static inline void omap_aess_reg_writel(struct omap_abe_aess *abe,
				u32 offset, u32 val)
{
	pm_runtime_get_sync(abe->dev);
	__raw_writel(val, (abe->io_base[OMAP_ABE_AESS] + offset));
	pm_runtime_put_sync(abe->dev);
}

/**
 * omap_aess_hw_configuration
 * @aess: Pointer on aess handle
 *
 * Initialize the AESS HW registers for MPU and DMA
 * request visibility.
 */
void omap_aess_hw_configuration(struct omap_abe_aess *aess)
{
	/* enable AESS auto gating (required to release all AESS clocks) */
	omap_aess_reg_writel(aess, OMAP_AESS_AUTO_GATING_ENABLE, 1);
	/* enables the DMAreq from AESS AESS_DMAENABLE_SET = 255 */
#if FIXME
	omap_aess_reg_writel(aess, OMAP_AESS_DMAENABLE_SET, DMA_ENABLE_ALL);
	/* enables the MCU IRQ from AESS to Cortex A9 */
	omap_aess_reg_writel(aess, OMAP_AESS_MCU_IRQENABLE_SET, INT_SET);
#endif
}

static ssize_t abe_read_mem(struct omap_abe_aess *abe, char *user_buf,
			       size_t count, loff_t *ppos, void *mem, int size)
{
	ssize_t ret = 0;

	pm_runtime_get_sync(abe->dev);
	set_current_state(TASK_INTERRUPTIBLE);

	if (*ppos >= size)
		goto out;

	if (*ppos + count > size)
		count = size - *ppos;

	memcpy(user_buf, mem + *ppos, count);

	*ppos += count;
	ret = count;
out:
	__set_current_state(TASK_RUNNING);
	pm_runtime_put_sync(abe->dev);
	return ret;
}

static void abe_probe(struct timer_list *t)
{
	struct omap_abe_aess *abe = from_timer(abe, t, timer);
	ssize_t ret;
	static u8 buf[PAGE_SIZE];
	loff_t ppos = 0;

	dev_info(abe->dev, "Dumping a page of ABE AESS pmem:\n");

	pm_runtime_enable(abe->dev);
	pm_runtime_irq_safe(abe->dev);

	/* aess_clk has to be enabled to access hal register.
	 * Disable the clk after it has been used.
	 */
	pm_runtime_get_sync(abe->dev);

	/* omap_aess_reset_hal() */
	omap_aess_hw_configuration(abe);

	memzero_explicit(buf, sizeof(buf));
	ret = abe_read_mem(abe, buf, sizeof(buf), &ppos, abe->io_base[OMAP_ABE_PMEM], 8192);
	pm_runtime_put_sync(abe->dev);
	if (ret <= 0)
		dev_err(abe->dev, "something went wrong: %d\n", ret);
	else {
		print_hex_dump(KERN_INFO, "pmem: ", DUMP_PREFIX_OFFSET, 16, 2, buf, sizeof(buf), false);
	}
}

static int abe_engine_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct omap_abe_aess *abe;
	int i;

	abe = devm_kzalloc(&pdev->dev, sizeof(struct omap_abe_aess), GFP_KERNEL);
	if (abe == NULL)
		return -ENOMEM;

	for (i = 0; i < OMAP_ABE_IO_RESOURCES; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   abe_memory_bank[i]);
		if (res == NULL) {
			dev_err(&pdev->dev, "no resource %s\n",
				abe_memory_bank[i]);
			return -ENODEV;
		}
		if (!devm_request_mem_region(&pdev->dev, res->start,
					resource_size(res), abe_memory_bank[i]))
			return -EBUSY;

		abe->io_base[i] = devm_ioremap(&pdev->dev, res->start,
					       resource_size(res));
		if (!abe->io_base[i])
			return -ENOMEM;

		dev_info(&pdev->dev, "mapped %s @ %pa -> %px (%zd)\n",
			 abe_memory_bank[i], &res->start, abe->io_base[i],
			 resource_size(res));
	}
	dev_set_drvdata(&pdev->dev, abe);
	abe->dev = &pdev->dev;

	timer_setup(&abe->timer, abe_probe, 0);
	mod_timer(&abe->timer, jiffies + msecs_to_jiffies(2000));

	return 0;
}

static int abe_engine_remove(struct platform_device *pdev)
{
	struct omap_abe_aess *abe = dev_get_drvdata(&pdev->dev);

	del_timer(&abe->timer);
	pm_runtime_disable(abe->dev);

	return 0;
}

static const struct of_device_id omap_aess_of_match[] = {
	{ .compatible = "ti,omap4-aess", },
	{ }
};
MODULE_DEVICE_TABLE(of, omap_aess_of_match);

static struct platform_driver omap_aess_driver = {
	.driver = {
		.name = "aess",
		.owner = THIS_MODULE,
		.of_match_table = omap_aess_of_match,
	},
	.probe = abe_engine_probe,
	.remove = abe_engine_remove,
};

module_platform_driver(omap_aess_driver);

MODULE_ALIAS("platform:omap-aess");
MODULE_DESCRIPTION("ASoC OMAP4 ABE");
MODULE_AUTHOR("Liam Girdwood <lrg@ti.com>");
MODULE_LICENSE("GPL");
