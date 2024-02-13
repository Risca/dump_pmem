/*
 * dump_pmem.c - initialize and dump pmem from aess
 */

#include <linux/module.h> /* Needed by all modules */
#include <linux/printk.h> /* Needed for pr_info() */

#include <asm/io.h>
#include <linux/io.h>
#include <linux/firmware.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <sound/soc.h>
#include <sound/soc-topology.h>

#define AESS_FW_NAME   "omap_aess-adfw.bin"

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

/* AESS memory bank IDs */
#define OMAP_AESS_BANK_DMEM	0
#define OMAP_AESS_BANK_CMEM	1
#define OMAP_AESS_BANK_SMEM	2
#define OMAP_AESS_BANK_PMEM	3
#define OMAP_AESS_BANK_AESS	4

static const char *abe_memory_bank[5] = {
	"dmem",
	"cmem",
	"smem",
	"pmem",
	"mpu"
};

/* Firmware coefficients and equalizers */
#define OMAP_AESS_MAX_FW_SIZE		(1024 * 128)

/*
 * AESS Firmware Header.
 * The AESS firmware blob has a header that describes each data section. This
 * way we can store coefficients etc in the firmware.
 */
struct fw_header {
	u32 version;	/* min version of AESS firmware required */
	u32 pmem_size;
	u32 cmem_size;
	u32 dmem_size;
	u32 smem_size;
};

/*
 * Coeffcient File Data.
 */
struct snd_soc_file_coeff_data {
	__le32 count; /* in elems */
	__le32 size;	/* total data size */
	__le32 id; /* associated mixer ID */
	/* data here */
} __attribute__((packed));

struct omap_abe_aess {
	struct device *dev;
	struct clk *clk;
	void __iomem *io_base[5];
	struct timer_list timer;
	struct snd_soc_component comp;
	struct device *card_dev;
	struct snd_soc_card card;
	bool card_registered;
	const struct firmware *fw;
	const void *fw_config;	// firmware config (within fw)
	const void *fw_data;	// firmware binary (within fw)
	struct fw_header fw_hdr;	// REVISIT: copy of first bytes of fw_data
};

static inline void omap_aess_reg_writel(struct omap_abe_aess *abe,
				u32 offset, u32 val)
{
	pm_runtime_get_sync(abe->dev);
	__raw_writel(val, (abe->io_base[OMAP_AESS_BANK_AESS] + offset));
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
/* linux-3.8 function */
static inline void omap_abe_mem_write(struct omap_abe_aess *abe, int bank,
				      u32 offset, u32 *src, size_t bytes)
{
	pm_runtime_get_sync(abe->dev);
	memcpy((void __force *)(abe->io_base[bank] + offset), src, bytes);
	pm_runtime_put_sync(abe->dev);
}
/* how mainline function is named */
#define omap_aess_write(aess,bank,offset,src,bytes) \
	omap_abe_mem_write(aess,bank,offset,src,bytes)

/**
 * omap_aess_load_fw_param - Load the AESS FW inside AESS memories
 * @aess: Pointer on aess handle
 * @data: Pointer on the AESS firmware (after the header)
 *
 * Load the different AESS memories PMEM/DMEM/SMEM/DMEM
 */
static void omap_aess_load_fw_param(struct omap_abe_aess *aess)
{
	u32 pmem_size, dmem_size, smem_size, cmem_size;
	u32 *pmem_ptr, *dmem_ptr, *smem_ptr, *cmem_ptr;

	/* Take FW memories banks sizes */
	pmem_size = aess->fw_hdr.pmem_size;
	cmem_size = aess->fw_hdr.cmem_size;
	dmem_size = aess->fw_hdr.dmem_size;
	smem_size = aess->fw_hdr.smem_size;
	pmem_ptr = (u32 *) aess->fw_data;
	cmem_ptr = pmem_ptr + (pmem_size >> 2);
	dmem_ptr = cmem_ptr + (cmem_size >> 2);
	smem_ptr = dmem_ptr + (dmem_size >> 2);

	omap_aess_write(aess, OMAP_AESS_BANK_PMEM, 0, pmem_ptr, pmem_size);
	omap_aess_write(aess, OMAP_AESS_BANK_CMEM, 0, cmem_ptr, cmem_size);
	omap_aess_write(aess, OMAP_AESS_BANK_SMEM, 0, smem_ptr, smem_size);
	omap_aess_write(aess, OMAP_AESS_BANK_DMEM, 0, dmem_ptr, dmem_size);
}

static void aess_dump_pmem(struct omap_abe_aess *abe)
{
	ssize_t ret;
	u8 buf[256] = {};
	loff_t ppos = 0;

	print_hex_dump(KERN_INFO, "fw pmem: ", DUMP_PREFIX_OFFSET, 16, 2, abe->fw_data, 256, false);

	dev_info(abe->dev, "Dumping a page of ABE AESS pmem:\n");

	ret = abe_read_mem(abe, buf, sizeof(buf), &ppos, abe->io_base[OMAP_AESS_BANK_PMEM], 8192);
	if (ret <= 0)
		dev_err(abe->dev, "something went wrong: %d\n", ret);
	else
		print_hex_dump(KERN_INFO, "pmem: ", DUMP_PREFIX_OFFSET, 16, 2, buf, sizeof(buf), false);
}

static void aess_dump_pmem_timer_cb(struct timer_list *t)
{
	struct omap_abe_aess *abe = from_timer(abe, t, timer);
	aess_dump_pmem(abe);
}

static int omap_aess_get_fw(struct omap_abe_aess *aess)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, AESS_FW_NAME, aess->dev);
	if (ret) {
		dev_err(aess->dev, "FW request failed: %d\n", ret);
		return ret;
	}

	if (unlikely(!fw->data)) {
		dev_err(aess->dev, "Loaded firmware is empty\n");
		release_firmware(fw);
		return -EINVAL;
	}

	aess->fw = fw;

	return ret;
}

static int aess_load_fw(struct snd_soc_component *component,
			struct snd_soc_tplg_hdr *hdr)
{
	struct omap_abe_aess *aess = snd_soc_component_get_drvdata(component);
	const void *fw_data = snd_soc_tplg_get_data(hdr);

	/* get firmware and coefficients header info */
	memcpy(&aess->fw_hdr, fw_data, sizeof(struct fw_header));
	if (hdr->payload_size > OMAP_AESS_MAX_FW_SIZE) {
		dev_err(aess->dev, "Firmware too large (%d bytes)\n",
			hdr->payload_size);
		return -ENOMEM;
	}
	dev_info(aess->dev, "AESS firmware size %d bytes\n", hdr->payload_size);
	dev_info(aess->dev, "AESS mem P %d C %d D %d S %d bytes\n",
		 aess->fw_hdr.pmem_size, aess->fw_hdr.cmem_size,
		 aess->fw_hdr.dmem_size, aess->fw_hdr.smem_size);

	dev_info(aess->dev, "AESS Firmware version %x\n", aess->fw_hdr.version);

	/* store AESS firmware for later context restore */
	aess->fw_data = fw_data + sizeof(struct fw_header);

	print_hex_dump(KERN_INFO, "fw pmem: ", DUMP_PREFIX_OFFSET, 16, 2, fw_data, 256, false);

	return 0;
}

static int aess_load_config(struct snd_soc_component *component,
			    struct snd_soc_tplg_hdr *hdr)
{
	struct omap_abe_aess *aess = snd_soc_component_get_drvdata(component);
	const void *fw_data = snd_soc_tplg_get_data(hdr);

	/* store AESS config for later context restore */
	dev_info(aess->dev, "AESS Config size %d bytes\n", hdr->payload_size);

	aess->fw_config = fw_data;

	return 0;
}

static int aess_load_coeffs(struct snd_soc_component *component,
			    struct snd_soc_tplg_hdr *hdr)
{
	const struct snd_soc_file_coeff_data *cd = snd_soc_tplg_get_data(hdr);

	dev_dbg(component->dev,"coeff %d size 0x%x with %d elems\n",
		cd->id, cd->size, cd->count);

	return 0;
}
/* callback to handle vendor data */
static int aess_vendor_load(struct snd_soc_component *component,
			    int index,
			    struct snd_soc_tplg_hdr *hdr)
{
	switch (hdr->type) {
	case SND_SOC_TPLG_TYPE_VENDOR_FW:
		return aess_load_fw(component, hdr);
	case SND_SOC_TPLG_TYPE_VENDOR_CONFIG:
		return aess_load_config(component, hdr);
	case SND_SOC_TPLG_TYPE_VENDOR_COEFF:
		return aess_load_coeffs(component, hdr);
	case SND_SOC_TPLG_TYPEVENDOR_CODEC:
	default:
		dev_err(component->dev, "vendor type %d:%d not supported\n",
			hdr->type, hdr->vendor_type);
		/* ignore errors */
		return 0;
	}
	return 0;
}

static int aess_fw_get_not_supported(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	printk("%s(%s)\n", __func__, kcontrol->id.name);
	return -ENOTSUPP;
}

static int aess_fw_put_dummy(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	printk("%s(%s)\n", __func__, kcontrol->id.name);
	return 0;
}

/* TODO: combine / sort */
#define OMAP_AESS_MIXER_DEFAULT	128
#define OMAP_AESS_MIXER_MONO	129
#define OMAP_AESS_MIXER_ROUTER	130
#define OMAP_AESS_MIXER_EQU	131
#define OMAP_AESS_MIXER_SWITCH	132
#define OMAP_AESS_MIXER_GAIN	133
#define OMAP_AESS_MIXER_VOLUME	134

static const struct snd_soc_tplg_kcontrol_ops omap_aess_fw_ops[] = {
{OMAP_AESS_MIXER_DEFAULT,	aess_fw_get_not_supported, aess_fw_put_dummy, NULL},
{OMAP_AESS_MIXER_MONO,		aess_fw_get_not_supported, aess_fw_put_dummy, NULL},
{OMAP_AESS_MIXER_ROUTER,	aess_fw_get_not_supported, aess_fw_put_dummy, NULL},
{OMAP_AESS_MIXER_EQU,		aess_fw_get_not_supported, aess_fw_put_dummy, NULL},
{OMAP_AESS_MIXER_SWITCH,	aess_fw_get_not_supported, aess_fw_put_dummy, NULL},
{OMAP_AESS_MIXER_GAIN,		aess_fw_get_not_supported, aess_fw_put_dummy, NULL},
{OMAP_AESS_MIXER_VOLUME,	aess_fw_get_not_supported, aess_fw_put_dummy, NULL},
};

static struct snd_soc_tplg_ops soc_tplg_ops = {
	.vendor_load	= aess_vendor_load,
	.io_ops		= omap_aess_fw_ops,
	.io_ops_count	= ARRAY_SIZE(omap_aess_fw_ops),
};

static int omap_aess_pcm_probe(struct snd_soc_component *component)
{
	struct omap_abe_aess *aess = container_of(component, struct omap_abe_aess, comp);
	int ret;

	snd_soc_component_set_drvdata(component, aess);

	dev_info(component->dev, "getting fw\n");

	ret = omap_aess_get_fw(aess);
	if (ret)
		return ret;

	pm_runtime_enable(aess->dev);
	pm_runtime_irq_safe(aess->dev);

	ret = snd_soc_tplg_component_load(component, &soc_tplg_ops, aess->fw);
	if (ret < 0) {
		dev_err(component->dev, "loading toplogy from AESS FW failed %d\n", ret);
		goto err_out;
	}

	dev_info(component->dev, "firmware loaded\n");

	/* aess_clk has to be enabled to access hal register.
	 * Disable the clk after it has been used.
	 */
	pm_runtime_get_sync(aess->dev);

	/* omap_aess_reset_hal() */
	omap_aess_hw_configuration(aess);

	/* omap_aess_load_fw() */
	omap_aess_load_fw_param(aess);

	/* the important bit */
	aess_dump_pmem(aess);

	pm_runtime_put_sync(aess->dev);

	return 0;
err_out:
	pm_runtime_disable(aess->dev);
	release_firmware(aess->fw);
	return ret;
}

static void omap_aess_pcm_remove(struct snd_soc_component *component)
{
	struct omap_abe_aess *aess = snd_soc_component_get_drvdata(component);
	int ret;

	ret = snd_soc_tplg_component_remove(component);
	if (ret)
		dev_warn(component->dev, "failed to remove topology\n");

	pm_runtime_disable(aess->dev);
	release_firmware(aess->fw);
	dev_info(component->dev, "component removed\n");
}

/* definitions copied from soc-topology-test.c */

/*
 * ASoC minimal boiler plate
 */
SND_SOC_DAILINK_DEF(dummy, DAILINK_COMP_ARRAY(COMP_DUMMY()));

SND_SOC_DAILINK_DEF(platform, DAILINK_COMP_ARRAY(COMP_PLATFORM("omap-aess-card")));

static struct snd_soc_dai_link omap_aess_dai_links[] = {
	{
		.name = "dump_pmem Audio Port",
		.id = 0,
		.stream_name = "Audio Playback/Capture",
		.nonatomic = 1,
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(dummy, dummy, platform),
	},
};

static const struct snd_soc_component_driver omap_aess_platform = {
	.name = "omap-aess",
	.probe = omap_aess_pcm_probe,
	.remove = omap_aess_pcm_remove,
};

static struct device_driver dump_pmem_card_drv = {
	.name = "omap-aess-card-driver",
};

static int omap_aess_register_card_and_component(struct omap_abe_aess *abe)
{
	int ret;

	/* setup a dummy card, register it, and then load topology from fw */

	abe->card_dev = root_device_register("omap-aess-card");
	abe->card_dev = get_device(abe->card_dev);
	if (!abe->card_dev)
		return -ENODEV;

	abe->card_dev->driver = &dump_pmem_card_drv;

	abe->card.dev = abe->card_dev;
	abe->card.name = "dummy";
	abe->card.owner = THIS_MODULE;
	abe->card.dai_link = omap_aess_dai_links;
	abe->card.num_links = ARRAY_SIZE(omap_aess_dai_links);
	abe->card.fully_routed = true;

	abe->card_registered = false;
	dev_info(abe->dev, "registering card\n");
	ret = snd_soc_register_card(&abe->card);
	if (ret == 0)
		abe->card_registered = true;
	else if (ret == -EPROBE_DEFER)
		dev_info(abe->dev, "card registration deferred\n");
	else {
		dev_err(abe->dev, "Failed to register card: %d\n", ret);
		goto out_err;
	}

	dev_info(abe->dev, "initializing component\n");
	ret = snd_soc_component_initialize(&abe->comp, &omap_aess_platform, abe->card_dev);
	if (ret != 0) {
		dev_err(abe->dev, "Failed to initialize component: %d\n", ret);
		snd_soc_unregister_card(&abe->card);
		goto out_err;
	}

	snd_soc_component_set_drvdata(&abe->comp, abe);

	dev_info(abe->dev, "adding component %s\n", abe->comp.name);
	ret = snd_soc_add_component(&abe->comp, NULL, 0);
	if (ret != 0) {
		dev_err(abe->dev, "Failed to add component: %d\n", ret);
		snd_soc_unregister_card(&abe->card);
		goto out_err;
	}

	dev_info(abe->dev, "Added snd component\n");

	if (!abe->card_registered) {
		dev_info(abe->dev, "retrying card register\n");
		ret = snd_soc_register_card(&abe->card);
		if (ret == 0)
			abe->card_registered = true;
		else {
			dev_warn(abe->dev, "card still not registered: %d\n", ret);
			goto out_err;
		}
	}

	return 0;
out_err:
	if (abe->card_registered)
		snd_soc_unregister_card(&abe->card);
	snd_soc_unregister_component(abe->card_dev);
	root_device_unregister(abe->card_dev);
	return ret;
}

static int abe_engine_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct omap_abe_aess *abe;
	int i;
	int ret;

	abe = devm_kzalloc(&pdev->dev, sizeof(struct omap_abe_aess), GFP_KERNEL);
	if (abe == NULL)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(abe_memory_bank); i++) {
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

	ret = omap_aess_register_card_and_component(abe);
	if (ret) {
		dev_info(&pdev->dev, "exiting with error %d\n", ret);
		return ret;
	}

	timer_setup(&abe->timer, aess_dump_pmem_timer_cb, 0);
	mod_timer(&abe->timer, jiffies + msecs_to_jiffies(2000));

	return 0;
}

static int abe_engine_remove(struct platform_device *pdev)
{
	struct omap_abe_aess *abe = dev_get_drvdata(&pdev->dev);

	del_timer(&abe->timer);

	if (abe->card_registered)
		snd_soc_unregister_card(&abe->card);
	snd_soc_unregister_component(abe->card_dev);
	root_device_unregister(abe->card_dev);
	snd_soc_unregister_component(&pdev->dev);

	dev_info(&pdev->dev, "removed\n");

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
