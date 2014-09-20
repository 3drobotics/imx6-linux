/*
 * Copyright 2011-2014 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/suspend.h>
#include <linux/genalloc.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <asm/cacheflush.h>
#include <asm/fncpy.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/tlb.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/map.h>

#include "common.h"
#include "hardware.h"

#define CCR				0x0
#define BM_CCR_WB_COUNT			(0x7 << 16)
#define BM_CCR_RBC_BYPASS_COUNT		(0x3f << 21)
#define BM_CCR_RBC_EN			(0x1 << 27)

#define CLPCR				0x54
#define BP_CLPCR_LPM			0
#define BM_CLPCR_LPM			(0x3 << 0)
#define BM_CLPCR_BYPASS_PMIC_READY	(0x1 << 2)
#define BM_CLPCR_ARM_CLK_DIS_ON_LPM	(0x1 << 5)
#define BM_CLPCR_SBYOS			(0x1 << 6)
#define BM_CLPCR_DIS_REF_OSC		(0x1 << 7)
#define BM_CLPCR_VSTBY			(0x1 << 8)
#define BP_CLPCR_STBY_COUNT		9
#define BM_CLPCR_STBY_COUNT		(0x3 << 9)
#define BM_CLPCR_COSC_PWRDOWN		(0x1 << 11)
#define BM_CLPCR_WB_PER_AT_LPM		(0x1 << 16)
#define BM_CLPCR_WB_CORE_AT_LPM		(0x1 << 17)
#define BM_CLPCR_BYP_MMDC_CH0_LPM_HS	(0x1 << 19)
#define BM_CLPCR_BYP_MMDC_CH1_LPM_HS	(0x1 << 21)
#define BM_CLPCR_MASK_CORE0_WFI		(0x1 << 22)
#define BM_CLPCR_MASK_CORE1_WFI		(0x1 << 23)
#define BM_CLPCR_MASK_CORE2_WFI		(0x1 << 24)
#define BM_CLPCR_MASK_CORE3_WFI		(0x1 << 25)
#define BM_CLPCR_MASK_SCU_IDLE		(0x1 << 26)
#define BM_CLPCR_MASK_L2CC_IDLE		(0x1 << 27)

#define CGPR				0x64
#define BM_CGPR_INT_MEM_CLK_LPM		(0x1 << 17)

#define MX6_INT_IOMUXC			32

unsigned long iram_tlb_base_addr;
unsigned long iram_tlb_phys_addr;

static void *suspend_iram_base;
static unsigned long iram_paddr;
static int (*suspend_in_iram_fn)(void *iram_vbase,
	unsigned long iram_pbase, unsigned int cpu_type);
static unsigned int cpu_type;
static void __iomem *ccm_base;
static unsigned long dcr;
static unsigned long pcr;

unsigned long save_ttbr1(void)
{
	unsigned long lttbr1;
	asm volatile(
		".align 4\n"
		"mrc p15, 0, %0, c2, c0, 1\n"
	: "=r" (lttbr1)
	);
	return lttbr1;
}

void restore_ttbr1(unsigned long ttbr1)
{
	asm volatile(
		".align 4\n"
		"mcr p15, 0, %0, c2, c0, 1\n"
	: : "r" (ttbr1)
	);
}

void imx6_set_cache_lpm_in_wait(bool enable)
{
	if ((cpu_is_imx6q() && imx_get_soc_revision() >
		IMX_CHIP_REVISION_1_1) ||
		(cpu_is_imx6dl() && imx_get_soc_revision() >
		IMX_CHIP_REVISION_1_0)) {
		u32 val;

		val = readl_relaxed(ccm_base + CGPR);
		if (enable)
			val |= BM_CGPR_INT_MEM_CLK_LPM;
		else
			val &= ~BM_CGPR_INT_MEM_CLK_LPM;
		writel_relaxed(val, ccm_base + CGPR);
	}
}

static void imx6_save_cpu_arch_regs(void)
{
	/* Save the Diagnostic Control Register. */
	asm volatile(
		"mrc p15, 0, %0, c15, c0, 1\n"
	: "=r" (dcr)
	);
	/* Save the Power Control Register. */
	asm volatile(
		"mrc p15, 0, %0, c15, c0, 0\n"
	: "=r" (pcr)
	);
}

static void imx6_restore_cpu_arch_regs(void)
{
	/* Restore the diagnostic Control Register. */
	asm volatile(
		"mcr p15, 0, %0, c15, c0, 1\n"
	: : "r" (dcr)
	);
	/* Restore the Power Control Register. */
	asm volatile(
		"mcr p15, 0, %0, c15, c0, 0\n"
	: : "r" (pcr)
	);
}

static void imx6_enable_rbc(bool enable)
{
	u32 val;

	/*
	 * need to mask all interrupts in GPC before
	 * operating RBC configurations
	 */
	imx_gpc_mask_all();

	/* configure RBC enable bit */
	val = readl_relaxed(ccm_base + CCR);
	val &= ~BM_CCR_RBC_EN;
	val |= enable ? BM_CCR_RBC_EN : 0;
	writel_relaxed(val, ccm_base + CCR);

	/* configure RBC count */
	val = readl_relaxed(ccm_base + CCR);
	val &= ~BM_CCR_RBC_BYPASS_COUNT;
	val |= enable ? BM_CCR_RBC_BYPASS_COUNT : 0;
	writel(val, ccm_base + CCR);

	/*
	 * need to delay at least 2 cycles of CKIL(32K)
	 * due to hardware design requirement, which is
	 * ~61us, here we use 65us for safe
	 */
	udelay(65);

	/* restore GPC interrupt mask settings */
	imx_gpc_restore_all();
}

static void imx6_enable_wb(bool enable)
{
	u32 val;

	/* configure well bias enable bit */
	val = readl_relaxed(ccm_base + CLPCR);
	val &= ~BM_CLPCR_WB_PER_AT_LPM;
	val |= enable ? BM_CLPCR_WB_PER_AT_LPM : 0;
	writel_relaxed(val, ccm_base + CLPCR);

	/* configure well bias count */
	val = readl_relaxed(ccm_base + CCR);
	val &= ~BM_CCR_WB_COUNT;
	val |= enable ? BM_CCR_WB_COUNT : 0;
	writel_relaxed(val, ccm_base + CCR);
}

int imx6_set_lpm(enum mxc_cpu_pwr_mode mode)
{
	u32 val = readl_relaxed(ccm_base + CLPCR);
	struct irq_desc *desc = irq_to_desc(MX6_INT_IOMUXC);

	/*
	 * CCM state machine has restriction, before enabling
	 * LPM mode, need to make sure last LPM mode is waked up
	 * by dsm_wakeup_signal, which means the wakeup source
	 * must be seen by GPC, then CCM will clean its state machine
	 * and re-sample necessary signal to decide whether it can
	 * enter LPM mode. We force irq #32 to be always pending,
	 * unmask it before we enable LPM mode and mask it after LPM
	 * is enabled, this flow will make sure CCM state machine in
	 * reliable status before entering LPM mode. Otherwise, CCM
	 * may enter LPM mode by mistake which will cause system bus
	 * locked by CPU access not finished, as when CCM enter
	 * LPM mode, CPU will stop running.
	 */
	imx_gpc_irq_unmask(&desc->irq_data);

	val &= ~BM_CLPCR_LPM;
	switch (mode) {
	case WAIT_CLOCKED:
		break;
	case WAIT_UNCLOCKED:
		val |= 0x1 << BP_CLPCR_LPM;
		val |= BM_CLPCR_ARM_CLK_DIS_ON_LPM;
		val &= ~BM_CLPCR_VSTBY;
		val &= ~BM_CLPCR_SBYOS;
		if (cpu_is_imx6sl())
			val |= BM_CLPCR_BYP_MMDC_CH0_LPM_HS;
		else
			val |= BM_CLPCR_BYP_MMDC_CH1_LPM_HS;
		break;
	case STOP_POWER_ON:
		val |= 0x2 << BP_CLPCR_LPM;
		val &= ~BM_CLPCR_VSTBY;
		val &= ~BM_CLPCR_SBYOS;
		if (cpu_is_imx6sl()) {
			val |= BM_CLPCR_BYPASS_PMIC_READY;
			val |= BM_CLPCR_BYP_MMDC_CH0_LPM_HS;
		} else {
			val |= BM_CLPCR_BYP_MMDC_CH1_LPM_HS;
		}
		break;
	case WAIT_UNCLOCKED_POWER_OFF:
		val |= 0x1 << BP_CLPCR_LPM;
		val &= ~BM_CLPCR_VSTBY;
		val &= ~BM_CLPCR_SBYOS;
		break;
	case STOP_POWER_OFF:
		val |= 0x2 << BP_CLPCR_LPM;
		val |= 0x3 << BP_CLPCR_STBY_COUNT;
		val |= BM_CLPCR_VSTBY;
		val |= BM_CLPCR_SBYOS;
		if (cpu_is_imx6sl()) {
			val |= BM_CLPCR_BYPASS_PMIC_READY;
			val |= BM_CLPCR_BYP_MMDC_CH0_LPM_HS;
		} else {
			val |= BM_CLPCR_BYP_MMDC_CH1_LPM_HS;
		}
		break;
	default:
		imx_gpc_irq_mask(&desc->irq_data);
		return -EINVAL;
	}

	writel_relaxed(val, ccm_base + CLPCR);
	imx_gpc_irq_mask(&desc->irq_data);

	return 0;
}

static int imx6_suspend_finish(unsigned long val)
{
	/*
	 * call low level suspend function in iram,
	 * as we need to float DDR IO.
	 */
	u32 ttbr1;

	ttbr1 = save_ttbr1();
	suspend_in_iram_fn(suspend_iram_base, iram_paddr, cpu_type);
	restore_ttbr1(ttbr1);
	return 0;
}

static int imx6_pm_enter(suspend_state_t state)
{
	struct regmap *g;

	/*
	 * L2 can exit by 'reset' or Inband beacon (from remote EP)
	 * toggling phy_powerdown has same effect as 'inband beacon'
	 * So, toggle bit18 of GPR1, used as a workaround of errata
	 * "PCIe PCIe does not support L2 Power Down"
	 */
	if (IS_ENABLED(CONFIG_PCI_IMX6)) {
		g = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
		if (IS_ERR(g)) {
			pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");
			return PTR_ERR(g);
		}
		regmap_update_bits(g, IOMUXC_GPR1, IMX6Q_GPR1_PCIE_TEST_PD,
				IMX6Q_GPR1_PCIE_TEST_PD);
	}

	switch (state) {
	case PM_SUSPEND_STANDBY:
		imx6_set_lpm(STOP_POWER_ON);
		imx6_set_cache_lpm_in_wait(true);
		imx_gpc_pre_suspend(false);
		if (cpu_is_imx6sl())
			imx6sl_set_wait_clk(true);
		/* Zzz ... */
		cpu_do_idle();
		if (cpu_is_imx6sl())
			imx6sl_set_wait_clk(false);
		imx_gpc_post_resume();
		imx6_set_lpm(WAIT_CLOCKED);
		break;
	case PM_SUSPEND_MEM:
		imx6_enable_wb(true);
		imx6_set_cache_lpm_in_wait(false);
		imx6_set_lpm(STOP_POWER_OFF);
		imx_gpc_pre_suspend(true);
		imx_anatop_pre_suspend();
		imx_set_cpu_jump(0, v7_cpu_resume);

		imx6_save_cpu_arch_regs();

		/* Zzz ... */
		cpu_suspend(0, imx6_suspend_finish);

		imx6_restore_cpu_arch_regs();

		if (!cpu_is_imx6sl())
			imx_smp_prepare();
		imx_anatop_post_resume();
		imx_gpc_post_resume();
		imx6_enable_rbc(false);
		imx6_enable_wb(false);
		imx6_set_cache_lpm_in_wait(true);
		imx6_set_lpm(WAIT_CLOCKED);
		break;
	default:
		return -EINVAL;
	}

	/*
	 * L2 can exit by 'reset' or Inband beacon (from remote EP)
	 * toggling phy_powerdown has same effect as 'inband beacon'
	 * So, toggle bit18 of GPR1, used as a workaround of errata
	 * "PCIe PCIe does not support L2 Power Down"
	 */
	if (IS_ENABLED(CONFIG_PCI_IMX6)) {
		regmap_update_bits(g, IOMUXC_GPR1, IMX6Q_GPR1_PCIE_TEST_PD,
				!IMX6Q_GPR1_PCIE_TEST_PD);
	}

	return 0;
}

static struct map_desc imx6_pm_io_desc[] __initdata = {
	imx_map_entry(MX6Q, MMDC_P0, MT_DEVICE),
	imx_map_entry(MX6Q, MMDC_P1, MT_DEVICE),
	imx_map_entry(MX6Q, SRC, MT_DEVICE),
	imx_map_entry(MX6Q, IOMUXC, MT_DEVICE),
	imx_map_entry(MX6Q, CCM, MT_DEVICE),
	imx_map_entry(MX6Q, ANATOP, MT_DEVICE),
	imx_map_entry(MX6Q, GPC, MT_DEVICE),
	imx_map_entry(MX6Q, L2, MT_DEVICE),
	imx_map_entry(MX6Q, IRAM_TLB, MT_MEMORY_NONCACHED),
};

void __init imx6_pm_map_io(void)
{
	unsigned long i;

	iotable_init(imx6_pm_io_desc, ARRAY_SIZE(imx6_pm_io_desc));

	/*
	 * Allocate IRAM for page tables to be used
	 * when DDR is in self-refresh.
	 */
	iram_tlb_phys_addr = MX6Q_IRAM_TLB_BASE_ADDR;
	iram_tlb_base_addr = IMX_IO_P2V(MX6Q_IRAM_TLB_BASE_ADDR);

	/* Set all entries to 0. */
	memset((void *)iram_tlb_base_addr, 0, SZ_16K);

	/*
	 * Make sure the IRAM virtual address has a mapping
	 * in the IRAM page table.
	 */
	i = (IMX_IO_P2V(MX6Q_IRAM_TLB_BASE_ADDR) >> 18) / 4;
	*((unsigned long *)iram_tlb_base_addr + i) =
		MX6Q_IRAM_TLB_BASE_ADDR | TT_ATTRIB_NON_CACHEABLE_1M;
	/*
	 * Make sure the AIPS1 virtual address has a mapping
	 * in the IRAM page table.
	 */
	i = (IMX_IO_P2V(MX6Q_AIPS1_BASE_ADDR) >> 18) / 4;
	*((unsigned long *)iram_tlb_base_addr + i) =
		MX6Q_AIPS1_BASE_ADDR | TT_ATTRIB_NON_CACHEABLE_1M;
	/*
	* Make sure the AIPS2 virtual address has a mapping
	* in the IRAM page table.
	*/
	i = (IMX_IO_P2V(MX6Q_AIPS2_BASE_ADDR) >> 18) / 4;
	*((unsigned long *)iram_tlb_base_addr + i) =
		MX6Q_AIPS2_BASE_ADDR | TT_ATTRIB_NON_CACHEABLE_1M;
	/*
	 * Make sure the AIPS2 virtual address has a mapping
	 * in the IRAM page table.
	 */
	i = (IMX_IO_P2V(MX6Q_L2_BASE_ADDR) >> 18) / 4;
	*((unsigned long *)iram_tlb_base_addr + i) =
		MX6Q_L2_BASE_ADDR | TT_ATTRIB_NON_CACHEABLE_1M;

}

static int imx6_pm_valid(suspend_state_t state)
{
	return (state == PM_SUSPEND_STANDBY || state == PM_SUSPEND_MEM);
}

static const struct platform_suspend_ops imx6_pm_ops = {
	.enter = imx6_pm_enter,
	.valid = imx6_pm_valid,
};

void imx6_pm_set_ccm_base(void __iomem *base)
{
	if (!base)
		pr_warn("ccm base is NULL!\n");
	ccm_base = base;
}

void __init imx6_pm_init(void)
{
	iram_paddr = MX6_SUSPEND_IRAM_ADDR;
	/* Get the virtual address of the suspend code. */
	suspend_iram_base = (void *)IMX_IO_P2V(MX6Q_IRAM_TLB_BASE_ADDR) +
			(iram_paddr - MX6Q_IRAM_TLB_BASE_ADDR);

	suspend_in_iram_fn = (void *)fncpy(suspend_iram_base,
		&imx6_suspend, MX6_SUSPEND_IRAM_SIZE);

	suspend_set_ops(&imx6_pm_ops);

	/* Set cpu_type for DSM */
	if (cpu_is_imx6q())
		cpu_type = MXC_CPU_IMX6Q;
	else if (cpu_is_imx6dl())
		cpu_type = MXC_CPU_IMX6DL;
	else
		cpu_type = MXC_CPU_IMX6SL;
}
