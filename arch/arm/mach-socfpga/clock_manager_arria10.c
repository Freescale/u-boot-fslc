// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016-2017 Intel Corporation
 */

#include <common.h>
#include <fdtdec.h>
#include <asm/io.h>
#include <dm.h>
#include <asm/arch/clock_manager.h>

static u32 eosc1_hz;
static u32 cb_intosc_hz;
static u32 f2s_free_hz;
static u32 cm_l4_main_clk_hz;
static u32 cm_l4_sp_clk_hz;
static u32 cm_l4_mp_clk_hz;
static u32 cm_l4_sys_free_clk_hz;

struct mainpll_cfg {
	u32 vco0_psrc;
	u32 vco1_denom;
	u32 vco1_numer;
	u32 mpuclk;
	u32 mpuclk_cnt;
	u32 mpuclk_src;
	u32 nocclk;
	u32 nocclk_cnt;
	u32 nocclk_src;
	u32 cntr2clk_cnt;
	u32 cntr3clk_cnt;
	u32 cntr4clk_cnt;
	u32 cntr5clk_cnt;
	u32 cntr6clk_cnt;
	u32 cntr7clk_cnt;
	u32 cntr7clk_src;
	u32 cntr8clk_cnt;
	u32 cntr9clk_cnt;
	u32 cntr9clk_src;
	u32 cntr15clk_cnt;
	u32 nocdiv_l4mainclk;
	u32 nocdiv_l4mpclk;
	u32 nocdiv_l4spclk;
	u32 nocdiv_csatclk;
	u32 nocdiv_cstraceclk;
	u32 nocdiv_cspdbclk;
};

struct perpll_cfg {
	u32 vco0_psrc;
	u32 vco1_denom;
	u32 vco1_numer;
	u32 cntr2clk_cnt;
	u32 cntr2clk_src;
	u32 cntr3clk_cnt;
	u32 cntr3clk_src;
	u32 cntr4clk_cnt;
	u32 cntr4clk_src;
	u32 cntr5clk_cnt;
	u32 cntr5clk_src;
	u32 cntr6clk_cnt;
	u32 cntr6clk_src;
	u32 cntr7clk_cnt;
	u32 cntr8clk_cnt;
	u32 cntr8clk_src;
	u32 cntr9clk_cnt;
	u32 emacctl_emac0sel;
	u32 emacctl_emac1sel;
	u32 emacctl_emac2sel;
	u32 gpiodiv_gpiodbclk;
};

struct alteragrp_cfg {
	u32 nocclk;
	u32 mpuclk;
};

static const struct socfpga_clock_manager *clock_manager_base =
	(struct socfpga_clock_manager *)SOCFPGA_CLKMGR_ADDRESS;

static int of_to_struct(const void *blob, int node, int cfg_len, void *cfg)
{
	if (fdtdec_get_int_array(blob, node, "altr,of_reg_value",
				 (u32 *)cfg, cfg_len)) {
		/* could not find required property */
		return -EINVAL;
	}

	return 0;
}

static int of_get_input_clks(const void *blob, int node, u32 *val)
{
	*val = fdtdec_get_uint(blob, node, "clock-frequency", 0);
	if (!*val)
		return -EINVAL;

	return 0;
}

static int of_get_clk_cfg(const void *blob, struct mainpll_cfg *main_cfg,
			  struct perpll_cfg *per_cfg,
			  struct alteragrp_cfg *altrgrp_cfg)
{
	int node, child, len;
	const char *node_name;

	node = fdtdec_next_compatible(blob, 0, COMPAT_ALTERA_SOCFPGA_CLK);
	if (node < 0)
		return -EINVAL;

	child = fdt_first_subnode(blob, node);
	if (child < 0)
		return -EINVAL;

	child = fdt_first_subnode(blob, child);
	if (child < 0)
		return -EINVAL;

	node_name = fdt_get_name(blob, child, &len);

	while (node_name) {
		if (!strcmp(node_name, "osc1")) {
			if (of_get_input_clks(blob, child, &eosc1_hz))
				return -EINVAL;
		} else if (!strcmp(node_name, "cb_intosc_ls_clk")) {
			if (of_get_input_clks(blob, child, &cb_intosc_hz))
				return -EINVAL;
		} else if (!strcmp(node_name, "f2s_free_clk")) {
			if (of_get_input_clks(blob, child, &f2s_free_hz))
				return -EINVAL;
		} else if (!strcmp(node_name, "main_pll")) {
			if (of_to_struct(blob, child,
					 sizeof(*main_cfg)/sizeof(u32),
					 main_cfg))
				return -EINVAL;
		} else if (!strcmp(node_name, "periph_pll")) {
			if (of_to_struct(blob, child,
					 sizeof(*per_cfg)/sizeof(u32),
					 per_cfg))
				return -EINVAL;
		} else if (!strcmp(node_name, "altera")) {
			if (of_to_struct(blob, child,
					 sizeof(*altrgrp_cfg)/sizeof(u32),
					 altrgrp_cfg))
				return -EINVAL;

			main_cfg->mpuclk = altrgrp_cfg->mpuclk;
			main_cfg->nocclk = altrgrp_cfg->nocclk;
		}
		child = fdt_next_subnode(blob, child);

		if (child < 0)
			break;

		node_name = fdt_get_name(blob, child, &len);
	}

	return 0;
}

/* calculate the intended main VCO frequency based on handoff */
static unsigned int cm_calc_handoff_main_vco_clk_hz
					(struct mainpll_cfg *main_cfg)
{
	unsigned int clk_hz;

	/* Check main VCO clock source: eosc, intosc or f2s? */
	switch (main_cfg->vco0_psrc) {
	case CLKMGR_MAINPLL_VCO0_PSRC_EOSC:
		clk_hz = eosc1_hz;
		break;
	case CLKMGR_MAINPLL_VCO0_PSRC_E_INTOSC:
		clk_hz = cb_intosc_hz;
		break;
	case CLKMGR_MAINPLL_VCO0_PSRC_F2S:
		clk_hz = f2s_free_hz;
		break;
	default:
		return 0;
	}

	/* calculate the VCO frequency */
	clk_hz /= 1 + main_cfg->vco1_denom;
	clk_hz *= 1 + main_cfg->vco1_numer;

	return clk_hz;
}

/* calculate the intended periph VCO frequency based on handoff */
static unsigned int cm_calc_handoff_periph_vco_clk_hz(
		struct mainpll_cfg *main_cfg, struct perpll_cfg *per_cfg)
{
	unsigned int clk_hz;

	/* Check periph VCO clock source: eosc, intosc, f2s or mainpll? */
	switch (per_cfg->vco0_psrc) {
	case CLKMGR_PERPLL_VCO0_PSRC_EOSC:
		clk_hz = eosc1_hz;
		break;
	case CLKMGR_PERPLL_VCO0_PSRC_E_INTOSC:
		clk_hz = cb_intosc_hz;
		break;
	case CLKMGR_PERPLL_VCO0_PSRC_F2S:
		clk_hz = f2s_free_hz;
		break;
	case CLKMGR_PERPLL_VCO0_PSRC_MAIN:
		clk_hz = cm_calc_handoff_main_vco_clk_hz(main_cfg);
		clk_hz /= main_cfg->cntr15clk_cnt;
		break;
	default:
		return 0;
	}

	/* calculate the VCO frequency */
	clk_hz /= 1 + per_cfg->vco1_denom;
	clk_hz *= 1 + per_cfg->vco1_numer;

	return clk_hz;
}

/* calculate the intended MPU clock frequency based on handoff */
static unsigned int cm_calc_handoff_mpu_clk_hz(struct mainpll_cfg *main_cfg,
					       struct perpll_cfg *per_cfg)
{
	unsigned int clk_hz;

	/* Check MPU clock source: main, periph, osc1, intosc or f2s? */
	switch (main_cfg->mpuclk_src) {
	case CLKMGR_MAINPLL_MPUCLK_SRC_MAIN:
		clk_hz = cm_calc_handoff_main_vco_clk_hz(main_cfg);
		clk_hz /= (main_cfg->mpuclk & CLKMGR_MAINPLL_MPUCLK_CNT_MSK)
			   + 1;
		break;
	case CLKMGR_MAINPLL_MPUCLK_SRC_PERI:
		clk_hz = cm_calc_handoff_periph_vco_clk_hz(main_cfg, per_cfg);
		clk_hz /= ((main_cfg->mpuclk >>
			   CLKMGR_MAINPLL_MPUCLK_PERICNT_LSB) &
			   CLKMGR_MAINPLL_MPUCLK_CNT_MSK) + 1;
		break;
	case CLKMGR_MAINPLL_MPUCLK_SRC_OSC1:
		clk_hz = eosc1_hz;
		break;
	case CLKMGR_MAINPLL_MPUCLK_SRC_INTOSC:
		clk_hz = cb_intosc_hz;
		break;
	case CLKMGR_MAINPLL_MPUCLK_SRC_FPGA:
		clk_hz = f2s_free_hz;
		break;
	default:
		return 0;
	}

	clk_hz /= main_cfg->mpuclk_cnt + 1;
	return clk_hz;
}

/* calculate the intended NOC clock frequency based on handoff */
static unsigned int cm_calc_handoff_noc_clk_hz(struct mainpll_cfg *main_cfg,
					       struct perpll_cfg *per_cfg)
{
	unsigned int clk_hz;

	/* Check MPU clock source: main, periph, osc1, intosc or f2s? */
	switch (main_cfg->nocclk_src) {
	case CLKMGR_MAINPLL_NOCCLK_SRC_MAIN:
		clk_hz = cm_calc_handoff_main_vco_clk_hz(main_cfg);
		clk_hz /= (main_cfg->nocclk & CLKMGR_MAINPLL_NOCCLK_CNT_MSK)
			 + 1;
		break;
	case CLKMGR_MAINPLL_NOCCLK_SRC_PERI:
		clk_hz = cm_calc_handoff_periph_vco_clk_hz(main_cfg, per_cfg);
		clk_hz /= ((main_cfg->nocclk >>
			   CLKMGR_MAINPLL_NOCCLK_PERICNT_LSB) &
			   CLKMGR_MAINPLL_NOCCLK_CNT_MSK) + 1;
		break;
	case CLKMGR_MAINPLL_NOCCLK_SRC_OSC1:
		clk_hz = eosc1_hz;
		break;
	case CLKMGR_MAINPLL_NOCCLK_SRC_INTOSC:
		clk_hz = cb_intosc_hz;
		break;
	case CLKMGR_MAINPLL_NOCCLK_SRC_FPGA:
		clk_hz = f2s_free_hz;
		break;
	default:
		return 0;
	}

	clk_hz /= main_cfg->nocclk_cnt + 1;
	return clk_hz;
}

/* return 1 if PLL ramp is required */
static int cm_is_pll_ramp_required(int main0periph1,
				   struct mainpll_cfg *main_cfg,
				   struct perpll_cfg *per_cfg)
{
	/* Check for main PLL */
	if (main0periph1 == 0) {
		/*
		 * PLL ramp is not required if both MPU clock and NOC clock are
		 * not sourced from main PLL
		 */
		if (main_cfg->mpuclk_src != CLKMGR_MAINPLL_MPUCLK_SRC_MAIN &&
		    main_cfg->nocclk_src != CLKMGR_MAINPLL_NOCCLK_SRC_MAIN)
			return 0;

		/*
		 * PLL ramp is required if MPU clock is sourced from main PLL
		 * and MPU clock is over 900MHz (as advised by HW team)
		 */
		if (main_cfg->mpuclk_src == CLKMGR_MAINPLL_MPUCLK_SRC_MAIN &&
		    (cm_calc_handoff_mpu_clk_hz(main_cfg, per_cfg) >
		     CLKMGR_PLL_RAMP_MPUCLK_THRESHOLD_HZ))
			return 1;

		/*
		 * PLL ramp is required if NOC clock is sourced from main PLL
		 * and NOC clock is over 300MHz (as advised by HW team)
		 */
		if (main_cfg->nocclk_src == CLKMGR_MAINPLL_NOCCLK_SRC_MAIN &&
		    (cm_calc_handoff_noc_clk_hz(main_cfg, per_cfg) >
		     CLKMGR_PLL_RAMP_NOCCLK_THRESHOLD_HZ))
			return 2;

	} else if (main0periph1 == 1) {
		/*
		 * PLL ramp is not required if both MPU clock and NOC clock are
		 * not sourced from periph PLL
		 */
		if (main_cfg->mpuclk_src != CLKMGR_MAINPLL_MPUCLK_SRC_PERI &&
		    main_cfg->nocclk_src != CLKMGR_MAINPLL_NOCCLK_SRC_PERI)
			return 0;

		/*
		 * PLL ramp is required if MPU clock are source from periph PLL
		 * and MPU clock is over 900MHz (as advised by HW team)
		 */
		if (main_cfg->mpuclk_src == CLKMGR_MAINPLL_MPUCLK_SRC_PERI &&
		    (cm_calc_handoff_mpu_clk_hz(main_cfg, per_cfg) >
		     CLKMGR_PLL_RAMP_MPUCLK_THRESHOLD_HZ))
			return 1;

		/*
		 * PLL ramp is required if NOC clock are source from periph PLL
		 * and NOC clock is over 300MHz (as advised by HW team)
		 */
		if (main_cfg->nocclk_src == CLKMGR_MAINPLL_NOCCLK_SRC_PERI &&
		    (cm_calc_handoff_noc_clk_hz(main_cfg, per_cfg) >
		     CLKMGR_PLL_RAMP_NOCCLK_THRESHOLD_HZ))
			return 2;
	}

	return 0;
}

static u32 cm_calculate_numer(struct mainpll_cfg *main_cfg,
			      struct perpll_cfg *per_cfg,
			      u32 safe_hz, u32 clk_hz)
{
	u32 cnt;
	u32 clk;
	u32 shift;
	u32 mask;
	u32 denom;

	if (main_cfg->mpuclk_src == CLKMGR_MAINPLL_MPUCLK_SRC_MAIN) {
		cnt = main_cfg->mpuclk_cnt;
		clk = main_cfg->mpuclk;
		shift = 0;
		mask = CLKMGR_MAINPLL_MPUCLK_CNT_MSK;
		denom = main_cfg->vco1_denom;
	} else if (main_cfg->nocclk_src == CLKMGR_MAINPLL_NOCCLK_SRC_MAIN) {
		cnt = main_cfg->nocclk_cnt;
		clk = main_cfg->nocclk;
		shift = 0;
		mask = CLKMGR_MAINPLL_NOCCLK_CNT_MSK;
		denom = main_cfg->vco1_denom;
	} else if (main_cfg->mpuclk_src == CLKMGR_MAINPLL_MPUCLK_SRC_PERI) {
		cnt = main_cfg->mpuclk_cnt;
		clk = main_cfg->mpuclk;
		shift = CLKMGR_MAINPLL_MPUCLK_PERICNT_LSB;
		mask = CLKMGR_MAINPLL_MPUCLK_CNT_MSK;
		denom = per_cfg->vco1_denom;
	} else if (main_cfg->nocclk_src == CLKMGR_MAINPLL_NOCCLK_SRC_PERI) {
		cnt = main_cfg->nocclk_cnt;
		clk = main_cfg->nocclk;
		shift = CLKMGR_MAINPLL_NOCCLK_PERICNT_LSB;
		mask = CLKMGR_MAINPLL_NOCCLK_CNT_MSK;
		denom = per_cfg->vco1_denom;
	} else {
		return 0;
	}

	return (safe_hz / clk_hz) * (cnt + 1) * (((clk >> shift) & mask) + 1) *
		(1 + denom) - 1;
}

/*
 * Calculate the new PLL numerator which is based on existing DTS hand off and
 * intended safe frequency (safe_hz). Note that PLL ramp is only modifying the
 * numerator while maintaining denominator as denominator will influence the
 * jitter condition. Please refer A10 HPS TRM for the jitter guide. Note final
 * value for numerator is minus with 1 to cater our register value
 * representation.
 */
static unsigned int cm_calc_safe_pll_numer(int main0periph1,
					   struct mainpll_cfg *main_cfg,
					   struct perpll_cfg *per_cfg,
					   unsigned int safe_hz)
{
	unsigned int clk_hz = 0;

	/* Check for main PLL */
	if (main0periph1 == 0) {
		/* Check main VCO clock source: eosc, intosc or f2s? */
		switch (main_cfg->vco0_psrc) {
		case CLKMGR_MAINPLL_VCO0_PSRC_EOSC:
			clk_hz = eosc1_hz;
			break;
		case CLKMGR_MAINPLL_VCO0_PSRC_E_INTOSC:
			clk_hz = cb_intosc_hz;
			break;
		case CLKMGR_MAINPLL_VCO0_PSRC_F2S:
			clk_hz = f2s_free_hz;
			break;
		default:
			return 0;
		}
	} else if (main0periph1 == 1) {
		/* Check periph VCO clock source: eosc, intosc, f2s, mainpll */
		switch (per_cfg->vco0_psrc) {
		case CLKMGR_PERPLL_VCO0_PSRC_EOSC:
			clk_hz = eosc1_hz;
			break;
		case CLKMGR_PERPLL_VCO0_PSRC_E_INTOSC:
			clk_hz = cb_intosc_hz;
			break;
		case CLKMGR_PERPLL_VCO0_PSRC_F2S:
			clk_hz = f2s_free_hz;
			break;
		case CLKMGR_PERPLL_VCO0_PSRC_MAIN:
			clk_hz = cm_calc_handoff_main_vco_clk_hz(main_cfg);
			clk_hz /= main_cfg->cntr15clk_cnt;
			break;
		default:
			return 0;
		}
	} else {
		return 0;
	}

	return cm_calculate_numer(main_cfg, per_cfg, safe_hz, clk_hz);
}

/* ramping the main PLL to final value */
static void cm_pll_ramp_main(struct mainpll_cfg *main_cfg,
			     struct perpll_cfg *per_cfg,
			     unsigned int pll_ramp_main_hz)
{
	unsigned int clk_hz = 0, clk_incr_hz = 0, clk_final_hz = 0;

	/* find out the increment value */
	if (main_cfg->mpuclk_src == CLKMGR_MAINPLL_MPUCLK_SRC_MAIN) {
		clk_incr_hz = CLKMGR_PLL_RAMP_MPUCLK_INCREMENT_HZ;
		clk_final_hz = cm_calc_handoff_mpu_clk_hz(main_cfg, per_cfg);
	} else if (main_cfg->nocclk_src == CLKMGR_MAINPLL_NOCCLK_SRC_MAIN) {
		clk_incr_hz = CLKMGR_PLL_RAMP_NOCCLK_INCREMENT_HZ;
		clk_final_hz = cm_calc_handoff_noc_clk_hz(main_cfg, per_cfg);
	}

	/* execute the ramping here */
	for (clk_hz = pll_ramp_main_hz + clk_incr_hz;
	     clk_hz < clk_final_hz; clk_hz += clk_incr_hz) {
		writel((main_cfg->vco1_denom <<
			CLKMGR_MAINPLL_VCO1_DENOM_LSB) |
			cm_calc_safe_pll_numer(0, main_cfg, per_cfg, clk_hz),
			&clock_manager_base->main_pll.vco1);
		mdelay(1);
		cm_wait_for_lock(LOCKED_MASK);
	}
	writel((main_cfg->vco1_denom << CLKMGR_MAINPLL_VCO1_DENOM_LSB) |
		main_cfg->vco1_numer, &clock_manager_base->main_pll.vco1);
	mdelay(1);
	cm_wait_for_lock(LOCKED_MASK);
}

/* ramping the periph PLL to final value */
static void cm_pll_ramp_periph(struct mainpll_cfg *main_cfg,
			       struct perpll_cfg *per_cfg,
			       unsigned int pll_ramp_periph_hz)
{
	unsigned int clk_hz = 0, clk_incr_hz = 0, clk_final_hz = 0;

	/* find out the increment value */
	if (main_cfg->mpuclk_src == CLKMGR_MAINPLL_MPUCLK_SRC_PERI) {
		clk_incr_hz = CLKMGR_PLL_RAMP_MPUCLK_INCREMENT_HZ;
		clk_final_hz = cm_calc_handoff_mpu_clk_hz(main_cfg, per_cfg);
	} else if (main_cfg->nocclk_src == CLKMGR_MAINPLL_NOCCLK_SRC_PERI) {
		clk_incr_hz = CLKMGR_PLL_RAMP_NOCCLK_INCREMENT_HZ;
		clk_final_hz = cm_calc_handoff_noc_clk_hz(main_cfg, per_cfg);
	}
	/* execute the ramping here */
	for (clk_hz = pll_ramp_periph_hz + clk_incr_hz;
	     clk_hz < clk_final_hz; clk_hz += clk_incr_hz) {
		writel((per_cfg->vco1_denom << CLKMGR_PERPLL_VCO1_DENOM_LSB) |
			cm_calc_safe_pll_numer(1, main_cfg, per_cfg, clk_hz),
			&clock_manager_base->per_pll.vco1);
		mdelay(1);
		cm_wait_for_lock(LOCKED_MASK);
	}
	writel((per_cfg->vco1_denom << CLKMGR_PERPLL_VCO1_DENOM_LSB) |
		per_cfg->vco1_numer, &clock_manager_base->per_pll.vco1);
	mdelay(1);
	cm_wait_for_lock(LOCKED_MASK);
}

/*
 * Setup clocks while making no assumptions of the
 * previous state of the clocks.
 *
 * Start by being paranoid and gate all sw managed clocks
 *
 * Put all plls in bypass
 *
 * Put all plls VCO registers back to reset value (bgpwr dwn).
 *
 * Put peripheral and main pll src to reset value to avoid glitch.
 *
 * Delay 5 us.
 *
 * Deassert bg pwr dn and set numerator and denominator
 *
 * Start 7 us timer.
 *
 * set internal dividers
 *
 * Wait for 7 us timer.
 *
 * Enable plls
 *
 * Set external dividers while plls are locking
 *
 * Wait for pll lock
 *
 * Assert/deassert outreset all.
 *
 * Take all pll's out of bypass
 *
 * Clear safe mode
 *
 * set source main and peripheral clocks
 *
 * Ungate clocks
 */

static int cm_full_cfg(struct mainpll_cfg *main_cfg, struct perpll_cfg *per_cfg)
{
	unsigned int pll_ramp_main_hz = 0, pll_ramp_periph_hz = 0,
		ramp_required;

	/* gate off all mainpll clock excpet HW managed clock */
	writel(CLKMGR_MAINPLL_EN_S2FUSER0CLKEN_SET_MSK |
		CLKMGR_MAINPLL_EN_HMCPLLREFCLKEN_SET_MSK,
		&clock_manager_base->main_pll.enr);

	/* now we can gate off the rest of the peripheral clocks */
	writel(0, &clock_manager_base->per_pll.en);

	/* Put all plls in external bypass */
	writel(CLKMGR_MAINPLL_BYPASS_RESET,
	       &clock_manager_base->main_pll.bypasss);
	writel(CLKMGR_PERPLL_BYPASS_RESET,
	       &clock_manager_base->per_pll.bypasss);

	/*
	 * Put all plls VCO registers back to reset value.
	 * Some code might have messed with them. At same time set the
	 * desired clock source
	 */
	writel(CLKMGR_MAINPLL_VCO0_RESET |
	       CLKMGR_MAINPLL_VCO0_REGEXTSEL_SET_MSK |
	       (main_cfg->vco0_psrc << CLKMGR_MAINPLL_VCO0_PSRC_LSB),
	       &clock_manager_base->main_pll.vco0);

	writel(CLKMGR_PERPLL_VCO0_RESET |
	       CLKMGR_PERPLL_VCO0_REGEXTSEL_SET_MSK |
	       (per_cfg->vco0_psrc << CLKMGR_PERPLL_VCO0_PSRC_LSB),
	       &clock_manager_base->per_pll.vco0);

	writel(CLKMGR_MAINPLL_VCO1_RESET, &clock_manager_base->main_pll.vco1);
	writel(CLKMGR_PERPLL_VCO1_RESET, &clock_manager_base->per_pll.vco1);

	/* clear the interrupt register status register */
	writel(CLKMGR_CLKMGR_INTR_MAINPLLLOST_SET_MSK |
		CLKMGR_CLKMGR_INTR_PERPLLLOST_SET_MSK |
		CLKMGR_CLKMGR_INTR_MAINPLLRFSLIP_SET_MSK |
		CLKMGR_CLKMGR_INTR_PERPLLRFSLIP_SET_MSK |
		CLKMGR_CLKMGR_INTR_MAINPLLFBSLIP_SET_MSK |
		CLKMGR_CLKMGR_INTR_PERPLLFBSLIP_SET_MSK |
		CLKMGR_CLKMGR_INTR_MAINPLLACHIEVED_SET_MSK |
		CLKMGR_CLKMGR_INTR_PERPLLACHIEVED_SET_MSK,
		&clock_manager_base->intr);

	/* Program VCO Numerator and Denominator for main PLL */
	ramp_required = cm_is_pll_ramp_required(0, main_cfg, per_cfg);
	if (ramp_required) {
		/* set main PLL to safe starting threshold frequency */
		if (ramp_required == 1)
			pll_ramp_main_hz = CLKMGR_PLL_RAMP_MPUCLK_THRESHOLD_HZ;
		else if (ramp_required == 2)
			pll_ramp_main_hz = CLKMGR_PLL_RAMP_NOCCLK_THRESHOLD_HZ;

		writel((main_cfg->vco1_denom << CLKMGR_MAINPLL_VCO1_DENOM_LSB) |
			cm_calc_safe_pll_numer(0, main_cfg, per_cfg,
					       pll_ramp_main_hz),
			&clock_manager_base->main_pll.vco1);
	} else
		writel((main_cfg->vco1_denom << CLKMGR_MAINPLL_VCO1_DENOM_LSB) |
			main_cfg->vco1_numer,
			&clock_manager_base->main_pll.vco1);

	/* Program VCO Numerator and Denominator for periph PLL */
	ramp_required = cm_is_pll_ramp_required(1, main_cfg, per_cfg);
	if (ramp_required) {
		/* set periph PLL to safe starting threshold frequency */
		if (ramp_required == 1)
			pll_ramp_periph_hz =
				CLKMGR_PLL_RAMP_MPUCLK_THRESHOLD_HZ;
		else if (ramp_required == 2)
			pll_ramp_periph_hz =
				CLKMGR_PLL_RAMP_NOCCLK_THRESHOLD_HZ;

		writel((per_cfg->vco1_denom << CLKMGR_PERPLL_VCO1_DENOM_LSB) |
			cm_calc_safe_pll_numer(1, main_cfg, per_cfg,
					       pll_ramp_periph_hz),
			&clock_manager_base->per_pll.vco1);
	} else
		writel((per_cfg->vco1_denom << CLKMGR_PERPLL_VCO1_DENOM_LSB) |
			per_cfg->vco1_numer,
			&clock_manager_base->per_pll.vco1);

	/* Wait for at least 5 us */
	udelay(5);

	/* Now deassert BGPWRDN and PWRDN */
	clrbits_le32(&clock_manager_base->main_pll.vco0,
		     CLKMGR_MAINPLL_VCO0_BGPWRDN_SET_MSK |
		     CLKMGR_MAINPLL_VCO0_PWRDN_SET_MSK);
	clrbits_le32(&clock_manager_base->per_pll.vco0,
		     CLKMGR_PERPLL_VCO0_BGPWRDN_SET_MSK |
		     CLKMGR_PERPLL_VCO0_PWRDN_SET_MSK);

	/* Wait for at least 7 us */
	udelay(7);

	/* enable the VCO and disable the external regulator to PLL */
	writel((readl(&clock_manager_base->main_pll.vco0) &
		~CLKMGR_MAINPLL_VCO0_REGEXTSEL_SET_MSK) |
		CLKMGR_MAINPLL_VCO0_EN_SET_MSK,
		&clock_manager_base->main_pll.vco0);
	writel((readl(&clock_manager_base->per_pll.vco0) &
		~CLKMGR_PERPLL_VCO0_REGEXTSEL_SET_MSK) |
		CLKMGR_PERPLL_VCO0_EN_SET_MSK,
		&clock_manager_base->per_pll.vco0);

	/* setup all the main PLL counter and clock source */
	writel(main_cfg->nocclk,
	       SOCFPGA_CLKMGR_ADDRESS + CLKMGR_MAINPLL_NOC_CLK_OFFSET);
	writel(main_cfg->mpuclk,
	       SOCFPGA_CLKMGR_ADDRESS + CLKMGR_ALTERAGRP_MPU_CLK_OFFSET);

	/* main_emaca_clk divider */
	writel(main_cfg->cntr2clk_cnt, &clock_manager_base->main_pll.cntr2clk);
	/* main_emacb_clk divider */
	writel(main_cfg->cntr3clk_cnt, &clock_manager_base->main_pll.cntr3clk);
	/* main_emac_ptp_clk divider */
	writel(main_cfg->cntr4clk_cnt, &clock_manager_base->main_pll.cntr4clk);
	/* main_gpio_db_clk divider */
	writel(main_cfg->cntr5clk_cnt, &clock_manager_base->main_pll.cntr5clk);
	/* main_sdmmc_clk divider */
	writel(main_cfg->cntr6clk_cnt, &clock_manager_base->main_pll.cntr6clk);
	/* main_s2f_user0_clk divider */
	writel(main_cfg->cntr7clk_cnt |
	       (main_cfg->cntr7clk_src << CLKMGR_MAINPLL_CNTR7CLK_SRC_LSB),
	       &clock_manager_base->main_pll.cntr7clk);
	/* main_s2f_user1_clk divider */
	writel(main_cfg->cntr8clk_cnt, &clock_manager_base->main_pll.cntr8clk);
	/* main_hmc_pll_clk divider */
	writel(main_cfg->cntr9clk_cnt |
	       (main_cfg->cntr9clk_src << CLKMGR_MAINPLL_CNTR9CLK_SRC_LSB),
	       &clock_manager_base->main_pll.cntr9clk);
	/* main_periph_ref_clk divider */
	writel(main_cfg->cntr15clk_cnt,
	       &clock_manager_base->main_pll.cntr15clk);

	/* setup all the peripheral PLL counter and clock source */
	/* peri_emaca_clk divider */
	writel(per_cfg->cntr2clk_cnt |
	       (per_cfg->cntr2clk_src << CLKMGR_PERPLL_CNTR2CLK_SRC_LSB),
	       &clock_manager_base->per_pll.cntr2clk);
	/* peri_emacb_clk divider */
	writel(per_cfg->cntr3clk_cnt |
	       (per_cfg->cntr3clk_src << CLKMGR_PERPLL_CNTR3CLK_SRC_LSB),
	       &clock_manager_base->per_pll.cntr3clk);
	/* peri_emac_ptp_clk divider */
	writel(per_cfg->cntr4clk_cnt |
	       (per_cfg->cntr4clk_src << CLKMGR_PERPLL_CNTR4CLK_SRC_LSB),
	       &clock_manager_base->per_pll.cntr4clk);
	/* peri_gpio_db_clk divider */
	writel(per_cfg->cntr5clk_cnt |
	       (per_cfg->cntr5clk_src << CLKMGR_PERPLL_CNTR5CLK_SRC_LSB),
	       &clock_manager_base->per_pll.cntr5clk);
	/* peri_sdmmc_clk divider */
	writel(per_cfg->cntr6clk_cnt |
	       (per_cfg->cntr6clk_src << CLKMGR_PERPLL_CNTR6CLK_SRC_LSB),
	       &clock_manager_base->per_pll.cntr6clk);
	/* peri_s2f_user0_clk divider */
	writel(per_cfg->cntr7clk_cnt, &clock_manager_base->per_pll.cntr7clk);
	/* peri_s2f_user1_clk divider */
	writel(per_cfg->cntr8clk_cnt |
	       (per_cfg->cntr8clk_src << CLKMGR_PERPLL_CNTR8CLK_SRC_LSB),
	       &clock_manager_base->per_pll.cntr8clk);
	/* peri_hmc_pll_clk divider */
	writel(per_cfg->cntr9clk_cnt, &clock_manager_base->per_pll.cntr9clk);

	/* setup all the external PLL counter */
	/* mpu wrapper / external divider */
	writel(main_cfg->mpuclk_cnt |
	       (main_cfg->mpuclk_src << CLKMGR_MAINPLL_MPUCLK_SRC_LSB),
	       &clock_manager_base->main_pll.mpuclk);
	/* NOC wrapper / external divider */
	writel(main_cfg->nocclk_cnt |
	       (main_cfg->nocclk_src << CLKMGR_MAINPLL_NOCCLK_SRC_LSB),
	       &clock_manager_base->main_pll.nocclk);
	/* NOC subclock divider such as l4 */
	writel(main_cfg->nocdiv_l4mainclk |
	       (main_cfg->nocdiv_l4mpclk <<
		CLKMGR_MAINPLL_NOCDIV_L4MPCLK_LSB) |
	       (main_cfg->nocdiv_l4spclk <<
		CLKMGR_MAINPLL_NOCDIV_L4SPCLK_LSB) |
	       (main_cfg->nocdiv_csatclk <<
		CLKMGR_MAINPLL_NOCDIV_CSATCLK_LSB) |
	       (main_cfg->nocdiv_cstraceclk <<
		CLKMGR_MAINPLL_NOCDIV_CSTRACECLK_LSB) |
	       (main_cfg->nocdiv_cspdbclk <<
		CLKMGR_MAINPLL_NOCDIV_CSPDBGCLK_LSB),
		&clock_manager_base->main_pll.nocdiv);
	/* gpio_db external divider */
	writel(per_cfg->gpiodiv_gpiodbclk,
	       &clock_manager_base->per_pll.gpiodiv);

	/* setup the EMAC clock mux select */
	writel((per_cfg->emacctl_emac0sel <<
		CLKMGR_PERPLL_EMACCTL_EMAC0SEL_LSB) |
	       (per_cfg->emacctl_emac1sel <<
		CLKMGR_PERPLL_EMACCTL_EMAC1SEL_LSB) |
	       (per_cfg->emacctl_emac2sel <<
		CLKMGR_PERPLL_EMACCTL_EMAC2SEL_LSB),
	       &clock_manager_base->per_pll.emacctl);

	/* at this stage, check for PLL lock status */
	cm_wait_for_lock(LOCKED_MASK);

	/*
	 * after locking, but before taking out of bypass,
	 * assert/deassert outresetall
	 */
	/* assert mainpll outresetall */
	setbits_le32(&clock_manager_base->main_pll.vco0,
		     CLKMGR_MAINPLL_VCO0_OUTRSTALL_SET_MSK);
	/* assert perpll outresetall */
	setbits_le32(&clock_manager_base->per_pll.vco0,
		     CLKMGR_PERPLL_VCO0_OUTRSTALL_SET_MSK);
	/* de-assert mainpll outresetall */
	clrbits_le32(&clock_manager_base->main_pll.vco0,
		     CLKMGR_MAINPLL_VCO0_OUTRSTALL_SET_MSK);
	/* de-assert perpll outresetall */
	clrbits_le32(&clock_manager_base->per_pll.vco0,
		     CLKMGR_PERPLL_VCO0_OUTRSTALL_SET_MSK);

	/* Take all PLLs out of bypass when boot mode is cleared. */
	/* release mainpll from bypass */
	writel(CLKMGR_MAINPLL_BYPASS_RESET,
	       &clock_manager_base->main_pll.bypassr);
	/* wait till Clock Manager is not busy */
	cm_wait_for_fsm();

	/* release perpll from bypass */
	writel(CLKMGR_PERPLL_BYPASS_RESET,
	       &clock_manager_base->per_pll.bypassr);
	/* wait till Clock Manager is not busy */
	cm_wait_for_fsm();

	/* clear boot mode */
	clrbits_le32(&clock_manager_base->ctrl,
		     CLKMGR_CLKMGR_CTL_BOOTMOD_SET_MSK);
	/* wait till Clock Manager is not busy */
	cm_wait_for_fsm();

	/* At here, we need to ramp to final value if needed */
	if (pll_ramp_main_hz != 0)
		cm_pll_ramp_main(main_cfg, per_cfg, pll_ramp_main_hz);
	if (pll_ramp_periph_hz != 0)
		cm_pll_ramp_periph(main_cfg, per_cfg, pll_ramp_periph_hz);

	/* Now ungate non-hw-managed clocks */
	writel(CLKMGR_MAINPLL_EN_S2FUSER0CLKEN_SET_MSK |
		CLKMGR_MAINPLL_EN_HMCPLLREFCLKEN_SET_MSK,
		&clock_manager_base->main_pll.ens);
	writel(CLKMGR_PERPLL_EN_RESET, &clock_manager_base->per_pll.ens);

	/* Clear the loss lock and slip bits as they might set during
	clock reconfiguration */
	writel(CLKMGR_CLKMGR_INTR_MAINPLLLOST_SET_MSK |
	       CLKMGR_CLKMGR_INTR_PERPLLLOST_SET_MSK |
	       CLKMGR_CLKMGR_INTR_MAINPLLRFSLIP_SET_MSK |
	       CLKMGR_CLKMGR_INTR_PERPLLRFSLIP_SET_MSK |
	       CLKMGR_CLKMGR_INTR_MAINPLLFBSLIP_SET_MSK |
	       CLKMGR_CLKMGR_INTR_PERPLLFBSLIP_SET_MSK,
	       &clock_manager_base->intr);

	return 0;
}

void cm_use_intosc(void)
{
	setbits_le32(&clock_manager_base->ctrl,
		     CLKMGR_CLKMGR_CTL_BOOTCLK_INTOSC_SET_MSK);
}

unsigned int cm_get_noc_clk_hz(void)
{
	unsigned int clk_src, divisor, nocclk, src_hz;

	nocclk = readl(&clock_manager_base->main_pll.nocclk);
	clk_src = (nocclk >> CLKMGR_MAINPLL_NOCCLK_SRC_LSB) &
		  CLKMGR_MAINPLL_NOCCLK_SRC_MSK;

	divisor = 1 + (nocclk & CLKMGR_MAINPLL_NOCDIV_MSK);

	if (clk_src == CLKMGR_PERPLLGRP_SRC_MAIN) {
		src_hz = cm_get_main_vco_clk_hz();
		src_hz /= 1 +
		(readl(SOCFPGA_CLKMGR_ADDRESS + CLKMGR_MAINPLL_NOC_CLK_OFFSET) &
		CLKMGR_MAINPLL_NOCCLK_CNT_MSK);
	} else if (clk_src == CLKMGR_PERPLLGRP_SRC_PERI) {
		src_hz = cm_get_per_vco_clk_hz();
		src_hz /= 1 +
		((readl(SOCFPGA_CLKMGR_ADDRESS +
			CLKMGR_MAINPLL_NOC_CLK_OFFSET) >>
			CLKMGR_MAINPLL_NOCCLK_PERICNT_LSB) &
			CLKMGR_MAINPLL_NOCCLK_CNT_MSK);
	} else if (clk_src == CLKMGR_PERPLLGRP_SRC_OSC1) {
		src_hz = eosc1_hz;
	} else if (clk_src == CLKMGR_PERPLLGRP_SRC_INTOSC) {
		src_hz = cb_intosc_hz;
	} else if (clk_src == CLKMGR_PERPLLGRP_SRC_FPGA) {
		src_hz = f2s_free_hz;
	} else {
		src_hz = 0;
	}

	return src_hz / divisor;
}

unsigned int cm_get_l4_noc_hz(unsigned int nocdivshift)
{
	unsigned int divisor2 = 1 <<
		((readl(&clock_manager_base->main_pll.nocdiv) >>
			nocdivshift) & CLKMGR_MAINPLL_NOCDIV_MSK);

	return cm_get_noc_clk_hz() / divisor2;
}

int cm_basic_init(const void *blob)
{
	struct mainpll_cfg main_cfg;
	struct perpll_cfg per_cfg;
	struct alteragrp_cfg altrgrp_cfg;
	int rval;

	/* initialize to zero for use case of optional node */
	memset(&main_cfg, 0, sizeof(main_cfg));
	memset(&per_cfg, 0, sizeof(per_cfg));
	memset(&altrgrp_cfg, 0, sizeof(altrgrp_cfg));

	rval = of_get_clk_cfg(blob, &main_cfg, &per_cfg, &altrgrp_cfg);
	if (rval)
		return rval;

	rval =  cm_full_cfg(&main_cfg, &per_cfg);

	cm_l4_main_clk_hz =
		cm_get_l4_noc_hz(CLKMGR_MAINPLL_NOCDIV_L4MAINCLK_LSB);

	cm_l4_mp_clk_hz = cm_get_l4_noc_hz(CLKMGR_MAINPLL_NOCDIV_L4MPCLK_LSB);

	cm_l4_sp_clk_hz = cm_get_l4_sp_clk_hz();

	cm_l4_sys_free_clk_hz = cm_get_noc_clk_hz() / 4;

	return rval;
}

unsigned long cm_get_mpu_clk_hz(void)
{
	u32 reg, clk_hz;
	u32 clk_src, mainmpuclk_reg;

	mainmpuclk_reg = readl(&clock_manager_base->main_pll.mpuclk);

	clk_src = (mainmpuclk_reg >> CLKMGR_MAINPLL_MPUCLK_SRC_LSB) &
		CLKMGR_MAINPLL_MPUCLK_SRC_MSK;

	reg = readl(&clock_manager_base->altera.mpuclk);
	/* Check MPU clock source: main, periph, osc1, intosc or f2s? */
	switch (clk_src) {
	case CLKMGR_MAINPLL_MPUCLK_SRC_MAIN:
		clk_hz = cm_get_main_vco_clk_hz();
		clk_hz /= (reg & CLKMGR_MAINPLL_MPUCLK_CNT_MSK) + 1;
		break;
	case CLKMGR_MAINPLL_MPUCLK_SRC_PERI:
		clk_hz = cm_get_per_vco_clk_hz();
		clk_hz /= (((reg >> CLKMGR_MAINPLL_MPUCLK_PERICNT_LSB) &
			   CLKMGR_MAINPLL_MPUCLK_CNT_MSK) + 1);
		break;
	case CLKMGR_MAINPLL_MPUCLK_SRC_OSC1:
		clk_hz = eosc1_hz;
		break;
	case CLKMGR_MAINPLL_MPUCLK_SRC_INTOSC:
		clk_hz = cb_intosc_hz;
		break;
	case CLKMGR_MAINPLL_MPUCLK_SRC_FPGA:
		clk_hz = f2s_free_hz;
		break;
	default:
		printf("cm_get_mpu_clk_hz invalid clk_src %d\n", clk_src);
		return 0;
	}

	clk_hz /= (mainmpuclk_reg & CLKMGR_MAINPLL_MPUCLK_CNT_MSK) + 1;

	return clk_hz;
}

unsigned int cm_get_per_vco_clk_hz(void)
{
	u32 src_hz = 0;
	u32 clk_src = 0;
	u32 numer = 0;
	u32 denom = 0;
	u32 vco = 0;

	clk_src = readl(&clock_manager_base->per_pll.vco0);

	clk_src = (clk_src >> CLKMGR_PERPLL_VCO0_PSRC_LSB) &
		CLKMGR_PERPLL_VCO0_PSRC_MSK;

	if (clk_src == CLKMGR_PERPLL_VCO0_PSRC_EOSC) {
		src_hz = eosc1_hz;
	} else if (clk_src == CLKMGR_PERPLL_VCO0_PSRC_E_INTOSC) {
		src_hz = cb_intosc_hz;
	} else if (clk_src == CLKMGR_PERPLL_VCO0_PSRC_F2S) {
		src_hz = f2s_free_hz;
	} else if (clk_src == CLKMGR_PERPLL_VCO0_PSRC_MAIN) {
		src_hz = cm_get_main_vco_clk_hz();
		src_hz /= (readl(&clock_manager_base->main_pll.cntr15clk) &
			CLKMGR_MAINPLL_CNTRCLK_MSK) + 1;
	} else {
		printf("cm_get_per_vco_clk_hz invalid clk_src %d\n", clk_src);
		return 0;
	}

	vco = readl(&clock_manager_base->per_pll.vco1);

	numer = vco & CLKMGR_PERPLL_VCO1_NUMER_MSK;

	denom = (vco >> CLKMGR_PERPLL_VCO1_DENOM_LSB) &
			CLKMGR_PERPLL_VCO1_DENOM_MSK;

	vco = src_hz;
	vco /= 1 + denom;
	vco *= 1 + numer;

	return vco;
}

unsigned int cm_get_main_vco_clk_hz(void)
{
	u32 src_hz, numer, denom, vco;

	u32 clk_src = readl(&clock_manager_base->main_pll.vco0);

	clk_src = (clk_src >> CLKMGR_MAINPLL_VCO0_PSRC_LSB) &
		CLKMGR_MAINPLL_VCO0_PSRC_MSK;

	if (clk_src == CLKMGR_MAINPLL_VCO0_PSRC_EOSC) {
		src_hz = eosc1_hz;
	} else if (clk_src == CLKMGR_MAINPLL_VCO0_PSRC_E_INTOSC) {
		src_hz = cb_intosc_hz;
	} else if (clk_src == CLKMGR_MAINPLL_VCO0_PSRC_F2S) {
		src_hz = f2s_free_hz;
	} else {
		printf("cm_get_main_vco_clk_hz invalid clk_src %d\n", clk_src);
		return 0;
	}

	vco = readl(&clock_manager_base->main_pll.vco1);

	numer = vco & CLKMGR_MAINPLL_VCO1_NUMER_MSK;

	denom = (vco >> CLKMGR_MAINPLL_VCO1_DENOM_LSB) &
			CLKMGR_MAINPLL_VCO1_DENOM_MSK;

	vco = src_hz;
	vco /= 1 + denom;
	vco *= 1 + numer;

	return vco;
}

unsigned int cm_get_l4_sp_clk_hz(void)
{
	return cm_get_l4_noc_hz(CLKMGR_MAINPLL_NOCDIV_L4SPCLK_LSB);
}

unsigned int cm_get_mmc_controller_clk_hz(void)
{
	u32 clk_hz = 0;
	u32 clk_input = 0;

	clk_input = readl(&clock_manager_base->per_pll.cntr6clk);
	clk_input = (clk_input >> CLKMGR_PERPLL_CNTR6CLK_SRC_LSB) &
		CLKMGR_PERPLLGRP_SRC_MSK;

	switch (clk_input) {
	case CLKMGR_PERPLLGRP_SRC_MAIN:
		clk_hz = cm_get_main_vco_clk_hz();
		clk_hz /= 1 + (readl(&clock_manager_base->main_pll.cntr6clk) &
			CLKMGR_MAINPLL_CNTRCLK_MSK);
		break;

	case CLKMGR_PERPLLGRP_SRC_PERI:
		clk_hz = cm_get_per_vco_clk_hz();
		clk_hz /= 1 + (readl(&clock_manager_base->per_pll.cntr6clk) &
			CLKMGR_PERPLL_CNTRCLK_MSK);
		break;

	case CLKMGR_PERPLLGRP_SRC_OSC1:
		clk_hz = eosc1_hz;
		break;

	case CLKMGR_PERPLLGRP_SRC_INTOSC:
		clk_hz = cb_intosc_hz;
		break;

	case CLKMGR_PERPLLGRP_SRC_FPGA:
		clk_hz = f2s_free_hz;
		break;
	}

	return clk_hz / 4;
}

unsigned int cm_get_spi_controller_clk_hz(void)
{
	return cm_get_l4_noc_hz(CLKMGR_MAINPLL_NOCDIV_L4MPCLK_LSB);
}

unsigned int cm_get_qspi_controller_clk_hz(void)
{
	return  cm_get_l4_noc_hz(CLKMGR_MAINPLL_NOCDIV_L4MAINCLK_LSB);
}

/* Override weak dw_spi_get_clk implementation in designware_spi.c driver */
int dw_spi_get_clk(struct udevice *bus, ulong *rate)
{
	*rate = cm_get_spi_controller_clk_hz();

	return 0;
}

void cm_print_clock_quick_summary(void)
{
	printf("MPU       %10ld kHz\n", cm_get_mpu_clk_hz() / 1000);
	printf("MMC         %8d kHz\n", cm_get_mmc_controller_clk_hz() / 1000);
	printf("QSPI        %8d kHz\n", cm_get_qspi_controller_clk_hz() / 1000);
	printf("SPI         %8d kHz\n", cm_get_spi_controller_clk_hz() / 1000);
	printf("EOSC1       %8d kHz\n", eosc1_hz / 1000);
	printf("cb_intosc   %8d kHz\n", cb_intosc_hz / 1000);
	printf("f2s_free    %8d kHz\n", f2s_free_hz / 1000);
	printf("Main VCO    %8d kHz\n", cm_get_main_vco_clk_hz() / 1000);
	printf("NOC         %8d kHz\n", cm_get_noc_clk_hz() / 1000);
	printf("L4 Main	    %8d kHz\n",
	       cm_get_l4_noc_hz(CLKMGR_MAINPLL_NOCDIV_L4MAINCLK_LSB) / 1000);
	printf("L4 MP       %8d kHz\n",
	       cm_get_l4_noc_hz(CLKMGR_MAINPLL_NOCDIV_L4MPCLK_LSB) / 1000);
	printf("L4 SP       %8d kHz\n", cm_get_l4_sp_clk_hz() / 1000);
	printf("L4 sys free %8d kHz\n", cm_l4_sys_free_clk_hz / 1000);
}
