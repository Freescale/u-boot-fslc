// SPDX-License-Identifier: GPL-2.0+
/*
 * Freescale i.MX28 I2C Driver
 *
 * Copyright (C) 2011 Marek Vasut <marek.vasut@gmail.com>
 * on behalf of DENX Software Engineering GmbH
 *
 * Partly based on Linux kernel i2c-mxs.c driver:
 * Copyright (C) 2011 Wolfram Sang, Pengutronix e.K.
 *
 * Which was based on a (non-working) driver which was:
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

#include <common.h>
#include <malloc.h>
#include <i2c.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/sys_proto.h>

#define	MXS_I2C_MAX_TIMEOUT	1000000

static struct mxs_i2c_regs *mxs_i2c_get_base(struct i2c_adapter *adap)
{
	if (adap->hwadapnr == 0)
		return (struct mxs_i2c_regs *)MXS_I2C0_BASE;
	else
		return (struct mxs_i2c_regs *)MXS_I2C1_BASE;
}

static unsigned int mxs_i2c_get_bus_speed(struct i2c_adapter *adap)
{
	struct mxs_i2c_regs *i2c_regs = mxs_i2c_get_base(adap);
	uint32_t clk = mxc_get_clock(MXC_XTAL_CLK);
	uint32_t timing0;

	timing0 = readl(&i2c_regs->hw_i2c_timing0);
	/*
	 * This is a reverse version of the algorithm presented in
	 * i2c_set_bus_speed(). Please refer there for details.
	 */
	return clk / ((((timing0 >> 16) - 3) * 2) + 38);
}

static uint mxs_i2c_set_bus_speed(struct i2c_adapter *adap, uint speed)
{
	struct mxs_i2c_regs *i2c_regs = mxs_i2c_get_base(adap);
	/*
	 * The timing derivation algorithm. There is no documentation for this
	 * algorithm available, it was derived by using the scope and fiddling
	 * with constants until the result observed on the scope was good enough
	 * for 20kHz, 50kHz, 100kHz, 200kHz, 300kHz and 400kHz. It should be
	 * possible to assume the algorithm works for other frequencies as well.
	 *
	 * Note it was necessary to cap the frequency on both ends as it's not
	 * possible to configure completely arbitrary frequency for the I2C bus
	 * clock.
	 */
	uint32_t clk = mxc_get_clock(MXC_XTAL_CLK);
	uint32_t base = ((clk / speed) - 38) / 2;
	uint16_t high_count = base + 3;
	uint16_t low_count = base - 3;
	uint16_t rcv_count = (high_count * 3) / 4;
	uint16_t xmit_count = low_count / 4;

	if (speed > 540000) {
		printf("MXS I2C: Speed too high (%d Hz)\n", speed);
		return -EINVAL;
	}

	if (speed < 12000) {
		printf("MXS I2C: Speed too low (%d Hz)\n", speed);
		return -EINVAL;
	}

	writel((high_count << 16) | rcv_count, &i2c_regs->hw_i2c_timing0);
	writel((low_count << 16) | xmit_count, &i2c_regs->hw_i2c_timing1);

	writel((0x0030 << I2C_TIMING2_BUS_FREE_OFFSET) |
		(0x0030 << I2C_TIMING2_LEADIN_COUNT_OFFSET),
		&i2c_regs->hw_i2c_timing2);

	return 0;
}

static void mxs_i2c_reset(struct i2c_adapter *adap)
{
	struct mxs_i2c_regs *i2c_regs = mxs_i2c_get_base(adap);
	int ret;
	int speed = mxs_i2c_get_bus_speed(adap);

	ret = mxs_reset_block(&i2c_regs->hw_i2c_ctrl0_reg);
	if (ret) {
		debug("MXS I2C: Block reset timeout\n");
		return;
	}

	writel(I2C_CTRL1_DATA_ENGINE_CMPLT_IRQ | I2C_CTRL1_NO_SLAVE_ACK_IRQ |
		I2C_CTRL1_EARLY_TERM_IRQ | I2C_CTRL1_MASTER_LOSS_IRQ |
		I2C_CTRL1_SLAVE_STOP_IRQ | I2C_CTRL1_SLAVE_IRQ,
		&i2c_regs->hw_i2c_ctrl1_clr);

	writel(I2C_QUEUECTRL_PIO_QUEUE_MODE, &i2c_regs->hw_i2c_queuectrl_set);

	mxs_i2c_set_bus_speed(adap, speed);
}

static void mxs_i2c_setup_read(struct i2c_adapter *adap, uint8_t chip, int len)
{
	struct mxs_i2c_regs *i2c_regs = mxs_i2c_get_base(adap);

	writel(I2C_QUEUECMD_RETAIN_CLOCK | I2C_QUEUECMD_PRE_SEND_START |
		I2C_QUEUECMD_MASTER_MODE | I2C_QUEUECMD_DIRECTION |
		(1 << I2C_QUEUECMD_XFER_COUNT_OFFSET),
		&i2c_regs->hw_i2c_queuecmd);

	writel((chip << 1) | 1, &i2c_regs->hw_i2c_data);

	writel(I2C_QUEUECMD_SEND_NAK_ON_LAST | I2C_QUEUECMD_MASTER_MODE |
		(len << I2C_QUEUECMD_XFER_COUNT_OFFSET) |
		I2C_QUEUECMD_POST_SEND_STOP, &i2c_regs->hw_i2c_queuecmd);

	writel(I2C_QUEUECTRL_QUEUE_RUN, &i2c_regs->hw_i2c_queuectrl_set);
}

static int mxs_i2c_write(struct i2c_adapter *adap, uchar chip, uint addr,
			 int alen, uchar *buf, int blen, int stop)
{
	struct mxs_i2c_regs *i2c_regs = mxs_i2c_get_base(adap);
	uint32_t data, tmp;
	int i, remain, off;
	int timeout = MXS_I2C_MAX_TIMEOUT;

	if ((alen > 4) || (alen == 0)) {
		debug("MXS I2C: Invalid address length\n");
		return -EINVAL;
	}

	if (stop)
		stop = I2C_QUEUECMD_POST_SEND_STOP;

	writel(I2C_QUEUECMD_PRE_SEND_START |
		I2C_QUEUECMD_MASTER_MODE | I2C_QUEUECMD_DIRECTION |
		((blen + alen + 1) << I2C_QUEUECMD_XFER_COUNT_OFFSET) | stop,
		&i2c_regs->hw_i2c_queuecmd);

	data = (chip << 1) << 24;

	for (i = 0; i < alen; i++) {
		data >>= 8;
		data |= ((char *)&addr)[alen - i - 1] << 24;
		if ((i & 3) == 2)
			writel(data, &i2c_regs->hw_i2c_data);
	}

	off = i;
	for (; i < off + blen; i++) {
		data >>= 8;
		data |= buf[i - off] << 24;
		if ((i & 3) == 2)
			writel(data, &i2c_regs->hw_i2c_data);
	}

	remain = 24 - ((i & 3) * 8);
	if (remain)
		writel(data >> remain, &i2c_regs->hw_i2c_data);

	writel(I2C_QUEUECTRL_QUEUE_RUN, &i2c_regs->hw_i2c_queuectrl_set);

	while (--timeout) {
		tmp = readl(&i2c_regs->hw_i2c_queuestat);
		if (tmp & I2C_QUEUESTAT_WR_QUEUE_EMPTY)
			break;
	}

	if (!timeout) {
		debug("MXS I2C: Failed transmitting data!\n");
		return -EINVAL;
	}

	return 0;
}

static int mxs_i2c_wait_for_ack(struct i2c_adapter *adap)
{
	struct mxs_i2c_regs *i2c_regs = mxs_i2c_get_base(adap);
	uint32_t tmp;
	int timeout = MXS_I2C_MAX_TIMEOUT;

	for (;;) {
		tmp = readl(&i2c_regs->hw_i2c_ctrl1);
		if (tmp & I2C_CTRL1_NO_SLAVE_ACK_IRQ) {
			debug("MXS I2C: No slave ACK\n");
			goto err;
		}

		if (tmp & (
			I2C_CTRL1_EARLY_TERM_IRQ | I2C_CTRL1_MASTER_LOSS_IRQ |
			I2C_CTRL1_SLAVE_STOP_IRQ | I2C_CTRL1_SLAVE_IRQ)) {
			debug("MXS I2C: Error (CTRL1 = %08x)\n", tmp);
			goto err;
		}

		if (tmp & I2C_CTRL1_DATA_ENGINE_CMPLT_IRQ)
			break;

		if (!timeout--) {
			debug("MXS I2C: Operation timed out\n");
			goto err;
		}

		udelay(1);
	}

	return 0;

err:
	mxs_i2c_reset(adap);
	return 1;
}

static int mxs_i2c_if_read(struct i2c_adapter *adap, uint8_t chip,
			   uint addr, int alen, uint8_t *buffer,
			   int len)
{
	struct mxs_i2c_regs *i2c_regs = mxs_i2c_get_base(adap);
	uint32_t tmp = 0;
	int timeout = MXS_I2C_MAX_TIMEOUT;
	int ret;
	int i;

	ret = mxs_i2c_write(adap, chip, addr, alen, NULL, 0, 0);
	if (ret) {
		debug("MXS I2C: Failed writing address\n");
		return ret;
	}

	ret = mxs_i2c_wait_for_ack(adap);
	if (ret) {
		debug("MXS I2C: Failed writing address\n");
		return ret;
	}

	mxs_i2c_setup_read(adap, chip, len);
	ret = mxs_i2c_wait_for_ack(adap);
	if (ret) {
		debug("MXS I2C: Failed reading address\n");
		return ret;
	}

	for (i = 0; i < len; i++) {
		if (!(i & 3)) {
			while (--timeout) {
				tmp = readl(&i2c_regs->hw_i2c_queuestat);
				if (!(tmp & I2C_QUEUESTAT_RD_QUEUE_EMPTY))
					break;
			}

			if (!timeout) {
				debug("MXS I2C: Failed receiving data!\n");
				return -ETIMEDOUT;
			}

			tmp = readl(&i2c_regs->hw_i2c_queuedata);
		}
		buffer[i] = tmp & 0xff;
		tmp >>= 8;
	}

	return 0;
}

static int mxs_i2c_if_write(struct i2c_adapter *adap, uint8_t chip,
			    uint addr, int alen, uint8_t *buffer,
			    int len)
{
	int ret;
	ret = mxs_i2c_write(adap, chip, addr, alen, buffer, len, 1);
	if (ret) {
		debug("MXS I2C: Failed writing address\n");
		return ret;
	}

	ret = mxs_i2c_wait_for_ack(adap);
	if (ret)
		debug("MXS I2C: Failed writing address\n");

	return ret;
}

static int mxs_i2c_probe(struct i2c_adapter *adap, uint8_t chip)
{
	int ret;
	ret = mxs_i2c_write(adap, chip, 0, 1, NULL, 0, 1);
	if (!ret)
		ret = mxs_i2c_wait_for_ack(adap);
	mxs_i2c_reset(adap);
	return ret;
}

static void mxs_i2c_init(struct i2c_adapter *adap, int speed, int slaveaddr)
{
	mxs_i2c_reset(adap);
	mxs_i2c_set_bus_speed(adap, speed);

	return;
}

U_BOOT_I2C_ADAP_COMPLETE(mxs0, mxs_i2c_init, mxs_i2c_probe,
			 mxs_i2c_if_read, mxs_i2c_if_write,
			 mxs_i2c_set_bus_speed,
			 CONFIG_SYS_I2C_SPEED, 0, 0)
U_BOOT_I2C_ADAP_COMPLETE(mxs1, mxs_i2c_init, mxs_i2c_probe,
			 mxs_i2c_if_read, mxs_i2c_if_write,
			 mxs_i2c_set_bus_speed,
			 CONFIG_SYS_I2C_SPEED, 0, 1)
