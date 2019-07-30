/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2012
 * Texas Instruments, <www.ti.com>
 */
#ifndef _I2C_AM33XX_H_
#define _I2C_AM33XX_H_

#define  I2C_BASE1		0x44E0B000
#define  I2C_BASE2		0x4802A000
#define  I2C_BASE3		0x4819C000

#define I2C_DEFAULT_BASE		I2C_BASE1

struct i2c {
	unsigned short revnb_lo;        /* 0x00 */
	unsigned short res1;
	unsigned short revnb_hi;        /* 0x04 */
	unsigned short res2[5];
	unsigned short sysc;		/* 0x10 */
	unsigned short res3[9];
	unsigned short irqstatus_raw;   /* 0x24 */
	unsigned short res4;
	unsigned short stat;            /* 0x28 */
	unsigned short res5;
	unsigned short ie;              /* 0x2C */
	unsigned short res6;
	unsigned short irqenable_clr;   /* 0x30 */
	unsigned short res7;
	unsigned short iv;              /* 0x34 */
	unsigned short res8[45];
	unsigned short syss;            /* 0x90 */
	unsigned short res9;
	unsigned short buf;             /* 0x94 */
	unsigned short res10;
	unsigned short cnt;             /* 0x98 */
	unsigned short res11;
	unsigned short data;            /* 0x9C */
	unsigned short res13;
	unsigned short res14;           /* 0xA0 */
	unsigned short res15;
	unsigned short con;             /* 0xA4 */
	unsigned short res16;
	unsigned short oa;              /* 0xA8 */
	unsigned short res17;
	unsigned short sa;              /* 0xAC */
	unsigned short res18;
	unsigned short psc;             /* 0xB0 */
	unsigned short res19;
	unsigned short scll;            /* 0xB4 */
	unsigned short res20;
	unsigned short sclh;            /* 0xB8 */
	unsigned short res21;
	unsigned short systest;         /* 0xBC */
	unsigned short res22;
	unsigned short bufstat;         /* 0xC0 */
	unsigned short res23;
};

#define I2C_IP_CLK			48000000
#define I2C_INTERNAL_SAMPLING_CLK	12000000

#endif /* _I2C_AM33XX_H_ */
