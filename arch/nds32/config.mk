# SPDX-License-Identifier: GPL-2.0+
#
# (C) Copyright 2000-2002
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# (C) Copyright 2011
# Shawn Lin, Andes Technology Corporation <nobuhiro@andestech.com>
# Macpaul Lin, Andes Technology Corporation <macpaul@andestech.com>
#

ifeq ($(CROSS_COMPILE),)
CROSS_COMPILE := nds32le-linux-
endif

CONFIG_STANDALONE_LOAD_ADDR = 0x300000 \
			      -T $(srctree)/examples/standalone/nds32.lds

PLATFORM_RELFLAGS	+= -fno-strict-aliasing -fno-common -mrelax
PLATFORM_RELFLAGS	+= -gdwarf-2
PLATFORM_CPPFLAGS	+= -D__nds32__ -G0 -ffixed-10 -fpie

LDFLAGS_u-boot		= --gc-sections --relax -pie
