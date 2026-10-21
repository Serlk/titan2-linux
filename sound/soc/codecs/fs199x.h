/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * fs199x.h -- Driver for the FourSemi FS199x Audio Amplifier
 *
 * Register layout as seen from the host (bus protocol only).
 *
 * Copyright (c) 2026 serlk <22211586+Serlk@users.noreply.github.com>
 */

#ifndef __FS199X_H__
#define __FS199X_H__

#define FS199X_00H_STATUS		0x00
#define FS199X_03H_DEVID		0x03
#define FS199X_05H_ANASTAT		0x05
#define FS199X_06H_DIGSTAT		0x06
#define FS199X_0BH_ACCKEY		0x0B
#define FS199X_0EH_CHIPINI		0x0E
#define FS199X_10H_PWRCTRL		0x10
#define FS199X_11H_SYSCTRL		0x11
#define FS199X_16H_VOLUME		0x16
#define FS199X_17H_I2SCTRL		0x17
#define FS199X_4CH_AMPCTRL		0x4C
#define FS199X_62H_CRAM_DATA_A		0x62
#define FS199X_66H_CRAM_ADDR_A		0x66
#define FS199X_82H_CRAM_DATA_B		0x82
#define FS199X_86H_CRAM_ADDR_B		0x86
#define FS199X_95H_PROT0		0x95
#define FS199X_96H_PROT1_HI		0x96
#define FS199X_97H_PROT1_LO		0x97
#define FS199X_A1H_PLLCTRL1		0xA1
#define FS199X_A2H_PLLCTRL2		0xA2
#define FS199X_A3H_PLLCTRL3		0xA3
#define FS199X_ABH_INTSTAT		0xAB
#define FS199X_C3H_REVCHK		0xC3
#define FS199X_F0H_OTP_STAT		0xF0
#define FS199X_F1H_SPKRE		0xF1
#define FS199X_REG_MAX			0xF8

/* 00h STATUS */
#define FS199X_00H_VDDPS		BIT(14)
#define FS199X_00H_CLO			BIT(10)
#define FS199X_00H_BOP			BIT(9)
#define FS199X_00H_OTW			BIT(8)
#define FS199X_00H_CLKS			BIT(6)
#define FS199X_00H_OCAMP		BIT(5)
#define FS199X_00H_UV1			BIT(4)
#define FS199X_00H_UV2			BIT(3)
#define FS199X_00H_OT			BIT(2)
#define FS199X_00H_PLLS			BIT(1)
#define FS199X_00H_OV			BIT(0)
#define FS199X_00H_NORMAL		(FS199X_00H_CLKS | FS199X_00H_PLLS)

/* 05h ANASTAT */
#define FS199X_05H_READY_MASK		GENMASK(3, 0)

/* 06h DIGSTAT */
#define FS199X_06H_BUSY_MASK		GENMASK(2, 0)
#define FS199X_06H_AMP_BUSY		BIT(1)

/* 0Bh ACCKEY */
#define FS199X_0BH_ACCKEY_ON		0xCA91
#define FS199X_0BH_ACCKEY_OFF		0x0000

/* 0Eh CHIPINI */
#define FS199X_0EH_INIT_DONE		GENMASK(1, 0)

/* 10h PWRCTRL */
#define FS199X_10H_ON			0x0000
#define FS199X_10H_PWDN			0x0001
#define FS199X_10H_I2C_RESET		0x0002

/* 11h SYSCTRL */
#define FS199X_11H_STANDBY		0x0022
#define FS199X_11H_PLAY			0x00EF

/* 16h VOLUME */
#define FS199X_16H_VOL_SHIFT		6
#define FS199X_16H_VOL_MASK		GENMASK(15, 6)
#define FS199X_16H_VOL_MAX		0x3FF

/* 17h I2SCTRL */
#define FS199X_17H_I2SSR_SHIFT		12
#define FS199X_17H_I2SSR_MASK		GENMASK(15, 12)
#define FS199X_17H_I2SF_MASK		GENMASK(2, 0)
#define FS199X_17H_I2SF_DSP		0x1
#define FS199X_17H_I2SF_LJ		0x2
#define FS199X_17H_I2SF_I2S		0x3
#define FS199X_17H_I2SF_RJ		0x7

/* 4Ch AMPCTRL */
#define FS199X_4CH_AMP_OFF		BIT(14)

/* 66h/86h CRAM address: burst access from a word address */
#define FS199X_CRAM_BURST		BIT(15)
#define FS199X_66H_MODEL		(FS199X_CRAM_BURST | 0x003C)
#define FS199X_66H_EFFECT_A		(FS199X_CRAM_BURST | 0x0000)
#define FS199X_66H_EFFECT_B		(FS199X_CRAM_BURST | 0x0091)
#define FS199X_86H_MODEL		(FS199X_CRAM_BURST | 0x0000)
#define FS199X_CRAM_CLEAR		0x0000

/* ABh INTSTAT (latched events) */
#define FS199X_ABH_VDDPS		BIT(14)
#define FS199X_ABH_OC			BIT(13)
#define FS199X_ABH_UV			BIT(12)
#define FS199X_ABH_OV			BIT(11)
#define FS199X_ABH_OT			BIT(10)
#define FS199X_ABH_BOV			BIT(8)
#define FS199X_ABH_BOL			BIT(6)
#define FS199X_ABH_BOP			BIT(5)
#define FS199X_ABH_OTW			BIT(4)
#define FS199X_ABH_CLKLOST		BIT(1)
/* events after which the output stays muted until the next start */
#define FS199X_ABH_FATAL		(FS199X_ABH_VDDPS | FS199X_ABH_OC | \
					 FS199X_ABH_OV | FS199X_ABH_OT)

/* C3h: revision check of the 0x05 device id */
#define FS199X_C3H_REV_MASK		GENMASK(7, 5)
#define FS199X_C3H_REV_OK		0x80

/* F0h OTP status: number of programmed speaker Re values */
#define FS199X_F0H_RE_COUNT		GENMASK(3, 0)

#endif /* __FS199X_H__ */
