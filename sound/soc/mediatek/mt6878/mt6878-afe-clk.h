/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6878-afe-clk.h  --  Mediatek MT6878 AFE Clock Control definitions
 *
 * Copyright (c) 2023 MediaTek Inc.
 * Copyright (c) 2025 MediaTek Inc.
 *  Author: Shu-wei Hsu <Shu-wei.Hsu@mediatek.com>
 *  Author: Darren Ye <darren.ye@mediatek.com>
 */

#ifndef _MT6878_AFE_CLOCK_CTRL_H_
#define _MT6878_AFE_CLOCK_CTRL_H_

#define MT6878_AFE_26M 26000000

/* APLL */
#define APLL1_W_NAME "APLL1"
#define APLL2_W_NAME "APLL2"

enum {
	MT6878_APLL1 = 0,
	MT6878_APLL2,
};

enum {
	/* topckgen muxes */
	MT6878_CLK_TOP_MUX_AUDIOINTBUS,
	MT6878_CLK_TOP_MUX_AUD_1,
	MT6878_CLK_TOP_MUX_AUD_2,
	MT6878_CLK_TOP_MUX_AUD_ENG1,
	MT6878_CLK_TOP_MUX_AUD_ENG2,
	MT6878_CLK_TOP_MUX_AUDIO_H,
	/* mux parents */
	MT6878_CLK_TOP_APLL1_CK,
	MT6878_CLK_TOP_APLL2_CK,
	MT6878_CLK_TOP_APLL1_D4,
	MT6878_CLK_TOP_APLL2_D4,
	MT6878_CLK_TOP_CLK26M,
	/* pericfg_ao bus gates */
	MT6878_CLK_PERAO_AUDIO_SLV,
	MT6878_CLK_PERAO_AUDIO_MST,
	MT6878_CLK_PERAO_AUDIO_INTBUS,
	MT6878_CLK_NUM,
};

struct mtk_base_afe;

int mt6878_get_apll_rate(struct mtk_base_afe *afe, int apll);
int mt6878_get_apll_by_rate(struct mtk_base_afe *afe, int rate);
int mt6878_get_apll_by_name(struct mtk_base_afe *afe, const char *name);
int mt6878_init_clock(struct mtk_base_afe *afe);
int mt6878_apll1_enable(struct mtk_base_afe *afe);
void mt6878_apll1_disable(struct mtk_base_afe *afe);
int mt6878_apll2_enable(struct mtk_base_afe *afe);
void mt6878_apll2_disable(struct mtk_base_afe *afe);
int mt6878_afe_enable_main_clock(struct mtk_base_afe *afe);
void mt6878_afe_disable_main_clock(struct mtk_base_afe *afe);
int mt6878_afe_enable_reg_rw_clk(struct mtk_base_afe *afe);
void mt6878_afe_disable_reg_rw_clk(struct mtk_base_afe *afe);

#endif
