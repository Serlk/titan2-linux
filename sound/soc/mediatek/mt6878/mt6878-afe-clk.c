// SPDX-License-Identifier: GPL-2.0
/*
 *  mt6878-afe-clk.c  --  Mediatek 6878 afe clock ctrl
 *
 *  Copyright (c) 2023 MediaTek Inc.
 *  Copyright (c) 2025 MediaTek Inc.
 *  Author: Shu-wei Hsu <Shu-wei.Hsu@mediatek.com>
 *  Author: Darren Ye <darren.ye@mediatek.com>
 */

#include <linux/clk.h>
#include <linux/regmap.h>

#include "mt6878-afe-clk.h"
#include "mt6878-afe-common.h"

static const char *aud_clks[MT6878_CLK_NUM] = {
	[MT6878_CLK_TOP_MUX_AUDIOINTBUS] = "top_aud_intbus",
	[MT6878_CLK_TOP_MUX_AUD_1] = "top_aud_1",
	[MT6878_CLK_TOP_MUX_AUD_2] = "top_aud_2",
	[MT6878_CLK_TOP_MUX_AUD_ENG1] = "top_aud_eng1",
	[MT6878_CLK_TOP_MUX_AUD_ENG2] = "top_aud_eng2",
	[MT6878_CLK_TOP_MUX_AUDIO_H] = "top_aud_h",
	[MT6878_CLK_TOP_APLL1_CK] = "apll1",
	[MT6878_CLK_TOP_APLL2_CK] = "apll2",
	[MT6878_CLK_TOP_APLL1_D4] = "apll1_d4",
	[MT6878_CLK_TOP_APLL2_D4] = "apll2_d4",
	[MT6878_CLK_TOP_CLK26M] = "clk26m",
	[MT6878_CLK_PERAO_AUDIO_SLV] = "peri_aud_slv",
	[MT6878_CLK_PERAO_AUDIO_MST] = "peri_aud_mst",
	[MT6878_CLK_PERAO_AUDIO_INTBUS] = "peri_aud_intbus",
};

static struct clk *mt6878_clk(struct mtk_base_afe *afe, int id)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;

	return afe_priv->clk[id];
}

static int mt6878_afe_enable_clk(struct mtk_base_afe *afe, int id)
{
	int ret;

	ret = clk_prepare_enable(mt6878_clk(afe, id));
	if (ret)
		dev_err(afe->dev, "failed to enable %s: %d\n", aud_clks[id], ret);

	return ret;
}

static void mt6878_afe_disable_clk(struct mtk_base_afe *afe, int id)
{
	clk_disable_unprepare(mt6878_clk(afe, id));
}

static int mt6878_afe_set_parent(struct mtk_base_afe *afe, int id, int parent)
{
	int ret;

	ret = clk_set_parent(mt6878_clk(afe, id), mt6878_clk(afe, parent));
	if (ret)
		dev_err(afe->dev, "failed to set %s parent to %s: %d\n",
			aud_clks[id], aud_clks[parent], ret);

	return ret;
}

static unsigned int get_top_cg_reg(unsigned int cg_type)
{
	switch (cg_type) {
	case MT6878_AUDIO_26M_EN_ON:
	case MT6878_AUDIO_APLL1_EN_ON:
	case MT6878_AUDIO_APLL2_EN_ON:
		return AUDIO_ENGEN_CON0;
	case MT6878_CG_AUDIO_HOPPING_CK:
	case MT6878_CG_AUDIO_F26M_CK:
	case MT6878_CG_APLL1_CK:
	case MT6878_CG_APLL2_CK:
	case MT6878_PDN_APLL_TUNER2:
	case MT6878_PDN_APLL_TUNER1:
		return AUDIO_TOP_CON4;
	default:
		return 0;
	}
}

static unsigned int get_top_cg_mask(unsigned int cg_type)
{
	switch (cg_type) {
	case MT6878_AUDIO_26M_EN_ON:
		return AUDIO_26M_EN_ON_MASK_SFT;
	case MT6878_AUDIO_APLL1_EN_ON:
		return AUDIO_APLL1_EN_ON_MASK_SFT;
	case MT6878_AUDIO_APLL2_EN_ON:
		return AUDIO_APLL2_EN_ON_MASK_SFT;
	case MT6878_CG_AUDIO_HOPPING_CK:
		return CG_AUDIO_HOPPING_CK_MASK_SFT;
	case MT6878_CG_AUDIO_F26M_CK:
		return CG_AUDIO_F26M_CK_MASK_SFT;
	case MT6878_CG_APLL1_CK:
		return CG_APLL1_CK_MASK_SFT;
	case MT6878_CG_APLL2_CK:
		return CG_APLL2_CK_MASK_SFT;
	case MT6878_PDN_APLL_TUNER2:
		return PDN_APLL_TUNER2_MASK_SFT;
	case MT6878_PDN_APLL_TUNER1:
		return PDN_APLL_TUNER1_MASK_SFT;
	default:
		return 0;
	}
}

/* AUDIO_ENGEN_CON0 bits are enables, AUDIO_TOP_CON4 bits are power-downs */
static bool top_cg_is_enable_bit(unsigned int cg_type)
{
	return get_top_cg_reg(cg_type) == AUDIO_ENGEN_CON0;
}

static int mt6878_afe_set_top_cg(struct mtk_base_afe *afe, unsigned int cg_type,
				 bool on)
{
	unsigned int mask = get_top_cg_mask(cg_type);
	unsigned int val = (on == top_cg_is_enable_bit(cg_type)) ? mask : 0;
	int ret;

	ret = regmap_update_bits(afe->regmap, get_top_cg_reg(cg_type), mask, val);
	if (ret)
		dev_err(afe->dev, "failed to switch top cg %u: %d\n", cg_type, ret);

	return ret;
}

/*
 * APLL1 (22.5792 MHz family): aud_1 -> apll1, aud_engen1 -> apll1_d4,
 * audio_h -> apll1. The muxes are switched back to 26 MHz when unused.
 */
static int apll_mux_setting(struct mtk_base_afe *afe, int apll, bool enable)
{
	int mux_aud = apll == MT6878_APLL1 ? MT6878_CLK_TOP_MUX_AUD_1 :
					     MT6878_CLK_TOP_MUX_AUD_2;
	int mux_eng = apll == MT6878_APLL1 ? MT6878_CLK_TOP_MUX_AUD_ENG1 :
					     MT6878_CLK_TOP_MUX_AUD_ENG2;
	int pll = apll == MT6878_APLL1 ? MT6878_CLK_TOP_APLL1_CK :
					 MT6878_CLK_TOP_APLL2_CK;
	int pll_d4 = apll == MT6878_APLL1 ? MT6878_CLK_TOP_APLL1_D4 :
					    MT6878_CLK_TOP_APLL2_D4;
	int ret;

	if (enable) {
		ret = mt6878_afe_enable_clk(afe, mux_aud);
		if (ret)
			return ret;
		ret = mt6878_afe_set_parent(afe, mux_aud, pll);
		if (ret)
			goto err_aud;

		ret = mt6878_afe_enable_clk(afe, mux_eng);
		if (ret)
			goto err_aud;
		ret = mt6878_afe_set_parent(afe, mux_eng, pll_d4);
		if (ret)
			goto err_eng;

		ret = mt6878_afe_enable_clk(afe, MT6878_CLK_TOP_MUX_AUDIO_H);
		if (ret)
			goto err_eng;
		ret = mt6878_afe_set_parent(afe, MT6878_CLK_TOP_MUX_AUDIO_H, pll);
		if (ret)
			goto err_h;

		return 0;
	}

	mt6878_afe_set_parent(afe, mux_eng, MT6878_CLK_TOP_CLK26M);
	mt6878_afe_set_parent(afe, mux_aud, MT6878_CLK_TOP_CLK26M);
	mt6878_afe_set_parent(afe, MT6878_CLK_TOP_MUX_AUDIO_H,
			      MT6878_CLK_TOP_CLK26M);
	ret = 0;
err_h:
	mt6878_afe_disable_clk(afe, MT6878_CLK_TOP_MUX_AUDIO_H);
err_eng:
	mt6878_afe_disable_clk(afe, mux_eng);
err_aud:
	mt6878_afe_disable_clk(afe, mux_aud);
	return ret;
}

int mt6878_apll1_enable(struct mtk_base_afe *afe)
{
	int ret;

	ret = apll_mux_setting(afe, MT6878_APLL1, true);
	if (ret)
		return ret;

	mt6878_afe_set_top_cg(afe, MT6878_CG_APLL1_CK, true);
	mt6878_afe_set_top_cg(afe, MT6878_PDN_APLL_TUNER1, true);

	/* sel 44.1kHz: 1, apll_div: 7, upper bound: 3 */
	regmap_update_bits(afe->regmap, AFE_APLL1_TUNER_CFG,
			   XTAL_EN_128FS_SEL_MASK_SFT | APLL_DIV_MASK_SFT |
			   UPPER_BOUND_MASK_SFT,
			   (0x1 << XTAL_EN_128FS_SEL_SFT) | (7 << APLL_DIV_SFT) |
			   (3 << UPPER_BOUND_SFT));
	regmap_update_bits(afe->regmap, AFE_APLL1_TUNER_CFG,
			   FREQ_TUNER_EN_MASK_SFT, 0x1 << FREQ_TUNER_EN_SFT);

	mt6878_afe_set_top_cg(afe, MT6878_AUDIO_APLL1_EN_ON, true);

	return 0;
}

void mt6878_apll1_disable(struct mtk_base_afe *afe)
{
	mt6878_afe_set_top_cg(afe, MT6878_AUDIO_APLL1_EN_ON, false);
	regmap_update_bits(afe->regmap, AFE_APLL1_TUNER_CFG,
			   FREQ_TUNER_EN_MASK_SFT, 0);
	mt6878_afe_set_top_cg(afe, MT6878_PDN_APLL_TUNER1, false);
	mt6878_afe_set_top_cg(afe, MT6878_CG_APLL1_CK, false);
	apll_mux_setting(afe, MT6878_APLL1, false);
}

int mt6878_apll2_enable(struct mtk_base_afe *afe)
{
	int ret;

	ret = apll_mux_setting(afe, MT6878_APLL2, true);
	if (ret)
		return ret;

	mt6878_afe_set_top_cg(afe, MT6878_CG_APLL2_CK, true);
	mt6878_afe_set_top_cg(afe, MT6878_PDN_APLL_TUNER2, true);

	/* sel 48kHz: 2, apll_div: 7, upper bound: 3 */
	regmap_update_bits(afe->regmap, AFE_APLL2_TUNER_CFG,
			   XTAL_EN_128FS_SEL_MASK_SFT | APLL_DIV_MASK_SFT |
			   UPPER_BOUND_MASK_SFT,
			   (0x2 << XTAL_EN_128FS_SEL_SFT) | (7 << APLL_DIV_SFT) |
			   (3 << UPPER_BOUND_SFT));
	regmap_update_bits(afe->regmap, AFE_APLL2_TUNER_CFG,
			   FREQ_TUNER_EN_MASK_SFT, 0x1 << FREQ_TUNER_EN_SFT);

	mt6878_afe_set_top_cg(afe, MT6878_AUDIO_APLL2_EN_ON, true);

	return 0;
}

void mt6878_apll2_disable(struct mtk_base_afe *afe)
{
	mt6878_afe_set_top_cg(afe, MT6878_AUDIO_APLL2_EN_ON, false);
	regmap_update_bits(afe->regmap, AFE_APLL2_TUNER_CFG,
			   FREQ_TUNER_EN_MASK_SFT, 0);
	mt6878_afe_set_top_cg(afe, MT6878_PDN_APLL_TUNER2, false);
	mt6878_afe_set_top_cg(afe, MT6878_CG_APLL2_CK, false);
	apll_mux_setting(afe, MT6878_APLL2, false);
}

int mt6878_get_apll_rate(struct mtk_base_afe *afe, int apll)
{
	if (apll != MT6878_APLL1 && apll != MT6878_APLL2) {
		dev_warn(afe->dev, "invalid apll %d\n", apll);
		return 0;
	}

	return clk_get_rate(mt6878_clk(afe, apll == MT6878_APLL1 ?
				       MT6878_CLK_TOP_APLL1_CK :
				       MT6878_CLK_TOP_APLL2_CK));
}

/* 48K: select APLL2; 44.1k: select APLL1 */
int mt6878_get_apll_by_rate(struct mtk_base_afe *afe, int rate)
{
	return (rate % 8000) ? MT6878_APLL1 : MT6878_APLL2;
}

int mt6878_get_apll_by_name(struct mtk_base_afe *afe, const char *name)
{
	if (strcmp(name, APLL1_W_NAME) == 0)
		return MT6878_APLL1;

	return MT6878_APLL2;
}

/* clocks needed for any AFE register access */
int mt6878_afe_enable_reg_rw_clk(struct mtk_base_afe *afe)
{
	int ret;

	/* infra/peri bus path to the AFE */
	ret = mt6878_afe_enable_clk(afe, MT6878_CLK_PERAO_AUDIO_INTBUS);
	if (ret)
		return ret;
	ret = mt6878_afe_enable_clk(afe, MT6878_CLK_PERAO_AUDIO_SLV);
	if (ret)
		goto err_slv;
	ret = mt6878_afe_enable_clk(afe, MT6878_CLK_PERAO_AUDIO_MST);
	if (ret)
		goto err_mst;

	/* AFE internal bus (AFE SRAM, registers) */
	ret = mt6878_afe_enable_clk(afe, MT6878_CLK_TOP_MUX_AUDIOINTBUS);
	if (ret)
		goto err_intbus;
	ret = mt6878_afe_set_parent(afe, MT6878_CLK_TOP_MUX_AUDIOINTBUS,
				    MT6878_CLK_TOP_CLK26M);
	if (ret)
		goto err_audio_h;

	ret = mt6878_afe_enable_clk(afe, MT6878_CLK_TOP_MUX_AUDIO_H);
	if (ret)
		goto err_audio_h;
	ret = mt6878_afe_set_parent(afe, MT6878_CLK_TOP_MUX_AUDIO_H,
				    MT6878_CLK_TOP_CLK26M);
	if (ret)
		goto err_set_h;

	return 0;

err_set_h:
	mt6878_afe_disable_clk(afe, MT6878_CLK_TOP_MUX_AUDIO_H);
err_audio_h:
	mt6878_afe_disable_clk(afe, MT6878_CLK_TOP_MUX_AUDIOINTBUS);
err_intbus:
	mt6878_afe_disable_clk(afe, MT6878_CLK_PERAO_AUDIO_MST);
err_mst:
	mt6878_afe_disable_clk(afe, MT6878_CLK_PERAO_AUDIO_SLV);
err_slv:
	mt6878_afe_disable_clk(afe, MT6878_CLK_PERAO_AUDIO_INTBUS);
	return ret;
}

void mt6878_afe_disable_reg_rw_clk(struct mtk_base_afe *afe)
{
	mt6878_afe_disable_clk(afe, MT6878_CLK_TOP_MUX_AUDIO_H);
	mt6878_afe_disable_clk(afe, MT6878_CLK_TOP_MUX_AUDIOINTBUS);
	mt6878_afe_disable_clk(afe, MT6878_CLK_PERAO_AUDIO_MST);
	mt6878_afe_disable_clk(afe, MT6878_CLK_PERAO_AUDIO_SLV);
	mt6878_afe_disable_clk(afe, MT6878_CLK_PERAO_AUDIO_INTBUS);
}

int mt6878_afe_enable_main_clock(struct mtk_base_afe *afe)
{
	return mt6878_afe_set_top_cg(afe, MT6878_AUDIO_26M_EN_ON, true);
}

void mt6878_afe_disable_main_clock(struct mtk_base_afe *afe)
{
	mt6878_afe_set_top_cg(afe, MT6878_AUDIO_26M_EN_ON, false);
}

int mt6878_init_clock(struct mtk_base_afe *afe)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	int i;

	afe_priv->clk = devm_kcalloc(afe->dev, MT6878_CLK_NUM,
				     sizeof(*afe_priv->clk), GFP_KERNEL);
	if (!afe_priv->clk)
		return -ENOMEM;

	for (i = 0; i < MT6878_CLK_NUM; i++) {
		afe_priv->clk[i] = devm_clk_get(afe->dev, aud_clks[i]);
		if (IS_ERR(afe_priv->clk[i]))
			return dev_err_probe(afe->dev, PTR_ERR(afe_priv->clk[i]),
					     "failed to get clock %s\n",
					     aud_clks[i]);
	}

	return 0;
}
