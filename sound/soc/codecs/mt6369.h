/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6369.h  --  mt6369 ALSA SoC audio codec driver
 *
 * Register definitions as in the MediaTek MT6878 BSP header, limited to
 * the ones the driver uses.
 *
 * Copyright (C) 2021 MediaTek Inc.
 * Author: Chih-hong Yang <chih-hong.yang@mediatek.com>
 */

#ifndef _MT6369_H_
#define _MT6369_H_

struct snd_soc_component;

#define MT6369_SMT_CON2                      0x23
#define MT6369_SMT_CON3                      0x24
#define MT6369_DRV_CON3                      0x2a
#define MT6369_DRV_CON4                      0x2b
#define MT6369_DRV_CON5                      0x2c
#define MT6369_DRV_CON6                      0x2d
#define MT6369_GPIO_DIR1                     0x8b
#define MT6369_GPIO_MODE4                    0xc7
#define MT6369_GPIO_MODE4_SET                0xc8
#define MT6369_GPIO_MODE4_CLR                0xc9
#define MT6369_GPIO_MODE5_SET                0xcb
#define MT6369_GPIO_MODE5_CLR                0xcc
#define MT6369_GPIO_MODE6_SET                0xce
#define MT6369_GPIO_MODE6_CLR                0xcf
#define MT6369_GPIO_MODE7_SET                0xd1
#define MT6369_GPIO_MODE7_CLR                0xd2
#define MT6369_AUD_TOP_CKPDN_CON0            0x230c
#define MT6369_AFE_UL_DL_CON0                0x2388
#define MT6369_AFE_DL_SRC2_CON0              0x2389
#define MT6369_AFE_UL_SRC_CON1               0x238b
#define MT6369_AUDIO_TOP_CON0                0x238f
#define MT6369_AFUNC_AUD_CON0                0x2392
#define MT6369_AFUNC_AUD_CON1                0x2393
#define MT6369_AFUNC_AUD_CON4                0x2396
#define MT6369_AFE_ADDA_MTKAIF_CFG0          0x23bc
#define MT6369_AFE_ADDA_MTKAIF_CFG1          0x23bd
#define MT6369_AFE_DCCLK_CFG0                0x23cd
#define MT6369_AFE_DCCLK_CFG1                0x23ce
#define MT6369_AFE_DCCLK_CFG2                0x23cf
#define MT6369_AUDIO_DIG_CFG0                0x23d0
#define MT6369_AUDIO_DIG_CFG1                0x23d1
#define MT6369_AUDIO_DIG_CFG2                0x23d2
#define MT6369_AFE_AUD_PAD_TOP               0x23d3
#define MT6369_AFE_DL_NLE_CFG                0x23d7
#define MT6369_AFE_MTKAIF_MUX_CFG0           0x23de
#define MT6369_AUDNCP_CLKDIV_CON0            0x248d
#define MT6369_AUDNCP_CLKDIV_CON1            0x248e
#define MT6369_AUDNCP_CLKDIV_CON2            0x248f
#define MT6369_AUDNCP_CLKDIV_CON3            0x2490
#define MT6369_AUDNCP_CLKDIV_CON4            0x2491
#define MT6369_AUDENC_ANA_CON0               0x2508
#define MT6369_AUDENC_ANA_CON1               0x2509
#define MT6369_AUDENC_ANA_CON2               0x250a
#define MT6369_AUDENC_ANA_CON3               0x250b
#define MT6369_AUDENC_ANA_CON6               0x250e
#define MT6369_AUDENC_ANA_CON10              0x2512
#define MT6369_AUDENC_ANA_CON11              0x2513
#define MT6369_AUDENC_ANA_CON12              0x2514
#define MT6369_AUDENC_ANA_CON17              0x2519
#define MT6369_AUDENC_ANA_CON18              0x251a
#define MT6369_AUDENC_ANA_CON19              0x251b
#define MT6369_AUDENC_ANA_CON20              0x251c
#define MT6369_AUDDEC_ANA_CON0               0x2588
#define MT6369_AUDDEC_ANA_CON1               0x2589
#define MT6369_AUDDEC_ANA_CON2               0x258a
#define MT6369_AUDDEC_ANA_CON3               0x258b
#define MT6369_AUDDEC_ANA_CON4               0x258c
#define MT6369_AUDDEC_ANA_CON5               0x258d
#define MT6369_AUDDEC_ANA_CON8               0x2590
#define MT6369_AUDDEC_ANA_CON10              0x2592
#define MT6369_AUDDEC_ANA_CON12              0x2594
#define MT6369_AUDDEC_ANA_CON13              0x2595
#define MT6369_AUDDEC_ANA_CON16              0x2598
#define MT6369_AUDDEC_ANA_CON17              0x2599
#define MT6369_AUDDEC_ANA_CON18              0x259a
#define MT6369_AUDDEC_ANA_CON19              0x259b
#define MT6369_AUDDEC_ANA_CON20              0x259c
#define MT6369_AUDDEC_ANA_CON21              0x259d
#define MT6369_AUDDEC_ANA_CON22              0x259e
#define MT6369_AUDDEC_ANA_CON23              0x259f
#define MT6369_AUDDEC_ANA_CON24              0x25a0
#define MT6369_AUDDEC_ANA_CON25              0x25a1
#define MT6369_AUDDEC_ANA_CON26              0x25a2
#define MT6369_AUDDEC_ANA_CON27              0x25a3
#define MT6369_ZCD_CON0                      0x2608
#define MT6369_ZCD_CON1                      0x2609
#define MT6369_ZCD_CON2                      0x260a
#define MT6369_ZCD_CON3                      0x260b
#define MT6369_ZCD_CON4                      0x260c
#define MT6369_ZCD_CON5                      0x260d

#define RG_AUDNCP_CK_PDN_SFT                     6
#define RG_AUDNCP_CK_PDN_MASK                    0x1
#define RG_AUDNCP_CK_PDN_MASK_SFT                (0x1 << 6)
#define RG_ZCD13M_CK_PDN_SFT                     5
#define RG_ZCD13M_CK_PDN_MASK                    0x1
#define RG_ZCD13M_CK_PDN_MASK_SFT                (0x1 << 5)
#define RG_AUDIF_CK_PDN_SFT                      2
#define RG_AUDIF_CK_PDN_MASK                     0x1
#define RG_AUDIF_CK_PDN_MASK_SFT                 (0x1 << 2)
#define RG_AUD_CK_PDN_SFT                        1
#define RG_AUD_CK_PDN_MASK                       0x1
#define RG_AUD_CK_PDN_MASK_SFT                   (0x1 << 1)
#define AFE_ON_SFT                               0
#define AFE_ON_MASK                              0x1
#define AFE_ON_MASK_SFT                          (0x1 << 0)
#define DL_2_SRC_ON_TMP_CTL_PRE_SFT              0
#define DL_2_SRC_ON_TMP_CTL_PRE_MASK             0x1
#define DL_2_SRC_ON_TMP_CTL_PRE_MASK_SFT         (0x1 << 0)
#define UL_SDM_3_LEVEL_CTL_SFT                   1
#define UL_SDM_3_LEVEL_CTL_MASK                  0x1
#define UL_SDM_3_LEVEL_CTL_MASK_SFT              (0x1 << 1)
#define UL_SRC_ON_TMP_CTL_SFT                    0
#define UL_SRC_ON_TMP_CTL_MASK                   0x1
#define UL_SRC_ON_TMP_CTL_MASK_SFT               (0x1 << 0)
#define PDN_AFE_CTL_SFT                          7
#define PDN_AFE_CTL_MASK                         0x1
#define PDN_AFE_CTL_MASK_SFT                     (0x1 << 7)
#define PDN_DAC_CTL_SFT                          6
#define PDN_DAC_CTL_MASK                         0x1
#define PDN_DAC_CTL_MASK_SFT                     (0x1 << 6)
#define PDN_ADC_CTL_SFT                          5
#define PDN_ADC_CTL_MASK                         0x1
#define PDN_ADC_CTL_MASK_SFT                     (0x1 << 5)
#define PDN_ADDA6_ADC_CTL_SFT                    4
#define PDN_ADDA6_ADC_CTL_MASK                   0x1
#define PDN_ADDA6_ADC_CTL_MASK_SFT               (0x1 << 4)
#define PWR_CLK_DIS_CTL_SFT                      2
#define PWR_CLK_DIS_CTL_MASK                     0x1
#define PWR_CLK_DIS_CTL_MASK_SFT                 (0x1 << 2)
#define PDN_AFE_DL_PREDIST_CTL_SFT               0
#define PDN_AFE_DL_PREDIST_CTL_MASK              0x1
#define PDN_AFE_DL_PREDIST_CTL_MASK_SFT          (0x1 << 0)
#define CCI_AFIFO_CLK_PWDB_SFT                   1
#define CCI_AFIFO_CLK_PWDB_MASK                  0x1
#define CCI_AFIFO_CLK_PWDB_MASK_SFT              (0x1 << 1)
#define RG_AUD_PAD_TOP_DAT_MISO_LOOPBACK_SFT     7
#define RG_AUD_PAD_TOP_DAT_MISO_LOOPBACK_MASK    0x1
#define RG_AUD_PAD_TOP_DAT_MISO_LOOPBACK_MASK_SFT (0x1 << 7)
#define RG_AUD_PAD_TOP_PHASE_MODE_SFT            0
#define RG_AUD_PAD_TOP_PHASE_MODE_MASK           0x7f
#define RG_AUD_PAD_TOP_PHASE_MODE_MASK_SFT       (0x7f << 0)
#define RG_AUD_PAD_TOP_DAT_MISO2_LOOPBACK_SFT    7
#define RG_AUD_PAD_TOP_DAT_MISO2_LOOPBACK_MASK   0x1
#define RG_AUD_PAD_TOP_DAT_MISO2_LOOPBACK_MASK_SFT (0x1 << 7)
#define RG_AUD_PAD_TOP_PHASE_MODE2_SFT           0
#define RG_AUD_PAD_TOP_PHASE_MODE2_MASK          0x7f
#define RG_AUD_PAD_TOP_PHASE_MODE2_MASK_SFT      (0x7f << 0)
#define RG_AUD_PAD_TOP_DAT_MISO3_LOOPBACK_SFT    7
#define RG_AUD_PAD_TOP_DAT_MISO3_LOOPBACK_MASK   0x1
#define RG_AUD_PAD_TOP_DAT_MISO3_LOOPBACK_MASK_SFT (0x1 << 7)
#define RG_AUD_PAD_TOP_PHASE_MODE3_SFT           0
#define RG_AUD_PAD_TOP_PHASE_MODE3_MASK          0x7f
#define RG_AUD_PAD_TOP_PHASE_MODE3_MASK_SFT      (0x7f << 0)
#define NLE_RCH_HPGAIN_SEL_SFT                   5
#define NLE_RCH_HPGAIN_SEL_MASK                  0x1
#define NLE_RCH_HPGAIN_SEL_MASK_SFT              (0x1 << 5)
#define NLE_LCH_HPGAIN_SEL_SFT                   2
#define NLE_LCH_HPGAIN_SEL_MASK                  0x1
#define NLE_LCH_HPGAIN_SEL_MASK_SFT              (0x1 << 2)
#define RG_ADDA_CH2_SEL_SFT                      2
#define RG_ADDA_CH2_SEL_MASK                     0x3
#define RG_ADDA_CH2_SEL_MASK_SFT                 (0x3 << 2)
#define RG_ADDA_CH1_SEL_SFT                      0
#define RG_ADDA_CH1_SEL_MASK                     0x3
#define RG_ADDA_CH1_SEL_MASK_SFT                 (0x3 << 0)
#define RG_DIVCKS_CHG_SFT                        0
#define RG_DIVCKS_CHG_MASK                       0x1
#define RG_DIVCKS_CHG_MASK_SFT                   (0x1 << 0)
#define RG_DIVCKS_ON_SFT                         0
#define RG_DIVCKS_ON_MASK                        0x1
#define RG_DIVCKS_ON_MASK_SFT                    (0x1 << 0)
#define RG_DIVCKS_PWD_NCP_SFT                    0
#define RG_DIVCKS_PWD_NCP_MASK                   0x1
#define RG_DIVCKS_PWD_NCP_MASK_SFT               (0x1 << 0)
#define RG_DIVCKS_PWD_NCP_ST_SEL_SFT             0
#define RG_DIVCKS_PWD_NCP_ST_SEL_MASK            0x3
#define RG_DIVCKS_PWD_NCP_ST_SEL_MASK_SFT        (0x3 << 0)
#define RG_AUDPREAMPLON_SFT                      0
#define RG_AUDPREAMPLON_MASK                     0x1
#define RG_AUDPREAMPLON_MASK_SFT                 (0x1 << 0)
#define RG_AUDPREAMPLDCCEN_SFT                   1
#define RG_AUDPREAMPLDCCEN_MASK                  0x1
#define RG_AUDPREAMPLDCCEN_MASK_SFT              (0x1 << 1)
#define RG_AUDPREAMPLDCPRECHARGE_SFT             2
#define RG_AUDPREAMPLDCPRECHARGE_MASK            0x1
#define RG_AUDPREAMPLDCPRECHARGE_MASK_SFT        (0x1 << 2)
#define RG_AUDPREAMPLINPUTSEL_SFT                6
#define RG_AUDPREAMPLINPUTSEL_MASK               0x3
#define RG_AUDPREAMPLINPUTSEL_MASK_SFT           (0x3 << 6)
#define RG_AUDPREAMPLGAIN_SFT                    0
#define RG_AUDPREAMPLGAIN_MASK                   0x7
#define RG_AUDPREAMPLGAIN_MASK_SFT               (0x7 << 0)
#define RG_AUDADCLPWRUP_SFT                      4
#define RG_AUDADCLPWRUP_MASK                     0x1
#define RG_AUDADCLPWRUP_MASK_SFT                 (0x1 << 4)
#define RG_AUDADCLINPUTSEL_SFT                   5
#define RG_AUDADCLINPUTSEL_MASK                  0x3
#define RG_AUDADCLINPUTSEL_MASK_SFT              (0x3 << 5)
#define RG_AUDPREAMPRON_SFT                      0
#define RG_AUDPREAMPRON_MASK                     0x1
#define RG_AUDPREAMPRON_MASK_SFT                 (0x1 << 0)
#define RG_AUDPREAMPRDCCEN_SFT                   1
#define RG_AUDPREAMPRDCCEN_MASK                  0x1
#define RG_AUDPREAMPRDCCEN_MASK_SFT              (0x1 << 1)
#define RG_AUDPREAMPRDCPRECHARGE_SFT             2
#define RG_AUDPREAMPRDCPRECHARGE_MASK            0x1
#define RG_AUDPREAMPRDCPRECHARGE_MASK_SFT        (0x1 << 2)
#define RG_AUDPREAMPRINPUTSEL_SFT                6
#define RG_AUDPREAMPRINPUTSEL_MASK               0x3
#define RG_AUDPREAMPRINPUTSEL_MASK_SFT           (0x3 << 6)
#define RG_AUDPREAMPRGAIN_SFT                    0
#define RG_AUDPREAMPRGAIN_MASK                   0x7
#define RG_AUDPREAMPRGAIN_MASK_SFT               (0x7 << 0)
#define RG_AUDADCRPWRUP_SFT                      4
#define RG_AUDADCRPWRUP_MASK                     0x1
#define RG_AUDADCRPWRUP_MASK_SFT                 (0x1 << 4)
#define RG_AUDADCRINPUTSEL_SFT                   5
#define RG_AUDADCRINPUTSEL_MASK                  0x3
#define RG_AUDADCRINPUTSEL_MASK_SFT              (0x3 << 5)
#define RG_AUDADCCLKSEL_SFT                      1
#define RG_AUDADCCLKSEL_MASK                     0x1
#define RG_AUDADCCLKSEL_MASK_SFT                 (0x1 << 1)
#define RG_AUDADCCLKSOURCE_SFT                   2
#define RG_AUDADCCLKSOURCE_MASK                  0x3
#define RG_AUDADCCLKSOURCE_MASK_SFT              (0x3 << 2)
#define RG_CLKSQ_EN_SFT                          0
#define RG_CLKSQ_EN_MASK                         0x1
#define RG_CLKSQ_EN_MASK_SFT                     (0x1 << 0)
#define RG_CLKSQ_IN_SEL_TEST_SFT                 1
#define RG_CLKSQ_IN_SEL_TEST_MASK                0x1
#define RG_CLKSQ_IN_SEL_TEST_MASK_SFT            (0x1 << 1)
#define RG_AUDPWDBMICBIAS0_SFT                   0
#define RG_AUDPWDBMICBIAS0_MASK                  0x1
#define RG_AUDPWDBMICBIAS0_MASK_SFT              (0x1 << 0)
#define RG_AUDMICBIAS0LOWPEN_SFT                 2
#define RG_AUDMICBIAS0LOWPEN_MASK                0x1
#define RG_AUDMICBIAS0LOWPEN_MASK_SFT            (0x1 << 2)
#define RG_AUDMICBIAS0VREF_SFT                   4
#define RG_AUDMICBIAS0VREF_MASK                  0x7
#define RG_AUDMICBIAS0VREF_MASK_SFT              (0x7 << 4)
#define RG_AUDPWDBMICBIAS1_SFT                   0
#define RG_AUDPWDBMICBIAS1_MASK                  0x1
#define RG_AUDPWDBMICBIAS1_MASK_SFT              (0x1 << 0)
#define RG_AUDMICBIAS1LOWPEN_SFT                 2
#define RG_AUDMICBIAS1LOWPEN_MASK                0x1
#define RG_AUDMICBIAS1LOWPEN_MASK_SFT            (0x1 << 2)
#define RG_AUDDACLPWRUP_VAUDP15_SFT              0
#define RG_AUDDACLPWRUP_VAUDP15_MASK             0x1
#define RG_AUDDACLPWRUP_VAUDP15_MASK_SFT         (0x1 << 0)
#define RG_AUDDACRPWRUP_VAUDP15_SFT              1
#define RG_AUDDACRPWRUP_VAUDP15_MASK             0x1
#define RG_AUDDACRPWRUP_VAUDP15_MASK_SFT         (0x1 << 1)
#define RG_AUD_DAC_PWR_UP_VA28_SFT               2
#define RG_AUD_DAC_PWR_UP_VA28_MASK              0x1
#define RG_AUD_DAC_PWR_UP_VA28_MASK_SFT          (0x1 << 2)
#define RG_AUD_DAC_PWL_UP_VA28_SFT               3
#define RG_AUD_DAC_PWL_UP_VA28_MASK              0x1
#define RG_AUD_DAC_PWL_UP_VA28_MASK_SFT          (0x1 << 3)
#define RG_AUDHPLMUXINPUTSEL_VAUDP15_SFT         0
#define RG_AUDHPLMUXINPUTSEL_VAUDP15_MASK        0x3
#define RG_AUDHPLMUXINPUTSEL_VAUDP15_MASK_SFT    (0x3 << 0)
#define RG_AUDHPRMUXINPUTSEL_VAUDP15_SFT         2
#define RG_AUDHPRMUXINPUTSEL_VAUDP15_MASK        0x3
#define RG_AUDHPRMUXINPUTSEL_VAUDP15_MASK_SFT    (0x3 << 2)
#define RG_HPLOUTSTGCTRL_VAUDP15_SFT             0
#define RG_HPLOUTSTGCTRL_VAUDP15_MASK            0x7
#define RG_HPLOUTSTGCTRL_VAUDP15_MASK_SFT        (0x7 << 0)
#define RG_HPROUTSTGCTRL_VAUDP15_SFT             4
#define RG_HPROUTSTGCTRL_VAUDP15_MASK            0x7
#define RG_HPROUTSTGCTRL_VAUDP15_MASK_SFT        (0x7 << 4)
#define RG_AUDREFN_DERES_EN_VAUDP15_SFT          0
#define RG_AUDREFN_DERES_EN_VAUDP15_MASK         0x1
#define RG_AUDREFN_DERES_EN_VAUDP15_MASK_SFT     (0x1 << 0)
#define RG_AUDHSPWRUP_VAUDP15_SFT                0
#define RG_AUDHSPWRUP_VAUDP15_MASK               0x1
#define RG_AUDHSPWRUP_VAUDP15_MASK_SFT           (0x1 << 0)
#define RG_AUDHSPWRUP_IBIAS_VAUDP15_SFT          1
#define RG_AUDHSPWRUP_IBIAS_VAUDP15_MASK         0x1
#define RG_AUDHSPWRUP_IBIAS_VAUDP15_MASK_SFT     (0x1 << 1)
#define RG_AUDHSMUXINPUTSEL_VAUDP15_SFT          2
#define RG_AUDHSMUXINPUTSEL_VAUDP15_MASK         0x3
#define RG_AUDHSMUXINPUTSEL_VAUDP15_MASK_SFT     (0x3 << 2)
#define RG_AUDHSSCDISABLE_VAUDP15_SFT            4
#define RG_AUDHSSCDISABLE_VAUDP15_MASK           0x1
#define RG_AUDHSSCDISABLE_VAUDP15_MASK_SFT       (0x1 << 4)
#define RG_AUDLOLPWRUP_VAUDP15_SFT               0
#define RG_AUDLOLPWRUP_VAUDP15_MASK              0x1
#define RG_AUDLOLPWRUP_VAUDP15_MASK_SFT          (0x1 << 0)
#define RG_AUDLOLPWRUP_IBIAS_VAUDP15_SFT         1
#define RG_AUDLOLPWRUP_IBIAS_VAUDP15_MASK        0x1
#define RG_AUDLOLPWRUP_IBIAS_VAUDP15_MASK_SFT    (0x1 << 1)
#define RG_AUDLOLMUXINPUTSEL_VAUDP15_SFT         2
#define RG_AUDLOLMUXINPUTSEL_VAUDP15_MASK        0x3
#define RG_AUDLOLMUXINPUTSEL_VAUDP15_MASK_SFT    (0x3 << 2)
#define RG_AUDLOLSCDISABLE_VAUDP15_SFT           4
#define RG_AUDLOLSCDISABLE_VAUDP15_MASK          0x1
#define RG_AUDLOLSCDISABLE_VAUDP15_MASK_SFT      (0x1 << 4)
#define RG_AUDZCDMUXSEL_VAUDP15_SFT              0
#define RG_AUDZCDMUXSEL_VAUDP15_MASK             0x7
#define RG_AUDZCDMUXSEL_VAUDP15_MASK_SFT         (0x7 << 0)
#define RG_AUDBIASADJ_0_HP_VAUDP15_SFT           0
#define RG_AUDBIASADJ_0_HS_VAUDP15_SFT           4
#define RG_AUDBIASADJ_0_LO_VAUDP15_SFT           0
#define RG_AUDIBIASPWRDN_VAUDP15_SFT             4
#define RG_AUDIBIASPWRDN_VAUDP15_MASK            0x1
#define RG_AUDIBIASPWRDN_VAUDP15_MASK_SFT        (0x1 << 4)
#define RG_AUDBIASADJ_1_VAUDP15_SFT              0
#define RG_RSTB_DECODER_VA28_SFT                 0
#define RG_RSTB_DECODER_VA28_MASK                0x1
#define RG_RSTB_DECODER_VA28_MASK_SFT            (0x1 << 0)
#define RG_AUDGLB_PWRDN_VA28_SFT                 4
#define RG_AUDGLB_PWRDN_VA28_MASK                0x1
#define RG_AUDGLB_PWRDN_VA28_MASK_SFT            (0x1 << 4)
#define RG_RSTB_ENCODER_VA28_SFT                 5
#define RG_RSTB_ENCODER_VA28_MASK                0x1
#define RG_RSTB_ENCODER_VA28_MASK_SFT            (0x1 << 5)
#define RG_HCLDO_EN_VA18_SFT                     0
#define RG_HCLDO_EN_VA18_MASK                    0x1
#define RG_HCLDO_EN_VA18_MASK_SFT                (0x1 << 0)
#define RG_HCLDO_REMOTE_SENSE_VA18_SFT           2
#define RG_HCLDO_REMOTE_SENSE_VA18_MASK          0x1
#define RG_HCLDO_REMOTE_SENSE_VA18_MASK_SFT      (0x1 << 2)
#define RG_LCLDO_EN_VA18_SFT                     4
#define RG_LCLDO_EN_VA18_MASK                    0x1
#define RG_LCLDO_EN_VA18_MASK_SFT                (0x1 << 4)
#define RG_LCLDO_REMOTE_SENSE_VA18_SFT           6
#define RG_LCLDO_REMOTE_SENSE_VA18_MASK          0x1
#define RG_LCLDO_REMOTE_SENSE_VA18_MASK_SFT      (0x1 << 6)
#define RG_LCLDO_ENC_EN_VA28_SFT                 0
#define RG_LCLDO_ENC_EN_VA28_MASK                0x1
#define RG_LCLDO_ENC_EN_VA28_MASK_SFT            (0x1 << 0)
#define RG_LCLDO_ENC_REMOTE_SENSE_VA28_SFT       2
#define RG_LCLDO_ENC_REMOTE_SENSE_VA28_MASK      0x1
#define RG_LCLDO_ENC_REMOTE_SENSE_VA28_MASK_SFT  (0x1 << 2)
#define RG_VA33REFGEN_EN_VA18_SFT                4
#define RG_VA33REFGEN_EN_VA18_MASK               0x1
#define RG_VA33REFGEN_EN_VA18_MASK_SFT           (0x1 << 4)
#define RG_VA28REFGEN_EN_VA28_SFT                5
#define RG_VA28REFGEN_EN_VA28_MASK               0x1
#define RG_VA28REFGEN_EN_VA28_MASK_SFT           (0x1 << 5)
#define RG_NVREG_EN_VAUDP15_SFT                  0
#define RG_NVREG_EN_VAUDP15_MASK                 0x1
#define RG_NVREG_EN_VAUDP15_MASK_SFT             (0x1 << 0)
#define RG_AUDZCDENABLE_SFT                      0
#define RG_AUDZCDENABLE_MASK                     0x1
#define RG_AUDZCDENABLE_MASK_SFT                 (0x1 << 0)
#define RG_AUDZCDGAINSTEPTIME_SFT                1
#define RG_AUDZCDGAINSTEPTIME_MASK               0x7
#define RG_AUDZCDGAINSTEPTIME_MASK_SFT           (0x7 << 1)
#define RG_AUDZCDGAINSTEPSIZE_SFT                4
#define RG_AUDZCDGAINSTEPSIZE_MASK               0x3
#define RG_AUDZCDGAINSTEPSIZE_MASK_SFT           (0x3 << 4)
#define RG_AUDZCDTIMEOUTMODESEL_SFT              6
#define RG_AUDZCDTIMEOUTMODESEL_MASK             0x1
#define RG_AUDZCDTIMEOUTMODESEL_MASK_SFT         (0x1 << 6)
#define DRBIAS_MASK                              0x7
#define DRBIAS_HP_SFT                            (RG_AUDBIASADJ_0_HP_VAUDP15_SFT)
#define DRBIAS_HP_MASK_SFT                       (DRBIAS_MASK << DRBIAS_HP_SFT)
#define DRBIAS_HS_SFT                            (RG_AUDBIASADJ_0_HS_VAUDP15_SFT)
#define DRBIAS_HS_MASK_SFT                       (DRBIAS_MASK << DRBIAS_HS_SFT)
#define DRBIAS_LO_SFT                            (RG_AUDBIASADJ_0_LO_VAUDP15_SFT)
#define DRBIAS_LO_MASK_SFT                       (DRBIAS_MASK << DRBIAS_LO_SFT)
#define IBIAS_MASK                               0x3
#define IBIAS_HP_SFT                             (RG_AUDBIASADJ_1_VAUDP15_SFT + 0)
#define IBIAS_HP_MASK_SFT                        (IBIAS_MASK << IBIAS_HP_SFT)
#define IBIAS_HS_SFT                             (RG_AUDBIASADJ_1_VAUDP15_SFT + 2)
#define IBIAS_HS_MASK_SFT                        (IBIAS_MASK << IBIAS_HS_SFT)
#define IBIAS_LO_SFT                             (RG_AUDBIASADJ_1_VAUDP15_SFT + 4)
#define IBIAS_LO_MASK_SFT                        (IBIAS_MASK << IBIAS_LO_SFT)
#define IBIAS_ZCD_SFT                            (RG_AUDBIASADJ_1_VAUDP15_SFT + 6)
#define IBIAS_ZCD_MASK_SFT                       (IBIAS_MASK << IBIAS_ZCD_SFT)
#define CODEC_MT6369_NAME                        "mtk-codec-mt6369"

#define CODEC_MT6369_NAME "mtk-codec-mt6369"

enum {
	MT6369_MTKAIF_PROTOCOL_1 = 0,
	MT6369_MTKAIF_PROTOCOL_2,
	MT6369_MTKAIF_PROTOCOL_2_CLK_P2,
};

enum {
	MT6369_AIF_1 = 0,	/* dl: hp, rcv, lo; ul: adc l/r */
	MT6369_AIF_NUM,
};

enum {
	AUDIO_ANALOG_VOLUME_HSOUTL,
	AUDIO_ANALOG_VOLUME_HSOUTR,
	AUDIO_ANALOG_VOLUME_HPOUTL,
	AUDIO_ANALOG_VOLUME_HPOUTR,
	AUDIO_ANALOG_VOLUME_LINEOUTL,
	AUDIO_ANALOG_VOLUME_LINEOUTR,
	AUDIO_ANALOG_VOLUME_TYPE_MAX
};

enum {
	MUX_MIC_TYPE_0, /* ain0, micbias 0 */
	MUX_MIC_TYPE_1, /* ain1, micbias 1 */
	MUX_MIC_TYPE_2, /* ain2, micbias 0 */
	MUX_PGA_L,
	MUX_PGA_R,
	MUX_HP_L,
	MUX_NUM,
};

enum {
	DEVICE_HP,
	DEVICE_LO,
	DEVICE_RCV,
	DEVICE_NUM
};

enum {
	HP_GAIN_CTL_ZCD = 0,
	HP_GAIN_CTL_NLE,
};

/* Supply widget subseq */
enum {
	/* common */
	SUPPLY_SEQ_AUD_GLB,
	SUPPLY_SEQ_DL_GPIO,
	SUPPLY_SEQ_UL_GPIO,
	SUPPLY_SEQ_CLKSQ,
	SUPPLY_SEQ_ADC_CLKGEN,
	SUPPLY_SEQ_TOP_CK,
	SUPPLY_SEQ_TOP_CK_LAST,
	SUPPLY_SEQ_DCC_CLK,
	SUPPLY_SEQ_MIC_BIAS,
	SUPPLY_SEQ_AUD_TOP,
	SUPPLY_SEQ_AUD_TOP_LAST,
	SUPPLY_SEQ_DL_SDM_FIFO_CLK,
	SUPPLY_SEQ_DL_SDM,
	SUPPLY_SEQ_DL_NCP,
	SUPPLY_SEQ_AFE,
	/* playback */
	SUPPLY_SEQ_DL_SRC,
	SUPPLY_SEQ_DL_ESD_RESIST,
	SUPPLY_SEQ_HP_MUTE,
	SUPPLY_SEQ_DL_LDO_VA33REFGEN,
	SUPPLY_SEQ_DL_LCLDO_REMOTE_SENSE,
	SUPPLY_SEQ_DL_HCLDO_REMOTE_SENSE,
	SUPPLY_SEQ_DL_LCLDO,
	SUPPLY_SEQ_DL_HCLDO,
	SUPPLY_SEQ_DL_NV,
	SUPPLY_SEQ_DL_IBIST,
	/* capture */
	SUPPLY_SEQ_UL_PGA,
	SUPPLY_SEQ_UL_ADC,
	SUPPLY_SEQ_UL_MTKAIF,
	SUPPLY_SEQ_UL_SRC,
};

enum {
	DRBIAS_4UA = 0,
	DRBIAS_5UA,
	DRBIAS_6UA,
};

enum {
	IBIAS_4UA = 0,
	IBIAS_5UA,
};

enum {
	IBIAS_ZCD_3UA = 0,
	IBIAS_ZCD_4UA,
};

enum {
	MIC_BIAS_1P7 = 0,
	MIC_BIAS_1P8,
	MIC_BIAS_1P9,
};

/* ZCD gain register index: 1 dB per step, 0x13...0x1e unused */
enum {
	DL_GAIN_8DB = 0,
	DL_GAIN_0DB = 8,
	DL_GAIN_N_10DB = 18,
	DL_GAIN_N_40DB = 0x1f,
};

/* values of the "mediatek,mic-type" DT property */
enum {
	MIC_TYPE_MUX_IDLE = 0,
	MIC_TYPE_MUX_ACC,
	MIC_TYPE_MUX_DMIC,
	MIC_TYPE_MUX_DCC,
	MIC_TYPE_MUX_DCC_ECM_DIFF,
	MIC_TYPE_MUX_DCC_ECM_SINGLE,
};

#define IS_DCC_BASE(x) ((x) == MIC_TYPE_MUX_DCC || \
			(x) == MIC_TYPE_MUX_DCC_ECM_DIFF || \
			(x) == MIC_TYPE_MUX_DCC_ECM_SINGLE)

enum {
	LO_MUX_OPEN = 0,
	LO_MUX_L_DAC,
	LO_MUX_3RD_DAC,
	LO_MUX_TEST_MODE,
	LO_MUX_MASK = 0x3,
};

enum {
	HP_MUX_OPEN = 0,
	HP_MUX_HPSPK,
	HP_MUX_HP,
	HP_MUX_MASK = 0x7,
};

enum {
	RCV_MUX_OPEN = 0,
	RCV_MUX_MUTE,
	RCV_MUX_VOICE_PLAYBACK,
	RCV_MUX_MASK = 0x3,
};

enum {
	PGA_MUX_NONE = 0,
	PGA_MUX_AIN0,
	PGA_MUX_AIN1,
	PGA_MUX_AIN2,
};

enum {
	UL_SRC_MUX_AMIC = 0,
	UL_SRC_MUX_DMIC,
};

enum {
	MISO_MUX_UL1_CH1 = 0,
	MISO_MUX_UL1_CH2,
	MISO_MUX_UL2_CH1,
	MISO_MUX_UL2_CH2,
};

enum {
	ADC_MUX_IDLE = 0,
	ADC_MUX_AIN0,
	ADC_MUX_PREAMPLIFIER,
	ADC_MUX_IDLE1,
};

void mt6369_set_mtkaif_protocol(struct snd_soc_component *cmpnt,
				int mtkaif_protocol);
void mt6369_mtkaif_calibration_enable(struct snd_soc_component *cmpnt);
void mt6369_mtkaif_calibration_disable(struct snd_soc_component *cmpnt);
void mt6369_set_mtkaif_calibration_phase(struct snd_soc_component *cmpnt,
					 int phase_1, int phase_2, int phase_3);

#endif /* _MT6369_H_ */
