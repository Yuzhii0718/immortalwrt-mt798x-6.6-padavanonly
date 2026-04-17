/* SPDX-License-Identifier: GPL-2.0 */
/*************************************************
 * FILE NAME:  air_en8811h.h
 * PURPOSE:
 *      EN8811H PHY Driver for Linux
 * NOTES:
 *
 *  Copyright (C) 2023 Airoha Technology Corp.
 *************************************************/
#ifndef __EN8811H_H
#define __EN8811H_H

#define MAX_RETRY                   25

#define EN8811H_DRIVER_VERSION  "v2.0.7"


/***************************************************************
 * The following led_cfg example is for reference only.
 * LED0 Link 2500/Blink 2500 TxRx   (GPIO5)    <-> BASE_T_LED0,
 * LED1 Link 1000/Blink 1000 TxRx   (GPIO4)    <-> BASE_T_LED1,
 * LED2 Link 100 /Blink 100  TxRx   (GPIO3)    <-> BASE_T_LED2,
 ***************************************************************/
/* User-defined.B */
#define AIR_LED0_ON     (LED_ON_EVT_LINK_2500M)
#define AIR_LED0_BLK    (LED_BLK_EVT_2500M_TX_ACT | LED_BLK_EVT_2500M_RX_ACT)
#define AIR_LED1_ON     (LED_ON_EVT_LINK_1000M)
#define AIR_LED1_BLK    (LED_BLK_EVT_1000M_TX_ACT | LED_BLK_EVT_1000M_RX_ACT)
#define AIR_LED2_ON     (LED_ON_EVT_LINK_100M)
#define AIR_LED2_BLK    (LED_BLK_EVT_100M_TX_ACT | LED_BLK_EVT_100M_RX_ACT)
/* User-defined.E */

/* CL45 MDIO control */
#define MII_MMD_ACC_CTL_REG         0x0d
#define MII_MMD_ADDR_DATA_REG       0x0e
#define MMD_OP_MODE_DATA            BIT(14)

#define LED_ON_CTRL(i)              (0x024 + ((i)*2))
#define LED_ON_EN                   (1 << 15)
#define LED_ON_POL                  (1 << 14)
#define LED_ON_EVT_MASK             (0x1ff)
/* LED ON Event Option.B */
#define LED_ON_EVT_LINK_2500M       (1 << 8)
#define LED_ON_EVT_FORCE            (1 << 6)
#define LED_ON_EVT_LINK_DOWN        (1 << 3)
#define LED_ON_EVT_LINK_100M        (1 << 1)
#define LED_ON_EVT_LINK_1000M       (1 << 0)
/* LED ON Event Option.E */

#define LED_BLK_CTRL(i)             (0x025 + ((i)*2))
#define LED_BLK_EVT_MASK            (0xfff)
/* LED Blinking Event Option.B*/
#define LED_BLK_EVT_2500M_RX_ACT    (1 << 11)
#define LED_BLK_EVT_2500M_TX_ACT    (1 << 10)
#define LED_BLK_EVT_FORCE           (1 << 9)
#define LED_BLK_EVT_100M_RX_ACT     (1 << 3)
#define LED_BLK_EVT_100M_TX_ACT     (1 << 2)
#define LED_BLK_EVT_1000M_RX_ACT    (1 << 1)
#define LED_BLK_EVT_1000M_TX_ACT    (1 << 0)
/* LED Blinking Event Option.E*/
#define EN8811H_LED_COUNT           3

#define LED_BCR                     (0x021)
#define LED_BCR_EXT_CTRL            (1 << 15)
#define LED_BCR_CLK_EN              (1 << 3)
#define LED_BCR_TIME_TEST           (1 << 2)
#define LED_BCR_MODE_MASK           (3)
#define LED_BCR_MODE_DISABLE        (0)

#define LED_ON_DUR                  (0x022)
#define LED_ON_DUR_MASK             (0xffff)

#define LED_BLK_DUR                 (0x023)
#define LED_BLK_DUR_MASK            (0xffff)

#define UNIT_LED_BLINK_DURATION     781

#define GET_BIT(val, bit) ((val & BIT(bit)) >> bit)

#define INVALID_DATA                0xffff
#define PBUS_INVALID_DATA           0xffffffff
#define EN8811H_DEFAULT_ADDR  0xf

struct air_cable_test_rsl {
	int status[4];
	unsigned int length[4];
};

struct air_base_t_led_cfg {
	u16 en;
	u16 gpio;
	u16 pol;
	u16 on_cfg;
	u16 blk_cfg;
};
enum air_led_gpio {
	AIR_LED2_GPIO3 = 3,
	AIR_LED1_GPIO4,
	AIR_LED0_GPIO5,
	AIR_LED_LAST
};

enum air_base_t_led {
	AIR_BASE_T_LED0,
	AIR_BASE_T_LED1,
	AIR_BASE_T_LED2,
	AIR_BASE_T_LED3
};

enum air_led_blk_dur {
	AIR_LED_BLK_DUR_32M,
	AIR_LED_BLK_DUR_64M,
	AIR_LED_BLK_DUR_128M,
	AIR_LED_BLK_DUR_256M,
	AIR_LED_BLK_DUR_512M,
	AIR_LED_BLK_DUR_1024M,
	AIR_LED_BLK_DUR_LAST
};

#define DEBUGFS_COUNTER		        "counter"
#define DEBUGFS_DRIVER_INFO	        "drvinfo"
#define DEBUGFS_PORT_MODE           "port_mode"
#define DEBUGFS_BUCKPBUS_OP         "buckpbus_op"
#define DEBUGFS_POLARITY            "polarity"
#define DEBUGFS_LINK_STATUS         "link_status"
#define DEBUGFS_DBG_REG_SHOW        "dbg_regs_show"
#define DEBUGFS_TEMPERATURE         "temp"
#define DEBUGFS_LP_SPEED            "lp_speed"
#define DEBUGFS_MII_CL22_OP         "cl22_op"
#define DEBUGFS_MII_CL45_OP         "cl45_op"
#define DEBUGFS_CABLE_DIAG          "cable_diag"
#define DEBUGFS_LED_MODE            "led_mode"

#define CMD_MAX_LENGTH 128

/* bits range: for example AIR_BITS_RANGE(16, 4) = 0x0F0000 */
#ifndef AIR_BITS_RANGE
#define AIR_BITS_RANGE(offset, range) GENMASK((offset) + (range) - 1U, (offset))
#endif /* End of AIR_BITS_RANGE */

/* bits offset right: for example AIR_BITS_OFF_R(0x1234, 8, 4) = 0x2 */
#ifndef AIR_BITS_OFF_R
#define AIR_BITS_OFF_R(val, offset, range) (((val) >> (offset)) & GENMASK((range) - 1U, 0))
#endif /* End of AIR_BITS_OFF_R */

/* bits offset left: for example AIR_BITS_OFF_L(0x1234, 8, 4) = 0x400 */
#ifndef AIR_BITS_OFF_L
#define AIR_BITS_OFF_L(val, offset, range) (((val) & GENMASK((range) - 1U, 0)) << (offset))
#endif /* End of AIR_BITS_OFF_L */

#define AIR_EN8811H_SET_VALUE(__out__, __val__, __offset__, __length__)	\
{							\
	(__out__) &= ~AIR_BITS_RANGE((__offset__), (__length__));			\
	(__out__) |= AIR_BITS_OFF_L((__val__), (__offset__), (__length__));	\
}

enum air_port_mode {
	AIR_PORT_MODE_FORCE_100,
	AIR_PORT_MODE_FORCE_1000,
	AIR_PORT_MODE_FORCE_2500,
	AIR_PORT_MODE_AUTONEGO,
	AIR_PORT_MODE_POWER_DOWN,
	AIR_PORT_MODE_POWER_UP,
	AIR_PORT_MODE_LAST = 0xFF,
};

enum air_polarity {
	AIR_POL_TX_REV_RX_NOR,
	AIR_POL_TX_NOR_RX_NOR,
	AIR_POL_TX_REV_RX_REV,
	AIR_POL_TX_NOR_RX_REV,
	AIR_POL_TX_NOR_RX_LAST = 0xff,
};

/* Link mode bit indices */
enum air_link_mode_bit {
	AIR_LINK_MODE_10baseT_Half_BIT	 = 0,
	AIR_LINK_MODE_10baseT_Full_BIT	 = 1,
	AIR_LINK_MODE_100baseT_Half_BIT	 = 2,
	AIR_LINK_MODE_100baseT_Full_BIT	 = 3,
	AIR_LINK_MODE_1000baseT_Full_BIT = 4,
	AIR_LINK_MODE_2500baseT_Full_BIT = 5,
};

enum air_led_force {
	AIR_LED_NORMAL = 0,
	AIR_LED_FORCE_OFF,
	AIR_LED_FORCE_ON,
	AIR_LED_FORCE_LAST = 0xff,
};

enum air_para {
	AIR_PARA_PRIV,
	AIR_PARA_PHYDEV,
	AIR_PARA_LAST = 0xff
};

enum air_port_cable_status {
	AIR_PORT_CABLE_STATUS_ERROR,
	AIR_PORT_CABLE_STATUS_OPEN,
	AIR_PORT_CABLE_STATUS_SHORT,
	AIR_PORT_CABLE_STATUS_NORMAL,
	AIR_PORT_CABLE_STATUS_LAST = 0xff
};

enum air_surge {
	AIR_SURGE_0R,
	AIR_SURGE_5R,
	AIR_SURGE_LAST = 0xff
};

enum air_port_cable_test_pair {
	AIR_PORT_CABLE_TEST_PAIR_A,
	AIR_PORT_CABLE_TEST_PAIR_B,
	AIR_PORT_CABLE_TEST_PAIR_C,
	AIR_PORT_CABLE_TEST_PAIR_D,
	AIR_PORT_CABLE_TEST_PAIR_ALL,
	AIR_PORT_CABLE_TEST_PAIR_LAST
};

enum air_cko {
	AIR_CKO_DIS,
	AIR_CKO_EN,
	AIR_CKO_LAST = 0xff
};

struct trrg_param_s {
	unsigned int TrRG_LSB     :5;
	unsigned int Reserved_21  :3;
	unsigned int TrRG_MSB     :5;
	unsigned int Reserved_29  :3;
	unsigned int Reserved_0   :1;
	unsigned int DATA_ADDR    :6;
	unsigned int NODE_ADDR    :4;
	unsigned int CH_ADDR      :2;
	unsigned int WR_RD_CTRL   :1;
	unsigned int Reserved_14  :1;
	unsigned int PKT_XMT_STA  :1;
};

union trrgdesc_s {
	struct trrg_param_s param;
	unsigned short     Raw[2];
	unsigned int       DescVal;
};

struct trrg_s {
	union trrgdesc_s TrRGDesc;
	unsigned int     RgMask;
};

struct hal_tr_data_s {
	unsigned short data_lo;
	unsigned char data_hi;
};

#ifndef unlikely
#  define unlikely(x)	(x)
#endif

#endif /* End of __EN8811H_H */
