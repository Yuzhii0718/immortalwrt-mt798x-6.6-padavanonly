// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for the Airoha EN8811H 2.5 Gigabit PHY.
 *
 * Limitations of the EN8811H:
 * - Only full duplex supported
 * - Forced speed (AN off) is not supported by hardware (100Mbps)
 *
 * Source originated from airoha's en8811h.c and en8811h.h v1.2.1
 *
 * Copyright (C) 2023 Airoha Technology Corp.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/phy.h>
#include <linux/firmware.h>
#include <linux/property.h>
#include <linux/wordpart.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0))
	#include <asm/unaligned.h>
#else
	#include <linux/unaligned.h>
#endif
#include <linux/crc32.h>
#include <linux/debugfs.h>
#include "air_en8811h.h"

#define EN8811H_PHY_ID		0x03a2a411

#define EN8811H_MD32_DM		"airoha/EthMD32.dm.bin"
#define EN8811H_MD32_DSP	"airoha/EthMD32.DSP.bin"

#define AIR_FW_ADDR_DM	0x00000000
#define AIR_FW_ADDR_DSP	0x00100000

/* MII Registers */
#define AIR_AUX_CTRL_STATUS		0x1d
#define   AIR_AUX_CTRL_STATUS_SPEED_MASK	GENMASK(4, 2)
#define   AIR_AUX_CTRL_STATUS_SPEED_100		0x4
#define   AIR_AUX_CTRL_STATUS_SPEED_1000	0x8
#define   AIR_AUX_CTRL_STATUS_SPEED_2500	0xc

#define AIR_EXT_PAGE_ACCESS		0x1f
#define   AIR_PHY_PAGE_STANDARD			0x0000
#define   AIR_PHY_PAGE_EXTENDED_4		0x0004

/* MII Registers Page 4*/
#define AIR_BPBUS_MODE			0x10
#define   AIR_BPBUS_MODE_ADDR_FIXED		0x0000
#define   AIR_BPBUS_MODE_ADDR_INCR		BIT(15)
#define AIR_BPBUS_WR_ADDR_HIGH		0x11
#define AIR_BPBUS_WR_ADDR_LOW		0x12
#define AIR_BPBUS_WR_DATA_HIGH		0x13
#define AIR_BPBUS_WR_DATA_LOW		0x14
#define AIR_BPBUS_RD_ADDR_HIGH		0x15
#define AIR_BPBUS_RD_ADDR_LOW		0x16
#define AIR_BPBUS_RD_DATA_HIGH		0x17
#define AIR_BPBUS_RD_DATA_LOW		0x18

/* Registers on MDIO_MMD_VEND1 */
#define EN8811H_PHY_FW_STATUS		0x8009
#define   EN8811H_PHY_READY			0x02

#define AIR_PHY_MCU_CMD_1		0x800c
#define AIR_PHY_MCU_CMD_1_MODE1			0x0
#define AIR_PHY_MCU_CMD_2		0x800d
#define AIR_PHY_MCU_CMD_2_MODE1			0x0
#define AIR_PHY_MCU_CMD_3		0x800e
#define AIR_PHY_MCU_CMD_3_MODE1			0x1101
#define AIR_PHY_MCU_CMD_3_DOCMD			0x1100
#define AIR_PHY_MCU_CMD_4		0x800f
#define AIR_PHY_MCU_CMD_4_MODE1			0x0002
#define AIR_PHY_MCU_CMD_4_INTCLR		0x00e4

/* Registers on MDIO_MMD_VEND2 */
#define AIR_PHY_LED_BCR			0x021
#define   AIR_PHY_LED_BCR_MODE_MASK		GENMASK(1, 0)
#define   AIR_PHY_LED_BCR_TIME_TEST		BIT(2)
#define   AIR_PHY_LED_BCR_CLK_EN		BIT(3)
#define   AIR_PHY_LED_BCR_EXT_CTRL		BIT(15)

#define AIR_PHY_LED_DUR_ON		0x022

#define AIR_PHY_LED_DUR_BLINK		0x023

#define AIR_PHY_LED_ON(i)	       (0x024 + ((i) * 2))
#define   AIR_PHY_LED_ON_MASK			(GENMASK(6, 0) | BIT(8))
#define   AIR_PHY_LED_ON_LINK1000		BIT(0)
#define   AIR_PHY_LED_ON_LINK100		BIT(1)
#define   AIR_PHY_LED_ON_LINK10			BIT(2)
#define   AIR_PHY_LED_ON_LINKDOWN		BIT(3)
#define   AIR_PHY_LED_ON_FDX			BIT(4) /* Full duplex */
#define   AIR_PHY_LED_ON_HDX			BIT(5) /* Half duplex */
#define   AIR_PHY_LED_ON_FORCE_ON		BIT(6)
#define   AIR_PHY_LED_ON_LINK2500		BIT(8)
#define   AIR_PHY_LED_ON_POLARITY		BIT(14)
#define   AIR_PHY_LED_ON_ENABLE			BIT(15)

#define AIR_PHY_LED_BLINK(i)	       (0x025 + ((i) * 2))
#define   AIR_PHY_LED_BLINK_1000TX		BIT(0)
#define   AIR_PHY_LED_BLINK_1000RX		BIT(1)
#define   AIR_PHY_LED_BLINK_100TX		BIT(2)
#define   AIR_PHY_LED_BLINK_100RX		BIT(3)
#define   AIR_PHY_LED_BLINK_10TX		BIT(4)
#define   AIR_PHY_LED_BLINK_10RX		BIT(5)
#define   AIR_PHY_LED_BLINK_COLLISION		BIT(6)
#define   AIR_PHY_LED_BLINK_RX_CRC_ERR		BIT(7)
#define   AIR_PHY_LED_BLINK_RX_IDLE_ERR		BIT(8)
#define   AIR_PHY_LED_BLINK_FORCE_BLINK		BIT(9)
#define   AIR_PHY_LED_BLINK_2500TX		BIT(10)
#define   AIR_PHY_LED_BLINK_2500RX		BIT(11)

/* Registers on BUCKPBUS */
#define EN8811H_CONTROL			0x3a9c
#define   EN8811H_CONTROL_INTERNAL		BIT(11)

#define EN8811H_2P5G_LPA		0x3b30
#define   EN8811H_2P5G_LPA_2P5G			BIT(0)

#define EN8811H_FW_VERSION		0x3b3c

#define EN8811H_POLARITY		0xca0f8
#define   EN8811H_POLARITY_TX_NORMAL		BIT(0)
#define   EN8811H_POLARITY_RX_REVERSE		BIT(1)

#define EN8811H_GPIO_OUTPUT		0xcf8b8
#define   EN8811H_GPIO_OUTPUT_345		(BIT(3) | BIT(4) | BIT(5))

#define EN8811H_HWTRAP1		0xcf914
#define EN8811H_HWTRAP1_CKO		BIT(12)

#define EN8811H_CLK_CGM		0xcf958
#define EN8811H_CLK_CGM_CKO		BIT(26)

#define EN8811H_FW_CTRL_1		0x0f0018
#define   EN8811H_FW_CTRL_1_START		0x0
#define   EN8811H_FW_CTRL_1_FINISH		0x1
#define EN8811H_FW_CTRL_2		0x800000
#define EN8811H_FW_CTRL_2_LOADING		BIT(11)

/* Led definitions */
#define EN8811H_LED_COUNT	3

/* Default LED setup:
 * GPIO5 <-> LED0  On: Link detected, blink Rx/Tx
 * GPIO4 <-> LED1  On: Link detected at 2500 and 1000 Mbps
 * GPIO3 <-> LED2  On: Link detected at 2500 and  100 Mbps
 */
#define AIR_DEFAULT_TRIGGER_LED0 (BIT(TRIGGER_NETDEV_LINK)      | \
				  BIT(TRIGGER_NETDEV_RX)        | \
				  BIT(TRIGGER_NETDEV_TX))
#define AIR_DEFAULT_TRIGGER_LED1 (BIT(TRIGGER_NETDEV_LINK_2500) | \
				  BIT(TRIGGER_NETDEV_LINK_1000))
#define AIR_DEFAULT_TRIGGER_LED2 (BIT(TRIGGER_NETDEV_LINK_2500) | \
				  BIT(TRIGGER_NETDEV_LINK_100))

struct led {
	unsigned long rules;
	unsigned long state;
};

#define clk_hw_to_en8811h_priv(_hw)			\
	container_of(_hw, struct en8811h_priv, hw)

struct en8811h_priv {
	u32			firmware_version;
	bool			mcu_needs_restart;
	struct led		led[EN8811H_LED_COUNT];
	bool			led_hw_control;
	struct clk_hw		hw;
	struct phy_device	*phydev;
	bool			restart_mcu;
	unsigned int		dm_crc32;
	unsigned int		dsp_crc32;
	char			buf[512];
	int			pol;
	int			surge;
	int			hw_addr;
	int			dts_addr;
	struct dentry		*debugfs_root;
	int			running_status;
	int			pair[4];
	int			an;
	int			link;
	int			speed;
	int			duplex;
	int			pause;
	int			asym_pause;
	u16			on_crtl[3];
	u16			blk_crtl[3];
	int			need_an;
	unsigned int		cko_is_enabled;
};

enum {
	AIR_PHY_LED_STATE_FORCE_ON,
	AIR_PHY_LED_STATE_FORCE_BLINK,
};

enum {
	AIR_PHY_LED_DUR_BLINK_32MS,
	AIR_PHY_LED_DUR_BLINK_64MS,
	AIR_PHY_LED_DUR_BLINK_128MS,
	AIR_PHY_LED_DUR_BLINK_256MS,
	AIR_PHY_LED_DUR_BLINK_512MS,
	AIR_PHY_LED_DUR_BLINK_1024MS,
};

enum {
	AIR_LED_DISABLE,
	AIR_LED_ENABLE,
};

enum {
	AIR_ACTIVE_LOW,
	AIR_ACTIVE_HIGH,
};

enum {
	AIR_LED_MODE_DISABLE,
	AIR_LED_MODE_USER_DEFINE,
};

#define AIR_PHY_LED_DUR_UNIT	781
#define AIR_PHY_LED_DUR (AIR_PHY_LED_DUR_UNIT << AIR_PHY_LED_DUR_BLINK_64MS)

static const unsigned long en8811h_led_trig = BIT(TRIGGER_NETDEV_FULL_DUPLEX) |
					      BIT(TRIGGER_NETDEV_LINK)        |
					      BIT(TRIGGER_NETDEV_LINK_10)     |
					      BIT(TRIGGER_NETDEV_LINK_100)    |
					      BIT(TRIGGER_NETDEV_LINK_1000)   |
					      BIT(TRIGGER_NETDEV_LINK_2500)   |
					      BIT(TRIGGER_NETDEV_RX)          |
					      BIT(TRIGGER_NETDEV_TX);
#ifdef CONFIG_AIR_EN8811H_PHY_DEBUGFS
static const char * const tx_rx_string[32] = {
	"Tx Reverse, Rx Normal",
	"Tx Normal, Rx Normal",
	"Tx Reverse, Rx Reverse",
	"Tx Normal, Rx Reverse",
};

static void airphy_debugfs_remove(struct phy_device *phydev);
static int airphy_debugfs_init(struct phy_device *phydev);
#endif

static int air_phy_read_page(struct phy_device *phydev)
{
	return __phy_read(phydev, AIR_EXT_PAGE_ACCESS);
}

static int air_phy_write_page(struct phy_device *phydev, int page)
{
	return __phy_write(phydev, AIR_EXT_PAGE_ACCESS, page);
}

static int __air_buckpbus_reg_write(struct phy_device *phydev,
				    u32 pbus_address, u32 pbus_data)
{
	int ret;

	ret = __phy_write(phydev, AIR_BPBUS_MODE, AIR_BPBUS_MODE_ADDR_FIXED);
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_WR_ADDR_HIGH,
			  upper_16_bits(pbus_address));
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_WR_ADDR_LOW,
			  lower_16_bits(pbus_address));
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_WR_DATA_HIGH,
			  upper_16_bits(pbus_data));
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_WR_DATA_LOW,
			  lower_16_bits(pbus_data));
	if (ret < 0)
		return ret;

	return 0;
}

static int air_buckpbus_reg_write(struct phy_device *phydev,
				  u32 pbus_address, u32 pbus_data)
{
	int saved_page;
	int ret = 0;

	saved_page = phy_select_page(phydev, AIR_PHY_PAGE_EXTENDED_4);

	if (saved_page >= 0) {
		ret = __air_buckpbus_reg_write(phydev, pbus_address,
					       pbus_data);
		if (ret < 0)
			phydev_err(phydev, "%s 0x%08x failed: %d\n", __func__,
				   pbus_address, ret);
	}

	return phy_restore_page(phydev, saved_page, ret);
}

static int __air_buckpbus_reg_read(struct phy_device *phydev,
				   u32 pbus_address, u32 *pbus_data)
{
	int pbus_data_low, pbus_data_high;
	int ret;

	ret = __phy_write(phydev, AIR_BPBUS_MODE, AIR_BPBUS_MODE_ADDR_FIXED);
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_RD_ADDR_HIGH,
			  upper_16_bits(pbus_address));
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_RD_ADDR_LOW,
			  lower_16_bits(pbus_address));
	if (ret < 0)
		return ret;

	pbus_data_high = __phy_read(phydev, AIR_BPBUS_RD_DATA_HIGH);
	if (pbus_data_high < 0)
		return pbus_data_high;

	pbus_data_low = __phy_read(phydev, AIR_BPBUS_RD_DATA_LOW);
	if (pbus_data_low < 0)
		return pbus_data_low;

	*pbus_data = pbus_data_low | (pbus_data_high << 16);
	return 0;
}

static int air_buckpbus_reg_read(struct phy_device *phydev,
				 u32 pbus_address, u32 *pbus_data)
{
	int saved_page;
	int ret = 0;

	saved_page = phy_select_page(phydev, AIR_PHY_PAGE_EXTENDED_4);

	if (saved_page >= 0) {
		ret = __air_buckpbus_reg_read(phydev, pbus_address, pbus_data);
		if (ret < 0)
			phydev_err(phydev, "%s 0x%08x failed: %d\n", __func__,
				   pbus_address, ret);
	}

	return phy_restore_page(phydev, saved_page, ret);
}

static int __air_buckpbus_reg_modify(struct phy_device *phydev,
				     u32 pbus_address, u32 mask, u32 set)
{
	int pbus_data_low, pbus_data_high;
	u32 pbus_data_old, pbus_data_new;
	int ret;

	ret = __phy_write(phydev, AIR_BPBUS_MODE, AIR_BPBUS_MODE_ADDR_FIXED);
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_RD_ADDR_HIGH,
			  upper_16_bits(pbus_address));
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_RD_ADDR_LOW,
			  lower_16_bits(pbus_address));
	if (ret < 0)
		return ret;

	pbus_data_high = __phy_read(phydev, AIR_BPBUS_RD_DATA_HIGH);
	if (pbus_data_high < 0)
		return pbus_data_high;

	pbus_data_low = __phy_read(phydev, AIR_BPBUS_RD_DATA_LOW);
	if (pbus_data_low < 0)
		return pbus_data_low;

	pbus_data_old = pbus_data_low | (pbus_data_high << 16);
	pbus_data_new = (pbus_data_old & ~mask) | set;
	if (pbus_data_new == pbus_data_old)
		return 0;

	ret = __phy_write(phydev, AIR_BPBUS_WR_ADDR_HIGH,
			  upper_16_bits(pbus_address));
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_WR_ADDR_LOW,
			  lower_16_bits(pbus_address));
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_WR_DATA_HIGH,
			  upper_16_bits(pbus_data_new));
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_WR_DATA_LOW,
			  lower_16_bits(pbus_data_new));
	if (ret < 0)
		return ret;

	return 0;
}

static int air_buckpbus_reg_modify(struct phy_device *phydev,
				   u32 pbus_address, u32 mask, u32 set)
{
	int saved_page;
	int ret = 0;

	saved_page = phy_select_page(phydev, AIR_PHY_PAGE_EXTENDED_4);

	if (saved_page >= 0) {
		ret = __air_buckpbus_reg_modify(phydev, pbus_address, mask,
						set);
		if (ret < 0)
			phydev_err(phydev, "%s 0x%08x failed: %d\n", __func__,
				   pbus_address, ret);
	}

	return phy_restore_page(phydev, saved_page, ret);
}

static int __air_write_buf(struct phy_device *phydev, u32 address,
			   const struct firmware *fw)
{
	unsigned int offset;
	int ret;
	u16 val;

	ret = __phy_write(phydev, AIR_BPBUS_MODE, AIR_BPBUS_MODE_ADDR_INCR);
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_WR_ADDR_HIGH,
			  upper_16_bits(address));
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, AIR_BPBUS_WR_ADDR_LOW,
			  lower_16_bits(address));
	if (ret < 0)
		return ret;

	for (offset = 0; offset < fw->size; offset += 4) {
		val = get_unaligned_le16(&fw->data[offset + 2]);
		ret = __phy_write(phydev, AIR_BPBUS_WR_DATA_HIGH, val);
		if (ret < 0)
			return ret;

		val = get_unaligned_le16(&fw->data[offset]);
		ret = __phy_write(phydev, AIR_BPBUS_WR_DATA_LOW, val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int air_write_buf(struct phy_device *phydev, u32 address,
			 const struct firmware *fw)
{
	int saved_page;
	int ret = 0;

	saved_page = phy_select_page(phydev, AIR_PHY_PAGE_EXTENDED_4);

	if (saved_page >= 0) {
		ret = __air_write_buf(phydev, address, fw);
		if (ret < 0)
			phydev_err(phydev, "%s 0x%08x failed: %d\n", __func__,
				   address, ret);
	}

	return phy_restore_page(phydev, saved_page, ret);
}

static int en8811h_wait_mcu_ready(struct phy_device *phydev)
{
	int ret, reg_value;

	/* Because of mdio-lock, may have to wait for multiple loads */
	ret = phy_read_mmd_poll_timeout(phydev, MDIO_MMD_VEND1,
					EN8811H_PHY_FW_STATUS, reg_value,
					reg_value == EN8811H_PHY_READY,
					20000, 7500000, true);
	if (ret) {
		phydev_err(phydev, "MCU not ready: 0x%x\n", reg_value);
		return -ENODEV;
	}

	return 0;
}

static int en8811h_load_firmware(struct phy_device *phydev)
{
	struct en8811h_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	const struct firmware *fw1, *fw2;
	int ret;

	ret = request_firmware_direct(&fw1, EN8811H_MD32_DM, dev);
	if (ret < 0)
		return ret;
#ifdef CONFIG_AIR_EN8811H_PHY_DEBUGFS
	priv->dm_crc32 = ~crc32(~0, fw1->data, fw1->size);
#endif
	ret = request_firmware_direct(&fw2, EN8811H_MD32_DSP, dev);
	if (ret < 0)
		goto en8811h_load_firmware_rel1;
#ifdef CONFIG_AIR_EN8811H_PHY_DEBUGFS
	priv->dsp_crc32 = ~crc32(~0, fw2->data, fw2->size);
#endif
	ret = air_buckpbus_reg_write(phydev, EN8811H_FW_CTRL_1,
				     EN8811H_FW_CTRL_1_START);
	if (ret < 0)
		goto en8811h_load_firmware_out;

	ret = air_buckpbus_reg_modify(phydev, EN8811H_FW_CTRL_2,
				      EN8811H_FW_CTRL_2_LOADING,
				      EN8811H_FW_CTRL_2_LOADING);
	if (ret < 0)
		goto en8811h_load_firmware_out;

	ret = air_write_buf(phydev, AIR_FW_ADDR_DM,  fw1);
	if (ret < 0)
		goto en8811h_load_firmware_out;

	ret = air_write_buf(phydev, AIR_FW_ADDR_DSP, fw2);
	if (ret < 0)
		goto en8811h_load_firmware_out;

	ret = air_buckpbus_reg_modify(phydev, EN8811H_FW_CTRL_2,
				      EN8811H_FW_CTRL_2_LOADING, 0);
	if (ret < 0)
		goto en8811h_load_firmware_out;

	ret = air_buckpbus_reg_modify(phydev, EN8811H_CONTROL,
				      EN8811H_CONTROL_INTERNAL,
				      EN8811H_CONTROL_INTERNAL);
	if (ret < 0)
		return ret;

	ret = air_buckpbus_reg_write(phydev, EN8811H_FW_CTRL_1,
				     EN8811H_FW_CTRL_1_FINISH);
	if (ret < 0)
		goto en8811h_load_firmware_out;

	ret = en8811h_wait_mcu_ready(phydev);

	air_buckpbus_reg_read(phydev, EN8811H_FW_VERSION,
			      &priv->firmware_version);
	phydev_info(phydev, "MD32 firmware version: %08x\n",
		    priv->firmware_version);

en8811h_load_firmware_out:
	release_firmware(fw2);

en8811h_load_firmware_rel1:
	release_firmware(fw1);

	if (ret < 0)
		phydev_err(phydev, "Load firmware failed: %d\n", ret);

	return ret;
}

static int en8811h_restart_mcu(struct phy_device *phydev)
{
	int ret;

	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, EN8811H_PHY_FW_STATUS, 0x0);
	if (ret < 0)
		return ret;

	ret = air_buckpbus_reg_write(phydev, EN8811H_FW_CTRL_1,
				     EN8811H_FW_CTRL_1_START);
	if (ret < 0)
		return ret;

	ret = air_buckpbus_reg_modify(phydev, EN8811H_CONTROL,
				      EN8811H_CONTROL_INTERNAL,
				      EN8811H_CONTROL_INTERNAL);
	if (ret < 0)
		return ret;

	ret = air_buckpbus_reg_write(phydev, EN8811H_FW_CTRL_1,
				     EN8811H_FW_CTRL_1_FINISH);
	if (ret < 0)
		return ret;

	return en8811h_wait_mcu_ready(phydev);
}

static int air_hw_led_on_set(struct phy_device *phydev, u8 index, bool on)
{
	struct en8811h_priv *priv = phydev->priv;
	bool changed;

	if (index >= EN8811H_LED_COUNT)
		return -EINVAL;

	if (on)
		changed = !test_and_set_bit(AIR_PHY_LED_STATE_FORCE_ON,
					    &priv->led[index].state);
	else
		changed = !!test_and_clear_bit(AIR_PHY_LED_STATE_FORCE_ON,
					       &priv->led[index].state);

	changed |= (priv->led[index].rules != 0);

	if (changed)
		return phy_modify_mmd(phydev, MDIO_MMD_VEND2,
				      AIR_PHY_LED_ON(index),
				      AIR_PHY_LED_ON_MASK,
				      on ? AIR_PHY_LED_ON_FORCE_ON : 0);

	return 0;
}

static int air_hw_led_blink_set(struct phy_device *phydev, u8 index,
				bool blinking)
{
	struct en8811h_priv *priv = phydev->priv;
	bool changed;

	if (index >= EN8811H_LED_COUNT)
		return -EINVAL;

	if (blinking)
		changed = !test_and_set_bit(AIR_PHY_LED_STATE_FORCE_BLINK,
					    &priv->led[index].state);
	else
		changed = !!test_and_clear_bit(AIR_PHY_LED_STATE_FORCE_BLINK,
					       &priv->led[index].state);

	changed |= (priv->led[index].rules != 0);

	if (changed)
		return phy_write_mmd(phydev, MDIO_MMD_VEND2,
				     AIR_PHY_LED_BLINK(index),
				     blinking ?
				     AIR_PHY_LED_BLINK_FORCE_BLINK : 0);
	else
		return 0;
}

static int air_led_blink_set(struct phy_device *phydev, u8 index,
			     unsigned long *delay_on,
			     unsigned long *delay_off)
{
	struct en8811h_priv *priv = phydev->priv;
	bool blinking = false;
	int err;

	if (index >= EN8811H_LED_COUNT)
		return -EINVAL;

	if (delay_on && delay_off && (*delay_on > 0) && (*delay_off > 0)) {
		blinking = true;
		*delay_on = 50;
		*delay_off = 50;
	}

	err = air_hw_led_blink_set(phydev, index, blinking);
	if (err)
		return err;

	/* led-blink set, so switch led-on off */
	err = air_hw_led_on_set(phydev, index, false);
	if (err)
		return err;

	/* hw-control is off*/
	if (!!test_bit(AIR_PHY_LED_STATE_FORCE_BLINK, &priv->led[index].state))
		priv->led[index].rules = 0;

	return 0;
}

static int air_led_brightness_set(struct phy_device *phydev, u8 index,
				  enum led_brightness value)
{
	struct en8811h_priv *priv = phydev->priv;
	int err;

	if (index >= EN8811H_LED_COUNT)
		return -EINVAL;

	/* led-on set, so switch led-blink off */
	err = air_hw_led_blink_set(phydev, index, false);
	if (err)
		return err;

	err = air_hw_led_on_set(phydev, index, (value != LED_OFF));
	if (err)
		return err;

	/* hw-control is off */
	if (!!test_bit(AIR_PHY_LED_STATE_FORCE_ON, &priv->led[index].state))
		priv->led[index].rules = 0;

	return 0;
}

static int air_led_hw_control_get(struct phy_device *phydev, u8 index,
				  unsigned long *rules)
{
	struct en8811h_priv *priv = phydev->priv;

	if (index >= EN8811H_LED_COUNT)
		return -EINVAL;

	*rules = priv->led[index].rules;

	return 0;
};

static int air_led_hw_control_set(struct phy_device *phydev, u8 index,
				  unsigned long rules)
{
	struct en8811h_priv *priv = phydev->priv;
	u16 on = 0, blink = 0;
	int ret;

	if (index >= EN8811H_LED_COUNT)
		return -EINVAL;

	priv->led[index].rules = rules;

	if (rules & BIT(TRIGGER_NETDEV_FULL_DUPLEX))
		on |= AIR_PHY_LED_ON_FDX;

	if (rules & (BIT(TRIGGER_NETDEV_LINK_10) | BIT(TRIGGER_NETDEV_LINK)))
		on |= AIR_PHY_LED_ON_LINK10;

	if (rules & (BIT(TRIGGER_NETDEV_LINK_100) | BIT(TRIGGER_NETDEV_LINK)))
		on |= AIR_PHY_LED_ON_LINK100;

	if (rules & (BIT(TRIGGER_NETDEV_LINK_1000) | BIT(TRIGGER_NETDEV_LINK)))
		on |= AIR_PHY_LED_ON_LINK1000;

	if (rules & (BIT(TRIGGER_NETDEV_LINK_2500) | BIT(TRIGGER_NETDEV_LINK)))
		on |= AIR_PHY_LED_ON_LINK2500;

	if (rules & BIT(TRIGGER_NETDEV_RX)) {
		blink |= AIR_PHY_LED_BLINK_10RX   |
			 AIR_PHY_LED_BLINK_100RX  |
			 AIR_PHY_LED_BLINK_1000RX |
			 AIR_PHY_LED_BLINK_2500RX;
	}

	if (rules & BIT(TRIGGER_NETDEV_TX)) {
		blink |= AIR_PHY_LED_BLINK_10TX   |
			 AIR_PHY_LED_BLINK_100TX  |
			 AIR_PHY_LED_BLINK_1000TX |
			 AIR_PHY_LED_BLINK_2500TX;
	}

	if (blink || on) {
		/* switch hw-control on, so led-on and led-blink are off */
		clear_bit(AIR_PHY_LED_STATE_FORCE_ON,
			  &priv->led[index].state);
		clear_bit(AIR_PHY_LED_STATE_FORCE_BLINK,
			  &priv->led[index].state);
	} else {
		priv->led[index].rules = 0;
	}

	ret = phy_modify_mmd(phydev, MDIO_MMD_VEND2, AIR_PHY_LED_ON(index),
			     AIR_PHY_LED_ON_MASK, on);

	if (ret < 0)
		return ret;

	return phy_write_mmd(phydev, MDIO_MMD_VEND2, AIR_PHY_LED_BLINK(index),
			     blink);
};

static int air_led_init(struct phy_device *phydev, u8 index, u8 state, u8 pol)
{
	int val = 0;
	int err;

	if (index >= EN8811H_LED_COUNT)
		return -EINVAL;

	if (state == AIR_LED_ENABLE)
		val |= AIR_PHY_LED_ON_ENABLE;
	else
		val &= ~AIR_PHY_LED_ON_ENABLE;

	if (pol == AIR_ACTIVE_HIGH)
		val |= AIR_PHY_LED_ON_POLARITY;
	else
		val &= ~AIR_PHY_LED_ON_POLARITY;

	err = phy_modify_mmd(phydev, MDIO_MMD_VEND2, AIR_PHY_LED_ON(index),
			     AIR_PHY_LED_ON_ENABLE |
			     AIR_PHY_LED_ON_POLARITY, val);

	if (err < 0)
		return err;

	return 0;
}

static int air_leds_init(struct phy_device *phydev, int num, int dur, int mode)
{
	struct en8811h_priv *priv = phydev->priv;
	int ret, i;

	ret = phy_write_mmd(phydev, MDIO_MMD_VEND2, AIR_PHY_LED_DUR_BLINK,
			    dur);
	if (ret < 0)
		return ret;

	ret = phy_write_mmd(phydev, MDIO_MMD_VEND2, AIR_PHY_LED_DUR_ON,
			    dur >> 1);
	if (ret < 0)
		return ret;

	switch (mode) {
	case AIR_LED_MODE_DISABLE:
		ret = phy_modify_mmd(phydev, MDIO_MMD_VEND2, AIR_PHY_LED_BCR,
				     AIR_PHY_LED_BCR_EXT_CTRL |
				     AIR_PHY_LED_BCR_MODE_MASK, 0);
		if (ret < 0)
			return ret;
		break;
	case AIR_LED_MODE_USER_DEFINE:
		ret = phy_modify_mmd(phydev, MDIO_MMD_VEND2, AIR_PHY_LED_BCR,
				     AIR_PHY_LED_BCR_EXT_CTRL |
				     AIR_PHY_LED_BCR_CLK_EN,
				     AIR_PHY_LED_BCR_EXT_CTRL |
				     AIR_PHY_LED_BCR_CLK_EN);
		if (ret < 0)
			return ret;
		break;
	default:
		phydev_err(phydev, "LED mode %d is not supported\n", mode);
		return -EINVAL;
	}

	for (i = 0; i < num; ++i) {
		ret = air_led_init(phydev, i, AIR_LED_ENABLE, AIR_ACTIVE_HIGH);
		if (ret < 0) {
			phydev_err(phydev, "LED%d init failed: %d\n", i, ret);
			return ret;
		}
		air_led_hw_control_set(phydev, i, priv->led[i].rules);
	}

	return 0;
}

static int en8811h_led_hw_is_supported(struct phy_device *phydev, u8 index,
				       unsigned long rules)
{
	if (index >= EN8811H_LED_COUNT)
		return -EINVAL;

	/* All combinations of the supported triggers are allowed */
	if (rules & ~en8811h_led_trig)
		return -EOPNOTSUPP;

	return 0;
};

static unsigned long en8811h_clk_recalc_rate(struct clk_hw *hw, unsigned long parent)
{
	struct en8811h_priv *priv = clk_hw_to_en8811h_priv(hw);
	struct phy_device *phydev = priv->phydev;
	u32 pbus_value;
	int ret;

	ret = air_buckpbus_reg_read(phydev, EN8811H_HWTRAP1, &pbus_value);
	if (ret < 0)
		return ret;

	return (pbus_value & EN8811H_HWTRAP1_CKO) ? 50000000 : 25000000;
}

static int en8811h_clk_enable(struct clk_hw *hw)
{
	struct en8811h_priv *priv = clk_hw_to_en8811h_priv(hw);
	struct phy_device *phydev = priv->phydev;

	return air_buckpbus_reg_modify(phydev, EN8811H_CLK_CGM,
				EN8811H_CLK_CGM_CKO, EN8811H_CLK_CGM_CKO);
}

static void en8811h_clk_disable(struct clk_hw *hw)
{
	struct en8811h_priv *priv = clk_hw_to_en8811h_priv(hw);
	struct phy_device *phydev = priv->phydev;

	air_buckpbus_reg_modify(phydev, EN8811H_CLK_CGM,
				EN8811H_CLK_CGM_CKO, 0);
}

static int en8811h_clk_is_enabled(struct clk_hw *hw)
{
	struct en8811h_priv *priv = clk_hw_to_en8811h_priv(hw);
	struct phy_device *phydev = priv->phydev;
	u32 pbus_value;
	int ret;

	ret = air_buckpbus_reg_read(phydev, EN8811H_CLK_CGM, &pbus_value);
	if (ret < 0)
		return ret;

	return (pbus_value & EN8811H_CLK_CGM_CKO);
}

static int en8811h_clk_save_context(struct clk_hw *hw)
{
	struct en8811h_priv *priv = clk_hw_to_en8811h_priv(hw);

	priv->cko_is_enabled = en8811h_clk_is_enabled(hw);

	return 0;
}

static void en8811h_clk_restore_context(struct clk_hw *hw)
{
	struct en8811h_priv *priv = clk_hw_to_en8811h_priv(hw);

	if (!priv->cko_is_enabled)
		en8811h_clk_disable(hw);
}

static const struct clk_ops en8811h_clk_ops = {
	.recalc_rate		= en8811h_clk_recalc_rate,
	.enable			= en8811h_clk_enable,
	.disable		= en8811h_clk_disable,
	.is_enabled		= en8811h_clk_is_enabled,
	.save_context		= en8811h_clk_save_context,
	.restore_context	= en8811h_clk_restore_context,
};

static int en8811h_clk_provider_setup(struct device *dev, struct clk_hw *hw)
{
	struct clk_init_data init;
	int ret;

	if (!IS_ENABLED(CONFIG_COMMON_CLK))
		return 0;

	init.name = devm_kasprintf(dev, GFP_KERNEL, "%s-cko",
				   fwnode_get_name(dev_fwnode(dev)));
	if (!init.name)
		return -ENOMEM;

	init.ops = &en8811h_clk_ops;
	init.flags = 0;
	init.num_parents = 0;
	hw->init = &init;

	ret = devm_clk_hw_register(dev, hw);
	if (ret)
		return ret;

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, hw);
}

static int en8811h_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct en8811h_priv *priv;
	u32 pbus_value;
	int ret;

	priv = devm_kzalloc(&phydev->mdio.dev, sizeof(struct en8811h_priv),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	phydev->priv = priv;

	ret = air_buckpbus_reg_write(phydev, 0x1e00d0, 0xf);
	ret |= air_buckpbus_reg_write(phydev, 0x1e0228, 0xf0);
	if (ret < 0)
		return ret;

	ret = en8811h_load_firmware(phydev);
	if (ret < 0)
		return ret;

	/* mcu has just restarted after firmware load */
	priv->mcu_needs_restart = false;
	priv->restart_mcu =	false;

	priv->led[0].rules = AIR_DEFAULT_TRIGGER_LED0;
	priv->led[1].rules = AIR_DEFAULT_TRIGGER_LED1;
	priv->led[2].rules = AIR_DEFAULT_TRIGGER_LED2;

	/* MDIO_DEVS1/2 empty, so set mmds_present bits here */
	phydev->c45_ids.mmds_present |= MDIO_DEVS_PMAPMD | MDIO_DEVS_AN;
#ifdef CONFIG_AIR_EN8811H_PHY_DEBUGFS
	ret = airphy_debugfs_init(phydev);
	if (ret < 0) {
		phydev_err(phydev, "air_debug_procfs_init fail. (ret=%d)\n", ret);
		airphy_debugfs_remove(phydev);
	}
#endif /* CONFIG_AIR_EN8811H_PHY_DEBUGFS */

	ret = air_leds_init(phydev, EN8811H_LED_COUNT, AIR_PHY_LED_DUR,
			    AIR_LED_MODE_DISABLE);
	if (ret < 0) {
		phydev_err(phydev, "Failed to disable leds: %d\n", ret);
		return ret;
	}

	priv->phydev = phydev;
	/* Co-Clock Output */
	ret = en8811h_clk_provider_setup(&phydev->mdio.dev, &priv->hw);
	if (ret)
		return ret;

	/* Configure led gpio pins as output */
	ret = air_buckpbus_reg_modify(phydev, EN8811H_GPIO_OUTPUT,
				      EN8811H_GPIO_OUTPUT_345,
				      EN8811H_GPIO_OUTPUT_345);
	if (ret < 0)
		return ret;

	if (!device_property_read_bool(dev, "airoha,phy-handle")) {
		ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, AIR_PHY_MCU_CMD_1,
				    AIR_PHY_MCU_CMD_1_MODE1);
		if (ret < 0)
			return ret;
		ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, AIR_PHY_MCU_CMD_2,
				    AIR_PHY_MCU_CMD_2_MODE1);
		if (ret < 0)
			return ret;
		ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, AIR_PHY_MCU_CMD_3,
				    AIR_PHY_MCU_CMD_3_MODE1);
		if (ret < 0)
			return ret;
		ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, AIR_PHY_MCU_CMD_4,
				    AIR_PHY_MCU_CMD_4_MODE1);
		if (ret < 0)
			return ret;

		/* Serdes polarity */
		pbus_value = 0;
		if (device_property_read_bool(dev, "airoha,pnswap-rx"))
			pbus_value |=  EN8811H_POLARITY_RX_REVERSE;
		else
			pbus_value &= ~EN8811H_POLARITY_RX_REVERSE;
		if (device_property_read_bool(dev, "airoha,pnswap-tx"))
			pbus_value &= ~EN8811H_POLARITY_TX_NORMAL;
		else
			pbus_value |=  EN8811H_POLARITY_TX_NORMAL;
		ret = air_buckpbus_reg_modify(phydev, EN8811H_POLARITY,
					      EN8811H_POLARITY_RX_REVERSE |
					      EN8811H_POLARITY_TX_NORMAL, pbus_value);
		if (ret < 0)
			return ret;

		ret = air_buckpbus_reg_modify(phydev, EN8811H_CONTROL,
					      EN8811H_CONTROL_INTERNAL, 0);
		if (ret < 0)
			return ret;

		phydev_info(phydev, "EN8811H initialize OK! (%s)\n",
			    EN8811H_DRIVER_VERSION);
	} else
		phydev_info(phydev, "EN8811H probe OK! (%s)\n",
			    EN8811H_DRIVER_VERSION);

	return 0;
}

static int en8811h_config_init(struct phy_device *phydev)
{
	struct en8811h_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	u32 pbus_value;
	int ret;

	ret = air_buckpbus_reg_write(phydev, 0x1e00d0, 0xf);
	ret |= air_buckpbus_reg_write(phydev, 0x1e0228, 0xf0);
	if (ret < 0)
		return ret;

	/* If restart happened in .probe(), no need to restart now */
	if (priv->mcu_needs_restart) {
		ret = en8811h_restart_mcu(phydev);
		if (ret < 0)
			return ret;
		priv->restart_mcu = true;
	} else {
		/* Next calls to .config_init() mcu needs to restart */
		priv->mcu_needs_restart = true;
	}

	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, AIR_PHY_MCU_CMD_1,
			    AIR_PHY_MCU_CMD_1_MODE1);
	if (ret < 0)
		return ret;
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, AIR_PHY_MCU_CMD_2,
			    AIR_PHY_MCU_CMD_2_MODE1);
	if (ret < 0)
		return ret;
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, AIR_PHY_MCU_CMD_3,
			    AIR_PHY_MCU_CMD_3_MODE1);
	if (ret < 0)
		return ret;
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, AIR_PHY_MCU_CMD_4,
			    AIR_PHY_MCU_CMD_4_MODE1);
	if (ret < 0)
		return ret;

	/* Serdes polarity */
	pbus_value = 0;
	if (device_property_read_bool(dev, "airoha,pnswap-rx"))
		pbus_value |=  EN8811H_POLARITY_RX_REVERSE;
	else
		pbus_value &= ~EN8811H_POLARITY_RX_REVERSE;
	if (device_property_read_bool(dev, "airoha,pnswap-tx"))
		pbus_value &= ~EN8811H_POLARITY_TX_NORMAL;
	else
		pbus_value |=  EN8811H_POLARITY_TX_NORMAL;
	ret = air_buckpbus_reg_modify(phydev, EN8811H_POLARITY,
				      EN8811H_POLARITY_RX_REVERSE |
				      EN8811H_POLARITY_TX_NORMAL, pbus_value);
	if (ret < 0)
		return ret;

	ret = air_leds_init(phydev, EN8811H_LED_COUNT, AIR_PHY_LED_DUR,
			    AIR_LED_MODE_USER_DEFINE);
	if (ret < 0) {
		phydev_err(phydev, "Failed to initialize leds: %d\n", ret);
		return ret;
	}

	if (!priv->restart_mcu) {
		ret = air_buckpbus_reg_write(phydev, 0x1e0228, 0xf);
		if (ret < 0)
			return ret;
	}

	ret = air_buckpbus_reg_modify(phydev, EN8811H_CONTROL,
				      EN8811H_CONTROL_INTERNAL, 0);
	if (ret < 0)
		return ret;

	ret = air_buckpbus_reg_read(phydev, EN8811H_CLK_CGM, &pbus_value);
	if (ret < 0)
		return ret;
	if (pbus_value & EN8811H_CLK_CGM_CKO) {
		ret = air_buckpbus_reg_read(phydev, EN8811H_HWTRAP1, &pbus_value);
		if (ret < 0)
			return ret;

		phydev_info(phydev, "CKO Output %dMHz - Enabled\n",
			    (pbus_value & EN8811H_HWTRAP1_CKO) ? 50 : 25);
	} else {
		phydev_info(phydev, "CKO Output - Disabled.\n");
	}

	phydev_info(phydev, "EN8811H initialize OK! (%s)\n",
		    EN8811H_DRIVER_VERSION);

	return 0;
}

static int en8811h_get_features(struct phy_device *phydev)
{
	linkmode_set_bit_array(phy_basic_ports_array,
			       ARRAY_SIZE(phy_basic_ports_array),
			       phydev->supported);

	return genphy_c45_pma_read_abilities(phydev);
}

static int en8811h_get_rate_matching(struct phy_device *phydev,
				     phy_interface_t iface)
{
	return RATE_MATCH_PAUSE;
}

static int en8811h_config_aneg(struct phy_device *phydev)
{
	bool changed = false;
	int ret;
	u32 adv;

	if (phydev->autoneg == AUTONEG_DISABLE) {
		phydev_warn(phydev, "Disabling autoneg is not supported\n");
		return -EINVAL;
	}

	adv = linkmode_adv_to_mii_10gbt_adv_t(phydev->advertising);

	ret = phy_modify_mmd_changed(phydev, MDIO_MMD_AN, MDIO_AN_10GBT_CTRL,
				     MDIO_AN_10GBT_CTRL_ADV2_5G, adv);
	if (ret < 0)
		return ret;
	if (ret > 0)
		changed = true;

	return __genphy_config_aneg(phydev, changed);
}

static int en8811h_read_status(struct phy_device *phydev)
{
	struct en8811h_priv *priv = phydev->priv;
	u32 pbus_value;
	int ret, val;

	ret = genphy_update_link(phydev);
	if (ret)
		return ret;

	phydev->master_slave_get = MASTER_SLAVE_CFG_UNSUPPORTED;
	phydev->master_slave_state = MASTER_SLAVE_STATE_UNSUPPORTED;
	phydev->speed = SPEED_UNKNOWN;
	phydev->duplex = DUPLEX_UNKNOWN;
	phydev->pause = 0;
	phydev->asym_pause = 0;
	phydev->rate_matching = RATE_MATCH_PAUSE;

	ret = genphy_read_master_slave(phydev);
	if (ret < 0)
		return ret;

	ret = genphy_read_lpa(phydev);
	if (ret < 0)
		return ret;

	/* Get link partner 2.5GBASE-T ability from vendor register */
	ret = air_buckpbus_reg_read(phydev, EN8811H_2P5G_LPA, &pbus_value);
	if (ret < 0)
		return ret;
	linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
			 phydev->lp_advertising,
			 pbus_value & EN8811H_2P5G_LPA_2P5G);

	/* Only supports full duplex */
	phydev->duplex = DUPLEX_FULL;

	if (phydev->autoneg_complete)
		phy_resolve_aneg_pause(phydev);

	if (!phydev->link)
		return 0;

	/* Get real speed from vendor register */
	val = phy_read(phydev, AIR_AUX_CTRL_STATUS);
	if (val < 0)
		return val;
	switch (val & AIR_AUX_CTRL_STATUS_SPEED_MASK) {
	case AIR_AUX_CTRL_STATUS_SPEED_2500:
		phydev->speed = SPEED_2500;
		break;
	case AIR_AUX_CTRL_STATUS_SPEED_1000:
		phydev->speed = SPEED_1000;
		break;
	case AIR_AUX_CTRL_STATUS_SPEED_100:
		phydev->speed = SPEED_100;
		break;
	}

	/* Firmware before version 24011202 has no vendor register 2P5G_LPA.
	 * Assume link partner advertised it if connected at 2500Mbps.
	 */
	if (priv->firmware_version < 0x24011202) {
		linkmode_mod_bit(ETHTOOL_LINK_MODE_2500baseT_Full_BIT,
				 phydev->lp_advertising,
				 phydev->speed == SPEED_2500);
	}

	return 0;
}

static int en8811h_clear_intr(struct phy_device *phydev)
{
	int ret;

	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, AIR_PHY_MCU_CMD_3,
			    AIR_PHY_MCU_CMD_3_DOCMD);
	if (ret < 0)
		return ret;

	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, AIR_PHY_MCU_CMD_4,
			    AIR_PHY_MCU_CMD_4_INTCLR);
	if (ret < 0)
		return ret;

	return 0;
}

static irqreturn_t en8811h_handle_interrupt(struct phy_device *phydev)
{
	int ret;

	ret = en8811h_clear_intr(phydev);
	if (ret < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}

	phy_trigger_machine(phydev);

	return IRQ_HANDLED;
}

static int air_phy_resume(struct phy_device *phydev)
{
	clk_restore_context();

	return genphy_resume(phydev);
}

#ifdef CONFIG_AIR_EN8811H_PHY_DEBUGFS
static void en8811h_remove(struct phy_device *phydev)
{
	struct en8811h_priv *priv = phydev->priv;

	if (priv) {
		phydev_info(phydev, "%s: airphy_debugfs_remove\n", __func__);
		airphy_debugfs_remove(phydev);
		kfree(priv);
	}
}
#endif

static struct phy_driver en8811h_driver[] = {
{
	PHY_ID_MATCH_MODEL(EN8811H_PHY_ID),
	.name			= "Airoha EN8811H",
	.probe			= en8811h_probe,
#ifdef CONFIG_AIR_EN8811H_PHY_DEBUGFS
	.remove         = en8811h_remove,
#endif
	.get_features		= en8811h_get_features,
	.config_init		= en8811h_config_init,
	.get_rate_matching	= en8811h_get_rate_matching,
	.config_aneg		= en8811h_config_aneg,
	.read_status		= en8811h_read_status,
	.config_intr		= en8811h_clear_intr,
	.handle_interrupt	= en8811h_handle_interrupt,
	.led_hw_is_supported	= en8811h_led_hw_is_supported,
	.read_page		= air_phy_read_page,
	.write_page		= air_phy_write_page,
	.led_blink_set		= air_led_blink_set,
	.led_brightness_set	= air_led_brightness_set,
	.led_hw_control_set	= air_led_hw_control_set,
	.led_hw_control_get	= air_led_hw_control_get,
	.resume			= air_phy_resume,
	.suspend		= genphy_suspend,
} };

module_phy_driver(en8811h_driver);

static struct mdio_device_id __maybe_unused en8811h_tbl[] = {
	{ PHY_ID_MATCH_MODEL(EN8811H_PHY_ID) },
	{ }
};

#ifdef CONFIG_AIR_EN8811H_PHY_DEBUGFS
#define phydev_mdio_bus(_dev) (_dev->mdio.bus)
#define phydev_addr(_dev) (_dev->mdio.addr)
#define phydev_dev(_dev) (&_dev->mdio.dev)
#define phydev_kobj(_dev) (&_dev->mdio.dev.kobj)

static struct trrg_s _fldRW_tr_reg__EcVarTrainingGain_ECNC_C8h = {
	.TrRGDesc.DescVal = 0x81900302,
	.RgMask =   0x0000000C
};

static struct trrg_s _fldRW_tr_reg__EcVarTrainingTime_ECNC_C8h = {
	.TrRGDesc.DescVal = 0x81900F04,
	.RgMask =   0x0000FFF0
};

static void air_polarity_help(void)
{
	pr_notice("\nUsage:\n"
		  "[debugfs] = /sys/kernel/debug/mdio-bus\':[phy_addr]\n"
		  "echo [tx polarity] [rx polarity] > /[debugfs]/polarity\n"
		  "option: tx_normal, tx_reverse, rx_normal, rx_revers\n");
}

static int air_set_polarity(struct phy_device *phydev, int tx_rx)
{
	int ret = 0;
	unsigned int pbus_data = 0;

	pr_debug("\nPolarit %s\n", tx_rx_string[tx_rx]);

	air_buckpbus_reg_read(phydev, 0xca0f8, &pbus_data);
	pbus_data &= ~(BIT(0) | BIT(1));
	pbus_data |= tx_rx;
	ret = air_buckpbus_reg_write(phydev, 0xca0f8, pbus_data);
	if (ret < 0)
		pr_notice("\n%s:air_buckpbus_reg_write fail\n", __func__);

	air_buckpbus_reg_read(phydev, 0xca0f8, &pbus_data);
	pr_notice("\nPolarity %s confirm....(%02lx)\n",
		  tx_rx_string[tx_rx], pbus_data & (BIT(0) | BIT(1)));

	return ret;
}

static int air_set_mode(struct phy_device *phydev, int dbg_mode)
{
	int ret = 0, val = 0;
	struct en8811h_priv *priv = phydev->priv;

	switch (dbg_mode) {
	case AIR_PORT_MODE_FORCE_100:
		pr_notice("\nForce 100M\n");
		val = (phy_read(phydev, MII_ADVERTISE) | BIT(8)) & ~BIT(12);
		ret = phy_write(phydev, MII_ADVERTISE, val);
		if (unlikely(ret < 0))
			break;
		val = phy_read(phydev, MII_CTRL1000) & ~BIT(9);
		ret = phy_write(phydev, MII_CTRL1000, val);
		if (unlikely(ret < 0))
			break;
		val = phy_read_mmd(phydev, 0x7, 0x20) & ~BIT(7);
		ret = phy_write_mmd(phydev, 0x7, 0x20, val);
		if (unlikely(ret < 0))
			break;
		val = phy_read(phydev, MII_BMCR) | BIT(9);
		ret = phy_write(phydev, MII_BMCR, val);
		if (unlikely(ret < 0))
			break;
		priv->need_an = 1;
		break;
	case AIR_PORT_MODE_FORCE_1000:
		pr_notice("\nForce 1000M\n");
		val = (phy_read(phydev, MII_ADVERTISE) & ~BIT(8)) | BIT(12);
		ret = phy_write(phydev, MII_ADVERTISE, val);
		if (unlikely(ret < 0))
			break;
		val = phy_read(phydev, MII_CTRL1000) | BIT(9);
		ret = phy_write(phydev, MII_CTRL1000, val);
		if (unlikely(ret < 0))
			break;
		val = phy_read_mmd(phydev, 0x7, 0x20) & ~BIT(7);
		ret = phy_write_mmd(phydev, 0x7, 0x20, val);
		if (unlikely(ret < 0))
			break;
		val = phy_read(phydev, MII_BMCR) | BIT(9);
		ret = phy_write(phydev, MII_BMCR, val);
		if (unlikely(ret < 0))
			break;
		priv->need_an = 1;
		break;
	case AIR_PORT_MODE_FORCE_2500:
		pr_notice("\nForce 2500M\n");
		val = (phy_read(phydev, MII_ADVERTISE) & ~BIT(8)) | BIT(12);
		ret = phy_write(phydev, MII_ADVERTISE, val);
		if (unlikely(ret < 0))
			break;
		val = phy_read(phydev, MII_CTRL1000) & ~BIT(9);
		ret = phy_write(phydev, MII_CTRL1000, val);
		if (unlikely(ret < 0))
			break;
		val = phy_read_mmd(phydev, 0x7, 0x20) | BIT(7);
		ret = phy_write_mmd(phydev, 0x7, 0x20, val);
		if (unlikely(ret < 0))
			break;
		val = phy_read(phydev, MII_BMCR) | BIT(9);
		ret = phy_write(phydev, MII_BMCR, val);
		if (unlikely(ret < 0))
			break;
		priv->need_an = 1;
		break;
	case AIR_PORT_MODE_AUTONEGO:
		pr_notice("\nAutonego mode\n");
		val = phy_read(phydev, MII_ADVERTISE) | BIT(8) | BIT(12);
		ret = phy_write(phydev, MII_ADVERTISE, val);
		if (unlikely(ret < 0))
			break;
		val = phy_read(phydev, MII_CTRL1000) | BIT(9);
		ret = phy_write(phydev, MII_CTRL1000, val);
		if (unlikely(ret < 0))
			break;
		val = phy_read_mmd(phydev, 0x7, 0x20) | BIT(7);
		ret = phy_write_mmd(phydev, 0x7, 0x20, val);
		if (unlikely(ret < 0))
			break;
		val = phy_read(phydev, MII_BMCR);
		val |= BMCR_ANENABLE;
		ret = phy_write(phydev, MII_BMCR, val);
		if (unlikely(ret < 0))
			break;
		if (priv->need_an) {
			val = phy_read(phydev, MII_BMCR);
			val |= BMCR_ANRESTART;
			ret = phy_write(phydev, MII_BMCR, val);
			if (unlikely(ret < 0))
				break;
			priv->need_an = 0;
			pr_notice("\nRe-an\n");
		}
		break;
	case AIR_PORT_MODE_POWER_DOWN:
		pr_notice("\nPower Down\n");
		val = phy_read(phydev, MII_BMCR) | BIT(11);
		ret = phy_write(phydev, MII_BMCR, val);
		break;
	case AIR_PORT_MODE_POWER_UP:
		pr_notice("\nPower Up\n");
		val = phy_read(phydev, MII_BMCR) & ~BIT(11);
		ret = phy_write(phydev, MII_BMCR, val);
		break;
	default:
		pr_notice("\nWrong Port mode\n");
		break;
	}
	return ret;
}

static int airoha_led_control(struct phy_device *phydev, int force_mode)
{
	int ret = 0, id, on_evt;
	struct en8811h_priv *priv = phydev->priv;

	for (id = 0; id < EN8811H_LED_COUNT; id++) {
		if (force_mode) {
			on_evt = phy_read_mmd(phydev, 0x1f,
					      LED_ON_CTRL(id));
			on_evt &= ~(LED_ON_EVT_LINK_2500M |
				  LED_ON_EVT_LINK_100M | LED_ON_EVT_LINK_1000M);
			if (force_mode == AIR_LED_FORCE_ON)
				on_evt |= LED_ON_EVT_FORCE;
			else
				on_evt &= ~LED_ON_EVT_FORCE;

			ret = phy_write_mmd(phydev, 0x1f,
					    LED_ON_CTRL(id), on_evt);
			if (ret < 0)
				return ret;
			ret = phy_write_mmd(phydev, 0x1f,
					    LED_BLK_CTRL(id), 0);
		} else {
			ret = phy_write_mmd(phydev, 0x1f,
					    LED_ON_CTRL(id), priv->on_crtl[id]);
			ret |= phy_write_mmd(phydev, 0x1f,
					     LED_BLK_CTRL(id), priv->blk_crtl[id]);
		}
		if (ret < 0)
			return ret;
	}
	return 0;
}

static void airphy_led_mode_help(void)
{
	pr_notice("\nUsage:\n"
		  "[debugfs] = /sys/kernel/debug/mdio-bus\':[phy_addr]\n"
		  "echo 0 > /[debugfs]/led_mode\n"
		  "echo 1 > /[debugfs]/led_mode\n"
		  "echo normal > /[debugfs]/led_mode\n");
}

static ssize_t airphy_led_mode(struct file *file, const char __user *ptr,
			       size_t len, loff_t *off)
{
	struct phy_device *phydev = file->private_data;
	char buf[32], cmd[32];
	int count = len, ret = 0;
	int num = 0;

	memset(buf, 0, 32);
	memset(cmd, 0, 32);

	if (count > sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, ptr, len))
		return -EFAULT;

	num = sscanf(buf, "%8s", cmd);
	if (num < 1 || num > 2)
		return -EFAULT;

	if (!strncmp("0", cmd, strlen("0")))
		ret = airoha_led_control(phydev, AIR_LED_FORCE_OFF);
	else if (!strncmp("1", cmd, strlen("1")))
		ret = airoha_led_control(phydev, AIR_LED_FORCE_ON);
	else if (!strncmp("normal", cmd, strlen("normal")))
		ret = airoha_led_control(phydev, AIR_LED_NORMAL);
	else if (!strncmp("help", cmd, strlen("help")))
		airphy_led_mode_help();

	if (ret < 0)
		return ret;

	return count;
}

static int airphy_info_show(struct seq_file *seq, void *v)
{
	struct phy_device *phydev = seq->private;
	struct en8811h_priv *priv = phydev->priv;
	unsigned int val = 0;

	seq_puts(seq, "<<AIR EN8811H Driver info>>\n");
	air_buckpbus_reg_read(phydev, 0xcf914, &val);
	seq_printf(seq, "| Boot mode            : %s\n",
		   ((val & BIT(24)) >> 24) ? "Flash" : "Download Code");
	seq_printf(seq, "| EthMD32.dm.bin  CRC32: %08x\n",
		   priv->dm_crc32);
	seq_printf(seq, "| EthMD32.DSP.bin CRC32: %08x\n",
		   priv->dsp_crc32);
	seq_printf(seq, "| MD32 FW Version      : %08x\n", priv->firmware_version);

	air_buckpbus_reg_read(phydev, 0x3a9c, &val);
	if (val & BIT(3))
		seq_puts(seq, "| Surge Protection     : 5R\n");
	else
		seq_puts(seq, "| Surge Protection     : 0R\n");

	air_buckpbus_reg_read(phydev, EN8811H_CLK_CGM, &val);
	if (val & EN8811H_CLK_CGM_CKO)
		seq_puts(seq, "| Co-Clock Ouput       : Enable\n");
	else
		seq_puts(seq, "| Co-Clock Ouput       : Disable\n");

	air_buckpbus_reg_read(phydev, 0xca0f8, &val);
	val &= 0x3;
	seq_printf(seq, "| Tx, Rx Polarity      : %s(%02d)\n",
		   tx_rx_string[val], val);
	seq_puts(seq, "\n");

	return 0;
}

static int airphy_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, airphy_info_show, inode->i_private);
}

static int airphy_fcm_counter_show(struct phy_device *phydev,
				   struct seq_file *seq)
{
	int ret = 0;
	u32 pkt_cnt = 0;

	seq_puts(seq, "|\t<<FCM Counter>>\n");
	seq_puts(seq, "| Rx from Line side_S      :");
	air_buckpbus_reg_read(phydev, 0xe0090, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Rx from Line side_T      :");
	air_buckpbus_reg_read(phydev, 0xe0094, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Tx to System side_S      :");
	air_buckpbus_reg_read(phydev, 0xe009c, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Tx to System side_T      :");
	air_buckpbus_reg_read(phydev, 0xe00A0, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Rx from System side_S    :");
	air_buckpbus_reg_read(phydev, 0xe0078, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Rx from System side_T    :");
	air_buckpbus_reg_read(phydev, 0xe007C, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Tx to Line side_S        :");
	air_buckpbus_reg_read(phydev, 0xe0084, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Tx to Line side_T        :");
	air_buckpbus_reg_read(phydev, 0xe0088, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Pause from Line side     :");
	air_buckpbus_reg_read(phydev, 0xe0098, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Pause to System side     :");
	air_buckpbus_reg_read(phydev, 0xe00A4, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Pause from System side   :");
	air_buckpbus_reg_read(phydev, 0xe0080, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Pause to Line side       :");
	air_buckpbus_reg_read(phydev, 0xe008C, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	ret = air_buckpbus_reg_write(phydev, 0xe0074, 0x3);
	if (ret < 0)
		return ret;
	return 0;
}

/**
 * airphy_mac_counter - Internal MAC counter
 * @seq: Pointer to the sequence file structure.
 * @phydev: target phy_device struct
 * NOTE: MAC counter should not be polled continuously.
 */
static int airphy_mac_counter_show(struct seq_file *seq,
				   struct phy_device *phydev)
{
	u32 pkt_cnt = 0;

	seq_puts(seq, "|\t<<MAC Counter>>\n");
	seq_puts(seq, "| Tx Error from System side:");
	air_buckpbus_reg_read(phydev, 0x131000, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Rx Error to System side  :");
	air_buckpbus_reg_read(phydev, 0x132000, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Tx from System side      :");
	air_buckpbus_reg_read(phydev, 0x131004, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| Rx to System Side        :");
	air_buckpbus_reg_read(phydev, 0x132004, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	return 0;
}

static int airphy_ss_counter_show(struct phy_device *phydev,
				  struct seq_file *seq)
{
	int ret = 0;
	u32 pkt_cnt = 0;

	seq_puts(seq, "|\t<<SS Counter>>\n");
	ret = air_buckpbus_reg_write(phydev, 0xC602C, 0x3);
	if (ret < 0)
		return ret;
	seq_puts(seq, "| TX Start                 :");
	air_buckpbus_reg_read(phydev, 0xC60B0, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| TX Terminal              :");
	air_buckpbus_reg_read(phydev, 0xC60B4, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| RX Start                 :");
	air_buckpbus_reg_read(phydev, 0xC60BC, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	seq_puts(seq, "| RX Terminal              :");
	air_buckpbus_reg_read(phydev, 0xC60C0, &pkt_cnt);
	seq_printf(seq, "%010u |\n", pkt_cnt);
	ret = air_buckpbus_reg_write(phydev, 0xC602C, 0x4);
	if (ret < 0)
		return ret;
	return 0;
}

static int air_get_autonego(struct phy_device *phydev, int *an)
{
	int reg;

	reg = phy_read(phydev, MII_BMCR);
	if (reg < 0)
		return -EINVAL;
	if (reg & BMCR_ANENABLE)
		*an = AUTONEG_ENABLE;
	else
		*an = AUTONEG_DISABLE;
	return 0;
}

static int air_ref_clk_speed(struct phy_device *phydev, int para)
{
	int ret;
	struct en8811h_priv *priv = phydev->priv;
	int saved_page;

	saved_page = phy_select_page(phydev, AIR_PHY_PAGE_STANDARD);
	ret = __phy_write(phydev, 0x1f, 0x0);
	/* Get real speed from vendor register */
	ret = __phy_read(phydev, AIR_AUX_CTRL_STATUS);
	if (ret < 0)
		return ret;
	switch (ret & AIR_AUX_CTRL_STATUS_SPEED_MASK) {
	case AIR_AUX_CTRL_STATUS_SPEED_2500:
		if (para == AIR_PARA_PRIV)
			priv->speed = SPEED_2500;
		else
			phydev->speed = SPEED_2500;
		break;
	case AIR_AUX_CTRL_STATUS_SPEED_1000:
		if (para == AIR_PARA_PRIV)
			priv->speed = SPEED_1000;
		else
			phydev->speed = SPEED_1000;
		break;
	case AIR_AUX_CTRL_STATUS_SPEED_100:
		if (para == AIR_PARA_PRIV)
			priv->speed = SPEED_100;
		else
			phydev->speed = SPEED_100;
		break;
	}
	return phy_restore_page(phydev, saved_page, ret);
}

static int air_read_status(struct phy_device *phydev)
{
	int ret = 0, reg = 0;
	struct en8811h_priv *priv = phydev->priv;

	priv->speed = SPEED_UNKNOWN;
	priv->duplex = DUPLEX_UNKNOWN;
	priv->pause = 0;
	priv->asym_pause = 0;

	reg = phy_read(phydev, MII_BMSR);
	if (reg < 0) {
		phydev_err(phydev, "MII_BMSR reg %d!\n", reg);
		return reg;
	}
	reg = phy_read(phydev, MII_BMSR);
	if (reg < 0) {
		phydev_err(phydev, "MII_BMSR reg %d!\n", reg);
		return reg;
	}
	if (reg & BMSR_LSTATUS) {
		priv->link = 1;
		ret = air_ref_clk_speed(phydev, AIR_PARA_PRIV);
		if (ret < 0)
			return ret;
		reg = phy_read(phydev, MII_ADVERTISE);
		if (reg < 0)
			return reg;
		priv->pause = GET_BIT(reg, 10);
		priv->asym_pause = GET_BIT(reg, 11);
	} else
		priv->link = 0;

	priv->duplex = DUPLEX_FULL;
	return 0;
}

static int airphy_counter_show(struct seq_file *seq, void *v)
{
	struct phy_device *phydev = seq->private;

	int ret = 0;
	u32 pkt_cnt = 0;
	struct en8811h_priv *priv = phydev->priv;

	ret = air_read_status(phydev);
	if (ret < 0)
		return ret;
	seq_puts(seq, "==========AIR PHY COUNTER==========\n");
	if (priv->link) {
		ret = airphy_ss_counter_show(phydev, seq);
		if (ret < 0)
			return ret;
	}
	ret = airphy_fcm_counter_show(phydev, seq);
	if (ret < 0)
		return ret;
	if (priv->link) {
		ret = airphy_mac_counter_show(seq, phydev);
		if (ret < 0)
			return ret;
	}
	if (priv->link && priv->speed == SPEED_2500) {
		seq_puts(seq, "|\t<<LS Counter>>\n");
		ret = air_buckpbus_reg_write(phydev, 0x30718, 0x10);
		if (ret < 0)
			return ret;
		ret = air_buckpbus_reg_write(phydev, 0x30718, 0x0);
		if (ret < 0)
			return ret;
		seq_puts(seq, "|\tBefore EF\n");
		seq_puts(seq, "| Tx to Line side_S        :");
		air_buckpbus_reg_read(phydev, 0x3071c, &pkt_cnt);
		seq_printf(seq, "%010u |\n", pkt_cnt);
		seq_puts(seq, "| Tx to Line side_T        :");
		air_buckpbus_reg_read(phydev, 0x30720, &pkt_cnt);
		seq_printf(seq, "%010u |\n", pkt_cnt);
		seq_puts(seq, "| Tx_ENC                   :");
		air_buckpbus_reg_read(phydev, 0x30724, &pkt_cnt);
		seq_printf(seq, "%010u |\n", pkt_cnt);
		seq_puts(seq, "| Rx from Line side_S      :");
		air_buckpbus_reg_read(phydev, 0x3072c, &pkt_cnt);
		seq_printf(seq, "%010u |\n", pkt_cnt);
		seq_puts(seq, "| Rx from Line side_T      :");
		air_buckpbus_reg_read(phydev, 0x30730, &pkt_cnt);
		seq_printf(seq, "%010u |\n", pkt_cnt);
		seq_puts(seq, "| Rx_DEC                   :");
		air_buckpbus_reg_read(phydev, 0x30728, &pkt_cnt);
		seq_printf(seq, "%010u |\n", pkt_cnt);
		seq_puts(seq, "|\tAfter EF\n");
		seq_puts(seq, "| Tx to Line side_S        :");
		air_buckpbus_reg_read(phydev, 0x30734, &pkt_cnt);
		seq_printf(seq, "%010u |\n", pkt_cnt);
		seq_puts(seq, "| Tx to Line side_T        :");
		air_buckpbus_reg_read(phydev, 0x30738, &pkt_cnt);
		seq_printf(seq, "%010u |\n", pkt_cnt);
		seq_puts(seq, "| Rx from Line side_S      :");
		air_buckpbus_reg_read(phydev, 0x30764, &pkt_cnt);
		seq_printf(seq, "%010u |\n", pkt_cnt);
		seq_puts(seq, "| Rx from Line side_T      :");
		air_buckpbus_reg_read(phydev, 0x30768, &pkt_cnt);
		seq_printf(seq, "%010u |\n\n", pkt_cnt);
		ret = air_buckpbus_reg_write(phydev, 0x30718, 0x13);
		ret |= air_buckpbus_reg_write(phydev, 0x30718, 0x3);
		ret |= air_buckpbus_reg_write(phydev, 0x30718, 0x10);
		ret |= air_buckpbus_reg_write(phydev, 0x30718, 0x0);
		if (ret < 0)
			return ret;
	}
	if (priv->link && ((priv->speed != SPEED_2500))) {
		seq_puts(seq, "|\t<<LS Counter>>\n");
		ret = phy_write(phydev, 0x1f, 1);
		if (ret < 0)
			return ret;
		seq_puts(seq, "| Rx from Line side        :");
		pkt_cnt = phy_read(phydev, 0x12) & 0x7fff;
		seq_printf(seq, "%010u |\n", pkt_cnt);
		seq_puts(seq, "| Rx Error from Line side  :");
		pkt_cnt = phy_read(phydev, 0x17) & 0xff;
		seq_printf(seq, "%010u |\n", pkt_cnt);
		ret = phy_write(phydev, 0x1f, 0);
		if (ret < 0)
			return ret;
		ret = phy_write(phydev, 0x1f, 0x52B5);
		ret |= phy_write(phydev, 0x10, 0xBF92);
		if (ret < 0)
			return ret;
		seq_puts(seq, "| Tx to Line side          :");
		pkt_cnt = (phy_read(phydev, 0x11) & 0x7ffe) >> 1;
		seq_printf(seq, "%010u |\n", pkt_cnt);
		seq_puts(seq, "| Tx Error to Line side    :");
		pkt_cnt = phy_read(phydev, 0x12);
		pkt_cnt &= 0x7f;
		seq_printf(seq, "%010u |\n\n", pkt_cnt);
		ret = phy_write(phydev, 0x1f, 0);
		if (ret < 0)
			return ret;
	}
	return ret;
}

static int airphy_counter_open(struct inode *inode, struct file *file)
{
	return single_open(file, airphy_counter_show, inode->i_private);
}

static ssize_t airphy_polarity_write(struct file *file, const char __user *ptr,
				     size_t len, loff_t *off)
{
	struct phy_device *phydev = file->private_data;
	char buf[32], param1[32], param2[32];
	int count = len, ret = 0, tx_rx = 0;

	memset(buf, 0, 32);
	memset(param1, 0, 32);
	memset(param2, 0, 32);

	if (count > sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, ptr, len))
		return -EFAULT;
	if (sscanf(buf, "%12s %12s", param1, param2) == -1)
		return -EFAULT;

	if (!strncmp("tx_normal", param1, strlen("tx_normal"))) {
		if (!strncmp("rx_normal", param2, strlen("rx_normal"))) {
			tx_rx = AIR_POL_TX_NOR_RX_NOR;
		} else if (!strncmp("rx_reverse", param2, strlen("rx_reverse"))) {
			tx_rx = AIR_POL_TX_NOR_RX_REV;
		} else {
			pr_notice("\nRx param is not correct.\n");
			return -EINVAL;
		}
	} else if (!strncmp("tx_reverse", param1, strlen("tx_reverse"))) {
		if (!strncmp("rx_normal", param2, strlen("rx_normal"))) {
			tx_rx = AIR_POL_TX_REV_RX_NOR;
		} else if (!strncmp("rx_reverse", param2, strlen("rx_reverse"))) {
			tx_rx = AIR_POL_TX_REV_RX_REV;
		} else {
			pr_notice("\nRx param is not correct.\n");
			return -EINVAL;
		}
	} else {
		air_polarity_help();
		return count;
	}
	pr_notice("\nSet Polarity %s\n", tx_rx_string[tx_rx]);
	ret = air_set_polarity(phydev, tx_rx);
	if (ret < 0)
		return ret;
	return count;
}

static void airphy_port_mode_help(void)
{
	pr_notice("\nUsage:\n"
		  "[debugfs] = /sys/kernel/debug/mdio-bus\':[phy_addr]\n"
		  "echo [mode] [para] > /[debugfs]/port_mode\n"
		  "echo re-an > /[debugfs]/port_mode\n"
		  "echo auto > /[debugfs]/port_mode\n"
		  "echo 2500 > /[debugfs]/port_mode\n"
		  "echo 1000 > /[debugfs]/port_mode\n"
		  "echo 100 > /[debugfs]/port_mode\n"
		  "echo power up/down >  /[debugfs]/port_mode\n");
}

static ssize_t airphy_port_mode(struct file *file, const char __user *ptr,
				size_t len, loff_t *off)
{
	struct phy_device *phydev = file->private_data;
	char buf[32], cmd[32], param[32];
	int count = len, ret = 0;
	int num = 0, val = 0;

	memset(buf, 0, 32);
	memset(cmd, 0, 32);
	memset(param, 0, 32);

	if (count > sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, ptr, len))
		return -EFAULT;

	num = sscanf(buf, "%8s %8s", cmd, param);
	if (num < 1 || num > 3)
		return -EFAULT;

	if (!strncmp("auto", cmd, strlen("auto"))) {
		ret = air_set_mode(phydev, AIR_PORT_MODE_AUTONEGO);
	} else if (!strncmp("2500", cmd, strlen("2500"))) {
		ret = air_set_mode(phydev, AIR_PORT_MODE_FORCE_2500);
	} else if (!strncmp("1000", cmd, strlen("1000"))) {
		ret = air_set_mode(phydev, AIR_PORT_MODE_FORCE_1000);
	} else if (!strncmp("100", cmd, strlen("100"))) {
		ret = air_set_mode(phydev, AIR_PORT_MODE_FORCE_100);
	} else if (!strncmp("re-an", cmd, strlen("re-an"))) {
		val = phy_read(phydev, MII_BMCR) | BIT(9);
		ret = phy_write(phydev, MII_BMCR, val);
	} else if (!strncmp("power", cmd, strlen("power"))) {
		if (!strncmp("down", param, strlen("down")))
			ret = air_set_mode(phydev, AIR_PORT_MODE_POWER_DOWN);
		else if (!strncmp("up", param, strlen("up")))
			ret = air_set_mode(phydev, AIR_PORT_MODE_POWER_UP);
	} else if (!strncmp("help", cmd, strlen("help"))) {
		airphy_port_mode_help();
	}

	if (ret < 0)
		return ret;

	return count;
}

static void airphy_debugfs_buckpbus_help(void)
{
	pr_notice("\nUsage:\n"
		  "[debugfs] = /sys/kernel/debug/mdio-bus\':[phy_addr]\n"
		  "Read:\n"
		  "echo r [buckpbus_register] > /[debugfs]/buckpbus_op\n"
		  "Write:\n"
		  "echo w [buckpbus_register] [value] > /[debugfs]/buckpbus_op\n");
}

static ssize_t airphy_debugfs_buckpbus(struct file *file,
				       const char __user *buffer, size_t count,
				       loff_t *data)
{
	struct phy_device *phydev = file->private_data;
	char buf[64];
	int ret = 0, i;
	unsigned int reg, val, num;

	memset(buf, 0, 64);
	if (count > sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	if (buf[0] == 'w') {
		if (sscanf(buf, "w %15x %15x", &reg, &val) == -1)
			return -EFAULT;

		pr_notice("\nphy=%d, reg=0x%x, val=0x%x\n",
			phydev_addr(phydev), reg, val);
		ret = air_buckpbus_reg_write(phydev, reg, val);
		if (ret < 0) {
			pr_notice("\nbuckpbus_reg_write fail\n");
			return -EIO;
		}
		air_buckpbus_reg_read(phydev, reg, &val);
		pr_notice("\nphy=%d, reg=0x%x, val=0x%x confirm..\n",
			phydev_addr(phydev), reg, val);
	} else if (buf[0] == 'r') {
		if (sscanf(buf, "r %15x", &reg) == -1)
			return -EFAULT;

		air_buckpbus_reg_read(phydev, reg, &val);
		pr_notice("\nphy=%d, reg=0x%x, val=0x%x\n",
		phydev_addr(phydev), reg, val);
	} else if (buf[0] == 'x') {
		if (sscanf(buf, "x %15x %6d", &reg, &num) == -1)
			return -EFAULT;
		if (num > 0x1000 || num == 0) {
			pr_notice("\nphy%d: number(0x%x) is invalid number\n",
				phydev_addr(phydev), num);
			return -EFAULT;
		}
		for (i = 0; i < num; i++) {
			air_buckpbus_reg_read(phydev, (reg + (i * 4)), &val);
			pr_notice("phy=%d, reg=0x%x, val=0x%x",
				phydev_addr(phydev), reg + (i * 4), val);
			pr_notice("");
		}
	} else
		airphy_debugfs_buckpbus_help();

	return count;
}

static int airphy_link_status(struct seq_file *seq, void *v)
{
	int ret = 0;
	struct phy_device *phydev = seq->private;
	struct en8811h_priv *priv = phydev->priv;

	ret = air_read_status(phydev);
	if (ret < 0)
		return ret;

	seq_printf(seq, "%s Information:\n", dev_name(phydev_dev(phydev)));
	seq_printf(seq, "\tPHYAD: %02d\n", phydev_addr(phydev));
	seq_printf(seq, "\tLink Status: %s\n", priv->link ? "UP" : "DOWN");
	if (priv->link) {
		ret = air_get_autonego(phydev, &priv->an);
		if (ret < 0)
			return ret;
		seq_printf(seq, "\tAuto-Nego: %s\n",
				priv->an ? "on" : "off");
		seq_puts(seq, "\tSpeed: ");
		if (priv->speed == SPEED_UNKNOWN)
			seq_printf(seq, "Unknown! (%i)\n", priv->speed);
		else
			seq_printf(seq, "%uMb/s\n", priv->speed);

		seq_printf(seq, "\tDuplex: %s\n",
			   priv->duplex ? "Full" : "Half");
		seq_puts(seq, "\n");
	}

	return ret;
}

static int airphy_link_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, airphy_link_status, inode->i_private);
}

static int dbg_regs_show(struct seq_file *seq, void *v)
{
	struct phy_device *phydev = seq->private;
	int reg;
	unsigned int val;

	seq_puts(seq, "\t<<DEBUG REG DUMP>>\n");
	for (reg = MII_BMCR; reg <= MII_STAT1000; reg++) {
		seq_printf(seq, "| RG_MII_REG_%02x         : 0x%08x |\n",
			   reg, phy_read(phydev, reg));
	}

	seq_printf(seq, "| RG_MII_2G5_ADV        : 0x%08x |\n",
		   phy_read_mmd(phydev, 0x7, 0x20));

	air_buckpbus_reg_read(phydev, 0x3b30, &val);
	seq_printf(seq, "| RG_MII_2G5_LP         : 0x%08x |\n", val);

	seq_printf(seq, "| RG_MII_REF_CLK        : 0x%08x |\n",
		   phy_read(phydev, 0x1d));

	air_buckpbus_reg_read(phydev, 0xcf910, &val);
	seq_printf(seq, "| RG_HW_STRAP1          : 0x%08x |\n", val);

	air_buckpbus_reg_read(phydev, 0xcf914, &val);
	seq_printf(seq, "| RG_HW_STRAP2          : 0x%08x |\n", val);

	air_buckpbus_reg_read(phydev, 0xca0f8, &val);
	seq_printf(seq, "| RG_PHY_ANA            : 0x%08x |\n", val);

	air_buckpbus_reg_read(phydev, 0xe0004, &val);
	seq_printf(seq, "| RG_SYS_LINK_MODE      : 0x%08x |\n", val);

	air_buckpbus_reg_read(phydev, 0xe000C, &val);
	seq_printf(seq, "| RG_FCM_CTRL           : 0x%08x |\n", val);

	air_buckpbus_reg_read(phydev, 0xe0020, &val);
	seq_printf(seq, "| RG_SS_PAUSE_TIME      : 0x%08x |\n", val);

	air_buckpbus_reg_read(phydev, 0xe002C, &val);
	seq_printf(seq, "| RG_MIN_IPG_NUM        : 0x%08x |\n", val);

	air_buckpbus_reg_read(phydev, 0xc0000, &val);
	seq_printf(seq, "| RG_CSR_AN0            : 0x%08x |\n", val);

	air_buckpbus_reg_read(phydev, 0xc0b04, &val);
	seq_printf(seq, "| RG_SS_LINK_STATUS     : 0x%08x |\n", val);

	air_buckpbus_reg_read(phydev, 0x1020c, &val);
	seq_printf(seq, "| RG_FN_PWR_CTRL_STATUS : 0x%08x |\n", val);

	seq_printf(seq, "| RG_MD32_FW_READY      : 0x%08x |\n",
		   phy_read_mmd(phydev, 0x1e, 0x8009));

	air_buckpbus_reg_read(phydev, 0x3a64, &val);
	seq_printf(seq, "| RG_RX_SYNC_CNT        : 0x%08x |\n", val);

	air_buckpbus_reg_read(phydev, 0x3a48, &val);
	seq_printf(seq, "| RG_WHILE_LOOP_COUNT   : 0x%08x |\n", val);

	air_buckpbus_reg_read(phydev, 0x3afc, &val);
	seq_printf(seq, "| RG_GOBL_CR            : 0x%08x |\n", val);

	air_buckpbus_reg_read(phydev, 0x3a9c, &val);
	seq_printf(seq, "| RG_SW_DBG_FLAG        : 0x%08x |\n", val);

	return 0;
}

static int airphy_dbg_regs_show_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_regs_show, inode->i_private);
}


static int airphy_temp_show(struct seq_file *seq, void *v)
{
	struct phy_device *phydev = seq->private;
	int ret = 0;
	u32 pbus_value = 0;

	seq_puts(seq, "<<AIR EN8811H Temp>>\n");
	ret |= phy_write_mmd(phydev, 0x1e, 0x800e, 0x1100);
	ret |= phy_write_mmd(phydev, 0x1e, 0x800f, 0xe5);
	if (ret < 0) {
		pr_notice("\nmii_cl45_write fail\n");
		return -EIO;
	}
	air_buckpbus_reg_read(phydev, 0x3B38, &pbus_value);
	seq_printf(seq, "| Temperature  : %dC |\n", pbus_value);
	seq_puts(seq, "\n");

	return 0;
}

static int airphy_temp_show_open(struct inode *inode, struct file *file)
{
	return single_open(file, airphy_temp_show, inode->i_private);
}


static unsigned int air_read_lp_speed(struct phy_device *phydev)
{
	int val = 0, ret = 0;
	int lpa, lpagb;
	int count = 15;
	struct en8811h_priv *priv = phydev->priv;

	if (priv->firmware_version < 0x24011202) {
		val = phy_read(phydev, MII_BMCR) | BIT(9);
		ret = phy_write(phydev, MII_BMCR, val);
		if (unlikely(ret < 0))
			return ret;
		msleep(1500);
		do {
			msleep(100);
			ret = phy_read_mmd(phydev, MDIO_MMD_AN, 0x21);
			ret &= BIT(5);
			if (ret)
				break;
			count--;
		} while (count);

		count = 10;
		do {
			msleep(500);
			val = phy_read(phydev, MII_BMSR);
			if (val < 0) {
				phydev_err(phydev, "MII_BMSR reg 0x%x!\n", val);
				return val;
			}
			val = phy_read(phydev, MII_BMSR);
			if (val < 0) {
				phydev_err(phydev, "MII_BMSR reg 0x%x!\n", val);
				return val;
			}
			phydev_dbg(phydev, "val 0x%x\n", val);
			if (val & BMSR_LSTATUS) {
				val = phy_read(phydev, MII_LPA);
				if (val < 0)
					return val;
				lpa = (val & (BIT(5) | BIT(6) | BIT(7) | BIT(8))) >> 5;
				val = phy_read(phydev, MII_STAT1000);
				if (val < 0)
					return val;
				lpagb = GET_BIT(val, 11) << 4;
				ret |= (lpagb | lpa);
				return ret;
			}
		} while (count--);
	} else {
		ret = phy_read(phydev, MII_BMSR);
		if (ret < 0) {
			phydev_err(phydev, "MII_BMSR reg 0x%x!\n", ret);
			return ret;
		}
		ret = phy_read(phydev, MII_BMSR);
		if (ret < 0) {
			phydev_err(phydev, "MII_BMSR reg 0x%x!\n", ret);
			return ret;
		}
		phydev_dbg(phydev, "val 0x%x\n", ret);
		if (ret & BMSR_LSTATUS) {
			air_buckpbus_reg_read(phydev, 0x3b30, &ret);
			ret = GET_BIT(ret, 0) << 5;
			lpa = phy_read(phydev, MII_LPA);
			if (lpa < 0)
				return lpa;
			lpa &= (BIT(5) | BIT(6) | BIT(7) | BIT(8));
			lpa >>= 5;
			lpagb = phy_read(phydev, MII_STAT1000);
			if (lpagb < 0)
				return lpagb;
			lpagb = GET_BIT(lpagb, 11) << 4;
			ret |= (lpagb | lpa);
			return ret;
		}
	}
	return 0;
}

static int airphy_lp_speed(struct seq_file *seq, void *v)
{
	unsigned int ret = 0, val = 0, did1 = 0, i;
	struct phy_device *phydev = seq->private;
	static const struct {
		unsigned int bit_index;
		const char *name;
	} mode_defs[] = {
		{ AIR_LINK_MODE_10baseT_Half_BIT,
		"10baseT/Half" },
		{ AIR_LINK_MODE_10baseT_Full_BIT,
		"10baseT/Full" },
		{ AIR_LINK_MODE_100baseT_Half_BIT,
		"100baseT/Half" },
		{ AIR_LINK_MODE_100baseT_Full_BIT,
		"100baseT/Full" },
		{ AIR_LINK_MODE_1000baseT_Full_BIT,
		"1000baseT/Full" },
		{ AIR_LINK_MODE_2500baseT_Full_BIT,
		"2500baseT/Full" }
	};

	seq_printf(seq, "%s Link Partner Ability:\n",
		   dev_name(phydev_dev(phydev)));
	ret = air_read_lp_speed(phydev);
	if (val < 0)
		return val;
	for (i = 0; i < ARRAY_SIZE(mode_defs); i++) {
		if (ret & BIT(mode_defs[i].bit_index)) {
			seq_printf(seq, "\t\t\t %s\n",
				   mode_defs[i].name);
			did1++;
		}
	}
	if (did1 == 0)
		seq_puts(seq, "\t\t\t Not reported\n");

	return 0;
}

static int airphy_lp_speed_open(struct inode *inode, struct file *file)
{
	return single_open(file, airphy_lp_speed, inode->i_private);
}

static void airphy_debugfs_mii_cl22_help(void)
{
	pr_notice("\nUsage:\n"
		  "[debugfs] = /sys/kernel/debug/mdio-bus\':[phy_addr]\n"
		  "Read:\n"
		  "echo r [phy_register] > /[debugfs]/mii_mgr_op\n"
		  "Write:\n"
		  "echo w [phy_register] [value] > /[debugfs]/mii_mgr_op\n");
}


static ssize_t airphy_debugfs_cl22(struct file *file,
				   const char __user *buffer, size_t count,
				   loff_t *data)
{
	struct phy_device *phydev = file->private_data;


	char buf[64];
	int ret = 0;
	unsigned int reg, val;

	memset(buf, 0, 64);
	if (count > sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	if (buf[0] == 'w') {
		if (sscanf(buf, "w %15x %15x", &reg, &val) == -1)
			return -EFAULT;

		pr_notice("\nphy=%d, reg=0x%x, val=0x%x\n",
			  phydev_addr(phydev), reg, val);
		ret = phy_write(phydev, reg, val);
		if (ret < 0) {
			pr_notice("\nmii_cl22_write fail\n");
			return -EIO;
		}
		val = phy_read(phydev, reg);
		pr_notice("\nphy=%d, reg=0x%x, val=0x%x confirm..\n",
			  phydev_addr(phydev), reg, val);
	} else if (buf[0] == 'r') {
		if (sscanf(buf, "r %15x", &reg) == -1)
			return -EFAULT;

		val = phy_read(phydev, reg);
		pr_notice("\nphy=%d, reg=0x%x, val=0x%x\n",
			  phydev_addr(phydev), reg, val);
	} else
		airphy_debugfs_mii_cl22_help();

	return count;
}

static void airphy_debugfs_mii_cl45_help(void)
{
	pr_notice("\nUsage:\n"
		  "[debugfs] = /sys/kernel/debug/mdio-bus\':[phy_addr]\n"
		  "Read:\n"
		  "echo r [device number] [phy_register] > /[debugfs]/cl45_op\n"
		  "Write:\n"
		  "echo w [device number] [phy_register] [value] > /[debugfs]/cl45_op\n");
}

static ssize_t airphy_debugfs_cl45(struct file *file,
				   const char __user *buffer, size_t count,
				   loff_t *data)
{
	struct phy_device *phydev = file->private_data;
	char buf[64];
	int ret = 0;
	unsigned int reg, val, devnum;

	memset(buf, 0, 64);
	if (count > sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	if (buf[0] == 'w') {
		if (sscanf(buf, "w %15x %15x %15x", &devnum, &reg, &val) == -1)
			return -EFAULT;

		pr_notice("\nphy=%d, devnum=0x%x, reg=0x%x, val=0x%x\n",
			  phydev_addr(phydev), devnum, reg, val);
		ret = phy_write_mmd(phydev, devnum, reg, val);
		if (ret < 0) {
			pr_notice("\nmii_cl45_write fail\n");
			return -EIO;
		}
		val = phy_read_mmd(phydev, devnum, reg);
		pr_notice("\nphy=%d, devnum=0x%x, reg=0x%x, val=0x%x confirm..\n",
			  phydev_addr(phydev), devnum, reg, val);
	} else if (buf[0] == 'r') {
		if (sscanf(buf, "r %15x %15x", &devnum, &reg) == -1)
			return -EFAULT;

		val = phy_read_mmd(phydev, devnum, reg);
		pr_notice("\nphy=%d, devnum=0x%x, reg=0x%x, val=0x%x\n",
			  phydev_addr(phydev), devnum, reg, val);
	} else
		airphy_debugfs_mii_cl45_help();

	return count;
}

static void
air_cable_pair_swap(int pair0, int pair1,
	struct air_cable_test_rsl *ptr_cable)
{
	int tmp_status;
	unsigned int tmp_length;

	tmp_status = ptr_cable->status[pair0];
	tmp_length = ptr_cable->length[pair0];
	ptr_cable->status[pair0] = ptr_cable->status[pair1];
	ptr_cable->length[pair0] = ptr_cable->length[pair1];
	ptr_cable->status[pair1] = tmp_status;
	ptr_cable->length[pair1] = tmp_length;
}

static int air_wait_md32_fw(struct phy_device *phydev)
{
	int reg;
	int retry = 60;

	do {
		msleep(1000);
		reg = phy_read_mmd(phydev, 0x1e, 0x8009);
		if (reg == EN8811H_PHY_READY) {
			phydev_dbg(phydev, "%s: PHY ready!\n", __func__);
			break;
		}
		if (!(retry % 10))
			phydev_info(phydev, "Waiting for MD32 FW ready....(%d)", reg);

		if (!retry) {
			phydev_err(phydev, "%s: MD32 FW is not ready.(Status: 0x%x)\n",
				   __func__, reg);
			return -EPERM;
		}
	} while (retry--);
	return 0;
}

static int air_cable_normal(struct phy_device *phydev,
			    int step_col_12, int step_col_13)
{
	int ret = 0, reg;

	ret |= phy_write(phydev, 0x1f, 0);
	reg = ((phy_read(phydev, 0x11)  & BIT(12)) >> 12);
	if (!reg) {
		reg = phy_read(phydev, MII_BMCR);
		ret |= phy_write(phydev,
				MII_BMCR, reg | BMCR_PDOWN);
		ret |= phy_write(phydev,
				MII_BMCR, reg & ~BMCR_PDOWN);
		phydev_dbg(phydev, "%s Power Down -> Power UP.\n", __func__);
		msleep(1000);
		ret |= phy_write(phydev, 0x1f, 0x1);
		ret |= phy_write(phydev, 0x18, 0x8000);
		ret |= air_wait_md32_fw(phydev);
		if (ret < 0) {
			phydev_err(phydev,
				   "%s: cable normal 1-1 fail!\n", __func__);
			return -EPERM;
		}
	}
	ret |= phy_write_mmd(phydev, 0x1e, 0x800e, 0x1100);
	ret |= phy_write_mmd(phydev, 0x1e, 0x800f, 0xd4);
	ret |= air_wait_md32_fw(phydev);
	if (ret < 0) {
		phydev_err(phydev,
			   "%s: cable normal 1-2 fail!\n", __func__);
		return -EPERM;
	}
	return 0;
}

static int airphy_cable_step2(struct phy_device *phydev)
{
	int ret = 0;
	int step_col_12 = 0;
	int step_col_13 = 0;


	ret = air_cable_normal(phydev, step_col_12, step_col_13);
	if (ret < 0) {
		phydev_err(phydev, "%s: air_cable_normal fail (ret=%d)\n",
			   __func__, ret);
		return -EPERM;
	}
	step_col_12 = phy_read_mmd(phydev, 0x1e, 0x800c);
	step_col_13 = phy_read_mmd(phydev, 0x1e, 0x800d);
	phydev_dbg(phydev, "%s step_col_12 %d, step_col_13 %d\n",
		   __func__, step_col_12, step_col_13);
	phydev_dbg(phydev, "%s: successful\n", __func__);
	return 0;
}

static int airphy_cable_step4(struct phy_device *phydev,
			      struct air_cable_test_rsl *cable_rsl)
{
	int reg = 0, status, test_pair = 0;
	unsigned int pbus_value = 0, cable_len, cable_info;
	int ret = 0;
	static const char * const pair_str[] = {"A", "B", "C", "D"};

	msleep(1000);
	ret |= phy_write_mmd(phydev, 0x1e, 0x800e, 0x1100);
	ret |= phy_write_mmd(phydev, 0x1e, 0x800f, 0xFA);
	ret |= air_wait_md32_fw(phydev);
	if (ret < 0) {
		phydev_err(phydev,
			   "%s: cable step4 fail!\n", __func__);
		return -EPERM;
	}
	ret |= phy_write(phydev, 0x1f, 0x0);
	reg = ((phy_read(phydev, 0x11)  & BIT(12)) >> 12);
	for (test_pair = 0; test_pair < 4; test_pair++) {
		air_buckpbus_reg_read(phydev, 0x11e0 + test_pair * 4, &cable_info);
		phydev_dbg(phydev, "pair%s cable_info(0x%x) 0x%x\n",
			   pair_str[test_pair], 0x11e0 + test_pair * 4, pbus_value);
		cable_len = cable_info & 0xffff;
		status = cable_info >> 16;
		if (!status) {
			phydev_dbg(phydev, "Pair %d, term=%d, len=%d\n",
				   test_pair, status, cable_len);
		} else {
			cable_rsl->status[test_pair] = status;
			cable_rsl->length[test_pair] = cable_len;
			phydev_dbg(phydev, "2.pair %s, status %d, cable_len %d\n",
				   pair_str[test_pair], cable_rsl->status[test_pair],
				   cable_rsl->length[test_pair]);
		}
	}
	if (!reg) {
		phydev_dbg(phydev, "%s: air_cable_pair_swap\n", __func__);
		air_cable_pair_swap(AIR_PORT_CABLE_TEST_PAIR_A,
				    AIR_PORT_CABLE_TEST_PAIR_B, cable_rsl);
		air_cable_pair_swap(AIR_PORT_CABLE_TEST_PAIR_C,
				    AIR_PORT_CABLE_TEST_PAIR_D, cable_rsl);
	}
	ret |= phy_write_mmd(phydev, 0x1E, 0x800D, 0x0);
	if (ret < 0)
		return ret;
	phydev_dbg(phydev, "%s successful\n", __func__);
	return 0;
}

static int airphy_cable_step1(struct phy_device *phydev)
{
	int ret = 0, pair, len = 0, reg, retry;
	static const char * const link_str[] = {"X", "100M", "1G", "2.5G", "LinkDown"};
	unsigned int pbus_value = 0;
	struct air_cable_test_rsl cable_rsl = {0};
	char str_out[CMD_MAX_LENGTH] = {0};
	struct en8811h_priv *priv = phydev->priv;
	int status;

	air_buckpbus_reg_read(phydev, 0x3b3c, &pbus_value);
	phydev_info(phydev, "MD32 FW Version: %x\n", pbus_value);
	msleep(1000);
	ret |= phy_write_mmd(phydev, 0x1E, 0x800e, 0x1100);
	ret |= phy_write_mmd(phydev, 0x1E, 0x800f, 0xd6);
	if (ret < 0)
		return ret;
	retry = 5;
	do {
		status = air_wait_md32_fw(phydev);
		if (!retry) {
			phydev_info(phydev, "%s: md32 fw is not ready(%d)", __func__, status);
			return -EPERM;
		}
		retry--;
	} while (status);
	air_buckpbus_reg_read(phydev, 0x11fc, &pbus_value);
	if ((pbus_value == 1) || (pbus_value == 3)) {
		phydev_info(phydev, "%s -> No Support!\n",
			    link_str[pbus_value]);
	} else if (pbus_value == 2) {
		phydev_info(phydev, "%s - Link-Up Mode!\n",
			    link_str[pbus_value]);
	} else if (pbus_value == 4) {
		phydev_info(phydev, "Link-Down Mode!\n");
	}
	if ((pbus_value == 2) || (pbus_value == 4)) {
		ret = airphy_cable_step2(phydev);
		if (ret < 0)
			goto phy_reset;
		ret = airphy_cable_step4(phydev, &cable_rsl);
		if (ret < 0)
			goto phy_reset;
	}
	phydev_info(phydev, "%7s %15s %15s %15s\n", "pair-a", "pair-b", "pair-c", "pair-d");
	phydev_info(phydev, "%7s %7s %7s %7s %7s %7s %7s %7s\n",
		    "status", "length", "status", "length", "status", "length", "status", "length");
	for (pair = 0; pair < 4; pair++) {
		if (cable_rsl.status[pair] == AIR_PORT_CABLE_STATUS_ERROR)
			len += snprintf(str_out + len, CMD_MAX_LENGTH, "%7s", "error");
		else if (cable_rsl.status[pair] == AIR_PORT_CABLE_STATUS_OPEN)
			len += snprintf(str_out + len, CMD_MAX_LENGTH, "%7s", " open");
		else if (cable_rsl.status[pair] == AIR_PORT_CABLE_STATUS_SHORT)
			len += snprintf(str_out + len, CMD_MAX_LENGTH, "%7s", " short");
		else if (cable_rsl.status[pair] == AIR_PORT_CABLE_STATUS_NORMAL)
			len += snprintf(str_out + len, CMD_MAX_LENGTH, "%7s", "normal");

		len += snprintf(str_out + len, CMD_MAX_LENGTH, "  %3d.%dm ",
			cable_rsl.length[pair] / 10, cable_rsl.length[pair] % 10);
		priv->pair[pair] = cable_rsl.length[pair];
	}
	phydev_info(phydev, "%s", str_out);
	phydev_info(phydev, "%s air_cable_diag sucessfull.\n", __func__);

	return 0;
phy_reset:
	phydev_info(phydev, "%s air_cable_diag fail.\n", __func__);
	phydev_info(phydev, "%s phy_reset.\n", __func__);
	ret |= phy_write(phydev, 0x1f, 0x0);
	reg = phy_read(phydev, MII_BMCR);
	ret |= phy_write(phydev, MII_BMCR, reg | BMCR_PDOWN);
	ret |= phy_write(phydev, MII_BMCR, reg & ~BMCR_PDOWN);
	if (ret < 0)
		return ret;
	msleep(1000);
	phydev_dbg(phydev, "%s Power Down -> Power UP.\n", __func__);

	return 0;
}

static void air_token_ring_write(struct phy_device *phydev, struct trrg_s TrRG,
				 unsigned int WrtVal)
{
	int data;

	unsigned int TmpVal;
	struct hal_tr_data_s TR_DATA;

	phy_lock_mdio_bus(phydev);
	data = __phy_read_mmd(phydev, 0x1e, 0x148);
	AIR_EN8811H_SET_VALUE(data, 1, 9, 1);
	__phy_write_mmd(phydev, 0x1e, 0x148, data);
	/* change page to 0x52b5 */
	__phy_write(phydev, 0x1F, 0x52B5);

	/* write addr */
	TrRG.TrRGDesc.param.WR_RD_CTRL = 1;
	__phy_write(phydev, 0x10, TrRG.TrRGDesc.Raw[1]);
	/* read data */
	TR_DATA.data_hi = __phy_read(phydev, 0x12);
	TR_DATA.data_lo = __phy_read(phydev, 0x11);
	TmpVal = ~TrRG.RgMask &
		 (((unsigned int)TR_DATA.data_hi) << 16 | (unsigned int)TR_DATA.data_lo);

	TrRG.TrRGDesc.param.WR_RD_CTRL = 0;
	WrtVal = (WrtVal<<TrRG.TrRGDesc.param.TrRG_LSB) | (TmpVal & ~TrRG.RgMask);
	/* write data */
	__phy_write(phydev, 0x12, (WrtVal >> 16) & 0x00FF);
	__phy_write(phydev, 0x11, WrtVal & 0xFFFF);
	__phy_write(phydev, 0x10, TrRG.TrRGDesc.Raw[1]);

	/* change page to 0 */
	__phy_write(phydev, 0x1F, 0x0);

	data = __phy_read_mmd(phydev, 0x1E, 0x148);
	AIR_EN8811H_SET_VALUE(data, 0, 0x9, 0x1);
	__phy_write_mmd(phydev, 0x1E, 0x148, data);
	phy_unlock_mdio_bus(phydev);
}

static void airphy_trigger_cable_diag(struct phy_device *phydev)
{
	struct en8811h_priv *priv = phydev->priv;

	priv->running_status = 1;
	air_token_ring_write(phydev, _fldRW_tr_reg__EcVarTrainingTime_ECNC_C8h, 0x2);
	air_token_ring_write(phydev, _fldRW_tr_reg__EcVarTrainingGain_ECNC_C8h, 0x0);
	airphy_cable_step1(phydev);
	air_token_ring_write(phydev, _fldRW_tr_reg__EcVarTrainingTime_ECNC_C8h, 0xf4);
	air_token_ring_write(phydev, _fldRW_tr_reg__EcVarTrainingGain_ECNC_C8h, 0x1);
	priv->running_status = 0;
}

static void airphy_cable_diag_help(void)
{
	pr_notice("\nUsage:\n"
		  "[debugfs] = /sys/kernel/debug/mdio-bus\':[phy_addr]\n"
		  "echo start > /[debugfs]/cable_diag\n");
}

static ssize_t airphy_cable_diag(struct file *file, const char __user *ptr,
				 size_t len, loff_t *off)
{
	struct phy_device *phydev = file->private_data;
	char buf[32], cmd[32];
	int count = len, ret = 0;
	int num = 0;

	memset(buf, 0, 32);
	memset(cmd, 0, 32);

	if (count > sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, ptr, len))
		return -EFAULT;

	num = sscanf(buf, "%s", cmd);
	if (num != 1)
		return -EFAULT;

	if (!strncmp("start", cmd, strlen("start")))
		airphy_trigger_cable_diag(phydev);
	else
		airphy_cable_diag_help();

	if (ret < 0)
		return ret;

	return count;
}

static const struct file_operations airphy_lp_speed_fops = {
	.owner = THIS_MODULE,
	.open = airphy_lp_speed_open,
	.read = seq_read,
	.llseek = noop_llseek,
	.release = single_release,
};

static const struct file_operations airphy_info_fops = {
	.owner = THIS_MODULE,
	.open = airphy_info_open,
	.read = seq_read,
	.llseek = noop_llseek,
	.release = single_release,
};

static const struct file_operations airphy_counter_fops = {
	.owner = THIS_MODULE,
	.open = airphy_counter_open,
	.read = seq_read,
	.llseek = noop_llseek,
	.release = single_release,
};

static const struct file_operations airphy_debugfs_buckpbus_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = airphy_debugfs_buckpbus,
	.llseek = noop_llseek,
};

static const struct file_operations airphy_port_mode_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = airphy_port_mode,
	.llseek = noop_llseek,
};

static const struct file_operations airphy_polarity_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = airphy_polarity_write,
	.llseek = noop_llseek,
};

static const struct file_operations airphy_link_status_fops = {
	.owner = THIS_MODULE,
	.open = airphy_link_status_open,
	.read = seq_read,
	.llseek = noop_llseek,
	.release = single_release,
};

static const struct file_operations airphy_dbg_reg_show_fops = {
	.owner = THIS_MODULE,
	.open = airphy_dbg_regs_show_open,
	.read = seq_read,
	.llseek = noop_llseek,
	.release = single_release,
};

static const struct file_operations airphy_temp_fops = {
	.owner = THIS_MODULE,
	.open = airphy_temp_show_open,
	.read = seq_read,
	.llseek = noop_llseek,
	.release = single_release,
};

static const struct file_operations airphy_debugfs_cl22_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = airphy_debugfs_cl22,
	.llseek = noop_llseek,
};

static const struct file_operations airphy_debugfs_cl45_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = airphy_debugfs_cl45,
	.llseek = noop_llseek,
};

static const struct file_operations airphy_cable_diag_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = airphy_cable_diag,
	.llseek = noop_llseek,
};

static const struct file_operations airphy_led_mode_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = airphy_led_mode,
	.llseek = noop_llseek,
};

int airphy_debugfs_init(struct phy_device *phydev)
{
	int ret = 0;
	struct en8811h_priv *priv = phydev->priv;
	struct dentry *dir = priv->debugfs_root;

	dir = debugfs_create_dir(dev_name(phydev_dev(phydev)), NULL);
	if (!dir) {
		dev_err(phydev_dev(phydev), "%s:err at %d\n",
					 __func__, __LINE__);
		ret = -ENOMEM;
	}
	debugfs_create_file(DEBUGFS_DRIVER_INFO, S_IFREG | 0444,
			    dir, phydev,
			    &airphy_info_fops);
	debugfs_create_file(DEBUGFS_COUNTER, S_IFREG | 0444,
			    dir, phydev,
			    &airphy_counter_fops);
	debugfs_create_file(DEBUGFS_BUCKPBUS_OP, S_IFREG | 0200,
			    dir, phydev,
			    &airphy_debugfs_buckpbus_fops);
	debugfs_create_file(DEBUGFS_PORT_MODE, S_IFREG | 0200,
			    dir, phydev,
			    &airphy_port_mode_fops);
	debugfs_create_file(DEBUGFS_POLARITY, S_IFREG | 0200,
			    dir, phydev,
			    &airphy_polarity_fops);
	debugfs_create_file(DEBUGFS_LINK_STATUS, S_IFREG | 0444,
			    dir, phydev,
			    &airphy_link_status_fops);
	debugfs_create_file(DEBUGFS_DBG_REG_SHOW, S_IFREG | 0444,
			    dir, phydev,
			    &airphy_dbg_reg_show_fops);
	debugfs_create_file(DEBUGFS_TEMPERATURE, S_IFREG | 0444,
			    dir, phydev,
			    &airphy_temp_fops);
	debugfs_create_file(DEBUGFS_LP_SPEED, S_IFREG | 0444,
			    dir, phydev,
			    &airphy_lp_speed_fops);
	debugfs_create_file(DEBUGFS_MII_CL22_OP, S_IFREG | 0200,
			    dir, phydev,
			    &airphy_debugfs_cl22_fops);
	debugfs_create_file(DEBUGFS_MII_CL45_OP, S_IFREG | 0200,
			    dir, phydev,
			    &airphy_debugfs_cl45_fops);
	debugfs_create_file(DEBUGFS_CABLE_DIAG, S_IFREG | 0200,
			    dir, phydev,
			    &airphy_cable_diag_fops);
	debugfs_create_file(DEBUGFS_LED_MODE, S_IFREG | 0200,
			    dir, phydev,
			    &airphy_led_mode_fops);
	priv->debugfs_root = dir;
	return ret;
}

static void airphy_debugfs_remove(struct phy_device *phydev)
{
	struct en8811h_priv *priv = phydev->priv;

	debugfs_remove_recursive(priv->debugfs_root);
	priv->debugfs_root = NULL;
}
#endif /*CONFIG_AIR_EN8811H_PHY_DEBUGFS*/

MODULE_DEVICE_TABLE(mdio, en8811h_tbl);
MODULE_FIRMWARE(EN8811H_MD32_DM);
MODULE_FIRMWARE(EN8811H_MD32_DSP);

MODULE_DESCRIPTION("Airoha EN8811H PHY drivers");
MODULE_AUTHOR("Airoha");
MODULE_AUTHOR("Eric Woudstra <ericwouds@gmail.com>");
MODULE_LICENSE("GPL");
