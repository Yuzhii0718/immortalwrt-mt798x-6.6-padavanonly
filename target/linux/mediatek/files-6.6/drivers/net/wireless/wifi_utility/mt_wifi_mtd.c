#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/concat.h>
#include <linux/mtd/partitions.h>
#if IS_ENABLED(CONFIG_MTD_UBI)
#include <linux/mtd/ubi.h>
#endif
#if defined (CONFIG_MIPS)
#include <asm/addrspace.h>
#endif

/*
 * Flash layouts which moved the factory data into a UBI volume (for example
 * the "OpenWrt UBI layout" used on several filogic boards) no longer provide
 * an MTD partition named "Factory"/"factory". The proprietary WiFi driver
 * still has to read its EEPROM from there, so fall back to reading the UBI
 * volume carrying that name when no matching MTD partition was found.
 */
#define WIFI_UBI_FACTORY_VOL	"factory"
#define WIFI_UBI_MAX_DEVICES	32

#if IS_ENABLED(CONFIG_MTD_UBI)
static int mt_ubi_read_volume_nm(const char *volname, loff_t from, size_t len,
				 unsigned char *buf)
{
	struct ubi_volume_desc *desc;
	struct ubi_volume_info vi;
	size_t done = 0;
	int ubi_num, err = -ENODEV;

	for (ubi_num = 0; ubi_num < WIFI_UBI_MAX_DEVICES; ubi_num++) {
		desc = ubi_open_volume_nm(ubi_num, volname, UBI_READONLY);
		if (IS_ERR(desc))
			continue;

		ubi_get_volume_info(desc, &vi);

		err = 0;
		while (done < len) {
			int lnum = (from + done) / vi.usable_leb_size;
			int offs = (from + done) % vi.usable_leb_size;
			size_t chunk = len - done;

			if (chunk > (size_t)(vi.usable_leb_size - offs))
				chunk = vi.usable_leb_size - offs;

			err = ubi_leb_read(desc, lnum, buf + done, offs, chunk, 0);
			if (err)
				break;
			done += chunk;
		}

		ubi_close_volume(desc);
		return err;
	}

	return err;
}
#else
static int mt_ubi_read_volume_nm(const char *volname, loff_t from, size_t len,
				 unsigned char *buf)
{
	return -ENODEV;
}
#endif

int mt_mtd_write_nm_wifi(char *name, loff_t to, size_t len, const u_char *buf)
{
	int ret = -1;
	size_t rdlen, wrlen;
	struct mtd_info *mtd;
	struct erase_info ei;
	u_char *bak = NULL;

	mtd = get_mtd_device_nm("Factory");

	if (IS_ERR(mtd))
		mtd = get_mtd_device_nm("factory");

	if (IS_ERR(mtd)) {
		printk("warning: ra_mtd_write: no \"Factory\" partition, "
		       "factory data is stored in read-only media\n");
		return -ENODEV;
	}

	if (len > mtd->erasesize) {
		put_mtd_device(mtd);
		return -E2BIG;
	}

	bak = kmalloc(mtd->erasesize, GFP_KERNEL);
	if (bak == NULL) {
		put_mtd_device(mtd);
		return -ENOMEM;
	}

	ret = mtd_read(mtd, 0, mtd->erasesize, &rdlen, bak);

	if (ret != 0) {
		put_mtd_device(mtd);
		kfree(bak);
		return ret;
	}

	if (rdlen != mtd->erasesize)
		printk("warning: ra_mtd_write: rdlen is not equal to erasesize\n");

	memcpy(bak + to, buf, len);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
	ei.mtd = mtd;
	ei.callback = NULL;
	ei.priv = 0;
#endif
	ei.addr = 0;
	ei.len = mtd->erasesize;
	ret = mtd_erase(mtd, &ei);

	if (ret != 0) {
		put_mtd_device(mtd);
		kfree(bak);
		return ret;
	}

	ret = mtd_write(mtd, 0, mtd->erasesize, &wrlen, bak);

	put_mtd_device(mtd);
	kfree(bak);
	return ret;
}
EXPORT_SYMBOL(mt_mtd_write_nm_wifi);


/*
 * The proprietary WiFi driver has no device tree support of its own, while the
 * DTS describes the per-band MAC addresses the same way it does for the mt76
 * driver, e.g.:
 *
 *	&wifi {
 *		band@0 {
 *			nvmem-cells = <&art_ethaddr 2>;
 *			nvmem-cell-names = "mac-address";
 *		};
 *		band@1 {
 *			nvmem-cells = <&art_ethaddr 3>;
 *			nvmem-cell-names = "mac-address";
 *		};
 *	};
 *
 * Read back such a MAC (nvmem cell or plain DT property) so the driver can use
 * it instead of the (often unpopulated) value in the factory EEPROM.
 *
 * Returns 0 on success, a negative errno otherwise.
 */
int mt_wifi_get_band_mac(int band, unsigned char *mac)
{
	struct device_node *wifi_np, *band_np;
	int ret = -ENODEV;
	u32 reg;

	if (!mac || band < 0)
		return -EINVAL;

	/*
	 * The WiFi node is /soc/wifi@18000000 (mediatek,mt798x-wmac), its
	 * per-band children are named band@N with "reg = <N>".
	 *
	 * Note: of_get_child_by_name() must not be used here, it compares
	 * names with of_node_name_eq() which ignores the unit address, so
	 * looking up "band@0" would never match. Match reg instead.
	 */
	for_each_node_by_name(wifi_np, "wifi") {
		for_each_child_of_node(wifi_np, band_np) {
			if (!of_property_read_u32(band_np, "reg", &reg) &&
			    reg == (u32)band)
				break;
		}

		if (!band_np)
			continue;

		ret = of_get_mac_address(band_np, mac);
		of_node_put(band_np);

		if (!ret) {
			of_node_put(wifi_np);
			pr_info("mt_wifi: band %d MAC from device tree = %pM\n",
				band, mac);
			return 0;
		}
	}
	of_node_put(wifi_np);

	pr_info("mt_wifi: no MAC address from device tree for band %d (err %d)\n",
		band, ret);

	return ret;
}
EXPORT_SYMBOL(mt_wifi_get_band_mac);

int mt_mtd_read_nm_wifi(char *name, loff_t from, size_t len, u_char *buf)
{
	int ret;
	size_t rdlen;
	struct mtd_info *mtd;

	mtd = get_mtd_device_nm("Factory");

	if (IS_ERR(mtd))
		mtd = get_mtd_device_nm("factory");

	if (IS_ERR(mtd)) {
		/*
		 * No matching MTD partition exists, the factory data may live
		 * in a UBI volume instead (MTD -> UBI layout migration). Both
		 * spellings are tried since the volume name depends on how the
		 * UBI image / volume was created.
		 */
		ret = mt_ubi_read_volume_nm(WIFI_UBI_FACTORY_VOL, from, len, buf);
		if (ret)
			ret = mt_ubi_read_volume_nm("Factory", from, len, buf);
		if (ret)
			printk("warning: ra_mtd_read_nm: no \"Factory\" partition/volume, ret=%d\n",
			       ret);
		return ret;
	}

	ret = mtd_read(mtd, from, len, &rdlen, buf);

	if (rdlen != len)
		printk("warning: ra_mtd_read_nm: rdlen is not equal to len\n");

	put_mtd_device(mtd);

	return ret;
}
EXPORT_SYMBOL(mt_mtd_read_nm_wifi);
