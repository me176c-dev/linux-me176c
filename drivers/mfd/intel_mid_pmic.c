#include <linux/mfd/intel_soc_pmic.h>
#include <linux/mfd/intel_mid_pmic.h>

extern struct intel_soc_pmic *intel_pmic;

int intel_mid_pmic_readb(int reg)
{
	int ret;
	unsigned int result;

	if (!intel_pmic)
		return -EIO;

	ret = regmap_read(intel_pmic->regmap, reg, &result);
	if (ret)
		return ret;

	return result;
}

int intel_mid_pmic_writeb(int reg, u8 val)
{
	if (!intel_pmic)
		return -EIO;

	return regmap_write(intel_pmic->regmap, reg, val);
}

int intel_mid_pmic_setb(int reg, u8 mask)
{
	if (!intel_pmic)
		return -EIO;

	// ((val & ~mask) | (mask & mask)) == (val | mask)
	return regmap_update_bits(intel_pmic->regmap, reg, mask, mask);
}

int intel_mid_pmic_clearb(int reg, u8 mask)
{
	if (!intel_pmic)
		return -EIO;

	// ((val & ~mask) | (0 & mask)) == (val & ~mask)
	return regmap_update_bits(intel_pmic->regmap, reg, mask, 0);
}
