#ifndef __INTEL_MID_PMIC_H__
#define __INTEL_MID_PMIC_H__

#include <linux/mfd/intel_soc_pmic.h>

extern struct intel_soc_pmic *intel_pmic;

static inline int intel_mid_pmic_readb(int reg)
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

static inline int intel_mid_pmic_writeb(int reg, u8 val)
{
	if (!intel_pmic)
		return -EIO;

	return regmap_write(intel_pmic->regmap, reg, val);
}

static inline int intel_mid_pmic_setb(int reg, u8 mask)
{
	if (!intel_pmic)
		return -EIO;

	// ((val & ~mask) | (mask & mask)) == (val | mask)
	return regmap_update_bits(intel_pmic->regmap, reg, mask, mask);
}

static inline int intel_mid_pmic_clearb(int reg, u8 mask)
{
	if (!intel_pmic)
		return -EIO;

	// ((val & ~mask) | (0 & mask)) == (val & ~mask)
	return regmap_update_bits(intel_pmic->regmap, reg, mask, 0);
}
#endif
