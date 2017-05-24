/*
 * bq24192_charger.c - Charger driver for TI BQ24192,BQ24191 and BQ24190
 *
 * Copyright (C) 2011 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 * Author: Raj Pandey <raj.pandey@intel.com>
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/power_supply.h>
#include <linux/sfi.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include "wakelock.h"
#include <linux/version.h>
#include <linux/usb/otg.h>
#include <linux/extcon.h>
#include <linux/acpi.h>

#include <asm/uaccess.h>
#include <asm/intel_scu_ipc.h>

#define DRV_NAME "bq24192_charger"
#define DEV_NAME "bq24192"
#include "intel_mid_pmic.h"
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>

#define R_PMIC_CHGRIRQ 0x0A
#define R_PMIC_MIRQS0  0x17
#define R_PMIC_MIRQSX  0x18


/*
 * D0, D1, D2 can be used to set current limits
 * and D3, D4, D5, D6 can be used to voltage limits
 */
#define BQ24192_INPUT_SRC_CNTL_REG		0x0
#define INPUT_SRC_CNTL_EN_HIZ			(1 << 7)
#define BATTERY_NEAR_FULL(a)			((a * 98)/100)
/*
 * set input voltage lim to 4.68V. This will help in charger
 * instability issue when duty cycle reaches 100%.
 */
#define INPUT_SRC_VOLT_LMT_DEF                 (3 << 4)
#define INPUT_SRC_VOLT_LMT_444                 (7 << 3)
#define INPUT_SRC_VOLT_LMT_468                 (5 << 4)
#define INPUT_SRC_VOLT_LMT_476                 (0xB << 3)

#define INPUT_SRC_VINDPM_MASK                  (0xF << 3)
#define INPUT_SRC_LOW_VBAT_LIMIT               3600
#define INPUT_SRC_MIDL_VBAT_LIMIT              4000
#define INPUT_SRC_MIDH_VBAT_LIMIT              4200
#define INPUT_SRC_HIGH_VBAT_LIMIT              4350

/* D0, D1, D2 represent the input current limit */
#define INPUT_SRC_CUR_LMT0		0x0	/* 100mA */
#define INPUT_SRC_CUR_LMT1		0x1	/* 150mA */
#define INPUT_SRC_CUR_LMT2		0x2	/* 500mA */
#define INPUT_SRC_CUR_LMT3		0x3	/* 900mA */
#define INPUT_SRC_CUR_LMT4		0x4	/* 1000mA */
#define INPUT_SRC_CUR_LMT5		0x5	/* 1200mA */
#define INPUT_SRC_CUR_LMT6		0x6	/* 2000mA */
#define INPUT_SRC_CUR_LMT7		0x7	/* 3000mA */

/*
 * D1, D2, D3 can be used to set min sys voltage limit
 * and D4, D5 can be used to control the charger
 */
#define BQ24192_POWER_ON_CFG_REG		0x1
#define POWER_ON_CFG_RESET			(1 << 7)
#define POWER_ON_CFG_I2C_WDTTMR_RESET		(1 << 6)
#define CHR_CFG_BIT_POS				4
#define CHR_CFG_BIT_LEN				2
#define POWER_ON_CFG_CHRG_CFG_DIS		(0 << 4)
#define POWER_ON_CFG_CHRG_CFG_EN		(1 << 4)
#define POWER_ON_CFG_CHRG_CFG_OTG		(3 << 4)
#define POWER_ON_CFG_BOOST_LIM			(1 << 0)

/*
 * Charge Current control register
 * with range from 500 - 4532mA
 */
#define BQ24192_CHRG_CUR_CNTL_REG		0x2
#define BQ24192_CHRG_CUR_OFFSET		512	/* 500 mA */
#define BQ24192_CHRG_CUR_LSB_TO_CUR	64	/* 64 mA */
#define BQ24192_GET_CHRG_CUR(reg) ((reg>>2)*BQ24192_CHRG_CUR_LSB_TO_CUR\
			+ BQ24192_CHRG_CUR_OFFSET) /* in mA */
#define BQ24192_CHRG_ITERM_OFFSET       128
#define BQ24192_CHRG_CUR_LSB_TO_ITERM   128

/* Pre charge and termination current limit reg */
#define BQ24192_PRECHRG_TERM_CUR_CNTL_REG	0x3
#define BQ24192_TERM_CURR_LIMIT_128		0	/* 128mA */
#define BQ24192_PRE_CHRG_CURR_256		(1 << 4)  /* 256mA */

/* Charge voltage control reg */
#define BQ24192_CHRG_VOLT_CNTL_REG	0x4
#define BQ24192_CHRG_VOLT_OFFSET	3504	/* 3504 mV */
#define BQ24192_CHRG_VOLT_LSB_TO_VOLT	16	/* 16 mV */
/* Low voltage setting 0 - 2.8V and 1 - 3.0V */
#define CHRG_VOLT_CNTL_BATTLOWV		(1 << 1)
/* Battery Recharge threshold 0 - 100mV and 1 - 300mV */
#define CHRG_VOLT_CNTL_VRECHRG		(0 << 0)
#define BQ24192_GET_CHRG_VOLT(reg) ((reg>>2)*BQ24192_CHRG_VOLT_LSB_TO_VOLT\
			+ BQ24192_CHRG_VOLT_OFFSET) /* in mV */

/* Charge termination and Timer control reg */
#define BQ24192_CHRG_TIMER_EXP_CNTL_REG		0x5
#define CHRG_TIMER_EXP_CNTL_EN_TERM		(1 << 7)
#define CHRG_TIMER_EXP_CNTL_TERM_STAT		(1 << 6)
/* WDT Timer uses 2 bits */
#define WDT_TIMER_BIT_POS			4
#define WDT_TIMER_BIT_LEN			2
#define CHRG_TIMER_EXP_CNTL_WDTDISABLE		(0 << 4)
#define CHRG_TIMER_EXP_CNTL_WDT40SEC		(1 << 4)
#define CHRG_TIMER_EXP_CNTL_WDT80SEC		(2 << 4)
#define CHRG_TIMER_EXP_CNTL_WDT160SEC		(3 << 4)
#define WDTIMER_RESET_MASK			0x40
/* Safety Timer Enable bit */
#define CHRG_TIMER_EXP_CNTL_EN_TIMER		(1 << 3)
/* Charge Timer uses 2bits(20 hrs) */
#define SFT_TIMER_BIT_POS			1
#define SFT_TIMER_BIT_LEN			2
#define CHRG_TIMER_EXP_CNTL_SFT_TIMER		(3 << 1)

#define BQ24192_CHRG_THRM_REGL_REG		0x6

#define BQ24192_MISC_OP_CNTL_REG		0x7
#define MISC_OP_CNTL_DPDM_EN			(1 << 7)
#define MISC_OP_CNTL_TMR2X_EN			(1 << 6)
#define MISC_OP_CNTL_BATFET_DIS			(1 << 5)
#define MISC_OP_CNTL_BATGOOD_EN			(1 << 4)
/* To mask INT's write 0 to the bit */
#define MISC_OP_CNTL_MINT_CHRG			(1 << 1)
#define MISC_OP_CNTL_MINT_BATT			(1 << 0)

#define BQ24192_SYSTEM_STAT_REG			0x8
/* D6, D7 show VBUS status */
#define SYSTEM_STAT_VBUS_BITS			(3 << 6)
#define SYSTEM_STAT_VBUS_UNKNOWN		0
#define SYSTEM_STAT_VBUS_HOST			(1 << 6)
#define SYSTEM_STAT_VBUS_ADP			(2 << 6)
#define SYSTEM_STAT_VBUS_OTG			(3 << 6)
/* D4, D5 show charger status */
#define SYSTEM_STAT_NOT_CHRG			(0 << 4)
#define SYSTEM_STAT_PRE_CHRG			(1 << 4)
#define SYSTEM_STAT_FAST_CHRG			(2 << 4)
#define SYSTEM_STAT_CHRG_DONE			(3 << 4)
#define SYSTEM_STAT_DPM				(1 << 3)
#define SYSTEM_STAT_PWR_GOOD			(1 << 2)
#define SYSTEM_STAT_THERM_REG			(1 << 1)
#define SYSTEM_STAT_VSYS_LOW			(1 << 0)
#define SYSTEM_STAT_CHRG_MASK			(3 << 4)

#define BQ24192_FAULT_STAT_REG			0x9
#define FAULT_STAT_WDT_TMR_EXP			(1 << 7)
#define FAULT_STAT_OTG_FLT			(1 << 6)
/* D4, D5 show charger fault status */
#define FAULT_STAT_CHRG_BITS			(3 << 4)
#define FAULT_STAT_CHRG_NORMAL			(0 << 4)
#define FAULT_STAT_CHRG_IN_FLT			(1 << 4)
#define FAULT_STAT_CHRG_THRM_FLT		(2 << 4)
#define FAULT_STAT_CHRG_TMR_FLT			(3 << 4)
#define FAULT_STAT_BATT_FLT			(1 << 3)
#define FAULT_STAT_BATT_TEMP_BITS		(3 << 0)

#define BQ24192_VENDER_REV_REG			0xA
/* D3, D4, D5 indicates the chip model number */
#define BQ24190_IC_VERSION			0x0
#define BQ24191_IC_VERSION			0x1
#define BQ24192_IC_VERSION			0x2
#define BQ24192I_IC_VERSION			0x3
#define BQ2419x_IC_VERSION			0x4

#define BQ24192_MAX_MEM		12
#define NR_RETRY_CNT		3

#define CHARGER_PS_NAME				"bq24192_charger"

#define CHARGER_TASK_JIFFIES		(HZ * 150)/* 150sec */
#define CHARGER_HOST_JIFFIES		(HZ * 60) /* 60sec */
#define FULL_THREAD_JIFFIES		(HZ * 30) /* 30sec */
#define TEMP_THREAD_JIFFIES		(HZ * 30) /* 30sec */

#define BATT_TEMP_MAX_DEF	60	/* 60 degrees */
#define BATT_TEMP_MIN_DEF	0

/* Max no. of tries to clear the charger from Hi-Z mode */
#define MAX_TRY		3

/* Max no. of tries to reset the bq24192i WDT */
#define MAX_RESET_WDT_RETRY 8

static struct power_supply *fg_psy;
struct bq24192_chip *chip_extern=NULL;
//extern int entry_mode;

//...........................................................................
static unsigned char WakeLockFlag=0;//lambert,0:already unlock,1:already lock

//.......................................................................................................
enum bq24192_chrgr_stat {
	BQ24192_CHRGR_STAT_UNKNOWN =0,
	BQ24192_CHRGR_STAT_CHARGING,
	BQ24192_CHRGR_STAT_FAULT,
	BQ24192_CHRGR_STAT_LOW_SUPPLY_FAULT,
	BQ24192_CHRGR_STAT_BAT_FULL,
};

struct bq24192_chip {
	struct i2c_client *client;
	struct power_supply *usb;
	struct delayed_work watchdog_work;
	struct delayed_work jeita_wrkr;
	struct mutex event_lock;
	/* Wake lock to prevent platform from going to S3 when charging */
	struct wake_lock wakelock;

	struct extcon_dev *vbus_edev, *id_edev, *charger_edev;
	struct notifier_block vbus_nb, charger_nb, otg_nb;
	struct work_struct charger_type_work;
	struct delayed_work otg_work;

	enum bq24192_chrgr_stat chgr_stat;
	int cc;
	int cv;
	int inlmt;
	int max_cc;
	int max_cv;
	int max_temp;
	int min_temp;
	int iterm;
	int cntl_state;
	int irq;
	bool boost_mode;
	bool online;
	bool present;
};

void charger_enabled_poweron(void);
int bq24192_chargeric_status(void);

// Webber +++++++++++++++++++++++++++
bool detect_charging(void);
// Webber ---------------------------

static int bq24192_reg_read_modify(struct i2c_client *client, u8 reg,
							u8 val, bool bit_set);

static struct i2c_client *bq24192_client;

static enum power_supply_property bq24192_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_TEMP_MAX,
	POWER_SUPPLY_PROP_TEMP_MIN
};

static void bq24192_ac_lock(void)
{
	unsigned long		flags;
	local_irq_save(flags);
	if(chip_extern){
		dev_dbg(&chip_extern->client->dev, "bq24192_ac_lock\n");
		wake_lock(&chip_extern->wakelock);
	}
	local_irq_restore(flags);

}
static void bq24192_ac_unlock(void)
{
	unsigned long		flags;
	local_irq_save(flags);
	if(chip_extern){
		printk("bq24192_ac_unlock\n");
		wake_unlock(&chip_extern->wakelock);
	}
	local_irq_restore(flags);
}
static int bq24192_write_reg(struct i2c_client *client, u8 reg, u8 value);
extern unsigned char cur_cable_status;
static inline int bq24192_set_inlmt(struct bq24192_chip *chip, int inlmt);
/*-------------------------------------------------------------------------*/
//--------------2014-1-1----distinction sdp/dcp----------------
static int USB_STATE,AC_STATE;

enum {
	UG31XX_NO_CABLE = 0,
	UG31XX_USB_PC_CABLE = 1,
	UG31XX_PAD_POWER = 2,
	UG31XX_AC_ADAPTER_CABLE = 3
};

enum {
	PWR_SUPPLY_AC = 0,
	PWR_SUPPLY_USB,
	PWR_SUPPLY_bq21455
};

static enum power_supply_property bq24192_pwr_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
        POWER_SUPPLY_PROP_PRESENT
};

static char *supply_list[] = {
	"ac",
	"usb",
};

static int bq24192_power_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	int ret = 0;

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		if((psy->desc->type == POWER_SUPPLY_TYPE_MAINS))
		{
			val->intval = AC_STATE;
		}
		else if((psy->desc->type == POWER_SUPPLY_TYPE_USB))
		{
			val->intval = USB_STATE;
		}
		else
		{
			val->intval = 0;
		}
		break;
        case POWER_SUPPLY_PROP_PRESENT:
            if (psy->desc->type == POWER_SUPPLY_TYPE_USB) {
                /* for ATD test to acquire the status about charger ic */
                ret = bq24192_chargeric_status();
                if (ret >= 0) {
                   val->intval = 1;
                } else
                   val->intval = 0;
            } else if (psy->desc->type == POWER_SUPPLY_TYPE_MAINS) {
                ret = bq24192_chargeric_status();
                if (ret >= 0) {
                   val->intval = 1;
                } else
                   val->intval = 0;
            } else
               ret = -EINVAL;
        break;
	default:
		return -EINVAL;
	}
	return ret;
}

static struct power_supply_desc bq24192_supply_desc[] = {
	{
		.name			= "ac",
		.type			= POWER_SUPPLY_TYPE_MAINS,
		.properties 		= bq24192_pwr_props,
		.num_properties	= ARRAY_SIZE(bq24192_pwr_props),
		.get_property		= bq24192_power_get_property,
	},
	{
		.name			= "usb",
		.type			= POWER_SUPPLY_TYPE_USB,
		.properties 		= bq24192_pwr_props,
		.num_properties 	= ARRAY_SIZE(bq24192_pwr_props),
		.get_property 		= bq24192_power_get_property,
	},
	{
		.name = "bq24155",
		.type = POWER_SUPPLY_TYPE_USB,
		.properties 		= bq24192_pwr_props,
		.num_properties 	= ARRAY_SIZE(bq24192_pwr_props),
		.get_property = bq24192_power_get_property,
	},
};

static struct power_supply_config bq24192_supply_config = {
	.supplied_to		= supply_list,
	.num_supplicants	= ARRAY_SIZE(supply_list),
};

static struct power_supply *bq24192_supply[3];

static unsigned char cable_status = EXTCON_NONE;

static int bq24192_cable_callback(unsigned char usb_cable_state)
{
	if (chip_extern->boost_mode) {
		cur_cable_status = UG31XX_NO_CABLE;
		cable_status = UG31XX_NO_CABLE;
		AC_STATE   = 0;
		USB_STATE = 0;
		power_supply_changed(bq24192_supply[PWR_SUPPLY_USB]);
		power_supply_changed(bq24192_supply[PWR_SUPPLY_AC]);
		return 0;
	}
	dev_dbg(&chip_extern->client->dev, "%s  usb_cable_state = %x, power source ok ? = %d \n", __func__, usb_cable_state,detect_charging()) ;

	if(usb_cable_state != cable_status)
	{
		cable_status = usb_cable_state;
		if((cable_status == EXTCON_CHG_USB_SDP) && (detect_charging() == true))
		{
                        charger_enabled_poweron();
			if(WakeLockFlag==0){
				 bq24192_ac_lock();
				 WakeLockFlag=1;
			}
			cur_cable_status = UG31XX_USB_PC_CABLE;
			AC_STATE   = 0;

			USB_STATE = 1;
			bq24192_set_inlmt(chip_extern,500);

			printk("%s  USB_IN  \n", __func__) ;

		}else
		{
			if((cable_status == EXTCON_CHG_USB_DCP) && (detect_charging() == true))
			{
                                charger_enabled_poweron();
				if(WakeLockFlag==0){
				      bq24192_ac_lock();
				      WakeLockFlag=1;
				}
				cur_cable_status = UG31XX_AC_ADAPTER_CABLE;
				AC_STATE   = 1;
				USB_STATE = 0;
                bq24192_set_inlmt(chip_extern,1200);

				printk("%s  AC_IN  \n", __func__) ;

			}else if((cable_status == EXTCON_NONE) && (detect_charging() == false))
			{
				if(WakeLockFlag==1){
					 WakeLockFlag=0;
					 bq24192_ac_unlock();
				}
				cur_cable_status = UG31XX_NO_CABLE;
				AC_STATE   = 0;
				USB_STATE = 0;

				printk("%s  CABLE_OUT  \n", __func__) ;

				//bq24192_write_reg(bq24192_client, BQ24192_INPUT_SRC_CNTL_REG, 0x32);
			}else{
				printk("%s  other condition ,  state =  %x , power source ok ? %d \n", __func__, usb_cable_state,detect_charging()) ;
			}
		}
		power_supply_changed(bq24192_supply[PWR_SUPPLY_USB]);
		power_supply_changed(bq24192_supply[PWR_SUPPLY_AC]);
		power_supply_changed(bq24192_supply[PWR_SUPPLY_bq21455]);
	}

	if (cur_cable_status != UG31XX_NO_CABLE)
		schedule_delayed_work(&chip_extern->jeita_wrkr, 30*HZ);
	return 0;
}

static int bq24192_powersupply_init(struct i2c_client *client)
{
	int i, ret;
	for (i = 0; i < ARRAY_SIZE(bq24192_supply); i++) {
		bq24192_supply[i] = power_supply_register(&client->dev, &bq24192_supply_desc[i],
					&bq24192_supply_config);
		if (IS_ERR(bq24192_supply[i])) {
			ret = PTR_ERR(bq24192_supply[i]);
			bq24192_supply[i] = NULL;
			printk("[%s] Failed to register power supply\n", __func__);
			while ((--i) >= 0) {
				power_supply_unregister(bq24192_supply[i]);
			}
			return ret;
		}
	}
  return 0;
}
//--------------2014-1-1----distinction sdp/dcp----------------ends

//Charge Portint Guide 2014-01-20------------------------------start
static inline int bq24192_set_inlmt(struct bq24192_chip *chip, int inlmt);
static inline int bq24192_set_iterm(struct bq24192_chip *chip, int iterm);
static inline int bq24192_set_cc(struct bq24192_chip *chip, int cc);
static inline int bq24192_set_cv(struct bq24192_chip *chip, int cv);
static int bq24192_read_reg(struct i2c_client *client, u8 reg);
static inline int bq24192_get_inlmt(struct bq24192_chip *chip);

static u8 chrg_vindpm_to_reg(int vindpm)
{
	u8 reg,reg1,value;

	value = bq24192_read_reg(chip_extern->client, BQ24192_INPUT_SRC_CNTL_REG);
	reg	= (vindpm-3880)/80;
	reg1 = (vindpm-3880)%80;

	if((reg1 > 0) && (reg1 < 80))
		reg += 1;

	reg = reg<<3;
	value = ((value&0x87)|reg);
	return value;
}

static inline int bq24192_set_vindpm(struct bq24192_chip *chip, int vindpm)
{
	u8 regval;

	dev_warn(&chip->client->dev, "%s:%d %d\n", __func__, __LINE__, vindpm);
	regval = chrg_vindpm_to_reg(vindpm);

	return bq24192_write_reg(chip->client, BQ24192_INPUT_SRC_CNTL_REG,
				regval);
}

static inline int bq24192_set_boostlim(u8 lim)
{
	u8 value;

	dev_warn(&chip_extern->client->dev, "%s\n", __func__);

	value = bq24192_read_reg(chip_extern->client, BQ24192_POWER_ON_CFG_REG);
	if(lim == 0)
		value = (value&0xFE);
	else
		value = (value|0x01);
	return bq24192_write_reg(chip_extern->client, BQ24192_POWER_ON_CFG_REG,value);
}

static inline int bq24192_set_sysmin(int sysmin)
{
	int value,regval;

	dev_warn(&chip_extern->client->dev, "%s:%d %d\n", __func__, __LINE__, sysmin);

	value = bq24192_read_reg(chip_extern->client, BQ24192_POWER_ON_CFG_REG);

	regval = (sysmin - 3000) / 100;
	if(((sysmin - 3000) % 100) > 0)
		regval += 1;
	regval = regval<<1;
	value = (value&0xF1)|regval;

	return bq24192_write_reg(chip_extern->client, BQ24192_POWER_ON_CFG_REG,value);
}

static int bq24192_clear_hiz(struct bq24192_chip *chip);
static int program_timers(struct bq24192_chip *chip, int wdt_duration,
				bool sfttmr_enable);
/*SDP 32 1d 54 11 b3 b4 73, 4b 6c 90 60*/
static void ChargeInit(unsigned int power_type)
{
	int ret,value;

	/* Clear the charger from Hi-Z */
	if(AC_STATE||USB_STATE)
	{
	   ret = bq24192_clear_hiz(chip_extern);
	   if (ret < 0)
		dev_warn(&chip_extern->client->dev, "HiZ clear failed:\n");
	}
	/*Set boost_limit 1.5a*/
	bq24192_set_boostlim(1);

	/*Set minimum system voltage limit 3.6v*/
	bq24192_set_sysmin(3600);

	/*BCOLD:0 , ICHG as reg02[7:2] programmed*/
	value = bq24192_read_reg(chip_extern->client, BQ24192_CHRG_CUR_CNTL_REG);
	value &= 0xFC;
	bq24192_write_reg(chip_extern->client, BQ24192_CHRG_CUR_CNTL_REG,value);

	/*boost voltage 4.998v,thermal regulation 120c*/
	bq24192_write_reg(chip_extern->client, BQ24192_CHRG_THRM_REGL_REG,0x73);


	if(power_type == EXTCON_CHG_USB_DCP){
		bq24192_set_inlmt(chip_extern,1200);		//00	inlim:1200mA
		}else{ //POWER_SUPPLY_CHARGER_TYPE_USB_SDP
			bq24192_set_inlmt(chip_extern,500);	//00	inlim:500mA
		}

		bq24192_set_vindpm(chip_extern,4360);	//00	vindpm:4.36v
		bq24192_set_iterm(chip_extern,256);		//03	BQ24192_PRE_CHRG_CURR_256 & termination current limit
		bq24192_set_cc(chip_extern,1856);		//02	fast charge current linit 1856ma
		bq24192_set_cv(chip_extern,4352);		//04	charge voltage 4.352v & pre-charge to fast charge 3.0v & 300mv

	/* Enable the WDT and Disable Safety timer */
	ret = program_timers(chip_extern, CHRG_TIMER_EXP_CNTL_WDT160SEC, false);
	if (ret < 0)
		dev_warn(&chip_extern->client->dev, "TIMER enable failed\n");

	/*Enable termination*/
	value = bq24192_read_reg(chip_extern->client, BQ24192_CHRG_TIMER_EXP_CNTL_REG);
	value |= 0x80;
	bq24192_write_reg(chip_extern->client, BQ24192_CHRG_TIMER_EXP_CNTL_REG,value);

	/*Mask INT*/
	value = bq24192_read_reg(chip_extern->client, BQ24192_MISC_OP_CNTL_REG);
	value &= 0xFC;
	bq24192_write_reg(chip_extern->client, BQ24192_MISC_OP_CNTL_REG,value);

	/*Clear fault register*/
	bq24192_read_reg(chip_extern->client, BQ24192_FAULT_STAT_REG);

}

//Charge Portint Guide 2014-01-20------------------------------end

static int bq24192_reg_read_modify(struct i2c_client *client, u8 reg,
							u8 val, bool bit_set);
extern bool is_upi_do_disablecharger;
void bq24192_charge_enable(int enable) {

        if (is_upi_do_disablecharger) {
           printk("=========== UPI gauge do stop charging, not do JETIA ==================\n");
           return;
        }

	if (enable) {
		dev_dbg(&chip_extern->client->dev, "Start charging\n");
		bq24192_reg_read_modify(chip_extern->client,BQ24192_POWER_ON_CFG_REG,0x10, true);
	} else {
		dev_dbg(&chip_extern->client->dev, "Stop charging\n");
		bq24192_reg_read_modify(chip_extern->client,BQ24192_POWER_ON_CFG_REG,0x10, false);
	}
}
EXPORT_SYMBOL(bq24192_charge_enable);

//Charge  jeita 2014-02-18------------------------------start
static int fg_chip_get_property(enum power_supply_property psp);
static struct power_supply *get_fg_chip_psy(void);
static int check_batt_psy(struct device *dev, void *data);
// [Webber]
// Change Jetia Temp Range according to ME176C Charger Porting Guide +++++++++++++++++++++++++++++++++
static int TempArry[5][2] = {{1000,500},{490,450},{440,150},{140,50},{40,-200}};
// [Webber] -----------------------------------------------------------------
static void JetiaWork(struct work_struct *work)
{
	static int temp,oldtemp=0,voltage;
	static u8 index=2;
        int capacity;
        capacity = fg_chip_get_property(POWER_SUPPLY_PROP_CAPACITY);

        if (cur_cable_status == UG31XX_NO_CABLE) {
            printk("## No charging source, no need do JEITA !!\n");
            return;
        }

        /* TODO bq24192_charge_enable(0);
		if (entry_mode == 4 && capacity == 100) {
            printk("## in charging mode, battery capacity is 100%% and full charger, no need do JEITA !!\n");
            schedule_delayed_work(&chip_extern->jeita_wrkr, 120*HZ);
            return;
        }*/

	temp = fg_chip_get_property(POWER_SUPPLY_PROP_TEMP);
	if((temp-oldtemp) > 0){
		if(temp > TempArry[index][0])
			index-= 1;
	}else{
		if(temp < TempArry[index][1])
			index+= 1;
	}
	printk("## ASUS JEITA rule, temp:%d,oldtemp:%d %d\n",temp,oldtemp,index);
	oldtemp = temp;
	if(index >= 4)
		index = 4;
	if(index <= 0)
		index = 0;

	switch (index)
	{
		case 0:
			printk("*** [%s] -  Temp >= (55-5(Debounce)) degree  ***\n",__func__);
			bq24192_charge_enable(0);
			// [Webber]
			// Case : Temp >= (55-5(Debounce)) degree celsius +++++++++++++++++++++++++++
			bq24192_set_cc(chip_extern,1536);
			bq24192_set_cv(chip_extern,4096);
			// [Webber] ---------------------------
			break;
		case 1:
			printk("*** [%s] -  (50-5(Debounce))  <= Temp <  (55-5(Debounce)) degree  ***\n",__func__);
			voltage = fg_chip_get_property(POWER_SUPPLY_PROP_VOLTAGE_NOW);
			voltage = voltage/1000;
			// [Webber]
			// Case : (50-5(Debounce))  <= Temp <  (55-5(Debounce)) degree celsius +++++++++++++++++++++++++++
			if(voltage <= 4096)
				bq24192_charge_enable(1);
			else
				bq24192_charge_enable(0);
			bq24192_set_cc(chip_extern,1536);
			bq24192_set_cv(chip_extern,4096);
			// [Webber] ---------------------------
			break;
		case 2:
			printk("*** [%s] -  (10+5(Debounce)) <= Temp <  (50-5(Debounce))  degree  ***\n",__func__);
			bq24192_charge_enable(1);
			// [Webber]
			// Case : (10+5(Debounce)) <= Temp <  (50-5(Debounce))  degree celsius +++++++++++++++++++++++++++
			bq24192_set_cc(chip_extern,1536);
			bq24192_set_cv(chip_extern,4352);
			// [Webber] ---------------------------
			break;
		case 3:
			printk("*** [%s] -  (0+5(Debounce)) <= Temp < (10+5(Debounce)) degree  ***\n",__func__);
			bq24192_charge_enable(1);
			// [Webber]
			// Case : (0+5(Debounce)) <= Temp < (10+5(Debounce)) degree celsius ++++++++++++++++++++++++++++
			bq24192_set_cc(chip_extern,768);
			bq24192_set_cv(chip_extern,4352);
			// [Webber] ----------------------------
			break;
		case 4:
			printk("*** [%s] -  Temp < (0+5(Debounce)) degree  ***\n",__func__);
			bq24192_charge_enable(0);
			// [Webber]
			// Case : Temp < (0+5(Debounce)) degree celsius ++++++++++++++++++++++++++++
			bq24192_set_cc(chip_extern,768);
			bq24192_set_cv(chip_extern,4352);
			// [Webber] ----------------------------
			break;
	}
	schedule_delayed_work(&chip_extern->jeita_wrkr, 30*HZ);
}
//Charge  jeita 2014-02-18------------------------------end
/*
 * Genenric register read/write interfaces to access registers in charger ic
 */

static int bq24192_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret == -EAGAIN || ret == -ETIMEDOUT)
			continue;
		else
			break;
	}

	if (ret < 0)
		dev_err(&client->dev, "I2C SMbus Write error:%d\n", ret);

	return ret;
}

static int bq24192_read_reg(struct i2c_client *client, u8 reg)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret == -EAGAIN || ret == -ETIMEDOUT)
			continue;
		else
			break;
	}

	if (ret < 0)
		dev_err(&client->dev, "I2C SMbus Read error:%d\n", ret);

	return ret;
}

#ifdef DEBUG
/*
 * This function dumps the bq24192 registers
 */
static void bq24192_dump_registers(struct bq24192_chip *chip)
{
	int ret;

	dev_info(&chip->client->dev, "%s\n", __func__);

	/* Input Src Ctrl register */
	ret = bq24192_read_reg(chip->client, BQ24192_INPUT_SRC_CNTL_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Input Src Ctrl reg read fail\n");
	dev_info(&chip->client->dev, "REG00 %x\n", ret);

	/* Pwr On Cfg register */
	ret = bq24192_read_reg(chip->client, BQ24192_POWER_ON_CFG_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Pwr On Cfg reg read fail\n");
	dev_info(&chip->client->dev, "REG01 %x\n", ret);

	/* Chrg Curr Ctrl register */
	ret = bq24192_read_reg(chip->client, BQ24192_CHRG_CUR_CNTL_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Chrg Curr Ctrl reg read fail\n");
	dev_info(&chip->client->dev, "REG02 %x\n", ret);

	/* Pre-Chrg Term register */
	ret = bq24192_read_reg(chip->client,
					BQ24192_PRECHRG_TERM_CUR_CNTL_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Pre-Chrg Term reg read fail\n");
	dev_info(&chip->client->dev, "REG03 %x\n", ret);

	/* Chrg Volt Ctrl register */
	ret = bq24192_read_reg(chip->client, BQ24192_CHRG_VOLT_CNTL_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Chrg Volt Ctrl reg read fail\n");
	dev_info(&chip->client->dev, "REG04 %x\n", ret);

	/* Chrg Term and Timer Ctrl register */
	ret = bq24192_read_reg(chip->client, BQ24192_CHRG_TIMER_EXP_CNTL_REG);
	if (ret < 0) {
		dev_warn(&chip->client->dev,
			"Chrg Term and Timer Ctrl reg read fail\n");
	}
	dev_info(&chip->client->dev, "REG05 %x\n", ret);

	/* Thermal Regulation register */
	ret = bq24192_read_reg(chip->client, BQ24192_CHRG_THRM_REGL_REG);
	if (ret < 0) {
		dev_warn(&chip->client->dev,
				"Thermal Regulation reg read fail\n");
	}
	dev_info(&chip->client->dev, "REG06 %x\n", ret);

	/* Misc Operations Ctrl register */
	ret = bq24192_read_reg(chip->client, BQ24192_MISC_OP_CNTL_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Misc Op Ctrl reg read fail\n");
	dev_info(&chip->client->dev, "REG07 %x\n", ret);

	/* System Status register */
	ret = bq24192_read_reg(chip->client, BQ24192_SYSTEM_STAT_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "System Status reg read fail\n");
	dev_info(&chip->client->dev, "REG08 %x\n", ret);

	/* Fault Status register */
	ret = bq24192_read_reg(chip->client, BQ24192_FAULT_STAT_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Fault Status reg read fail\n");
	dev_info(&chip->client->dev, "REG09 %x\n", ret);

	/* Vendor Revision register */
	ret = bq24192_read_reg(chip->client, BQ24192_VENDER_REV_REG);
	if (ret < 0)
		dev_warn(&chip->client->dev, "Vendor Rev reg read fail\n");
	dev_info(&chip->client->dev, "REG0A %x\n", ret);
}
#endif

/*
 * If the bit_set is TRUE then val 1s will be SET in the reg else val 1s will
 * be CLEARED
 */
static int bq24192_reg_read_modify(struct i2c_client *client, u8 reg,
							u8 val, bool bit_set)
{
	int ret;

	ret = bq24192_read_reg(client, reg);

	if (bit_set)
		ret |= val;
	else
		ret &= (~val);

	ret = bq24192_write_reg(client, reg, ret);

	return ret;
}

/*
 * This function verifies if the bq24192i charger chip is in Hi-Z
 * If yes, then clear the Hi-Z to resume the charger operations
 */
static int bq24192_clear_hiz(struct bq24192_chip *chip)
{
	int ret, count;

	dev_info(&chip->client->dev, "%s\n", __func__);

	for (count = 0; count < MAX_TRY; count++) {
		/*
		 * Read the bq24192i REG00 register for charger Hi-Z mode.
		 * If it is in Hi-Z, then clear the Hi-Z to resume the charging
		 * operations.
		 */
		ret = bq24192_read_reg(chip->client,
				BQ24192_INPUT_SRC_CNTL_REG);
		if (ret < 0) {
			dev_warn(&chip->client->dev,
					"Input src cntl read failed\n");
			goto i2c_error;
		}

		if (ret & INPUT_SRC_CNTL_EN_HIZ) {
			dev_warn(&chip->client->dev,
						"Charger IC in Hi-Z mode\n");
#ifdef DEBUG
			bq24192_dump_registers(chip);
#endif
			/* Clear the Charger from Hi-Z mode */
			ret = (chip->inlmt & ~INPUT_SRC_CNTL_EN_HIZ);

			/* Write the values back */
			ret = bq24192_write_reg(chip->client,
					BQ24192_INPUT_SRC_CNTL_REG, ret);
			if (ret < 0) {
				dev_warn(&chip->client->dev,
						"Input src cntl write failed\n");
				goto i2c_error;
			}
			msleep(150);
		} else {
			dev_info(&chip->client->dev,
						"Charger is not in Hi-Z\n");
			break;
		}
	}
	return ret;
i2c_error:
	dev_err(&chip->client->dev, "%s\n", __func__);
	return ret;
}

/* check_batt_psy -check for whether power supply type is battery
 * @dev : Power Supply dev structure
 * @data : Power Supply Driver Data
 * Context: can sleep
 *
 * Return true if power supply type is battery
 *
 */
static int check_batt_psy(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);

	/* check for whether power supply type is battery */
	if (psy->desc->type == POWER_SUPPLY_TYPE_BATTERY) {
		fg_psy = psy;
		return 1;
	}
	return 0;
}

/**
 * get_fg_chip_psy - identify the Fuel Gauge Power Supply device
 * Context: can sleep
 *
 * Return Fuel Gauge power supply structure
 */
static struct power_supply *get_fg_chip_psy(void)
{
	if (fg_psy)
		return fg_psy;

	/* loop through power supply class */
	class_for_each_device(power_supply_class, NULL, NULL,
			check_batt_psy);
	return fg_psy;
}

/**
 * fg_chip_get_property - read a power supply property from Fuel Gauge driver
 * @psp : Power Supply property
 *
 * Return power supply property value
 *
 */
static int fg_chip_get_property(enum power_supply_property psp)
{
	union power_supply_propval val;
	int ret = -ENODEV;

	if (!fg_psy)
		fg_psy = get_fg_chip_psy();
	if (fg_psy) {
		ret = fg_psy->desc->get_property(fg_psy, psp, &val);
		if (!ret)
			return val.intval;
	}
	return ret;
}

/**
 * bq24192_get_charger_health - to get the charger health status
 *
 * Returns charger health status
 */
int bq24192_get_charger_health(void)
{
	int ret_status, ret_fault;
	struct bq24192_chip *chip =
		i2c_get_clientdata(bq24192_client);

	dev_dbg(&chip->client->dev, "%s\n", __func__);

	ret_fault = bq24192_read_reg(chip->client, BQ24192_FAULT_STAT_REG);
	if (ret_fault < 0) {
		dev_warn(&chip->client->dev,
			"read reg failed %s\n", __func__);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}
	/* Check if the error VIN condition occured */
	ret_status = bq24192_read_reg(chip->client, BQ24192_SYSTEM_STAT_REG);
	if (ret_status < 0) {
		dev_warn(&chip->client->dev,
			"read reg failed %s\n", __func__);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (!(ret_status & SYSTEM_STAT_PWR_GOOD) &&
	((ret_fault & FAULT_STAT_CHRG_BITS) == FAULT_STAT_CHRG_IN_FLT))
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	if (!(ret_status & SYSTEM_STAT_PWR_GOOD) &&
	((ret_status & SYSTEM_STAT_VBUS_BITS) == SYSTEM_STAT_VBUS_UNKNOWN))
		return POWER_SUPPLY_HEALTH_DEAD;

	return POWER_SUPPLY_HEALTH_GOOD;
}

/***********************************************************************/

/* convert the input current limit value
 * into equivalent register setting.
 * Note: ilim must be in mA.
 */
static u8 chrg_ilim_to_reg(int ilim)
{
	u8 reg = 0;

	/* Set the input source current limit
	 * between 100 to 1500mA */
	if (ilim <= 100)
		reg |= INPUT_SRC_CUR_LMT0;
	else if (ilim <= 150)
		reg |= INPUT_SRC_CUR_LMT1;
	else if (ilim <= 500)
		reg |= INPUT_SRC_CUR_LMT2;
	else if (ilim <= 900)
		reg |= INPUT_SRC_CUR_LMT3;
	else if (ilim <= 1000)
		reg |= INPUT_SRC_CUR_LMT4;
	else if (ilim <= 1200)
		reg |= INPUT_SRC_CUR_LMT5;
	else if (ilim <= 2000)
		reg |= INPUT_SRC_CUR_LMT6;
	else
		reg |= INPUT_SRC_CUR_LMT7;

	return reg;
}

static u8 chrg_iterm_to_reg(int iterm)
{
	u8 reg;

	if (iterm <= BQ24192_CHRG_ITERM_OFFSET)
		reg = 0;
	else
		reg = ((iterm - BQ24192_CHRG_ITERM_OFFSET) /
			BQ24192_CHRG_CUR_LSB_TO_ITERM);
	return reg;
}

/* convert the charge current value
 * into equivalent register setting
 */
static u8 chrg_cur_to_reg(int cur)
{
	u8 reg;

	if (cur <= BQ24192_CHRG_CUR_OFFSET)
		reg = 0x0;
	else
		reg = ((cur - BQ24192_CHRG_CUR_OFFSET) /
				BQ24192_CHRG_CUR_LSB_TO_CUR);

	if(((cur - BQ24192_CHRG_CUR_OFFSET) % BQ24192_CHRG_CUR_LSB_TO_CUR) > 0)
		reg += 1;

	/* D0, D1 bits of Charge Current
	 * register are not used */
	reg = reg << 2;
	return reg;
}

/* convert the charge voltage value
 * into equivalent register setting
 */
static u8 chrg_volt_to_reg(int volt)
{
	u8 reg,reg1;

	if (volt <= BQ24192_CHRG_VOLT_OFFSET)
		reg = 0x0;
	else
		reg = (volt - BQ24192_CHRG_VOLT_OFFSET) /
				BQ24192_CHRG_VOLT_LSB_TO_VOLT;

	reg1 = (volt - BQ24192_CHRG_VOLT_OFFSET) %
				BQ24192_CHRG_VOLT_LSB_TO_VOLT;
	if((reg1 >0)&&(reg1 < 16))
		reg += 1;

	reg = (reg << 2) | CHRG_VOLT_CNTL_BATTLOWV;
	return reg;
}

/*
 * chip->event_lock need to be acquired before calling this function
 * to avoid the race condition
 */
static int program_timers(struct bq24192_chip *chip, int wdt_duration,
				bool sfttmr_enable)
{
	int ret;

	/* Read the timer control register */
	ret = bq24192_read_reg(chip->client, BQ24192_CHRG_TIMER_EXP_CNTL_REG);
	if (ret < 0) {
		dev_warn(&chip->client->dev, "TIMER CTRL reg read failed\n");
		return ret;
	}

	/* First disable watchdog */
	ret &= 0xCF;
	bq24192_write_reg(chip->client,
				BQ24192_CHRG_TIMER_EXP_CNTL_REG,
				ret);

	/* Read the timer control register */
	ret = bq24192_read_reg(chip->client, BQ24192_CHRG_TIMER_EXP_CNTL_REG);
	if (ret < 0) {
		dev_warn(&chip->client->dev, "TIMER CTRL reg read failed\n");
		return ret;
	}

	/* Program the time with duration passed */
	ret &= 0xCF;
	ret |=  wdt_duration;

	/* Enable/Disable the safety timer */
	if (sfttmr_enable)
		ret |= CHRG_TIMER_EXP_CNTL_EN_TIMER;
	else
		ret &= ~CHRG_TIMER_EXP_CNTL_EN_TIMER;

	/* Program the TIMER CTRL register */
	ret = bq24192_write_reg(chip->client,
				BQ24192_CHRG_TIMER_EXP_CNTL_REG,
				ret);
	if (ret < 0)
		dev_warn(&chip->client->dev, "TIMER CTRL I2C write failed\n");

	return ret;
}

/* This function should be called with the mutex held */
static int reset_wdt_timer(struct bq24192_chip *chip)
{
	int ret = 0, i;

	/* reset WDT timer */
	for (i = 0; i < MAX_RESET_WDT_RETRY; i++) {
		ret = bq24192_reg_read_modify(chip->client,
						BQ24192_POWER_ON_CFG_REG,
						WDTIMER_RESET_MASK, true);
		if (ret < 0)
			dev_warn(&chip->client->dev, "I2C write failed:%s\n",
							__func__);
	}
	return ret;
}

static inline int bq24192_set_cc(struct bq24192_chip *chip, int cc)
{
	u8 regval,value;

	dev_dbg(&chip->client->dev, "%s:%d %d\n", __func__, __LINE__, cc);
	regval = chrg_cur_to_reg(cc);
	value = bq24192_read_reg(chip->client, BQ24192_CHRG_CUR_CNTL_REG);
	value = ((value&0x03)|regval);

	return bq24192_write_reg(chip->client, BQ24192_CHRG_CUR_CNTL_REG,
				value);
}

static inline int bq24192_set_cv(struct bq24192_chip *chip, int cv)
{
	u8 regval;

	dev_dbg(&chip->client->dev, "%s:%d %d\n", __func__, __LINE__, cv);
	regval = chrg_volt_to_reg(cv);

	return bq24192_write_reg(chip->client, BQ24192_CHRG_VOLT_CNTL_REG,
					(regval | CHRG_VOLT_CNTL_VRECHRG)|CHRG_VOLT_CNTL_BATTLOWV);
}

static inline int bq24192_set_inlmt(struct bq24192_chip *chip, int inlmt)
{
	u8 regval,value;

	dev_warn(&chip->client->dev, "%s:%d %d\n", __func__, __LINE__, inlmt);
	chip->inlmt = inlmt;
	regval = chrg_ilim_to_reg(inlmt);
	value = bq24192_read_reg(chip->client, BQ24192_INPUT_SRC_CNTL_REG);
	regval = ((value&0xF8)|regval);
	return bq24192_write_reg(chip->client, BQ24192_INPUT_SRC_CNTL_REG,
				regval);
}

static inline int bq24192_set_iterm(struct bq24192_chip *chip, int iterm)
{
	u8 reg_val;

	if (iterm > BQ24192_CHRG_ITERM_OFFSET)
		dev_warn(&chip->client->dev,
			"%s ITERM set for >128mA", __func__);

	reg_val = chrg_iterm_to_reg(iterm);
	//msleep(500);

	return bq24192_write_reg(chip->client,
			BQ24192_PRECHRG_TERM_CUR_CNTL_REG,
				(BQ24192_PRE_CHRG_CURR_256 | reg_val));
}

static inline int bq24192_get_inlmt(struct bq24192_chip *chip) {
	u8 regval;
        int value = 100;

        regval = bq24192_read_reg(chip->client, BQ24192_INPUT_SRC_CNTL_REG);
	regval &= 0x07;
        switch (regval) {
            case 0:
               value = 100;
               break;
            case 1:
               value = 150;
               break;
            case 2:
               value = 500;
               break;
            case 3:
               value = 900;
               break;
            case 4:
               value = 1000;
               break;
            case 5:
               value = 1200;
               break;
            case 6:
               value = 2000;
               break;
            case 7:
               value = 3000;
               break;
        }
        dev_warn(&chip->client->dev, "%s:%d ,input current = %d mA\n", __func__, __LINE__, value);

	return value;
}

int bq24192_chargeric_status (void) {
	int ret;
	struct bq24192_chip *chip=NULL;

	chip = chip_extern;
	if((chip->client == NULL)||(chip == NULL)) {
		printk("chip->client == null\n");
		return -1;
	}

        mutex_lock(&chip_extern->event_lock);
	ret = bq24192_read_reg(chip->client, BQ24192_SYSTEM_STAT_REG);
        mutex_unlock(&chip_extern->event_lock);

	if (ret < 0)
		dev_err(&chip->client->dev, "STATUS register read failed\n");

	return ret;
}

int bq24192_is_charging(void)
{
	int ret;
	struct bq24192_chip *chip=NULL;

        if(chip_extern == NULL)
	{
		printk("chip_extern == null\n");
		return BQ24192_CHRGR_STAT_FAULT;
	}

	chip = chip_extern;
	ret = bq24192_read_reg(chip->client, BQ24192_SYSTEM_STAT_REG);
	if (ret < 0)
		dev_err(&chip->client->dev, "STATUS register read failed\n");

	ret &= SYSTEM_STAT_CHRG_MASK;

	switch (ret) {
	case SYSTEM_STAT_NOT_CHRG:
		chip->chgr_stat = BQ24192_CHRGR_STAT_FAULT;
		break;
	case SYSTEM_STAT_CHRG_DONE:
		chip->chgr_stat = BQ24192_CHRGR_STAT_BAT_FULL;
		break;
	case SYSTEM_STAT_PRE_CHRG:
	case SYSTEM_STAT_FAST_CHRG:
		chip->chgr_stat = BQ24192_CHRGR_STAT_CHARGING;
		break;
	default:
		break;
	}

	return chip->chgr_stat;
}

// Webber [TT-559041] ++++++++++++++++++++++++++++++++++++
bool detect_charging(void)
{
	int ret;
	struct bq24192_chip *chip=NULL;

        if(chip_extern == NULL)
	{
		printk("chip_extern == null\n");
		return BQ24192_CHRGR_STAT_FAULT;
	}

	chip = chip_extern;
	ret = bq24192_read_reg(chip->client, BQ24192_SYSTEM_STAT_REG);
	if (ret < 0)
		dev_err(&chip->client->dev, "STATUS register read failed\n");

	ret &= SYSTEM_STAT_PWR_GOOD;

	dev_dbg(&chip->client->dev, "[%s] ret = %d  \n", __func__ , ret);

	if ( ret == SYSTEM_STAT_PWR_GOOD )
	{
	        dev_dbg(&chip->client->dev, "[%s] charging\n", __func__);
		return (true);
	}else{
		dev_dbg(&chip->client->dev, "[%s] isn't charging\n", __func__);
		return (false);
	}

}
// Webber -----------------------------------


static int bq24192_usb_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct bq24192_chip *chip = power_supply_get_drvdata(psy);
	int ret = 0;

	dev_dbg(&chip->client->dev, "%s %d\n", __func__, psp);
	if (mutex_is_locked(&chip->event_lock)) {
		dev_dbg(&chip->client->dev,
			"%s: mutex is already acquired",
				__func__);
	}
	mutex_lock(&chip->event_lock);

	switch (psp) {

	case POWER_SUPPLY_PROP_PRESENT:
		chip->present = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		chip->online = val->intval;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = bq24192_set_cc(chip, val->intval);
		if (!ret)
			chip->cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq24192_set_cv(chip, val->intval);
		if (!ret)
			chip->cv = val->intval;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		chip->max_cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		chip->max_cv = val->intval;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq24192_set_inlmt(chip, val->intval);
		if (!ret)
			chip->inlmt = val->intval;
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		chip->max_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_TEMP_MIN:
		chip->min_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		chip->cntl_state = val->intval;
		break;
	default:
		ret = -ENODATA;
	}

	mutex_unlock(&chip->event_lock);
	return ret;
}

static int bq24192_usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct bq24192_chip *chip = power_supply_get_drvdata(psy);

	dev_dbg(&chip->client->dev, "%s %d\n", __func__, psp);
	mutex_lock(&chip->event_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->present;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq24192_get_charger_health();
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->max_cc;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = chip->max_cv;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = chip->cc;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = chip->cv;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = chip->inlmt;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = chip->cntl_state;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = 3;
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		val->intval = chip->max_temp;
		break;
	case POWER_SUPPLY_PROP_TEMP_MIN:
		val->intval = chip->min_temp;
		break;
	default:
		mutex_unlock(&chip->event_lock);
		return -EINVAL;
	}

	mutex_unlock(&chip->event_lock);
	return 0;
}

/* IRQ handler for charger Interrupts configured to GPIO pin */
static irqreturn_t bq24192_irq_isr(int irq, void *devid)
{
	struct bq24192_chip *chip = (struct bq24192_chip *)devid;

	/**TODO: This hanlder will be used for charger Interrupts */
	dev_dbg(&chip->client->dev,
		"IRQ Handled for charger interrupt: %d\n", irq);

	return IRQ_WAKE_THREAD;
}

/* IRQ handler for charger Interrupts configured to GPIO pin */
static irqreturn_t bq24192_irq_thread(int irq, void *devid)
{
	struct bq24192_chip *chip = (struct bq24192_chip *)devid;
	int reg_status, reg_fault,pending;
    int ret=0;

	pending = intel_mid_pmic_readb(R_PMIC_CHGRIRQ);
	if(pending&0x01){
	  intel_mid_pmic_writeb(R_PMIC_CHGRIRQ,pending);
	}
	/* Check if battery fault condition occured. Reading the register
	   value two times to get reliable reg value, recommended by vendor*/
	reg_fault = bq24192_read_reg(chip->client, BQ24192_FAULT_STAT_REG);
	if (reg_fault < 0)
		dev_err(&chip->client->dev, "FAULT register read failed:\n");


	if(reg_fault & FAULT_STAT_OTG_FLT){

        ret=bq24192_reg_read_modify(chip_extern->client,
		                 BQ24192_POWER_ON_CFG_REG,0x20, false);
        if (ret < 0) {
		 printk("Error in writing the control register\n");
		}
       printk("bq24192 otg fault\n");
	}
	dev_dbg(&chip->client->dev, "FAULT reg %x\n", reg_fault);
	reg_fault = bq24192_read_reg(chip->client, BQ24192_FAULT_STAT_REG);
	if (reg_fault < 0)
		dev_err(&chip->client->dev, "FAULT register read failed:\n");
	dev_dbg(&chip->client->dev, "FAULT reg %x\n", reg_fault);
	/*
	 * check the bq24192 status/fault registers to see what is the
	 * source of the interrupt
	 */
	reg_status = bq24192_read_reg(chip->client, BQ24192_SYSTEM_STAT_REG);
	if (reg_status < 0)
		dev_err(&chip->client->dev, "STATUS register read failed:\n");
	dev_dbg(&chip->client->dev, "STATUS reg %x\n", reg_status);

	if( (reg_status&0x30) == 0x20){
		if(cable_status == EXTCON_CHG_USB_DCP){
			ret = bq24192_read_reg(chip->client, BQ24192_INPUT_SRC_CNTL_REG);
			printk("Input source: %x\n",ret);
			bq24192_set_inlmt(chip_extern,1200);
		}
	}
	return IRQ_HANDLED;
}

extern int intel_mid_pmic_writeb(int reg, u8 val);

void charger_enabled_poweron() {
       int ret = 0;

        mutex_lock(&chip_extern->event_lock);
	ret=bq24192_reg_read_modify(chip_extern->client, BQ24192_INPUT_SRC_CNTL_REG,0x80,false);
	if (ret < 0) {
		mutex_unlock(&chip_extern->event_lock);
		printk("Error in writing the control register\n");
		return;
	}
	mutex_unlock(&chip_extern->event_lock);

	mutex_lock(&chip_extern->event_lock);
	ret=bq24192_reg_read_modify(chip_extern->client,
	BQ24192_POWER_ON_CFG_REG,0x10, true);
	if (ret < 0) {
		mutex_unlock(&chip_extern->event_lock);
		printk("Error in writing the control register\n");
		 return;
	}
	mutex_unlock(&chip_extern->event_lock);
}

static struct power_supply_desc bq24192_usb_desc = {
	.name = CHARGER_PS_NAME,
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = bq24192_usb_props,
	.num_properties = ARRAY_SIZE(bq24192_usb_props),
	.get_property = bq24192_usb_get_property,
	.set_property = bq24192_usb_set_property,
};

static char *bq24192_supplied_to[] = {
	"battery",
};

static void bq24192_watchdog_work(struct work_struct *work)
{
	struct bq24192_chip *chip = container_of(work,
									struct bq24192_chip, watchdog_work.work);
	reset_wdt_timer(chip);
	schedule_delayed_work(&chip->watchdog_work, 120*HZ);
}

static int bq24192_vbus_notifier(struct notifier_block *nb,
	unsigned long event, void *ptr)
{
	struct bq24192_chip *chip = container_of(nb, struct bq24192_chip, vbus_nb);

	if (event && chip->boost_mode) {
		/*
		 * VBUS may trigger when in OTG boost mode.
		 * This is incorrect, so cancel the event since running
		 * charger detection would be pointless.
		 */
		return NOTIFY_BAD;
	}

	/*
	 * When power source is removed from the OTG adapter,
	 * we need to enable the OTG boost to power it from battery.
	 *
	 * TODO: Right now, the USB devices are disconnected for a short moment.
	 *   This could be only avoided by enabling OTG boost all the time..
	 */
	 if (!event && !chip->boost_mode)
		 schedule_delayed_work(&chip->otg_work, msecs_to_jiffies(100));

	return NOTIFY_DONE;
}

static void bq24192_update_charger_state(struct work_struct *work)
{
	struct bq24192_chip *chip = container_of(work,
									struct bq24192_chip, charger_type_work);

	if (extcon_get_state(chip->charger_edev, EXTCON_CHG_USB_DCP) == 1) {
		bq24192_cable_callback(EXTCON_CHG_USB_DCP);
	} else if (extcon_get_state(chip->charger_edev, EXTCON_CHG_USB_SDP) == 1) {
		bq24192_cable_callback(EXTCON_CHG_USB_SDP);
	} else {
		bq24192_cable_callback(EXTCON_NONE);
	}
}

static int bq24192_charger_notifier(struct notifier_block *nb,
	unsigned long event, void *ptr)
{
	struct bq24192_chip *chip = container_of(nb,
									struct bq24192_chip, charger_nb);
	schedule_work(&chip->charger_type_work);
	return NOTIFY_OK;
}

static void bq24192_otg_boost(struct bq24192_chip *chip, bool enable)
{
	if (chip->boost_mode == enable)
		return;

	dev_info(&chip->client->dev, "Update OTG boost: %d\n", enable);

	bq24192_reg_read_modify(chip->client,
			BQ24192_POWER_ON_CFG_REG, 0x20, enable);
	chip->boost_mode = enable;
}

static void bq24192_update_otg_state(struct work_struct *work)
{
	struct bq24192_chip *chip = container_of(work,
									struct bq24192_chip, otg_work.work);

	/*
	 * Enable 5V boost for OTG if host mode is activated
	 * and there is no charger/power source inserted.
	 */
	bq24192_otg_boost(chip,
		extcon_get_state(chip->id_edev, EXTCON_USB_HOST) == 1
			&& !extcon_get_state(chip->vbus_edev, EXTCON_USB));
}

static int bq24192_otg_notifier(struct notifier_block *nb,
	unsigned long event, void *ptr)
{
	struct bq24192_chip *chip = container_of(nb, struct bq24192_chip, otg_nb);
	schedule_delayed_work(&chip->otg_work, msecs_to_jiffies(300));
	return NOTIFY_DONE;
}

#define CRYSTAL_COVE_IRQ_CHGR		4

static int bq24192_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bq24192_chip *chip;
	struct power_supply_config usb_config = {0};
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"SMBus doesn't support BYTE transactions\n");
		return -EIO;
	}

	chip = kzalloc(sizeof(struct bq24192_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	/*assigning default value for min and max temp*/
	chip->min_temp = BATT_TEMP_MIN_DEF;
	chip->max_temp = BATT_TEMP_MAX_DEF;
	i2c_set_clientdata(client, chip);

	ret = bq24192_read_reg(client, BQ24192_VENDER_REV_REG);
	if (ret < 0) {
		dev_err(&client->dev, "i2c read err:%d\n", ret);
		i2c_set_clientdata(client, NULL);
		kfree(chip);
		return -EIO;
	}

	/* D3, D4, D5 indicates the chip model number */
	ret = (ret >> 3) & 0x07;
	if (ret != BQ2419x_IC_VERSION) {
		dev_err(&client->dev, "device version mismatch: %x\n", ret);
		i2c_set_clientdata(client, NULL);
		kfree(chip);
		return -EIO;
	}

        bq24192_client = client;
	chip_extern = chip;
	chip->client = client;

	/*
	 * Request for charger chip gpio.This will be used to
	 * register for an interrupt handler for servicing charger
	 * interrupts
	 */
	if (intel_pmic) {
		chip->irq = regmap_irq_get_virq(intel_pmic->irq_chip_data,
						CRYSTAL_COVE_IRQ_CHGR);
		if (chip->irq < 0) {
			dev_err(&chip->client->dev,
				"chgr_int_n GPIO is not available\n");
		} else {
			ret = request_threaded_irq(chip->irq,
					bq24192_irq_isr, bq24192_irq_thread,
					IRQF_TRIGGER_FALLING, "BQ24192", chip);
			if (ret) {
				dev_warn(&bq24192_client->dev,
					"failed to register irq for pin %d\n",
					chip->irq);
			} else {
				dev_warn(&bq24192_client->dev,
					"registered charger irq for pin %d\n",
					chip->irq);
			}
		}
	}

        enable_irq_wake(chip->irq);
	intel_mid_pmic_writeb(R_PMIC_MIRQS0,0x00);
	intel_mid_pmic_writeb(R_PMIC_MIRQSX,0x00);

	/*
	 * Get extcon devices (might as well hard-code since this driver is
	 * too bad to be used anywhere else...)
	 */
	chip->vbus_edev = extcon_get_extcon_dev("crystal_cove_pwrsrc");
	if (!chip->vbus_edev) {
		dev_err(&client->dev, "VBUS extcon device does not exist\n");
		return -ENODEV;
	}
	if (IS_ERR(chip->vbus_edev))
		return PTR_ERR(chip->vbus_edev);

	chip->id_edev = extcon_get_extcon_dev("INT3496:00");
	if (!chip->id_edev) {
		dev_err(&client->dev, "ID extcon device does not exist\n");
		return -ENODEV;
	}
	if (IS_ERR(chip->id_edev))
		return PTR_ERR(chip->id_edev);

	chip->charger_edev = extcon_get_extcon_dev("phy-dwc3.1.auto.ulpi.0");
	if (!chip->charger_edev) {
		dev_err(&client->dev, "Charger extcon device does not exist\n");
		return -ENODEV;
	}
	if (IS_ERR(chip->charger_edev))
		return PTR_ERR(chip->charger_edev);

	ChargeInit(EXTCON_CHG_USB_SDP);

	INIT_DELAYED_WORK(&chip->watchdog_work, bq24192_watchdog_work);
	INIT_DELAYED_WORK(&chip->jeita_wrkr, JetiaWork);
	schedule_delayed_work(&chip->watchdog_work, 60*HZ);
	mutex_init(&chip->event_lock);

	/* Initialize the wakelock */
	wake_lock_init(&chip->wakelock, WAKE_LOCK_SUSPEND,
						"ctp_charger_wakelock");

	/* register bq24192 usb with power supply subsystem */
	if (true) { // !chip->pdata->slave_mode
		usb_config.drv_data = chip;
		usb_config.supplied_to = bq24192_supplied_to;
		usb_config.num_supplicants = ARRAY_SIZE(bq24192_supplied_to);
		chip->max_cc = 1216;
		chip->max_cv = 4200;
		chip->chgr_stat = BQ24192_CHRGR_STAT_UNKNOWN;
		chip->usb = power_supply_register(&client->dev, &bq24192_usb_desc,
						&usb_config);
		if (IS_ERR(chip->usb)) {
			ret = PTR_ERR(chip->usb);
			dev_err(&client->dev, "failed:power supply register\n");
			i2c_set_clientdata(client, NULL);
			kfree(chip);
			return ret;
		}
	}
	/* Init Runtime PM State */
	//pm_runtime_put_noidle(&chip->client->dev);
	//pm_schedule_suspend(&chip->client->dev, MSEC_PER_SEC);

	ret = bq24192_powersupply_init(client);
	if (ret)
		return ret;

	INIT_WORK(&chip->charger_type_work, bq24192_update_charger_state);
	INIT_DELAYED_WORK(&chip->otg_work, bq24192_update_otg_state);

	/* Setup extcon notifiers */
	chip->vbus_nb.priority = 1; /* Higher priority to cancel event to PHY */
	chip->vbus_nb.notifier_call = bq24192_vbus_notifier;
	ret = devm_extcon_register_notifier(&client->dev, chip->vbus_edev,
					EXTCON_USB, &chip->vbus_nb);
	if (ret) {
		dev_err(&client->dev,
			"Failed to register VBUS extcon notifier: %d\n", ret);
	}

	chip->charger_nb.notifier_call = bq24192_charger_notifier;
	ret = devm_extcon_register_notifier_all(
			&client->dev, chip->charger_edev, &chip->charger_nb);
	if (ret) {
		dev_err(&client->dev,
			"Failed to register charger extcon notifier: %d\n", ret);
	}

	chip->otg_nb.notifier_call = bq24192_otg_notifier;
	ret = devm_extcon_register_notifier(&client->dev, chip->id_edev,
					EXTCON_USB_HOST, &chip->otg_nb);
	if (ret) {
		dev_err(&client->dev,
			"Failed to register OTG ID extcon notifier: %d\n", ret);
	}

	/* Check initial charger state */
	schedule_work(&chip->charger_type_work);
	flush_work(&chip->charger_type_work);

	/* Check initial OTG state */
	schedule_work(&chip->otg_work.work);
	flush_work(&chip->otg_work.work);

	intel_mid_pmic_writeb(0x24, 0xaa);//vaild battery detection threshold 3.5v
	return 0;
}

static int bq24192_remove(struct i2c_client *client)
{
	struct bq24192_chip *chip = i2c_get_clientdata(client);

	if (true) // !chip->pdata->slave_mode
		power_supply_unregister(chip->usb);

	if (chip->irq > 0)
		free_irq(chip->irq, chip);

	i2c_set_clientdata(client, NULL);
	wake_lock_destroy(&chip->wakelock);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static void bq24192_shutdown(struct i2c_client *client)
{
	struct bq24192_chip *chip = i2c_get_clientdata(client);

	// Disable OTG
	bq24192_otg_boost(chip, false);
}

static int bq24192_suspend(struct device *dev)
{
	struct bq24192_chip *chip = dev_get_drvdata(dev);
	int ret;

	printk("bq24192_suspend\n");
	cancel_delayed_work_sync(&chip->watchdog_work);
	if (chip->irq > 0) {
		/*
		 * Once the WDT is expired all bq24192 registers gets
		 * set to default which means WDT is programmed to 40s
		 * and if there is no charger connected, no point
		 * feeding the WDT. Since reg07[1] is set to default,
		 * charger will interrupt SOC every 40s which is not
		 * good for S3. In this case we need to free chgr_int_n
		 * interrupt so that no interrupt from charger wakes
		 * up the platform in case of S3. Interrupt will be
		 * re-enabled on charger connect.
		 */
		if (chip->irq > 0)
			free_irq(chip->irq, chip);
	}
	reset_wdt_timer(chip);
	ret = program_timers(chip, CHRG_TIMER_EXP_CNTL_WDTDISABLE, false);
	if (ret < 0)
		dev_warn(&chip->client->dev, "TIMER enable failed\n");
	dev_dbg(&chip->client->dev, "bq24192 suspend\n");
	return 0;
}

static int bq24192_resume(struct device *dev)
{
	struct bq24192_chip *chip = dev_get_drvdata(dev);
	int ret;

	printk("bq24192_resume\n");
	if (chip->irq > 0) {
		ret = request_threaded_irq(chip->irq,
				bq24192_irq_isr, bq24192_irq_thread,
				IRQF_TRIGGER_FALLING, "BQ24192", chip);
		if (ret) {
			dev_warn(&bq24192_client->dev,
				"failed to register irq for pin %d\n",
				chip->irq);
		} else {
			dev_warn(&bq24192_client->dev,
				"registered charger irq for pin %d\n",
				chip->irq);
		}
	}
	ChargeInit(cable_status);

	dev_dbg(&chip->client->dev, "bq24192 resume\n");
	schedule_delayed_work(&chip->watchdog_work, 60*HZ);
	return 0;
}
#else
#define bq24192_suspend NULL
#define bq24192_resume NULL
#endif

#ifdef CONFIG_PM
static int bq24192_runtime_suspend(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bq24192_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bq24192_runtime_idle(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#else
#define bq24192_runtime_suspend	NULL
#define bq24192_runtime_resume		NULL
#define bq24192_runtime_idle		NULL
#endif

static const struct i2c_device_id bq24192_id[] = {
	{ DEV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, bq24192_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id bq24192_acpi_match[] = {
	{ "TIBQ2419", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, bq24192_acpi_match);
#endif

static const struct dev_pm_ops bq24192_pm_ops = {
	.suspend		= bq24192_suspend,
	.resume			= bq24192_resume,
	.runtime_suspend	= bq24192_runtime_suspend,
	.runtime_resume		= bq24192_runtime_resume,
	.runtime_idle		= bq24192_runtime_idle,
};

static struct i2c_driver bq24192_i2c_driver = {
	.driver	= {
		.name	= DEV_NAME,
		.owner	= THIS_MODULE,
		.acpi_match_table = ACPI_PTR(bq24192_acpi_match),
		.pm	= &bq24192_pm_ops,
	},
	.probe		= bq24192_probe,
	.remove		= bq24192_remove,
	.id_table	= bq24192_id,
	.shutdown   = bq24192_shutdown,
};

static int __init bq24192_init(void)
{
	return i2c_add_driver(&bq24192_i2c_driver);
}
module_init(bq24192_init);
static void __exit bq24192_exit(void)
{
	i2c_del_driver(&bq24192_i2c_driver);
}
module_exit(bq24192_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_AUTHOR("Raj Pandey <raj.pandey@intel.com>");
MODULE_DESCRIPTION("BQ24192 Charger Driver");
MODULE_LICENSE("GPL");
