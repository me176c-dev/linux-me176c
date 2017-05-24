#include <linux/module.h>
#include <linux/i2c.h>

#define	BQ24192_I2C_ADAPTER	0

static struct i2c_board_info bq24192_i2c_board_info = {
	I2C_BOARD_INFO("bq24192", 0x6b),
};

static int __init bq24192_platform_init(void)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;

	adapter = i2c_get_adapter(BQ24192_I2C_ADAPTER);
	if (!adapter) {
		pr_err("Cannot get I2C adapter for bq24192\n");
		goto err;
	}

	client = i2c_new_device(adapter, &bq24192_i2c_board_info);
	if (!client) {
		pr_err("Failed to create new I2C device for bq24192\n");
		goto err;
	}

err:
	return 0;
}
module_init(bq24192_platform_init);
