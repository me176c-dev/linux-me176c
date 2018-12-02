#ifndef	__I2CSEQ_LINUX_H__
#define	__I2CSEQ_LINUX_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#define	I2C_SEQOPS_HASHTABLE_SIZE	7

typedef struct _i2c_seqops_entry_t {
	const char *name;
	const char *fw_path;		//relate to "/system/firmware/", refer to function "request_firmware"
	const char **pkg_names;
	int is_missing_pkg;
	int missing_pkg_index;
	struct mutex mutex;
	struct i2c_client *client;
	struct device_attribute dev_attr;
	int is_create_attr_file;
	int fw_loadstatus;
	struct list_head ht[I2C_SEQOPS_HASHTABLE_SIZE];
	__u8 *fw_data;			//may be NULL
	size_t fw_len;
	__u32 fw_version;
	void *list_array;
	size_t list_array_len;
} i2c_seqops_entry_t;

#define __I2C_SEQOPS_ENTRY_INIT(in_name, in_fw_path, in_pkg_names) \
		{ .name = #in_name, \
		  .fw_path = in_fw_path, \
		  .pkg_names = in_pkg_names, \
		  .mutex = __MUTEX_INITIALIZER(in_name.mutex) }

#define	DEFINE_I2C_SEQOPS_ENTRY(name, fw_path, pkg_names)	\
	struct _i2c_seqops_entry_t name = __I2C_SEQOPS_ENTRY_INIT(name, fw_path, pkg_names)

int i2c_seqops_init(i2c_seqops_entry_t *entry, struct i2c_client *client);

int i2c_seqops_updateclient(i2c_seqops_entry_t *entry, struct i2c_client *client);

void i2c_seqops_clear(i2c_seqops_entry_t *entry);

int i2c_seqops_reloadfw(i2c_seqops_entry_t *entry, int force);

int i2c_seqops_exec(i2c_seqops_entry_t *entry, const char *pkg_name);


#endif	/* __I2CSEQ_LINUX_H__ */
