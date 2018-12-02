#include "i2c_seqops.h"
#include "i2cseq_type.h"
#include <linux/firmware.h>
#include <linux/list.h>

#define I2C_SEQOPS_TRANS_RETRY	5

static __u32 i2c_seqops_calc_checksum(void* pdata, size_t len)
{
	//calculate checksum
	__u32 checksum = 0;
	__u32 size32 = len/4;
	__u32 *pPack32 = (__u32*)pdata;
	size_t	i = 0;
	size_t size8 = 0;
	__u8	*pPack8 = NULL;

	while(size32) {
		checksum += *pPack32;
		++pPack32;
		--size32;
	}

	size8 = len%4;
	pPack8 = (__u8*)pPack32;
	for (i = 0; i < size8; i++)
	{
		checksum += ((__u32)*pPack8)<<(i*8);
		++pPack8;
	}

	return 0-checksum;
}

typedef struct _ht_node_t {
	struct list_head list;
	i2cseq_pkg_head_t *pkg;
	size_t pkg_name_len;
} ht_node_t;

static int i2c_seqops_hashfunc(const char *pkg_name)
{
	int seed = 131;
	int key = 0;

	if (!pkg_name) {
		return 0;
	}

	//BKDR hash function
	while (*pkg_name) {
		key = key * seed + (*(pkg_name++));
	}

	key = key % I2C_SEQOPS_HASHTABLE_SIZE;

	return key < 0 ? (key + I2C_SEQOPS_HASHTABLE_SIZE) : key;
}

//initialize the hash table
static void i2c_seqops_ht_init(i2c_seqops_entry_t *entry)
{
	int i = 0;
	for (i = 0; i < sizeof(entry->ht)/sizeof(struct list_head); i++) {
		INIT_LIST_HEAD(entry->ht + i);
	}
	entry->list_array = NULL;
	entry->list_array_len = 0;
}

static void i2c_seqops_ht_reset(i2c_seqops_entry_t *entry)
{
	int i = 0;
	for (i = 0; i < sizeof(entry->ht)/sizeof(struct list_head); i++) {
		INIT_LIST_HEAD(entry->ht + i);
	}
}

static void i2c_seqops_ht_clear(i2c_seqops_entry_t *entry)
{
	i2c_seqops_ht_reset(entry);
	if (entry->list_array != NULL) {
		kfree(entry->list_array);
		entry->list_array = NULL;
	}
	entry->list_array_len = 0;
}

typedef enum _i2c_seqops_check_pre_micro_type_t {
	t_null = 0,
	t_start,
	t_end,
	t_start_end,
	t_repeat,
} i2c_seqops_check_pre_micro_type_t;

typedef struct _i2c_seqops_check_micro_t {
	i2c_seqops_check_pre_micro_type_t type;
	__u16 subopcode;
} i2c_seqops_check_micro_t;

typedef struct _i2c_seqops_check_component_t {
	__u16 opcode;
	 const i2c_seqops_check_micro_t *micro;	//[3];
} i2c_seqops_check_component_t;

static const i2c_seqops_check_micro_t g_check_c_end[3] =
{
	{
		.type = t_start_end,
	},
};

static const i2c_seqops_check_micro_t g_check_c_wreg[3] =
{
	{
		.type = t_start_end,
	},
};

static const i2c_seqops_check_micro_t g_check_c_wbits[3] =
{
	{
		.type = t_start,
		.subopcode = I2CSEQ_SUBOPCODE_WBITS_V,
	},
	{
		.type = t_end,
		.subopcode = I2CSEQ_SUBOPCODE_WBITS_M,
	},
};

static const i2c_seqops_check_micro_t g_check_c_pbits[3] =
{
	{
		.type = t_start,
		.subopcode = I2CSEQ_SUBOPCODE_PBITS_V,
	},
	{
		.type = t_end,
		.subopcode = I2CSEQ_SUBOPCODE_PBITS_DM,
	},
};

static const i2c_seqops_check_micro_t g_check_c_burst[3] =
{
	{
		.type = t_start,
		.subopcode = I2CSEQ_SUBOPCODE_BURST_S,
	},
	{
		.type = t_repeat,
		.subopcode = I2CSEQ_SUBOPCODE_BURST_V,
	},
	{
		.type = t_end,
		.subopcode = I2CSEQ_SUBOPCODE_BURST_E,
	},
};

static const i2c_seqops_check_micro_t g_check_c_delay[3] =
{
	{
		.type = t_start_end,
	},
};

static const struct _i2c_seqops_check_component_t g_check_table[] =
{
	{
		.opcode = I2CSEQ_OPCODE_END,
		.micro = g_check_c_end,
	},
	{
		.opcode = I2CSEQ_OPCODE_WREG,
		.micro = g_check_c_wreg,
	},
	{
		.opcode = I2CSEQ_OPCODE_WBITS,
		.micro = g_check_c_wbits,
	},
	{
		.opcode = I2CSEQ_OPCODE_PBITS,
		.micro = g_check_c_pbits,
	},
	{
		.opcode = I2CSEQ_OPCODE_BURST,
		.micro = g_check_c_burst,
	},
	{
		.opcode = I2CSEQ_OPCODE_DELAY,
		.micro = g_check_c_delay,
	},
};

#define I2C_SEQOPS_CHECKTABLE_SIZE	(sizeof(g_check_table)/sizeof(struct _i2c_seqops_check_component_t))
static const i2c_seqops_check_micro_t *i2c_seqops_get_checkmicro(__u16 opcode)
{
	if (I2CSEQ_OPCODE(opcode) >= I2C_SEQOPS_CHECKTABLE_SIZE) {
		return NULL;
	}

	return g_check_table[I2CSEQ_OPCODE(opcode)].micro;
}

static int i2c_seqops_checkfw(const struct firmware *fw)
{
	i2cseq_file_header_t *f_header = (i2cseq_file_header_t *)fw->data;
	size_t header_size = sizeof(i2cseq_file_header_t);
	i2cseq_pkg_mapping_node_t *pkg_mapnode = NULL;
	i2cseq_pkg_head_t *pkg_head = NULL;
	__u16 addr_size = 0;
	size_t op_num = 0;
	i2cseq_ops_t *ops = NULL;
	const i2c_seqops_check_micro_t *micro = NULL;
	int bEnd = 1;
	__u16 pre_opcode = 0;	//full opcode
	__u16 cur_opcode = 0;	//full opcode
	int i = 0, j = 0;

	//check file header

	if (fw->size < header_size) {
		pr_info("%s: check header size fail without packages map\n", __func__);
		return -EINVAL;
	}

	header_size += f_header->pkg_num * sizeof(i2cseq_pkg_mapping_node_t);
	if (fw->size < header_size) {
		pr_info("%s: check header size fail with packages map\n", __func__);
		return -EINVAL;
	}

	//signature
	if (f_header->signature != I2CSEQ_FILE_SIGNATURE) {
		pr_info("%s: check header signature fail\n", __func__);
		return -EINVAL;
	}

	//version
	if (f_header->version != I2CSEQ_FILE_VERSION(0, 2)) {
		pr_info("%s: file protocol version 0x%08X is not supported!\n",
				__func__, f_header->version);
		return -EINVAL;
	}

	//header checksum
	if (i2c_seqops_calc_checksum(f_header, header_size) != 0) {
		pr_info("%s: check header checksum fail\n", __func__);
		return -EINVAL;
	}

	//package number
	if (f_header->pkg_num == 0) {
		pr_info("%s: not pakages in the file!\n", __func__);
		return -EINVAL;
	}

	//address size
	switch (f_header->addr_size) {
	case I2CSEQ_ADDRSIZE_BYTE:
		break;
	case I2CSEQ_ADDRSIZE_WORD:
		break;
	default:
		pr_info("%s: Address size(0x%04X) is not supported!\n",
				__func__, f_header->addr_size);
		return -EINVAL;
	}
	addr_size = f_header->addr_size;
	pr_info("%s: addr_size 0x%04x\n", __func__, addr_size);

	//check package names and size
	pkg_mapnode = (i2cseq_pkg_mapping_node_t *)(f_header + 1);
	for (i = 0; i < f_header->pkg_num; ++i) {
		//check package name length
		if (strlen(pkg_mapnode->name) >= sizeof(pkg_mapnode->name)) {
			pr_info("%s: package %d's is too long\n",
					__func__, i);
			return -EINVAL;
		}

		if (strlen(pkg_mapnode->name) == 0) {
			pr_info("%s: package %d's name is empty!\n", __func__, i);
			return -EINVAL;
		}

		if (fw->size < (sizeof(i2cseq_pkg_head_t) + pkg_mapnode->f_offset)) {
			pr_info("%s: package \"%s\"(%d)''s size seems to be wrong\n",
				__func__, pkg_mapnode->name, i);
			return -EINVAL;
		}

		pkg_head = (i2cseq_pkg_head_t *)(fw->data + pkg_mapnode->f_offset);
		if (strlen(pkg_head->name) >= sizeof(pkg_head->name) ||
		    strcmp(pkg_head->name, pkg_mapnode->name) != 0) {
			pr_info("%s: the name in package(%d) header is invalid!\n",
					__func__, i);
			return -EINVAL;
		}

		//check type
		if (pkg_head->type != I2CSEQ_PACKAGE_TYPE_OPS) {
			pr_info("%s: package \"%s(%d)\"'s type %d is not supported!\n",
					__func__, pkg_head->name, i, pkg_head->type);
			return -EINVAL;
		}

		//check size
		if (pkg_head->data_len % sizeof(i2cseq_ops_t) != 0) {
			pr_info("%s: package \"%s(%d)\"'s data length %d is invalid!\n",
					__func__, pkg_head->name, i, pkg_head->data_len);
			return -EINVAL;
		}

		//check checksum
		if (i2c_seqops_calc_checksum(
				pkg_head,
				sizeof(i2cseq_pkg_head_t) + pkg_head->data_len) != 0) {
			pr_info("%s: package \"%s(%d)\"'s checksum is invalid!\n",
					__func__, pkg_head->name, i);
			return -EINVAL;
		}

		//
		op_num = pkg_head->data_len / sizeof(i2cseq_ops_t);
		if (op_num < 1) {
			pr_info("%s: package \"%s(%d)\"'s at least has one operation.!\n",
				__func__, pkg_head->name, i);
			return -EINVAL;
		}

		ops = (i2cseq_ops_t *)(pkg_head + 1);
		bEnd = 1;
		//pre_opcode = 0;
		for (j = 0; j < op_num; ++j) {
			cur_opcode = ops[j].opcode;

			if (I2CSEQ_ADDRSIZE(cur_opcode) != addr_size) {
				pr_info("%s: package \"%s(%d)\"'s %d'th FULL OPCODE(0x%04x) address size is not matched!\n",
					__func__, pkg_head->name, i, j, cur_opcode);
				return -EINVAL;
			}

			micro = i2c_seqops_get_checkmicro(cur_opcode);
			if (micro == NULL) {
				pr_info("%s: package \"%s(%d)\"'s %d'th FULL OPCODE(0x%04x) is not supported!\n",
					__func__, pkg_head->name, i, j, cur_opcode);
				return -EINVAL;
			}

			switch (micro[I2CSEQ_SUBOPCODE(cur_opcode)>>I2CSEQ_SUBOPCODE_BIT_OFFSET].type) {
			case t_start:
				//End by type "t_end"
				if (bEnd != 1) {
					pr_info("%s: package \"%s(%d)\"'s %d'th FULL OPCODE(0x%04x) sequence check fail !\n",
						__func__, pkg_head->name, i, j, cur_opcode);
					return -EINVAL;
				}
				bEnd = 0;
				break;
			case t_end:
				if (pre_opcode != I2CSEQ_OPCODE(cur_opcode) || bEnd == 1) {
					pr_info("%s: package \"%s(%d)\"'s %d'th FULL OPCODE(0x%04x) sequence check fail!\n",
						__func__, pkg_head->name, i, j, cur_opcode);
					return -EINVAL;
				}
				bEnd = 1;
				break;
			case t_start_end:
				//new operation components
				if (bEnd != 1) {
					pr_info("%s: package \"%s(%d)\"'s %d'th FULL OPCODE(0x%04x) sequence check fail !\n",
						__func__, pkg_head->name, i, j, cur_opcode);
					return -EINVAL;
				}
				bEnd = 1;
				break;
			case t_repeat:
				if (pre_opcode != I2CSEQ_OPCODE(cur_opcode) || bEnd == 1) {
					pr_info("%s: package \"%s(%d)\"'s %d'th FULL OPCODE(0x%04x) sequence check fail!\n",
						__func__, pkg_head->name, i, j, cur_opcode);
					return -EINVAL;
				}
				break;
			default:
				pr_info("%s: package \"%s(%d)\"'s %d'th FULL OPCODE(0x%04x) is not supported!\n",
					__func__, pkg_head->name, i, j, cur_opcode);
				return -EINVAL;
				break;
			}
			pre_opcode = I2CSEQ_OPCODE(cur_opcode);

			if (j == (op_num - 1) &&
			    I2CSEQ_OPCODE(cur_opcode) != I2CSEQ_OPCODE_END) {
				pr_info("%s: package \"%s(%d)\"'s %d'th FULL OPCODE(0x%04x) must be I2CSEQ_OPCODE_END!\n",
					__func__, pkg_head->name, i, j, cur_opcode);
				return -EINVAL;
			}
		}

		pr_info("%s: check package \"%s(%d)\" PASS\n",
			__func__, pkg_head->name, i);

		pkg_mapnode++;
	}

	return 0;
}

static int i2c_seqops_addfw2ht(i2c_seqops_entry_t *entry)
{
	i2cseq_file_header_t *f_header = (i2cseq_file_header_t *)entry->fw_data;
	i2cseq_pkg_mapping_node_t *pkg_mapnode = (i2cseq_pkg_mapping_node_t *)(f_header + 1);
	ht_node_t *ht_node = NULL;
	i2cseq_pkg_head_t *pkg_head = NULL;
	int i = 0;

	entry->fw_version = f_header->fw_version;

	//allocate memory for lists
	if (entry->list_array_len < f_header->pkg_num * sizeof(ht_node_t)) {
		if (entry->list_array != NULL) {
			kfree(entry->list_array);
		}
		entry->list_array_len = f_header->pkg_num * sizeof(ht_node_t);
		entry->list_array = kmalloc(entry->list_array_len, GFP_KERNEL);
		if (entry->list_array == NULL) {
			entry->list_array_len = 0;
			pr_err("%s: out of memory 0x%x\n",
					__func__,
					f_header->pkg_num * sizeof(ht_node_t));
			return -ENOMEM;
		}
	}
	ht_node = (ht_node_t *)entry->list_array;

	for (i = 0; i < f_header->pkg_num; ++i) {
		pkg_head = (i2cseq_pkg_head_t *)(entry->fw_data + pkg_mapnode->f_offset);
		ht_node[i].pkg = pkg_head;
		ht_node[i].pkg_name_len = strlen(pkg_head->name);
		list_add_tail(
			&ht_node[i].list,
			&entry->ht[i2c_seqops_hashfunc(pkg_head->name)]);

		pkg_mapnode++;
	}

	return 0;
}

static const i2cseq_pkg_head_t *i2c_seqops_name2pkg(i2c_seqops_entry_t *entry, const char *name)
{
	ht_node_t *node;
	i2cseq_pkg_head_t * pkg = NULL;
	int k = i2c_seqops_hashfunc(name);
	size_t name_len = strlen(name);

	list_for_each_entry(node, &entry->ht[k], list) {
		if (node->pkg_name_len == name_len &&
		    strcmp(node->pkg->name, name) == 0) {
			pkg = node->pkg;
		}
	}

	return pkg;
}

static void i2c_seqops_set_missing_pkgs(i2c_seqops_entry_t *entry, int is_missing, int index)
{
	mutex_lock(&entry->mutex);
	entry->is_missing_pkg = is_missing;
	entry->missing_pkg_index = index;
	mutex_unlock(&entry->mutex);
}

static void i2c_seqops_get_missing_pkgs(i2c_seqops_entry_t *entry, int *is_missing, int *index)
{
	mutex_lock(&entry->mutex);
	*is_missing = entry->is_missing_pkg;
	*index = entry->missing_pkg_index;
	mutex_unlock(&entry->mutex);
}

static int i2c_seqops_check_missing_pkgs(i2c_seqops_entry_t *entry)
{
	int i = 0;
	if (!entry->pkg_names) {
		pr_info("%s: checking packages is NULL\n", __func__);
		i2c_seqops_set_missing_pkgs(entry, 0, 0);
		return -EINVAL;
	}

	mutex_lock(&entry->mutex);
	while (entry->pkg_names[i]) {
		if (!i2c_seqops_name2pkg(entry, entry->pkg_names[i])) {
			mutex_unlock(&entry->mutex);
			i2c_seqops_set_missing_pkgs(entry, 1, i);
			pr_info("%s: miss package \"%s\"\n", __func__, entry->pkg_names[i]);
			return -EEXIST;
		}
		++i;
	}
	mutex_unlock(&entry->mutex);

	i2c_seqops_set_missing_pkgs(entry, 0, 0);

	return 0;
}

static void i2c_seqops_set_loadstatus(i2c_seqops_entry_t *entry, int status)
{
	mutex_lock(&entry->mutex);
	entry->fw_loadstatus = status;
	mutex_unlock(&entry->mutex);
}

static int i2c_seqops_get_loadstatus(i2c_seqops_entry_t *entry)
{
	int status = 0;
	mutex_lock(&entry->mutex);
	status = entry->fw_loadstatus;
	mutex_unlock(&entry->mutex);
	return status;
}

static int _i2c_seqops_reloadfw(i2c_seqops_entry_t *entry)
{
	int ret = 0;
	const struct firmware *fw = NULL;

	if (!entry || entry->fw_path == NULL) {
		pr_info("%s: invalid parameters\n", __func__);
		return -EINVAL;
	}

	request_firmware(&fw, entry->fw_path, &entry->client->dev);
	if (fw == NULL || fw->data == NULL) {
		pr_err("%s: unable to load firmware \"%s\"\n",
				__func__, entry->fw_path);
		return -EEXIST;
	}

	ret = i2c_seqops_checkfw(fw);
	if (ret != 0) {
		goto l_exit;
	}

	i2c_seqops_set_missing_pkgs(entry, 0, 0);

	mutex_lock(&entry->mutex);

	i2c_seqops_ht_reset(entry);

	//copy firmware
	if (entry->fw_len < fw->size) {
		if (entry->fw_data != NULL) {
			kfree(entry->fw_data);
		}
		entry->fw_len = fw->size;
		entry->fw_data = (__u8 *)kmalloc(entry->fw_len, GFP_KERNEL);
		if (entry->fw_data == NULL) {
			entry->fw_len = 0;
			ret = -ENOMEM;
			pr_info("%s: out of memory 0x%x\n", __func__, fw->size);
			goto l_unlock;
		}
	}

	memcpy(entry->fw_data, fw->data, fw->size);

	ret = i2c_seqops_addfw2ht(entry);

l_unlock:
	mutex_unlock(&entry->mutex);
	if (ret < 0) {
		goto l_exit;
	}

	ret = i2c_seqops_check_missing_pkgs(entry);

l_exit:
	release_firmware(fw);

	return ret;
}

int i2c_seqops_reloadfw(i2c_seqops_entry_t *entry, int force)
{
	int ret = 0;

	if (force == 0 &&
	    i2c_seqops_get_loadstatus(entry) == 0) {
		//pr_info("%s: the firmware is already loaded!\n", __func__);
		return 0;
	}

	ret = _i2c_seqops_reloadfw(entry);
	i2c_seqops_set_loadstatus(entry, ret);

	return ret;
}

EXPORT_SYMBOL(i2c_seqops_reloadfw);

static void i2c_seqops_release_fw(i2c_seqops_entry_t *entry)
{
	if (entry->fw_data != NULL) {
		kfree(entry->fw_data);
		entry->fw_data = NULL;
	}
	entry->fw_len = 0;
}

static ssize_t i2c_seqops_dev_attr_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	i2c_seqops_entry_t *entry = container_of(attr, i2c_seqops_entry_t, dev_attr);
	int is_miss = 0, miss_index = 0;
	int load_status = i2c_seqops_get_loadstatus(entry);
	i2c_seqops_get_missing_pkgs(entry, &is_miss, &miss_index);
	if (is_miss) {
		return snprintf(buf, PAGE_SIZE,
				"load firmware \"%s\" status: %d\nmiss package:%s\n",
				entry->fw_path, load_status, entry->pkg_names[miss_index]);
	} else {
		return snprintf(buf, PAGE_SIZE,
				"load firmware \"%s\" status: %d, fw_version=%d.%d.%d.%d\n",
				entry->fw_path, load_status,
				(entry->fw_version>>24)&0xff,
				(entry->fw_version>>16)&0xff,
				(entry->fw_version>>8)&0xff,
				(entry->fw_version>>0)&0xff);
	}
}

static ssize_t i2c_seqops_dev_attr_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	i2c_seqops_entry_t *entry = container_of(attr, i2c_seqops_entry_t, dev_attr);
	int ret = _i2c_seqops_reloadfw(entry);
	i2c_seqops_set_loadstatus(entry, ret);
	return strnlen(buf, PAGE_SIZE);
}

int i2c_seqops_init(i2c_seqops_entry_t *entry, struct i2c_client *client)
{
	int ret = 0;
	int endian = 1;

	//big endian is not supported
	if (*(char*)&endian != 1) {
		pr_err("%s: big endian is not supported!\n", __func__);
		return -EFAULT;
	}

	if (!client || !entry->fw_path) {
		return -EINVAL;
	}

	if (!entry->pkg_names) {
		return -EINVAL;
	}

	entry->client = client;

	i2c_seqops_ht_init(entry);

	entry->fw_data = NULL;
	entry->fw_len = 0;
	entry->is_missing_pkg = 0;
	entry->missing_pkg_index = 0;

	entry->dev_attr.show = i2c_seqops_dev_attr_show;
	entry->dev_attr.store = i2c_seqops_dev_attr_store;
	entry->dev_attr.attr.mode = S_IRUGO | S_IWUSR;
	entry->dev_attr.attr.name = "i2c_seqops";

	ret = device_create_file(&client->dev, &entry->dev_attr);
	if (ret != 0) {
		pr_err("%s: create attribute file fail!\n", __func__);
		return ret;
	}
	mutex_lock(&entry->mutex);
	entry->is_create_attr_file = 1;
	mutex_unlock(&entry->mutex);

	ret = _i2c_seqops_reloadfw(entry);

	i2c_seqops_set_loadstatus(entry, ret);

	if (ret != 0) {
		pr_err("%s: load firmware \"%s\" fail\n", __func__, entry->fw_path);
	}

	return ret;
}
EXPORT_SYMBOL(i2c_seqops_init);

int i2c_seqops_updateclient(i2c_seqops_entry_t *entry, struct i2c_client *client)
{
	if (entry == NULL || client == NULL) {
		return -EINVAL;
	}

	mutex_lock(&entry->mutex);
	entry->client = client;
	mutex_unlock(&entry->mutex);

	return 0;
}
EXPORT_SYMBOL(i2c_seqops_updateclient);


void i2c_seqops_clear(i2c_seqops_entry_t *entry)
{
	mutex_lock(&entry->mutex);
	i2c_seqops_ht_clear(entry);
	i2c_seqops_release_fw(entry);
	if (entry->is_create_attr_file) {
		device_remove_file(&entry->client->dev, &entry->dev_attr);
		entry->is_create_attr_file = 0;
	}
	mutex_unlock(&entry->mutex);
}
EXPORT_SYMBOL(i2c_seqops_clear);

static int i2c_seqops_wreg(
	struct i2c_client *client,
	int addr_size,
	__u16 addr,
	int value_size,
	__u32 value)
{
	int ret = 0;
	__u8 data[6];
	struct i2c_msg msg;
	int retry = 0;

	if (client->adapter == NULL) {
		pr_info("%s: no client->adapter\n", __func__);
		return -ENODEV;
	}

	switch (addr_size) {
	case 1:
		data[0] =  (__u8)addr;
		break;
	case 2:
		*((__u16*)data) = cpu_to_be16(addr);
		break;
	default:
		pr_info("%s: invalid address size(%d)", __func__, addr_size);
		return -EINVAL;
		break;
	}

	switch (value_size) {
	case 1:
		data[addr_size] = (__u8)value;
		break;
	case 2:
		*((__u16*)&data[addr_size]) = cpu_to_be16((__u16)value);
		break;
	case 4:
		*((__u32*)&data[addr_size]) = cpu_to_be32(value);
		break;
	default:
		pr_info("%s: invalid value size(%d)", __func__, value_size);
		return -EINVAL;
		break;
	}

	memset(&msg, 0, sizeof(msg));

	for(retry = 0; retry < I2C_SEQOPS_TRANS_RETRY; ++retry) {
		msg.addr = client->addr;
		msg.flags = 0;
		msg.len = addr_size + value_size;
		msg.buf = data;

		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret >= 0) {
			return 0;
		}

		dev_err(&client->dev, "retrying... %d", retry + 1);
	}

	dev_err(&client->dev, "%s: addr_size %d addr 0x%04x value_size %d value %08x fail",
			__func__,
			addr_size, addr,
			value_size, value);

	return ret;
}

//return: negative error code
//	  or the processed operations count
static int i2c_seqops_exec_op_burst(
	struct i2c_client *client,
	const i2cseq_ops_t *ops)
{
	int count = 0;
	size_t buf_len = 0;
	int ret = 0;
	i2cseq_ops_t const *p = ops + 1;
	u8 *buf = NULL;
	u8 *buf_ptr = NULL;
	int i = 0;
	__u16 addr = 0;
	int addr_size = 0;
	struct i2c_msg msg;

	if (client->adapter == NULL) {
		pr_info("%s: no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (I2CSEQ_FULLOPCODE(ops->opcode) != (I2CSEQ_OPCODE_BURST|I2CSEQ_SUBOPCODE_BURST_S)) {
		pr_info("%s: not burst ops\n",
			__func__);
		return -EINVAL;
	}

	switch (I2CSEQ_ADDRSIZE(ops->opcode)) {
	case I2CSEQ_ADDRSIZE_BYTE:
		addr = (__u8)(ops->addr);
		break;
	case I2CSEQ_ADDRSIZE_WORD:
		addr = cpu_to_be16(ops->addr);
		break;
	default:
		break;
	}
	addr_size = I2CSEQ_ADDRSIZE_BYTES(ops->opcode);
	buf_len += addr_size;

	//count first.
	while(I2CSEQ_SUBOPCODE(p->opcode) != I2CSEQ_SUBOPCODE_BURST_E) {
		buf_len += I2CSEQ_VALUESIZE_BYTES(p->opcode);
		count++;
		p++;
	}

	if (count == 0) {
		ret = 2;
		goto l_ret;
	}

	//pr_info("%s: Burst addr=0x%x count = %d buf_len=0x%x\n ",
	//	__func__, ops->addr, count, buf_len);

	//allocate memory
	buf = kmalloc(buf_len, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto l_ret;
	}

	if (addr_size == 1) {
		*buf = (__u8)addr;
	} else {
		*((__u16*)buf) = addr;
	}

	p = ops + 1;
	buf_ptr = buf + addr_size;
	for (i = 0; i < count; ++i) {
		switch(I2CSEQ_VALUESIZE_BYTES(p->opcode)) {
		case 1:
			*buf_ptr = (u8)(p->value);
			break;
		case 2:
			*((u16*)buf_ptr) = cpu_to_be16((u16)(p->value));
			break;
		case 4:
			*((u32*)buf_ptr) = cpu_to_be32((u32)(p->value));
			break;
		}
		buf_ptr += I2CSEQ_VALUESIZE_BYTES(p->opcode);
		p++;
	}

	memset(&msg, 0, sizeof(msg));
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = buf_len;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);

	kfree(buf);

	if (ret < 0) {
		pr_info("%s: burst transport %d bytes fail\n",
			__func__, buf_len);
		goto l_ret;
	}

	ret = count + 2;

l_ret:
	return ret;

}

static int i2c_seqops_rreg(
	struct i2c_client *client,
	int addr_size,
	__u16 addr,
	int value_size,
	__u32 *value)
{
	int ret = 0;
	struct i2c_msg msg[2];
	__u8 c_addr[2];
	__u32 u32_value = 0;

	if (client->adapter == NULL) {
		pr_info("%s: no client->adapter\n", __func__);
		return -ENODEV;
	}

	switch (addr_size) {
	case 1:
		c_addr[0] =  (__u8)addr;
		break;
	case 2:
		*((__u16*)c_addr) = cpu_to_be16(addr);
		break;
	default:
		pr_info("%s: invalid address size(%d)", __func__, addr_size);
		return -EINVAL;
		break;
	}

	switch (value_size) {
	case 1:
		break;
	case 2:
		break;
	case 4:
		break;
	default:
		pr_info("%s: invalid value size(%d)", __func__, value_size);
		return -EINVAL;
		break;
	}

	memset(msg, 0, sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = c_addr;
	msg[0].len = addr_size;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &u32_value;
	msg[1].len = (__u16)value_size;

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret >=0){
		switch (value_size) {
		case 1:
			*value = (__u32)((__u8)u32_value);
			break;
		case 2:
			*value = (__u32)(be16_to_cpu((__u16)u32_value));
			break;
		case 4:
			*value = be32_to_cpu(u32_value);
			break;
		default:
			return -EINVAL;
		}
	} else {
		dev_err(&client->dev, "%s: addr_size %d addr 0x%04x value_size %d fail",
				__func__,
				addr_size, addr,
				value_size);
	}

	return ret;
}

//return: negative error code
//	  or the processed operations count
static int i2c_seqops_exec_op_wbits(
	struct i2c_client *client,
	const i2cseq_ops_t *ops)
{
	int ret = 0;
	__u32 value = 0;

	//check opcode
	if (I2CSEQ_FULLOPCODE(ops->opcode) != (I2CSEQ_OPCODE_WBITS|I2CSEQ_SUBOPCODE_WBITS_V)) {
		pr_info("%s: not WBITS ops\n",
			__func__);
		return -EINVAL;
	}

	ret = i2c_seqops_rreg(
		client,
		I2CSEQ_ADDRSIZE_BYTES(ops->opcode),
		ops->addr,
		I2CSEQ_VALUESIZE_BYTES(ops->opcode),
		&value);
	if (ret < 0) {
		pr_info("%s: read from address 0x%04x fail\n",
			__func__, ops->addr);
		return ret;
	}

	value &= ~(ops+1)->value;		//mask
	value |= ops->value & (ops+1)->value;

	ret = i2c_seqops_wreg(
		client,
		I2CSEQ_ADDRSIZE_BYTES(ops->opcode),
		ops->addr,
		I2CSEQ_VALUESIZE_BYTES(ops->opcode),
		value);

	if (ret < 0) {
		pr_info("%s: write to address 0x%04x fail\n",
			__func__, ops->addr);
		return ret;
	}

	return 2;
}

//return: negative error code
//	  or the processed operations count
static int i2c_seqops_exec_op_pbits(
	struct i2c_client *client,
	const i2cseq_ops_t *ops)
{
	int ret = 0;
	__u32 value = 0;
	int delay_unit = 1;	//ms
	int delay = 0;

	//check opcode
	if (I2CSEQ_FULLOPCODE(ops->opcode) != (I2CSEQ_OPCODE_PBITS|I2CSEQ_SUBOPCODE_PBITS_V)) {
		pr_info("%s: not PBITS ops\n",
			__func__);
		return -EINVAL;
	}

	if ((ops + 1)->addr > 200) {
		delay_unit = 100;	//ms
	} else if ((ops + 1)->addr > 50) {
		delay_unit = 10;	//ms
	}

	for (delay = 0; delay <  (ops + 1)->addr; delay += delay_unit) {
		ret = i2c_seqops_rreg(
			client,
			I2CSEQ_ADDRSIZE_BYTES(ops->opcode),
			ops->addr,
			I2CSEQ_VALUESIZE_BYTES(ops->opcode),
			&value);
		if (ret < 0) {
			pr_info("%s: read from address 0x%04x fail\n",
				__func__, ops->addr);
			return ret;
		}

		if ( (value & (ops + 1)->value) == (ops->value & (ops + 1)->value)) {
			return 2;
		}

		mdelay(delay_unit);
	}

	return -EAGAIN;
}

static int i2c_seqops_exec_pkg(i2c_seqops_entry_t *entry, i2cseq_pkg_head_t *pkg)
{
	int ret = 0;
	i2cseq_ops_t *ops = (i2cseq_ops_t *)(pkg + 1);

	while (I2CSEQ_OPCODE(ops->opcode) != I2CSEQ_OPCODE_END) {
		switch (I2CSEQ_OPCODE(ops->opcode)) {
		case I2CSEQ_OPCODE_WREG:
			ret = i2c_seqops_wreg(
				entry->client,
				I2CSEQ_ADDRSIZE_BYTES(ops->opcode),
				ops->addr,
				I2CSEQ_VALUESIZE_BYTES(ops->opcode),
				ops->value);
			if (ret >= 0) {
				ret = 1;
			}
			break;

		case I2CSEQ_OPCODE_WBITS:
			ret = i2c_seqops_exec_op_wbits(entry->client, ops);
			break;

		case I2CSEQ_OPCODE_PBITS:
			ret = i2c_seqops_exec_op_pbits(entry->client, ops);
			break;

		case I2CSEQ_OPCODE_BURST:
			ret = i2c_seqops_exec_op_burst(entry->client, ops);
			break;

		case I2CSEQ_OPCODE_DELAY:
			mdelay(ops->value);
			ret = 1;
			break;
		default:
			ret = -EEXIST;
			break;
		}

		if (ret < 0) {
			pr_info("%s: exec package \"%s\" at %d't operation fail, err=%d\n",
				__func__, pkg->name,
				ops - (i2cseq_ops_t *)(pkg + 1), ret);
			return ret;
		}

		ops += ret;
	}

	return 0;
}

int i2c_seqops_exec(i2c_seqops_entry_t *entry, const char *pkg_name)
{
	int ret = 0;
	i2cseq_pkg_head_t *pkg = NULL;

	if (pkg_name == NULL) {
		return -EINVAL;
	}

	mutex_lock(&entry->mutex);

	pkg = i2c_seqops_name2pkg(entry, pkg_name);
	if (pkg == NULL) {
		pr_info("%s: package \"%s\" is not exist\n", __func__, pkg_name);
		ret = -EEXIST;
		goto l_exit;
	}

	ret = i2c_seqops_exec_pkg(entry, pkg);

l_exit:
	mutex_unlock(&entry->mutex);

	return ret;
}
EXPORT_SYMBOL(i2c_seqops_exec);
