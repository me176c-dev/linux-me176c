#ifndef	__I2CSEQ_TYPE_H__
#define __I2CSEQ_TYPE_H__

#ifdef _WINDOWS

#include <Windows.h>

#ifndef __u8
#define __u8	UINT8
#endif

#ifndef	__u16
#define __u16	UINT16
#endif

#ifndef	__u32
#define __u32	UINT32
#endif

#else

//include linux head files
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#endif

#pragma pack(1)

//===============================================================
//I2C sequence operations
//===============================================================

typedef struct _i2cseq_ops_t {
	__u16	opcode;		//operation code
				//bit[15 14]:	address size
				//		0-byte
				//		1-word
				//
				//bit[13 12]:	reg size
				//		0-byte
				//		1-word
				//		2-double word
				//
				//bit[11 10]:	reserved
				//
				//bit[09-08]:	sub ID
				//
				//bit[07-00]:	operation ID

	__u16	addr;		//address
	__u32	value;		//reg value
} i2cseq_ops_t;

//==================Address size==================
#define I2CSEQ_ADDRSIZE_BIT_OFFSET	14
#define I2CSEQ_ADDRSIZE_BIT_MASK	0xC000
#define I2CSEQ_ADDRSIZE(op)		(op & I2CSEQ_ADDRSIZE_BIT_MASK)
#define I2CSEQ_ADDRSIZE_BYTE		(0 << I2CSEQ_ADDRSIZE_BIT_OFFSET)
#define I2CSEQ_ADDRSIZE_WORD		(1 << I2CSEQ_ADDRSIZE_BIT_OFFSET)
#define I2CSEQ_SETADDRSIZE(op, size)	((op) & (~I2CSEQ_ADDRSIZE_BIT_MASK) | size)
#define I2CSEQ_ADDRSIZE_BYTES(op)	(1<<(I2CSEQ_ADDRSIZE(op)>> I2CSEQ_ADDRSIZE_BIT_OFFSET))

//==================Value size==================
#define I2CSEQ_VALUESIZE_BIT_OFFSET	12
#define I2CSEQ_VALUESIZE_BIT_MASK	0x3000
#define I2CSEQ_VALUESIZE(op)		(op & I2CSEQ_VALUESIZE_BIT_MASK)
#define I2CSEQ_VALUESIZE_BYTE		(0 << I2CSEQ_VALUESIZE_BIT_OFFSET)
#define I2CSEQ_VALUESIZE_WORD		(1 << I2CSEQ_VALUESIZE_BIT_OFFSET)
#define I2CSEQ_VALUESIZE_DWORD		(2 << I2CSEQ_VALUESIZE_BIT_OFFSET)
#define I2CSEQ_SETVALUESIZE(op, size)	((op) & (~I2CSEQ_VALUESIZE_BIT_MASK) | size)
#define I2CSEQ_VALUESIZE_BYTES(op)	(1<<(I2CSEQ_VALUESIZE(op)>>I2CSEQ_VALUESIZE_BIT_OFFSET))

//==================Operation sub ID==================
#define I2CSEQ_SUBOPCODE_BIT_OFFSET	8
#define I2CSEQ_SUBOPCODE_BIT_MASK	0x300
#define I2CSEQ_SUBOPCODE(op)		(op & I2CSEQ_SUBOPCODE_BIT_MASK)
#define I2CSEQ_SUBOPCODE_RIGHTSHIFT(op)	(I2CSEQ_SUBOPCODE(op)>>I2CSEQ_SUBOPCODE_BIT_OFFSET)
#define I2CSEQ_SUBOPCODE_FORM(subid)	(subid << I2CSEQ_SUBOPCODE_BIT_OFFSET)
#define I2CSEQ_SETSUBOPCODE(op,subcode)	((op) & (~I2CSEQ_SUBOPCODE_BIT_MASK) | (subcode))

//==================Operation ID==================
#define I2CSEQ_OPCODE_BIT_OFFSET	0
#define I2CSEQ_OPCODE_BIT_MASK		0xff
#define I2CSEQ_OPCODE(op)		(op & I2CSEQ_OPCODE_BIT_MASK)
#define I2CSEQ_SETOPCODE(op, opcode)		((op) & (~I2CSEQ_OPCODE_BIT_MASK) | (opcode))

#define I2CSEQ_FULLOPCODE(op)		(I2CSEQ_OPCODE(op)|I2CSEQ_SUBOPCODE(op))

#define I2CSEQ_OPCODE_END	0		//end
	//Format:	{END, 0, 0}

#define I2CSEQ_OPCODE_WREG	1		//write reg
	//Format:	{WREG, addr, value}

#define I2CSEQ_OPCODE_WBITS	2		//write reg bits
#define I2CSEQ_SUBOPCODE_WBITS_V	I2CSEQ_SUBOPCODE_FORM(0)	//value
#define I2CSEQ_SUBOPCODE_WBITS_M	I2CSEQ_SUBOPCODE_FORM(1)	//mask - if the bits in mask is 1,
									//       then these bits is replaced by the same bits in value.
	//Format:	{WBITS|WBITS_V, addr, value}
	//		{WBITS|WBITS_M, 0,    mask}

#define I2CSEQ_OPCODE_PBITS	3		//pull bits
#define I2CSEQ_SUBOPCODE_PBITS_V	I2CSEQ_SUBOPCODE_FORM(0)	//value
#define I2CSEQ_SUBOPCODE_PBITS_DM	I2CSEQ_SUBOPCODE_FORM(1)	//delay(ms) and mask
									//mask - if the bits in mask is 1,
									//       then these bits is replaced by the same bits in value.
	//Format:	{PBITS|PBITS_V, addr, value}
	//		{PBITS|PBITS_DM, delay, mask}			//delay is not limited by ASize

#define I2CSEQ_OPCODE_BURST	4		//burst
#define I2CSEQ_SUBOPCODE_BURST_S	I2CSEQ_SUBOPCODE_FORM(0)	//address and length
#define I2CSEQ_SUBOPCODE_BURST_V	I2CSEQ_SUBOPCODE_FORM(1)	//values
#define I2CSEQ_SUBOPCODE_BURST_E	I2CSEQ_SUBOPCODE_FORM(2)	//End
	//Format:	{BURST|BURST_S, addr, length}	//seem to be useless
	//		{BURST|BURST_V, 0, value}
	//		.......
	//		{BURST|BURST_V, 0, value}
	//		{BURST|BURST_E, 0, 0}

#define I2CSEQ_OPCODE_DELAY	5		//delay(ms)
	//Format:	{DELAY, 0, delay}


//===============================================================
//package define
//===============================================================
#define I2CSEQ_PKG_NAME_LEN		32
typedef struct _i2cseq_pkg_head_t {
	__u8		name[I2CSEQ_PKG_NAME_LEN];
	__u8		type;			//type:	  0 - sequence ops
						//	  1 - TODO binary data, like FW.
						//	others - reserved
	__u8		reserved[3];
	__u32		data_len;		//length: the length of data
	__u32		checksum;		//checksum
	//__u8		data[]
} i2cseq_pkg_head_t;

//==================package type==================
#define I2CSEQ_PACKAGE_TYPE_OPS		0
#define I2CSEQ_PACKAGE_TYPE_BINARY	1

typedef struct _i2cseq_pkg_ops_t {
	__u8		name[32];
	__u8		type;			//type:	  0 - sequence ops
						//	  1 - binary data, like FW.
						//	others - reserved
	__u8		reserved[3];
	__u32		data_len;		//length: the length of data
	__u32		checksum;		//checksum
	i2cseq_ops_t	ops[1];			//operations
} i2cseq_pkg_ops_t;

typedef struct _i2cseq_pkg_binary_t {
	__u8		name[32];
	__u8		type;			//type:	  0 - sequence ops
						//	  1 - binary data, like FW.
						//	others - reserved
	__u8		reserved[3];
	__u32		data_len;		//length: the length of data
	__u32		checksum;		//checksum
	__u8		binary[1];		//binary data
} i2cseq_pkg_binary_t;

//===============================================================
//file header define
//===============================================================
typedef struct _i2cseq_file_header_t {
	__u32		signature;		//'2SEQ'
	__u32		version;		//version
	__u32		fw_version;		//firmware version
	__u16		addr_size;		//I2CSEQ_ADDRSIZE_BYTE/I2CSEQ_ADDRSIZE_WORD
	__u16		reserved_align;
	__u32		pkg_num;		//packages number
	__u32		checksum;		//header checksum, including the mapping nodes
	//i2cseq_pkg_mapping_node_t mapping[];
} i2cseq_file_header_t;

#define I2CSEQ_FILE_SIGNATURE	0x32534551	//'2SEQ'
#define I2CSEQ_FILE_VERSION(m, s) ((((__u32)m) << 16) | (((__u32)s)&0xffff))

typedef struct _i2cseq_pkg_mapping_node {
	__u8		name[I2CSEQ_PKG_NAME_LEN];
	__u32		f_offset;		//file offset
} i2cseq_pkg_mapping_node_t;


#pragma pack()

#endif	/* __I2CSEQ_TYPE_H__ */
