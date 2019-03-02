/* Foxconn added , BokeeLi, 201/10/06 */
/* Note:
   (1) Refer to Mulberry project
   (2) Reserve a partition for store Manufacture data
*/

#include <linux/string.h>
#include <linux/types.h>
#include <linux/stat.h>
#include <linux/fs.h>

#include <linux/unistd.h>
#include "Testflag.h"
#include <asm/uaccess.h>

#define FAILED -1;
#define OK 1;
/*J6000092 add for proinfo partition cache start*/
static struct manuf_data fih_proinfo_data;
static int read_flag = 0;
static int write_flag = 0;
static struct manuf_data manuf_data;
/*J6000092 add for proinfo partition cache end*/

int write_ef(struct manuf_data * wdata)
{
	int len = 0;

	struct file *pid_filp = NULL;
//	char temp[80];
	mm_segment_t oldfs;
//	int pid_len;
        loff_t pos = 0;

	printk("write_ef()\n");

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	pid_filp = filp_open(MANUF_FILE_LOCATION, O_RDWR, 0);//O_RDONLY

	if(!IS_ERR(pid_filp))
	{
		strncpy(wdata->sync_info, MANUF_SYNC_INFO_STRING, strlen(MANUF_SYNC_INFO_STRING));
		//len = pid_filp->f_op->write(pid_filp, (char *)&wdata->magic,sizeof(struct manuf_data), &pid_filp->f_pos);
		vfs_write(pid_filp, (char __user *)&wdata->magic, sizeof(struct manuf_data), &pos);

		// added by sinkin after write action
		vfs_fsync(pid_filp, 0);
		printk("write_ef() len = %d\n", len);

		filp_close(pid_filp, NULL);

		if (len != sizeof(struct manuf_data))
		{
			return FILE_CORRUPTED;
		}
	}
	else
	{
		printk("write_ef() open file fail %s\n", MANUF_FILE_LOCATION);
		return FILE_NOT_FOUND;
	}

	set_fs(oldfs);

	/*J6000092 add for proinfo partition cache start*/
	memcpy(&fih_proinfo_data, wdata, sizeof(struct manuf_data));
	write_flag = 1;
	/*J6000092 add for proinfo partition cache end*/

	return 0;
}


int read_ef(struct manuf_data * rdata)
{
	int len = 0;
	//int pid_len;
	//char temp[80];

	mm_segment_t oldfs;
	struct file *pid_filp = NULL;
        loff_t pos = 0;

	printk("read_ef() read_flag = %d, write_flag = %d\n", read_flag, write_flag);

	/*J6000092 modify for proinfo partition cache start*/
	if(read_flag == 0 || write_flag == 1)
	{
		oldfs = get_fs();
		set_fs(KERNEL_DS);

		pid_filp = filp_open(MANUF_FILE_LOCATION, O_RDONLY, 0);

		if(!IS_ERR(pid_filp))
		{
			//len = pid_filp->f_op->read(pid_filp, (char *)&rdata->magic, sizeof(struct manuf_data), &pid_filp->f_pos);
			vfs_read(pid_filp, (char __user *)&rdata->magic, sizeof(struct manuf_data), &pos);
			filp_close(pid_filp, NULL);

			if (len != sizeof(struct manuf_data))
			{
				return FILE_CORRUPTED;
			}
		}
		else
		{
			printk("read_ef() failed to open PTR_ERR(pid_filp) = %ld\n", PTR_ERR(pid_filp));

			return FILE_NOT_FOUND;
		}
		set_fs(oldfs);

		memcpy(&fih_proinfo_data, rdata, sizeof(struct manuf_data));
		read_flag = 1;
		write_flag = 0;
	}
	else
	{
		memcpy(rdata, &fih_proinfo_data, sizeof(struct manuf_data));        
	}
	/*J6000092 modify for proinfo partition cache end*/

	return 0;
}


int fih_read_pid(char* pid_str)
{
	//struct manuf_data manuf_data;
	int access;
	//int pid_len;

	printk("fih_read_pid\n");

	memset(&manuf_data, '\0', sizeof(struct manuf_data) );

	/* read manuf data from file system*/
	access = read_ef(&manuf_data);

	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("Read failed\n");
		return READ_MANUFACTURE_FAIL;
	}

	/* check PID name */
	if(strcmp(manuf_data.productid.name, MANUF_PRODUCTID_STRING) == 0 && manuf_data.productid.length <= MANUF_PRODUCTID_LEN)
	{
		printk("pid=%s, len=%d\n", manuf_data.productid.productid, manuf_data.productid.length);
		strncpy(pid_str, manuf_data.productid.productid, manuf_data.productid.length);
	}
	else
	{
		printk("PID tag (0x%s 0x%x)mismatch)\n", manuf_data.productid.name, manuf_data.productid.length);
		return FAILED;
	}

	//printk("pid_str=%s, len=%d\n", pid_str, strlen(pid_str));

	return OK;
}


int fih_write_pid(char* pid_str)
{
	//struct manuf_data manuf_data;
	int access;

	//int temp_len = 0;

	//printk("fih_write_pid with pid=%s, len=%d\n", pid_str, strlen(pid_str));

	memset(&manuf_data, '\0', sizeof(struct manuf_data));

	/* read manuf data from file system*/
	access = read_ef(&manuf_data);

	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("Write pid failed 1\n");
		return READ_MANUFACTURE_FAIL;
	}

	//copy testflag and write to flash
	/* clear product_id and copy written string */
	memset(manuf_data.productid.name, 0x0, MANUF_NAME_LEN);
	memset(manuf_data.productid.productid, 0x0, MANUF_PRODUCTID_LEN);
	strncpy(manuf_data.productid.name, MANUF_PRODUCTID_STRING, strlen(MANUF_PRODUCTID_STRING));
	strncpy(manuf_data.productid.productid, pid_str, strlen(pid_str));
	manuf_data.productid.length = strlen(pid_str);
	strncpy(manuf_data.sync_info, MANUF_SYNC_INFO_STRING, strlen(MANUF_SYNC_INFO_STRING));

	access = write_ef(&manuf_data);

	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("Write pid failed 2\n");
		return WRITE_MANUFACTURE_FAIL;
	}

	return OK;
}


int fih_read_CAVIS(char* pid_str, int i)
{
	//struct manuf_data manuf_data;
	int access;
	//int pid_len;

	printk("fih_read_CAVIS()\n");

	memset(&manuf_data, '\0', sizeof(struct manuf_data));

	/* read manuf data from file system*/
	access = read_ef(&manuf_data);

	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("Read CAVIS failed\n");
		return READ_MANUFACTURE_FAIL;
	}

	/* check PID name */
	if(i == 1)
	{
		if(strlen(manuf_data.CAVISflag.Register) <= sizeof(manuf_data.CAVISflag.Register))
		{
			//printk("pid=%s, len=%d\n", manuf_data.CAVISflag.Register, strlen(manuf_data.CAVISflag.Register));
			strncpy(pid_str, manuf_data.CAVISflag.Register, strlen(manuf_data.CAVISflag.Register));
		}
		else
		{
			printk("1 Error\n");
			return FAILED;
		}
	}
	else if(i == 2)
	{
		if(strlen(manuf_data.CAVISflag.Retry) <= sizeof(manuf_data.CAVISflag.Retry))
		{
			//printk("pid=%s, len=%d\n", manuf_data.CAVISflag.Retry, strlen(manuf_data.CAVISflag.Retry));
			strncpy(pid_str, manuf_data.CAVISflag.Retry, strlen(manuf_data.CAVISflag.Retry));
		}
		else
		{
			printk("2 Error\n");
			return FAILED;
		}
	}
	//printk("pid_str=%s, len=%d, num=%d\n", pid_str, strlen(pid_str),i);

	return OK;
}

int fih_write_CAVIS(char* pid_str, int i)
{
	//struct manuf_data manuf_data;
	int access;

	//int temp_len = 0;

	//printk("[DW]fih_write_CAVIS with cavis=%s, len=%d, num=%d\n", pid_str, strlen(pid_str),i);

	memset(&manuf_data, '\0', sizeof(struct manuf_data));

	/* read manuf data from file system*/
	access = read_ef(&manuf_data);

	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("Write failed1\n");
		return READ_MANUFACTURE_FAIL;
	}

	if(i == 1)
	{
		memset(manuf_data.CAVISflag.Register, 0x0, sizeof(char)*5);
		strncpy(manuf_data.CAVISflag.Register, pid_str, strlen(pid_str));
	}
	else if(i == 2)
	{
		memset(manuf_data.CAVISflag.Retry, 0x0, sizeof(char)*5);
		strncpy(manuf_data.CAVISflag.Retry, pid_str, strlen(pid_str));
	}
	access = write_ef(&manuf_data);

	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("Write failed2\n");
		return WRITE_MANUFACTURE_FAIL;
	}

	return OK;
};

int fih_read_ps_thd(u8 cali[7])
{
	//struct manuf_data manuf_data;
	int access, i, size;

	printk("[DW] %s\n", __func__);
	memset(&manuf_data, '\0', sizeof(struct manuf_data) );

	/* read manuf data from file system*/
	access = read_ef(&manuf_data);
	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s Read failed\n", __func__);
		return READ_MANUFACTURE_FAIL;
	}

	size = sizeof(manuf_data.PSENSORflag.data);
	memcpy(cali, manuf_data.PSENSORflag.data, size);

	for (i = 0; i < size; ++i)
	{
		printk("psensor manuf read data[%d] = %d\n", i, cali[i]);
	}

	return OK;
}

int fih_write_ps_thd(u8 cali[7])
{
	//struct manuf_data manuf_data;
	int access, i, size;

	printk("[DW] %s\n", __func__);
	memset(&manuf_data, '\0', sizeof(struct manuf_data));

	/* read manuf data from file system*/
	access = read_ef(&manuf_data);
	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s Write failed1\n", __func__);
		return READ_MANUFACTURE_FAIL;
	}

	size = sizeof(manuf_data.PSENSORflag.data);
	memcpy(manuf_data.PSENSORflag.data, cali, size);

	for (i = 0; i < size; ++i)
	{
		printk("psensor manuf write data[%d] = %d\n", i, manuf_data.PSENSORflag.data[i]);
	}

	access = write_ef(&manuf_data);

	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s Write failed2\n", __func__);
		return WRITE_MANUFACTURE_FAIL;
	}

	return OK;
};

int fih_read_als_slope(u8 cali[16])
{
	//struct manuf_data manuf_data;
	int access, i, size;

	printk("[DW] %s\n", __func__);
	memset(&manuf_data, '\0', sizeof(struct manuf_data));

	//read manuf data from file system
	access = read_ef(&manuf_data);
	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s Read failed\n", __func__);
		return READ_MANUFACTURE_FAIL;
	}

	size = sizeof(manuf_data.als_flag.data);
	memcpy(cali, manuf_data.als_flag.data, size);

	for (i = 0; i < size; ++i)
	{
		printk("als manuf read data[%d] = %d\n", i, cali[i]);
	}

	return OK;
}

int fih_write_als_slope(u8 cali[16])
{
	//struct manuf_data manuf_data;
	int access, i, size;

	printk("[DW] %s\n", __func__);
	memset(&manuf_data, '\0', sizeof(struct manuf_data)); //clean buffer

	// read manuf data from file system
	access = read_ef(&manuf_data);
	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s Write failed1\n", __func__);
		return READ_MANUFACTURE_FAIL;
	}

	size = sizeof(manuf_data.als_flag.data);
	memcpy(manuf_data.als_flag.data, cali, size);

	for (i = 0; i < size; ++i)
	{
		printk("als manuf write data[%d] = %d\n", i, manuf_data.als_flag.data[i]);
	}

	access = write_ef(&manuf_data);

	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s Write failed2\n", __func__);
		return WRITE_MANUFACTURE_FAIL;
	}

	return OK;
}

int fih_read_tp_rawdata_range(struct manuf_tp_rawdata_range_t *p)
{
	//struct manuf_data manuf_data;
	int access;//, size;

	printk("[Kernel] %s\n", __func__);
	memset(&manuf_data, '\0', sizeof(struct manuf_data));

	/* read manuf data from file system*/
	access = read_ef(&manuf_data);
	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s Read manuf_data failed\n", __func__);
		return READ_MANUFACTURE_FAIL;
	}
	memcpy(p, &(manuf_data.tp_rawdata_range), sizeof(struct manuf_tp_rawdata_range_t));

	printk("read touch panel rawdata range: \n");
	printk("rawdata_screen_low = %d\n", manuf_data.tp_rawdata_range.rawdata_screen_low);
	printk("rawdata_screen_high = %d\n", manuf_data.tp_rawdata_range.rawdata_screen_high);
	printk("rawdata_key_low = %d\n", manuf_data.tp_rawdata_range.rawdata_key_low);
	printk("rawdata_key_high = %d\n", manuf_data.tp_rawdata_range.rawdata_key_high);

	return OK;
}

int fih_write_tp_rawdata_range(struct manuf_tp_rawdata_range_t *p)
{
	//struct manuf_data manuf_data;
	int access;

	printk("[Kernel] %s\n", __func__);
	memset(&manuf_data, '\0', sizeof(struct manuf_data));

	/* read manuf data from file system*/
	access = read_ef(&manuf_data);
	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s 1.read manuf_data failed\n", __func__);
		return READ_MANUFACTURE_FAIL;
	}

	memcpy(&manuf_data.tp_rawdata_range, p, sizeof(struct manuf_tp_rawdata_range_t));

	printk("write touch panel rawdata range: \n");
	printk("rawdata_screen_low = %d\n", manuf_data.tp_rawdata_range.rawdata_screen_low);
	printk("rawdata_screen_high = %d\n", manuf_data.tp_rawdata_range.rawdata_screen_high);
	printk("rawdata_key_low = %d\n", manuf_data.tp_rawdata_range.rawdata_key_low);
	printk("rawdata_key_high = %d\n", manuf_data.tp_rawdata_range.rawdata_key_high);

	access = write_ef(&manuf_data);

	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s 2.write manuf_data failed\n", __func__);
		return WRITE_MANUFACTURE_FAIL;
	}

	return OK;
};
/* add gsensor calibration for alex 20141205 begin*/

int fih_read_gsensor_cali(struct manuf_gsensor_cali *p)
{
	int access;//, size;
	//struct manuf_data manuf_data;

	printk("[Kernel] %s\n", __func__);
	memset(&manuf_data, '\0', sizeof(struct manuf_data) );

	/* read manuf data from file system*/
	access = read_ef(&manuf_data);
	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s Read manuf_data failed\n", __func__);
		return READ_MANUFACTURE_FAIL;
	}
	memcpy(p, &(manuf_data.gcali), sizeof(struct manuf_gsensor_cali));

	printk("read gsensor calibration data \n");
	printk("calibration x = %d\n", manuf_data.gcali.cali_x);
	printk("calibration y = %d\n", manuf_data.gcali.cali_y);
	printk("calibration z = %d\n", manuf_data.gcali.cali_z);

	return OK;
}

int fih_write_gsensor_cali(struct manuf_gsensor_cali *p)
{
	//struct manuf_data manuf_data;
	int access;

	printk("[Kernel] %s\n", __func__);
	memset(&manuf_data, '\0', sizeof(struct manuf_data)); //clean buffer

	/* read manuf data from file system*/
	access = read_ef(&manuf_data);
	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s 1.read manuf_data failed\n", __func__);
		return READ_MANUFACTURE_FAIL;
	}

	memcpy(&manuf_data.gcali, p, sizeof(struct manuf_gsensor_cali));

	printk("read gsensor calibration data \n");
	printk("calibration x = %d\n", manuf_data.gcali.cali_x);
	printk("calibration y = %d\n", manuf_data.gcali.cali_y);
	printk("calibration z = %d\n", manuf_data.gcali.cali_z);

	access = write_ef(&manuf_data);

	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s 2.write manuf_data failed\n", __func__);
		return WRITE_MANUFACTURE_FAIL;
	}

	return OK;
};
/* add gsensor calibration for alex 20141205 end*/


int fih_read_skuid(char* skuid)
{
	int access;//, size;
	//struct manuf_data manuf_data;

	memset(&manuf_data, '\0', sizeof(struct manuf_data) );

	//read manuf data from file system
	access = read_ef(&manuf_data);
	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("%s Read manuf_data failed\n", __func__);
		return READ_MANUFACTURE_FAIL;
	}

	if (skuid != NULL)
	{
		strncpy(skuid, manuf_data.SKUflag.SKUID, 5);
	}

	printk("fih_read_skuid skuid = %s, SKUflag = %s\n", skuid, manuf_data.SKUflag.SKUID);

	return OK;
}

