/* Foxconn added , BokeeLi, 2011/10/06 */
/* Note:
   (1) Refer to Mulberry project
   (2) Reserve a partition for store Manufacture data

*/
#ifndef _TESTFLAG_H /*_TESTFLAG_H */
#define _TESTFLAG_H


#define MANUFACTURE_SIZE (1024*1024)
#define MANUFACTURE_OFFSET (0x100000)
#define MANUFACTURE_CALIBRATION_FILE_OFFSET (0x1000)  // 4096 bytes

#define MANUF_FILE_LOCATION "/dev/block/platform/mtk-msdc.0/11230000.msdc0/by-name/proinfo"

#define MANUF_MAX_DATA_LEN 64

#define MANUF_MAGIC_LEN 8
#define MANUF_SYNC_INFO_LEN 8
#define MANUF_VERSION_LEN 8
#define MANUF_NAME_LEN 32
#define MANUF_DATA_LEN 4
#define MANUF_HARDWAREID_LEN 16
#define MANUF_PRODUCTID_LEN 32
#define MANUF_PRODUCTID2_LEN 32
#define MANUF_BT_ADDR_LEN 12
#define MANUF_UICOLOR_LEN 10
#define MANUF_MAC_ADDR_LEN 17
#define MANUF_SECRET_NUM_LEN 32
#define MANUF_BOOTFLAG_LEN 16
#define MANUF_DOWNLOADFLAG_LEN 16
#define MANUF_ONETIMEFTMFLAG_LEN 16
#define MANUF_RESERVED_LEN 32
#define MANUF_CALIBRATION_FILENAME_LEN 128
#define MANUF_CALIBRATION_DATA_SIZE 4*1024 //4K
#define MANUF_SECINFO_LEN 16
#define FIH_CERT_LENGTH   (256)

#define MANUF_SKUID_LEN 16 
#define MULTICDA_ID_OFFSET (4*1024)  //4KB
#define MANUF_SKUID_CHANGE_FLAG "SKUchange"
#define INVALID_PTN               -1

#define MANUF_MAGIC_STRING "MANUDONE"
#define MANUF_SYNC_INFO_STRING "MANUSYNC"
#define MANUF_VERSION_STRING "VERSION"
#define MANUF_HARDWAREID_STRING "HARDWAREID"
#define MANUF_PRODUCTID_STRING "PRODUCTID"
#define MANUF_PRODUCTID2_STRING "PRODUCTID2"
#define MANUF_BT_ADDR_STRING "BTADDR"
#define MANUF_MAC_ADDR_STRING "MACADDR"
#define MANUF_SECRET_NUM_STRING "SECRET"
#define MANUF_BOOTFLAG_STRING "BOOTFLAG"
#define MANUF_DOWNLOADFLAG_STRING "DOWNLOADFLAG"
#define MANUF_ONETIMEFTMFLAG_STRING "ONETIMEFTMFLAG"
#define MANUF_SKUFLAG_STRING "SKUFLAG"
#define MANUF_CAVISFLAG_STRING "CAVISFLAG"
#define MANUF_ROOTFLAG_STRING "ROOTFLAG"
#define MANUF_PSENSORFLAG_STRING "PSENSORFLAG"
#define MANUF_ALSFLAG_STRING "ALSFLAG"
#define MANUF_TPDATAFLAG_STRING "TPDATAFLAG"
#define MANUF_UICOLOR_STRING "UICOLOR"
#define MANUF_SECCHECK_STRING "SECCHECK"
#define MANUF_SECINFO_STRING "SECURITYINFO"
#define MANUF_CALIBRATION_DATA_STRING "CAL_DATA"
#define MANUF_GSENSOR_STRING "GSENOR_CALI"
#define MANUF_GYRO_STRING	"GYRO_CALI"
#define MANUF_ALS_STRING	"ALS_CALI"
#define MANUF_SIM_CARD_SLOT_STRING	"SIM_CARD_SLOT"

/* status */
#define SUCCESS         0x00
#define FAIL            0x01


#define INVALID_MODE        0x55
#define FILE_NOT_FOUND      0x03
#define FILE_CORRUPTED      0x04


/* return for PID read/write */
#define READ_MANUFACTURE_FAIL  (-1)
#define WRITE_MANUFACTURE_FAIL  (-2)


/* Note : if you add a new structure, please add reserved data in this feature for extension */
struct manuf_version
{
	char name[MANUF_NAME_LEN];
	unsigned int length;
	char version[MANUF_VERSION_LEN];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_hardware_id
{
	char name[MANUF_NAME_LEN];
	unsigned int length;
	char hardwareid[MANUF_HARDWAREID_LEN];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_product_id
{
	char name[MANUF_NAME_LEN];
	unsigned int length;
	char productid[MANUF_PRODUCTID_LEN];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_product_id_2
{
	char name[MANUF_NAME_LEN];
	unsigned int length;
	char productid2[MANUF_PRODUCTID2_LEN];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_bluetooth_bt_addr
{
	char name[MANUF_NAME_LEN];
	unsigned int length;
	char btaddr[MANUF_BT_ADDR_LEN];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_wifi_mac_addr
{
	char name[MANUF_NAME_LEN];
	unsigned int length;
	char macaddr[MANUF_MAC_ADDR_LEN];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_wifi_secret_num
{
	char name[MANUF_NAME_LEN];
	unsigned int length;
	char secret[MANUF_SECRET_NUM_LEN];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_boot_flag
{
	char name[MANUF_NAME_LEN];
	unsigned int length;
	char bootflag[MANUF_BOOTFLAG_LEN];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_download_flag
{
	char name[MANUF_NAME_LEN];
	unsigned int length;
	char downloadflag[MANUF_DOWNLOADFLAG_LEN];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_onetimeftm_flag
{
	char name[MANUF_NAME_LEN];
	unsigned int length;
	char onetimeftmflag[MANUF_ONETIMEFTMFLAG_LEN];
	char reserved[MANUF_RESERVED_LEN];
};

// porting for SKUID&Multi-Splash
struct manuf_SKU_flag
{
	char name[MANUF_NAME_LEN];
	char change_flag[MANUF_SKUID_LEN];
	char SKUID[60];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_CAVIS
{
	char Register[8];
	char Retry[8];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_PSENSOR
{
	// bit 0: match number
	// bit 1: status
	// bit 2: high threshold
	// bit 3: low threshold
	// bit 4: max crosstalk
	// bit 5: avg crosstalk
	// bit 6: min crosstalk
	u8 data[7];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_als
{
	// bit 0: match number
	// bit 1: status
	// bit 2~15: slope
	u8 data[16];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_uicolor
{
	char name[MANUF_NAME_LEN];
	unsigned int length;
	char uicolor[MANUF_UICOLOR_LEN];
	char reserved[MANUF_RESERVED_LEN];
};

struct manuf_tp_rawdata_range_t
{
	int rawdata_screen_low;
	int rawdata_screen_high;
	int rawdata_key_low;
	int rawdata_key_high;
};

struct manuf_root
{
	unsigned int status;
	char reserved[MANUF_RESERVED_LEN];
};

/* add gsensor calibration for alex 20141205 begin*/
struct manuf_gsensor_cali
{
	int cali_x;
	int cali_y;
	int cali_z;
};
/* add gsensor calibration for alex 20141205 end*/

/* add for gyro calibration 20150912 by xurd begin*/
struct manuf_gyro_cali
{
	int cali_x;
	int cali_y;
	int cali_z;
};
struct manuf_als_cali
{
	int cali_data;
};

/* add for gyro calibration 20150912 by xurd end*/


/*20150715, for store security information begin*/
struct manuf_need_sec_chk
{
	char sec_bt_chk;
	char sec_dl_chk;
};

struct manuf_sec_info
{
	bool lk_flag;
	bool boot_flag;
	bool recovery_flag;
	char lk[MANUF_SECINFO_LEN+1];
	char boot[MANUF_SECINFO_LEN+1];
	char recovery[MANUF_SECINFO_LEN+1];
	unsigned int lk_size; 		// lk write data size
	unsigned int boot_size; 
	unsigned int recovery_size;
};
/*20150715, for store security information end*/

struct manuf_sim_info
{
	int sim_card_slot;
};

struct cert_timecount_st
{
    unsigned char en_uid_index;
    unsigned char rest_count;
    unsigned char en_uid[FIH_CERT_LENGTH];
};


struct manuf_data
{
	char magic[MANUF_MAGIC_LEN];
	char sync_info[MANUF_SYNC_INFO_LEN];
	struct manuf_version version;
	struct manuf_hardware_id hardwareid;
	struct manuf_product_id productid;
	struct manuf_product_id_2 productid2;
	struct manuf_bluetooth_bt_addr btaddr;
	struct manuf_wifi_mac_addr macaddr;
	struct manuf_wifi_secret_num secret;
	struct manuf_boot_flag bootflag;
	struct manuf_download_flag downloadflag;
	struct manuf_onetimeftm_flag onetimeftmflag;
	struct manuf_SKU_flag SKUflag;
	struct manuf_CAVIS CAVISflag;
	struct manuf_root rootflag;
	struct manuf_PSENSOR PSENSORflag;
	struct manuf_als als_flag;
    struct manuf_tp_rawdata_range_t tp_rawdata_range;
	struct manuf_uicolor uicolor;
	struct manuf_sim_info sim_info;
	struct manuf_gsensor_cali gcali;		// add gsensor calibration for alex 20141205 
	struct manuf_gyro_cali gycali;			// add for gyro calibration 20150912 by xurd
	struct manuf_als_cali alscali;			// add for als calibration 20151029 by xurd
	struct manuf_need_sec_chk is_need_chk;
	struct manuf_sec_info sec_info;
	struct cert_timecount_st cert_set;
};

struct manuf_file_data
{
    char magic[MANUF_MAGIC_LEN];
    unsigned int manuf_length;
    char filename[MANUF_CALIBRATION_FILENAME_LEN];
    char file_raw_data[MANUF_CALIBRATION_DATA_SIZE];
};

int fih_read_ps_thd(u8 data[7]);
int fih_write_ps_thd(u8 data[7]);
int fih_read_CAVIS(char* pid_str,int i);
int fih_write_CAVIS(char* pid_str,int i);
int fih_read_pid(char* pid_str);
int fih_write_pid(char* pid_str);

int fih_read_als_slope(u8 data[16]);
int fih_write_als_slope(u8 data[16]);

int fih_read_tp_rawdata_range(struct manuf_tp_rawdata_range_t *p);
int fih_write_tp_rawdata_range(struct manuf_tp_rawdata_range_t *p);
/* add gsensor calibration for alex 20141205 begin*/
int fih_read_gsensor_cali(struct manuf_gsensor_cali *p);  
int fih_write_gsensor_cali(struct manuf_gsensor_cali *p);
/* add gsensor calibration for alex 20141205 end*/

int fih_read_skuid(char* skuid);

#endif /* _TESTFLAG_H */
