/**+===========================================================================
  File: proc_info.h
============================================================================+*/
//#define fver_BLOCK 				"/dev/block/platform/mtk-msdc.0/11230000.msdc0/by-name/sys_info"
#define fver_BLOCK                              "/dev/block/platform/bootdevice/by-name/systeminfo"
#define wifimac_BLOCK                              "/vendor/nvdata/APCFG/APRDEB/WIFI"
//#define wifimac_BLOCK                              "/data/misc/wifi_mac"
#define btmac_BLOCK                                "/vendor/nvdata/APCFG/APRDEB/BT_Addr"

#define STATUSROOT_LOCATION 	"/BBSYS/status.cfg"
//#define FIH_HWID_INFO 		"/system/etc/Fih_Hwid_Info"
#define FIH_HWID_INFO 			"/vendor/etc/Fih_Hwid_Info"
#define FIH_HWID_TAG_SIZE 16
#define FIH_HWID_TAG 			"FIH_HWID_INFO"

#define MODULE_PROC 			"MODULE"
#define FVER_PROC 				"fver"
#define MODEL_PROC 				"devmodel"
#define HWMODEL_PROC 			"hwmodel"
#define DRAM_PROC 				"draminfo"
#define EMMC_PROC 				"emmcinfo"
#define CPU_PROC 				"cpu_info"
#define LCM_PROC 				"LCM"
#define POWERONCAUSE_PROC 		"poweroncause"
#define BASEBAND_PROC 			"baseband"
#define BASEBAND_SETTINGS_PROC 	"baseband_settings"
#define PCBA_DESCRIPTION_PROC 	"pcba_description"
#define HWIDV_PROC 				"hwidv"
#define SIM1_proc 				"simstate1"
#define SIM2_proc 				"simstate2"
#define IMEI_PROC 				"imei"
#define IMEI2_PROC 				"imei2"
#define CAVIS_D 				"DeviceRegistered"
#define CAVIS_R 				"RetryTime"
#define PRODUCTID_PROC 			"productid"
#define TOUCH_INFO   			"Touch"
#define AUDIO_PROC 				"fih_audio_ver"       
#define PROC_STATUSROOT 		"statusroot"
#define UICOLOR_PROC 			"custom_sw_color" 
#define HWINFO_PROC  			"hwid_info"	
#define LCM_CABC  				"CABC_settings"
#define SIM_CARD_SLOT_PROC		"sim_card_slot"
#define BANDINFO				"bandinfo"
#define EFUSE_INFO_PROC			"efuse_state"
#define OTG_LAST_FLAG			"otg_last_flag"				
#define SIM_NUMBER				"simnum"
#define FQCXMLPATH				"fqc_xml"
#define RAMRESULT_PROC                  "dramtest_result"   //sunjie +
/* PDA: for lcm runin { */
#define LCM0_AWER_CNT			"awer_cnt"
#define LCM0_AWER_STATUS		"awer_status"
/* PDA: for lcm runin } */
#define LCM0_FS_CURR			"fs_curr"
#define WIFI_MAC		"wifi_mac"
#define BT_MAC					"bt_mac"

int otg_last_flag = 0;

struct systeminfo {
	char	model_name[16];
	char	lcm_info[20];
	char	dram[20];
	char	emmc[20];
	char	cpu_name[16];
	char	phase_settings[16];
	char    phase[16];
	char	module[128];
	char    bandinfo[128];
	char    hw_family[16];
	char    pcba_description[112];
	char	hac[8];
	char	sim_num[8];
	char    audio_para[30];
};

struct fih_hwid_info {
	int	    project_id;
	int	    phase_id;
	int	    module_id;
	char	project_name[16];
	char	phase_sw[16];
	char	phase_hw[16];
	char	cpu_name[16];
	char	module[128];
	char 	bandinfo[128];
	char	hw_family[16];
	char    pcba_description[112];
	char	hac[8];
	char	sim_num[8];
};


struct systeminfo model[] = {
/*************************unknow**************************/
		{
			.model_name 	=		"unknow",
			.lcm_info		=		"unknow",
			.dram			=		"unknow",
			.emmc			=		"unknow",
			.cpu_name		=		"unknow",
			.phase_settings =		"unknow",
			.phase			=		"unknow",
			.module 		=		"unknow",
			.bandinfo		=		"unknow",
			.hw_family		=		"unknow",
			.hac			=		"unknow",
			.sim_num		=		"unknow",
			.audio_para 	=		"unknow"
		},
		{
			.model_name     =       "unknow",
			.lcm_info       =       "unknow",
			.dram           =       "unknow",
			.emmc           =       "unknow",
			.cpu_name       =       "unknow",
			.phase_settings =       "unknow",
			.phase          =       "unknow",
			.module         =       "unknow",
			.bandinfo       =       "unknow",
			.hw_family      =       "unknow",
			.hac			=		"unknow",
			.sim_num		=		"unknow",
			.audio_para     =       "unknow"
		},
		{
			.model_name 	=		"unknow",
			.lcm_info		=		"unknow",
			.dram			=		"unknow",
			.emmc			=		"unknow",
			.cpu_name		=		"unknow",
			.phase_settings =		"unknow",
			.phase			=		"unknow",
			.module 		=		"unknow",
			.bandinfo		=		"unknow",
			.hw_family		=		"unknow",
			.hac			=		"unknow",
			.sim_num		=		"unknow",
			.audio_para 	=		"unknow"
		},
		{
			.model_name 	=		"unknow",
			.lcm_info		=		"unknow",
			.dram			=		"unknow",
			.emmc			=		"unknow",
			.cpu_name		=		"unknow",
			.phase_settings =		"unknow",
			.phase			=		"unknow",
			.module 		=		"unknow",
			.bandinfo		=		"unknow",
			.hw_family		=		"unknow",
			.hac			=		"unknow",
			.sim_num		=		"unknow",
			.audio_para 	=		"unknow"
		},
		{
			.model_name 	=		"unknow",
			.lcm_info		=		"unknow",
			.dram			=		"unknow",
			.emmc			=		"unknow",
			.cpu_name		=		"unknow",
			.phase_settings =		"unknow",
			.phase			=		"unknow",
			.module 		=		"unknow",
			.bandinfo		=		"unknow",
			.hw_family		=		"unknow",
			.hac			=		"unknow",
			.sim_num		=		"unknow",
			.audio_para 	=		"unknow"
		},
		{
			.model_name 	=		"unknow",
			.lcm_info		=		"unknow",
			.dram			=		"unknow",
			.emmc			=		"unknow",
			.cpu_name		=		"unknow",
			.phase_settings =		"unknow",
			.phase			=		"unknow",
			.module 		=		"unknow",
			.bandinfo		=		"unknow",
			.hw_family		=		"unknow",
			.hac			=		"unknow",
			.sim_num		=		"unknow",
			.audio_para 	=		"unknow"
		},
};
