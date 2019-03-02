#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>

// OEM
extern char fih_skuid[8];

static int version_proc_show(struct seq_file *m, void *v)
{
	int length = 0, offset = 0;
	char old_linux_version[256] = {0}, new_linux_version[256] = {0};
	char *timezone = NULL, *at = NULL, *user = NULL, *machine = NULL, *left_parentheses = NULL, *right_parentheses = NULL;	

	printk("%s: SW version(skuid) = %s\n", __func__, fih_skuid);
	if(strncmp(fih_skuid, "600ID", 5) != 0)
	{
		seq_printf(m, linux_proc_banner,
			utsname()->sysname,
			utsname()->release,
			utsname()->version);
		return 0;
	}
	else
	{
		snprintf(old_linux_version, 256, linux_proc_banner, utsname()->sysname, utsname()->release, utsname()->version);
		/*
		Linux version 3.4.0-g8bf076e (fihtdc@fihtdc-sw5) (gcc version 4.9.x-google 20140827 (prerelease) (GCC) ) 
		#11 SMP PREEMPT Tue Aug 30 15:00:42 CST 2016
		*/

		// copy before @
		left_parentheses = strchr(old_linux_version,'(');
		right_parentheses = strchr(old_linux_version,')');

		at = strchr(old_linux_version,'@');
		machine = at + 1;
		user = left_parentheses + 1;
		length = left_parentheses-old_linux_version+1;
		offset = length;

		strncpy(new_linux_version, old_linux_version, length);

		// set user@machine as evercoss@android
		strncpy(&new_linux_version[offset], "Tsm-0@tsm-Server0", 17);
		offset += 17;

		// copy ) and others
		length = strlen(old_linux_version) - strlen(right_parentheses)+1;
		snprintf(new_linux_version, 256, "%s%s", new_linux_version, right_parentheses);
		//strncpy(&new_linux_version[offset], right_parentheses, length);

		timezone = strstr(new_linux_version, "CST");

		if(timezone != NULL)
		{
			memcpy(timezone, "WIB", 3);
		}

		seq_printf(m, new_linux_version);

		return 0;
	}
}

static int version_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, version_proc_show, NULL);
}

static const struct file_operations version_proc_fops = {
	.open		= version_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_version_init(void)
{
	proc_create("version", 0, NULL, &version_proc_fops);
	return 0;
}
fs_initcall(proc_version_init);
