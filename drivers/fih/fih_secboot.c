#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include "fih_secboot.h"
#include <linux/seq_file.h>

static inline bool is_tampered(void)
{
	static const char yellow_state[] = "androidboot.verifiedbootstate=yellow";
	static const char red_state[] = "androidboot.verifiedbootstate=red";

	return ( (strstr(saved_command_line, yellow_state) != NULL) || (strstr(saved_command_line, red_state) != NULL) );
}

static inline bool is_unlocked(void)
{
	static const char unlocked[] = "androidboot.verifiedbootstate=orange";

	return (strstr(saved_command_line, unlocked) != NULL);
}

static inline bool is_not_securityfused(void)
{
	static const char fused[] = "androidboot.securityfused=false";

	return (strstr(saved_command_line, fused) != NULL);
}

static int proc_read_enabled_state(struct seq_file *m, void *v)
{
	if (is_not_securityfused())
		seq_printf(m, "0\n"); /* Disabled */
	else {
		seq_printf(m, "1\n");
	}

	return 0;
}

static int secboot_enabled_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_read_enabled_state, NULL);
};

static struct file_operations secboot_enabled_file_ops = {
	.owner   = THIS_MODULE,
	.open    = secboot_enabled_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int proc_read_unlocked_state(struct seq_file *m, void *v)
{
	if (is_unlocked()) {
		seq_printf(m, "1\n");
	} else {
		seq_printf(m, "0\n");
	}

	return 0;
}

static int secboot_unlocked_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_read_unlocked_state, NULL);
};

static struct file_operations secboot_unlocked_file_ops = {
	.owner   = THIS_MODULE,
	.open    = secboot_unlocked_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int proc_read_tampered_state(struct seq_file *m, void *v)
{
	if (is_tampered()) {
		seq_printf(m, "1\n"); /* Tampered */
	} else {
		seq_printf(m, "0\n");
	}

	return 0;
}

static int secboot_tampered_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_read_tampered_state, NULL);
};

static struct file_operations secboot_tampered_file_ops = {
	.owner   = THIS_MODULE,
	.open    = secboot_tampered_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int proc_read_serialno_state(struct seq_file *m, void *v)
{
	unsigned int *addr = NULL;

	if (NULL == addr)
		seq_printf(m, "00000000\n");
	else {
		seq_printf(m, "%08X\n", *addr);
	}

	return 0;
}

static int secboot_serialno_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_read_serialno_state, NULL);
};

static struct file_operations secboot_serialno_file_ops = {
	.owner   = THIS_MODULE,
	.open    = secboot_serialno_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};


static struct {
		char *name;
		struct file_operations *ops;
} *p, fih_secboot[] = {
	{"secboot/enabled", &secboot_enabled_file_ops},
	{"secboot/unlocked", &secboot_unlocked_file_ops},
	{"secboot/tampered", &secboot_tampered_file_ops},
	{"secboot/serialno", &secboot_serialno_file_ops},
	{NULL}, };


static int __init fih_secboot_init(void)
{
	proc_mkdir("secboot", NULL);
	for (p = fih_secboot; p->name; p++) {
		if (proc_create(p->name, 0, NULL, p->ops) == NULL) {
			pr_err("%s: fail to create proc/%s\n", __func__, p->name);
		}
	}

	return (0);
}

static void __exit fih_secboot_exit(void)
{
	for (p = fih_secboot; p->name; p++) {
		remove_proc_entry(p->name, NULL);
	}
}

late_initcall(fih_secboot_init);
module_exit(fih_secboot_exit);
