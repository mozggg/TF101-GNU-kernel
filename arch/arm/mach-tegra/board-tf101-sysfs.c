#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include "board-tf101.h"

static struct kobject *tf101_kobj;

static ssize_t hw_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	s += sprintf(s, "%02x\n", tf101_hw);
	return s - buf;
}

static struct kobj_attribute hw_attr =
	__ATTR(hw, 0440, hw_show, NULL);

#define CHK_ERR(x) if (x) { \
	pr_err("%s: sysfs_create_file fail(%d)!", __func__, x); \
	return x; \
}

static int __init tf101_sysfs_init(void)
{
	tf101_kobj = kobject_create_and_add("tf101", firmware_kobj);
	if (!tf101_kobj) {
		pr_err("%s: tf101_kobj create fail\n", __func__);
		return -ENODEV;
	}

	CHK_ERR(sysfs_create_file(tf101_kobj, &hw_attr.attr));

	return 0;
}

static void __exit tf101_sysfs_exit(void)
{
	sysfs_remove_file(tf101_kobj, &hw_attr.attr);
	kobject_del(tf101_kobj);
}

late_initcall(tf101_sysfs_init);
module_exit(tf101_sysfs_exit);
