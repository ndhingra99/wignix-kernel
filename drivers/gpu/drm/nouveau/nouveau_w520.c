// SPDX-License-Identifier: MIT
/*
 * ThinkPad W520: performance level status for the Quadro 2000M.
 *
 * The driver runs the engine clocks at the highest level on mains power
 * and at the level below it on battery; memory stays at the highest
 * level's clock.  A read-only "pstate" attribute on the GPU's PCI device lists
 * the levels in the format of the debugfs file of the same name; the last
 * line is read back from the clock registers.
 *
 *   cat /sys/bus/pci/devices/0000:01:00.0/pstate
 */
#include "nouveau_drv.h"
#include "nouveau_w520.h"

#include <core/w520.h>
#include <nvif/class.h>
#include <nvif/if0001.h>

#include <linux/pm_runtime.h>

struct nouveau_w520 {
	struct nvif_object ctrl;
};

static ssize_t
pstate_show(struct device *d, struct device_attribute *a, char *buf)
{
	struct nouveau_drm *drm = dev_get_drvdata(d);
	struct drm_device *dev = drm->dev;
	struct nvif_object *ctrl = &drm->w520->ctrl;
	struct nvif_control_pstate_info_v0 info = {};
	ssize_t len = 0;
	int ret, i;

	ret = pm_runtime_get_sync(dev->dev);
	if (ret < 0 && ret != -EACCES) {
		pm_runtime_put_autosuspend(dev->dev);
		return ret;
	}

	ret = nvif_mthd(ctrl, NVIF_CONTROL_PSTATE_INFO, &info, sizeof(info));
	if (ret)
		goto done;

	for (i = 0; i < info.count + 1; i++) {
		const s32 state = i < info.count ? i :
			NVIF_CONTROL_PSTATE_ATTR_V0_STATE_CURRENT;
		struct nvif_control_pstate_attr_v0 attr = {
			.state = state,
			.index = 0,
		};

		ret = nvif_mthd(ctrl, NVIF_CONTROL_PSTATE_ATTR,
				&attr, sizeof(attr));
		if (ret)
			goto done;

		if (i < info.count)
			len += sysfs_emit_at(buf, len, "%02x:", attr.state);
		else
			len += sysfs_emit_at(buf, len, "%s:",
					     info.pwrsrc == 0 ? "DC" :
					     info.pwrsrc == 1 ? "AC" : "--");

		attr.index = 0;
		do {
			attr.state = state;
			ret = nvif_mthd(ctrl, NVIF_CONTROL_PSTATE_ATTR,
					&attr, sizeof(attr));
			if (ret)
				goto done;

			len += sysfs_emit_at(buf, len, " %s %d",
					     attr.name, attr.min);
			if (attr.min != attr.max)
				len += sysfs_emit_at(buf, len, "-%d", attr.max);
			len += sysfs_emit_at(buf, len, " %s", attr.unit);
		} while (attr.index);

		if (state >= 0) {
			if (info.ustate_ac == state)
				len += sysfs_emit_at(buf, len, " AC");
			if (info.ustate_dc == state)
				len += sysfs_emit_at(buf, len, " DC");
			if (info.pstate == state)
				len += sysfs_emit_at(buf, len, " *");
		} else {
			if (info.ustate_ac < -1)
				len += sysfs_emit_at(buf, len, " AC");
			if (info.ustate_dc < -1)
				len += sysfs_emit_at(buf, len, " DC");
		}

		len += sysfs_emit_at(buf, len, "\n");
	}

done:
	pm_runtime_put_autosuspend(dev->dev);
	return ret ? ret : len;
}

#ifdef W520_PSTATE_RW
/* Development builds only: request a level by hand, e.g. "0f" or "ac:03". */
static ssize_t
pstate_store(struct device *d, struct device_attribute *a,
	     const char *ubuf, size_t len)
{
	struct nouveau_drm *drm = dev_get_drvdata(d);
	struct drm_device *dev = drm->dev;
	struct nvif_control_pstate_user_v0 args = { .pwrsrc = -EINVAL };
	char buf[32] = {}, *tmp, *cur = buf;
	long value;
	int ret;

	if (len >= sizeof(buf))
		return -EINVAL;

	memcpy(buf, ubuf, len);
	if ((tmp = strchr(buf, '\n')))
		*tmp = '\0';

	if (!strncasecmp(cur, "dc:", 3)) {
		args.pwrsrc = 0;
		cur += 3;
	} else
	if (!strncasecmp(cur, "ac:", 3)) {
		args.pwrsrc = 1;
		cur += 3;
	}

	if (!strcasecmp(cur, "none")) {
		args.ustate = NVIF_CONTROL_PSTATE_USER_V0_STATE_UNKNOWN;
	} else {
		ret = kstrtol(cur, 16, &value);
		if (ret)
			return ret;
		args.ustate = value;
	}

	ret = pm_runtime_get_sync(dev->dev);
	if (ret < 0 && ret != -EACCES) {
		pm_runtime_put_autosuspend(dev->dev);
		return ret;
	}

	ret = nvif_mthd(&drm->w520->ctrl, NVIF_CONTROL_PSTATE_USER,
			&args, sizeof(args));
	pm_runtime_put_autosuspend(dev->dev);
	if (ret < 0)
		return ret;

	return len;
}

static DEVICE_ATTR_RW(pstate);
#else
static DEVICE_ATTR_RO(pstate);
#endif

void
nouveau_w520_init(struct nouveau_drm *drm)
{
	struct nouveau_w520 *w520;

	if (!nvkm_w520(nvxx_device(drm)))
		return;

	w520 = kzalloc_obj(*w520);
	if (!w520)
		return;

	if (nvif_object_ctor(&drm->client.device.object, "w520Ctrl", 0,
			     NVIF_CLASS_CONTROL, NULL, 0, &w520->ctrl)) {
		kfree(w520);
		return;
	}

	drm->w520 = w520;
	if (device_create_file(drm->dev->dev, &dev_attr_pstate)) {
		nvif_object_dtor(&w520->ctrl);
		drm->w520 = NULL;
		kfree(w520);
	}
}

void
nouveau_w520_fini(struct nouveau_drm *drm)
{
	if (!drm->w520)
		return;

	device_remove_file(drm->dev->dev, &dev_attr_pstate);
	nvif_object_dtor(&drm->w520->ctrl);
	kfree(drm->w520);
	drm->w520 = NULL;
}
