/* SPDX-License-Identifier: MIT */
#ifndef __NVKM_W520_H__
#define __NVKM_W520_H__
#include <core/device.h>
#include <core/option.h>

#include <linux/dmi.h>

/* ThinkPad W520 with the Quadro 2000M (GF106GLM, NVC3, DDR3).
 *
 * Performance level switching is implemented for this board only; every
 * other Fermi keeps the stock behaviour.  nouveau.config=NvW520Reclock=0
 * turns it off.
 */
static inline bool
nvkm_w520(struct nvkm_device *device)
{
	const char *version = dmi_get_system_info(DMI_PRODUCT_VERSION);

	return device->chipset == 0xc3 &&
	       version && strstr(version, "ThinkPad W520") &&
	       nvkm_boolopt(device->cfgopt, "NvW520Reclock", true);
}
#endif
