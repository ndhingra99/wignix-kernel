// SPDX-License-Identifier: MIT
/*
 * Memory clock switching for the ThinkPad W520's Quadro 2000M (GF106GLM
 * with DDR3).
 *
 * The transition is built from the board's own VBIOS tables (memory map,
 * per-strap memory config and timing entries of every level) and from the
 * state the registers are in, and is run by the PMU's MEMX process while
 * framebuffer access is paused.  The sequence follows Ben Skeggs' DDR3
 * work for Fermi (nouveau devel-clk, 2017), which matches what the NVIDIA
 * driver uploads on a GF106 with DDR3, with the memory reference PLL
 * handled the way that driver does it.
 *
 * A VBIOS field that is zero in every level is never touched: the
 * registers behind it keep what the VBIOS init scripts put there.
 */
#include "ram.h"

#include <core/option.h>
#include <subdev/bios.h>
#include <subdev/bios/gpio.h>
#include <subdev/bios/pll.h>
#include <subdev/bios/ramcfg.h>
#include <subdev/bios/rammap.h>
#include <subdev/bios/timing.h>
#include <subdev/clk.h>
#include <subdev/clk/pll.h>
#include <subdev/gpio.h>
#include <subdev/pmu.h>

#include <linux/unaligned.h>

enum w520_ram_mode {
	W520_RAM_DIV,
	W520_RAM_PLL,
};

struct w520_ram_level {
	u32 min, max; /* MHz */
	u8  map[16];
	u8  cfg[16];
	struct nvbios_ramcfg bios;
};

struct w520_ram {
	struct nvkm_ram *ram;
	struct nvbios_pll refpll;
	struct nvbios_pll mempll;

	struct w520_ram_level level[8];
	int level_nr;
	u8  map_any[16];
	u8  cfg_any[16];
	bool any_RC;
	bool any_CWL;
	bool any_DLLoff;
	bool fbvddq_differs;

	/* transition being built */
	struct nvkm_memx *memx;
	const struct w520_ram_level *c;
	u32 freq;
	enum w520_ram_mode from;
	enum w520_ram_mode mode;
	bool again;
	u32 dsrc, dctl;
	u32 rsrc, rctl;
	u32 rpll, mpll;
	bool rpll_set;
	u32 r611200;
	bool stepped;

	int words;
	bool failed;
	int reg_nr;
	struct {
		u32 addr;
		u32 data;
	} reg[128];
};

/* MEMX keeps its script in a 0x800 byte buffer. */
#define W520_RAM_WORDS 480

static u32
w520_bits(u8 data, u8 mask)
{
	return (data & mask) >> __ffs(mask);
}

#define MAP(c,o,m) w520_bits((c)->map[o], (m))
#define CFG(c,o,m) w520_bits((c)->cfg[o], (m))
#define ANY_MAP(w,o,m) (((w)->map_any[o] & (m)) != 0)
#define ANY_CFG(w,o,m) (((w)->cfg_any[o] & (m)) != 0)

/******************************************************************************
 * Script building, with a shadow of every register the script touches so
 * later steps see the values earlier steps will have written.
 *****************************************************************************/
static u32
w520_rd32(struct w520_ram *w, u32 addr)
{
	struct nvkm_device *device = w->ram->fb->subdev.device;
	int i;

	for (i = 0; i < w->reg_nr; i++) {
		if (w->reg[i].addr == addr)
			return w->reg[i].data;
	}

	if (w->reg_nr == ARRAY_SIZE(w->reg)) {
		w->failed = true;
		return nvkm_rd32(device, addr);
	}

	w->reg[w->reg_nr].addr = addr;
	w->reg[w->reg_nr].data = nvkm_rd32(device, addr);
	return w->reg[w->reg_nr++].data;
}

static void
w520_wr32(struct w520_ram *w, u32 addr, u32 data)
{
	int i;

	w520_rd32(w, addr);
	for (i = 0; i < w->reg_nr; i++) {
		if (w->reg[i].addr == addr)
			w->reg[i].data = data;
	}

	nvkm_memx_wr32(w->memx, addr, data);
	w->words += 3;
}

/* Writes only when the value changes; returns the previous value. */
static u32
w520_mask(struct w520_ram *w, u32 addr, u32 mask, u32 data)
{
	u32 prev = w520_rd32(w, addr);
	u32 next = (prev & ~mask) | data;

	if (next != prev)
		w520_wr32(w, addr, next);
	return prev;
}

static void
w520_force(struct w520_ram *w, u32 addr, u32 mask, u32 data)
{
	w520_wr32(w, addr, (w520_rd32(w, addr) & ~mask) | data);
}

static void
w520_nsec(struct w520_ram *w, u32 nsec)
{
	nvkm_memx_nsec(w->memx, nsec);
	w->words += 2;
}

static void
w520_wait(struct w520_ram *w, u32 addr, u32 mask, u32 data, u32 nsec)
{
	nvkm_memx_wait(w->memx, addr, mask, data, nsec);
	w->words += 5;
}

/******************************************************************************
 * Pieces of the transition
 *****************************************************************************/
static void
w520_ram_timing(struct w520_ram *w)
{
	const struct nvbios_ramcfg *t = &w->c->bios;
	u32 mask, data;

	data = (t->timing_10_RP  & 0x7f) << 24 |
	       (t->timing_10_RAS & 0x7f) << 17 |
	       (t->timing_10_RFC & 0xff) << 8;
	mask = 0x7ffeff00;
	if (w->any_RC) {
		data |= t->timing_10_RC;
		mask |= 0x000000ff;
	}
	w520_mask(w, 0x10f290, mask, data);

	data = (t->timing_10_RCDWR & 0x3f) << 20 |
	       (t->timing_10_RCDRD & 0x3f) << 14;
	if (w->any_DLLoff && t->ramcfg_DLLoff)
		data |= (t->timing_10_CL & 0x7f) - 1;
	else
		data |= (t->timing_10_CL & 0x7f);
	mask = 0x03ffc07f;
	if (w->any_CWL) {
		data |= (t->timing_10_CWL & 0x7f) << 7;
		mask |= 0x00003f80;
	}
	w520_mask(w, 0x10f294, mask, data);

	w520_mask(w, 0x10f298, 0x007f7f00, (t->timing_10_WR  & 0x7f) << 16 |
					   (t->timing_10_WTR & 0x7f) << 8);
	w520_mask(w, 0x10f29c, 0x0001ffff, (t->timing_10_20 & 0xff) << 9 |
					   (t->timing_10_21 & 0x0f) << 5 |
					   (t->timing_10_13 & 0x1f));
	w520_mask(w, 0x10f2a0, 0x001f8000, (t->timing_10_RRD & 0x3f) << 15);
}

static void
w520_ram_r10f658(struct w520_ram *w)
{
	const struct w520_ram_level *c = w->c;

	w520_mask(w, 0x10f658, 0x00ffffff, CFG(c, 0x06, 0xf0) << 20 |
					   CFG(c, 0x06, 0x0f) << 16 |
					   CFG(c, 0x05, 0xf0) << 12 |
					   CFG(c, 0x05, 0x0f) << 8 |
					   CFG(c, 0x05, 0xf0) << 4 |
					   CFG(c, 0x05, 0x0f) << 0);
}

static void
w520_ram_r10f830(struct w520_ram *w, bool on)
{
	w520_mask(w, 0x10f830, 0x00000006, on ? 0x00000006 : 0x00000000);
}

static void
w520_ram_r137370(struct w520_ram *w, bool enable)
{
	w520_wr32(w, 0x137370, enable ? w->ram->part_mask : 0x00000000);
	w520_wr32(w, 0x137380, enable ? 0x00000001 : 0x00000000);
}

static void
w520_ram_r132018(struct w520_ram *w, u32 lowspeed, u32 map_04_01, bool mask14)
{
	u32 data = 0x00001000 | lowspeed << 28;
	u32 mask = 0x10009000;

	if (w->mode == W520_RAM_DIV)
		data |= 0x00008000;
	if (map_04_01 || mask14) {
		data |= 0x00004000 * map_04_01;
		mask |= 0x00004000;
	}
	w520_mask(w, 0x132018, mask, data);
}

static void
w520_ram_dll_reset(struct w520_ram *w)
{
	if (!(w520_rd32(w, 0x10f304) & 0x00000001)) {
		w520_force(w, 0x10f300, 0x00000100, 0x00000100);
		w520_nsec(w, 1000);
		w520_mask(w, 0x10f300, 0x00000100, 0x00000000);
		w520_nsec(w, 1000);
	}
}

/* Restart the memory reference PLL, then lock MPLL onto it. */
static void
w520_ram_plls(struct w520_ram *w)
{
	u32 ctrl = w520_rd32(w, 0x10fe20) & ~0x00000007;

	w520_wr32(w, 0x10fe20, ctrl);
	w520_wr32(w, 0x137320, w->rsrc);
	w520_wr32(w, 0x137330, w->rctl);
	if (w->rpll_set)
		w520_wr32(w, 0x10fe24, w->rpll);
	w520_wr32(w, 0x10fe20, ctrl | 0x00000001);
	w520_wait(w, 0x137390, 0x00020000, 0x00020000, 64000);
	w520_wr32(w, 0x10fe20, ctrl | 0x00000005);

	w520_wr32(w, 0x132004, w->mpll);
	w520_mask(w, 0x132000, 0x00000001, 0x00000001);
	w520_wait(w, 0x137390, 0x00000002, 0x00000002, 64000);
}

/* FBVDDQ select, GPIO function 0x18. */
static void
w520_ram_fbvddq(struct w520_ram *w, u32 state)
{
	static const u32 regs[] = { 0x00e104, 0x00e108, 0x00e120, 0x00e124 };
	struct nvkm_gpio *gpio = w->ram->fb->subdev.device->gpio;
	struct dcb_gpio_func func;
	u32 reg, sh, prev;

	if (nvkm_gpio_find(gpio, 0, 0x18, DCB_GPIO_UNUSED, &func))
		return;

	reg = regs[(func.line >> 3) & 3];
	sh = (func.line & 0x7) << 2;
	prev = w520_rd32(w, reg);
	if (prev & (8 << sh))
		state = !state;
	if (!(func.log[1] & 1))
		state = !state;

	w520_mask(w, reg, 0x3 << sh, (state | 0x2) << sh);
	w520_nsec(w, 64000);
}

static u32
w520_ram_fb_access(struct w520_ram *w, bool access, u32 r100b0c)
{
	if (!access) {
		r100b0c = w520_mask(w, 0x100b0c, 0x000000ff, r100b0c) & 0xff;
		nvkm_memx_wait_vblank(w->memx);
		w->r611200 = w520_rd32(w, 0x611200);
		w520_wr32(w, 0x611200, w->r611200 & ~0x00000030);
		w520_wait(w, 0x611200, 0x00000003, 0x00000000, 2000000);
		nvkm_memx_block(w->memx);
		w->words += 4;
	} else {
		nvkm_memx_unblock(w->memx);
		w->words += 1;
		w520_mask(w, 0x100b0c, 0x000000ff, r100b0c);
		w520_wr32(w, 0x611200, w->r611200);
	}

	return r100b0c;
}

static int
w520_ram_script(struct w520_ram *w)
{
	struct nvkm_ram *ram = w->ram;
	struct nvkm_device *device = ram->fb->subdev.device;
	const struct w520_ram_level *c = w->c;
	const u32 locknsec = DIV_ROUND_UP(540000, w->freq) * 1000;
	const u32 lowspeed = w->freq <= 750000;
	const bool slow = w->freq < 405000;
	const bool div_from = w->from == W520_RAM_DIV;
	const bool div_mode = w->mode == W520_RAM_DIV;
	const u32 map_04_01 = !ANY_MAP(w, 0x04, 0x01) || MAP(c, 0x04, 0x01);
	const bool any_02_10 = ANY_CFG(w, 0x02, 0x10);
	const u32 cfg_02_10 = CFG(c, 0x02, 0x10);
	u32 r10f200_11, r100b0c, mask, data, mr[3];
	int ret, part;

	/* Mode register values for the target level. */
	ram->mr[0] = w520_rd32(w, 0x10f300);
	ram->mr[1] = w520_rd32(w, 0x10f304);
	ram->mr[2] = w520_rd32(w, 0x10f320);
	ram->target.bios = c->bios;
	ram->target.freq = w->freq;
	ram->next = &ram->target;
	ret = nvkm_sddr3_calc(ram);
	if (ret)
		return ret;
	mr[0] = ram->mr[0] & ~0x00000100;
	mr[1] = ram->mr[1];
	mr[2] = ram->mr[2];

	/* Clock preparations that do not need framebuffer access paused. */
	if (!div_from) {
		w520_wr32(w, 0x137310, w->dctl);
		w520_wr32(w, 0x137300, w->dsrc);
	} else
	if (!div_mode) {
		w520_ram_plls(w);
		w520_ram_r132018(w, lowspeed, map_04_01, false);
	}

	if (!div_from || !div_mode)
		w520_ram_r137370(w, true);

	if (any_02_10) {
		if (!cfg_02_10) {
			w520_mask(w, 0x10f808, 0x40000000, 0x40000000);
			mask = 0x00000180;
		} else {
			mask = 0x00000600;
		}
		w520_mask(w, 0x10f824, mask, 0x00000000);
	}

	/* Wait for vblank and pause framebuffer access. */
	r100b0c = w520_ram_fb_access(w, false, 0x12);

	r10f200_11 = w520_mask(w, 0x10f200, 0x00000800, 0x00000000) & 0x00000800;
	if (ANY_MAP(w, 0x04, 0x02))
		r10f200_11 = MAP(c, 0x04, 0x02);

	if (!(w520_rd32(w, 0x10f304) & 0x001) && (mr[1] & 0x001)) {
		w520_wr32(w, 0x10f314, 0x00000001);
		w520_mask(w, 0x10f304, 0x00000001, 0x00000001);
		w520_nsec(w, 1000);
	}

	if (any_02_10 && !cfg_02_10) {
		w520_mask(w, 0x10f808, 0x04000000, 0x04000000);
		w520_mask(w, 0x1373ec, 0x00030000, 0x00020000);
	}

	/* Refresh must suit both ends of the transition. */
	data = w520_rd32(w, 0x10f290) & 0x0000ff00;
	data = max(data, (u32)c->bios.timing_10_RFC << 8);
	w520_mask(w, 0x10f290, 0x0000ff00, data);

	/* Precharge, enter self-refresh. */
	w520_wr32(w, 0x10f314, 0x00000001);
	w520_wr32(w, 0x10f210, 0x00000000);
	w520_wr32(w, 0x10f310, 0x00000001);
	w520_wr32(w, 0x10f310, 0x00000001);
	w520_nsec(w, 1000);
	w520_wr32(w, 0x10f090, 0x00000060);
	w520_wr32(w, 0x10f090, 0xc000007e);

	if (!div_from) {
		/* Onto the divider while MPLL is turned off. */
		w520_mask(w, 0x137360, 0x00000001, 0x00000001);
		if (!(w520_rd32(w, 0x10f830) & 0x00000006))
			w520_ram_r10f830(w, slow);
		w520_ram_r137370(w, false);
		w520_mask(w, 0x132018, 0x00004000, 0x00000000);
		w520_mask(w, 0x132000, 0x00000001, 0x00000000);
	}

	if (w->fbvddq_differs &&
	    CFG(c, 0x04, 0x08) == nvkm_gpio_get(device->gpio, 0, 0x18,
						DCB_GPIO_UNUSED))
		w520_ram_fbvddq(w, !CFG(c, 0x04, 0x08));

	if (!div_from && !div_mode)
		w520_ram_plls(w);

	if (div_from && div_mode)
		w520_wr32(w, 0x137310, w->dctl);

	w520_mask(w, 0x10f874, 0x04000000, lowspeed << 26);

	if (map_04_01)
		w520_mask(w, 0x10f824, 0x00006000, 0x00000000);
	else
		w520_ram_r10f658(w);

	w520_ram_r132018(w, lowspeed, map_04_01, true);
	if (map_04_01)
		w520_nsec(w, 20000);

	if (ANY_MAP(w, 0x04, 0x08)) {
		if (MAP(c, 0x04, 0x08)) {
			w520_force(w, 0x10f660, 0x0000ffff,
				   c->cfg[0x08] << 8 |
				   CFG(c, 0x07, 0xf0) << 4 |
				   CFG(c, 0x07, 0x0f));

			if (mask = 0, data = 0, ANY_CFG(w, 0x03, 0x0f)) {
				data |= CFG(c, 0x03, 0x0f) << 16;
				mask |= 0x000f0000;
			}
			if (ANY_CFG(w, 0x09, 0xf0)) {
				data |= CFG(c, 0x09, 0xf0) << 20;
				data |= CFG(c, 0x09, 0x0f) << 0;
				mask |= 0x00f0000f;
			}
			w520_mask(w, 0x10f668, mask, data);
		}

		data = MAP(c, 0x04, 0x08) ? 0x00000000 : 0x00002000;
		w520_mask(w, 0x10f910, 0x00003000, data);
		w520_mask(w, 0x10f914, 0x00003000, data);
	}

	if (!map_04_01)
		w520_mask(w, 0x10f824, 0x00006000, 0x00006000);
	else
		w520_ram_r10f658(w);

	if (!ANY_MAP(w, 0x04, 0x08) || !MAP(c, 0x04, 0x08)) {
		data = 0x22222222 * lowspeed;
		for (part = 0; part < ram->parts; part++) {
			if (!(ram->part_mask & BIT(part)))
				continue;
			w520_mask(w, 0x110620 + (part * 0x1000), 0x22222222, data);
			w520_mask(w, 0x110628 + (part * 0x1000), 0x22222222, data);
			w520_mask(w, 0x110630 + (part * 0x1000), 0x22222222, data);
		}
	}

	if (!div_mode) {
		/* Onto MPLL. */
		w520_ram_r10f830(w, slow);
		w520_ram_r137370(w, true);
		w520_mask(w, 0x137360, 0x00000001, 0x00000000);
	}

	/* Leave self-refresh, let the DLL lock. */
	w520_wr32(w, 0x10f090, 0x4000007f);
	w520_wr32(w, 0x10f210, 0x80000000);
	w520_nsec(w, locknsec);

	/* Mode registers. */
	if (w520_mask(w, 0x10f304, 0x00000245, mr[1] & 0x00000245) !=
	    w520_rd32(w, 0x10f304))
		w520_nsec(w, 1000);

	w520_ram_dll_reset(w);

	if (w520_mask(w, 0x10f320, 0x00000038, mr[2] & 0x00000038) !=
	    w520_rd32(w, 0x10f320))
		w520_nsec(w, 1000);
	w520_force(w, 0x10f300, 0x00000e74, mr[0] & 0x00000e74);
	w520_nsec(w, 1000);

	/* Timings. */
	w520_mask(w, 0x10f224, 0x001f0000, (c->bios.timing_10_18 & 0x1f) << 16);
	w520_ram_timing(w);

	if (ANY_CFG(w, 0x02, 0x08))
		w520_mask(w, 0x10f200, 0x00001000,
			  CFG(c, 0x02, 0x08) ? 0x00000000 : 0x00001000);

	if (mask = 0, data = 0, ANY_CFG(w, 0x02, 0x20)) {
		data |= 0xf0000000 * CFG(c, 0x02, 0x20);
		mask |= 0xf0000000;
	}
	if (ANY_CFG(w, 0x02, 0x04)) {
		data |= 0x01000000 * !CFG(c, 0x02, 0x04);
		mask |= 0x01000000;
	}
	if (ANY_CFG(w, 0x04, 0x03)) {
		data |= CFG(c, 0x04, 0x03) << 8;
		mask |= 0x00000300;
	}
	w520_mask(w, 0x10f604, mask, data);

	if (ANY_CFG(w, 0x02, 0x01))
		w520_mask(w, 0x10f614, 0x00000100, 0x00000100 * CFG(c, 0x02, 0x01));
	if (ANY_CFG(w, 0x02, 0x02))
		w520_mask(w, 0x10f610, 0x00000100, 0x00000100 * CFG(c, 0x02, 0x02));

	mask = 0x14000020;
	data = 0x00000000;
	if (any_02_10) {
		data |= 0x04000000 * !cfg_02_10;
		data |= 0x08000004 *  cfg_02_10;
		mask |= 0x08000004;
	}
	if (ANY_CFG(w, 0x02, 0x08)) {
		data |= 0x00100000 * !CFG(c, 0x02, 0x08);
		mask |= 0x00100000;
	}
	if (ANY_CFG(w, 0x02, 0x04)) {
		data |= 0x12800000 * !CFG(c, 0x02, 0x04);
		mask |= 0x12800000;
	}
	w520_mask(w, 0x10f808, mask, data);

	if (any_02_10)
		w520_mask(w, 0x1373ec, 0x00030000, cfg_02_10 ? 0 : 0x00020000);

	if (mask = 0, data = 0, ANY_CFG(w, 0x02, 0x01)) {
		for (part = 0; part < ram->parts; part++) {
			if (!(ram->part_mask & BIT(part)))
				continue;
			if (CFG(c, 0x02, 0x01))
				data |= BIT(part * 2);
			mask |= BIT(part * 2);
		}
		w520_mask(w, 0x1373f8, mask, data);
	}

	if (ANY_CFG(w, 0x0d, 0x0f))
		w520_wr32(w, 0x10f870, 0x11111111 * CFG(c, 0x0d, 0x0f));

	if (ANY_CFG(w, 0x03, 0x0f))
		w520_mask(w, 0x10f618, 0xffffffff, 0x11111111 * CFG(c, 0x03, 0x0f));

	if (mask = 0, data = 0, ANY_CFG(w, 0x02, 0x20)) {
		data |= 0x04000000 * CFG(c, 0x02, 0x20);
		mask |= 0x04000000;
	}
	if (ANY_CFG(w, 0x02, 0x01)) {
		data |= 0x08000000 * CFG(c, 0x02, 0x01);
		mask |= 0x08000000;
	}
	if (ANY_CFG(w, 0x04, 0x03)) {
		data |= CFG(c, 0x04, 0x03) << 24;
		mask |= 0x03000000;
	}
	w520_mask(w, 0x100c00, mask, data);

	if (!c->bios.ramcfg_DLLoff) {
		w520_ram_dll_reset(w);
		w520_nsec(w, locknsec);
	} else {
		w520_nsec(w, 5000);
	}

	if (div_from && !div_mode) {
		/* Pulse sequence the NVIDIA driver issues when memory moves
		 * from the divider to MPLL.
		 */
		const u32 base = w520_rd32(w, 0x10f324) & ~0x00000701;
		const u32 keep = w520_rd32(w, 0x10f324) & 0x00000700;

		w520_wr32(w, 0x10f324, base | 0x00000301);
		w520_wr32(w, 0x10f324, base | 0x00000601);
		w520_wr32(w, 0x10f324, base | keep | 0x00000001);
		w520_wr32(w, 0x10f324, base | 0x00000300);
		w520_wr32(w, 0x10f324, base | 0x00000600);
		w520_wr32(w, 0x10f324, base | keep);
	}

	w520_mask(w, 0x10f830, 0x01000000, 0x01000000);
	w520_mask(w, 0x10f830, 0x01000000, 0x00000000);

	/* Resume framebuffer access. */
	w520_ram_fb_access(w, true, r100b0c);

	if (r10f200_11)
		w520_mask(w, 0x10f200, 0x00000800, 0x00000800);

	if (w->failed || w->words > W520_RAM_WORDS)
		return -ENOSPC;
	return 0;
}

/******************************************************************************
 * Clock sources
 *****************************************************************************/
static int
w520_ram_src(struct w520_ram *w, u32 rsrc, u32 rctl, u32 khz,
	     u32 *dsrc, u32 *dctl)
{
	struct nvkm_device *device = w->ram->fb->subdev.device;
	int ref, div;

	*dctl = nvkm_rd32(device, rctl) | 0x80000000;
	*dsrc = nvkm_rd32(device, rsrc) | 0x00000003;

	if (!(*dsrc & 0x00000100))
		ref = nvkm_clk_read(device->clk, nv_clk_src_sppll0);
	else
		ref = nvkm_clk_read(device->clk, nv_clk_src_sppll1);
	if (ref <= 0)
		return -EINVAL;

	div = ((ref * 2) / khz) - 2;
	if (div < 0 || div > 63)
		return -EINVAL;

	if (*dsrc & 0x00000100)
		*dctl = (*dctl & ~0x00003f00) | div << 8;
	else
		*dctl = (*dctl & ~0x0000003f) | div;
	return (ref * 2) / (div + 2);
}

static int
w520_ram_calc_pll(struct w520_ram *w)
{
	struct nvkm_subdev *subdev = &w->ram->fb->subdev;
	struct nvkm_device *device = subdev->device;
	struct nvbios_pll pll;
	u32 coef, target = w->mempll.refclk;
	int khz, ref, N, M, P;

	/* Clock memory runs from while MPLL is changed. */
	khz = w520_ram_src(w, 0x137300, 0x137310, 405000, &w->dsrc, &w->dctl);
	if (khz < 0)
		return khz;

	/* Reference PLL input. */
	ref = w520_ram_src(w, 0x137320, 0x137330, w->refpll.refclk,
			   &w->rsrc, &w->rctl);
	if (ref < 0)
		return ref;

	/* Reference PLL: keep the VBIOS coefficients when they are sane. */
	coef = nvkm_rd32(device, 0x10fe24);
	N = (coef & 0x0000ff00) >> 8;
	M = (coef & 0x000000ff);
	P = (coef & 0x003f0000) >> 16;
	khz = (M && P) ? ref * N / M / P : 0;
	w->rpll_set = false;
	if (!khz || abs(khz - (int)target) > target / 100) {
		pll = w->refpll;
		pll.refclk = ref;
		khz = gt215_pll_calc(subdev, &pll, target, &N, NULL, &M, &P);
		if (khz <= 0)
			return khz ? khz : -ERANGE;
		w->rpll = (P << 16) | (N << 8) | M;
		w->rpll_set = true;
	}

	pll = w->mempll;
	pll.refclk = khz;
	khz = gt215_pll_calc(subdev, &pll, w->freq, &N, NULL, &M, &P);
	if (khz <= 0)
		return khz ? khz : -ERANGE;

	w->mpll = (P << 16) | (N << 8) | M;
	w->mode = W520_RAM_PLL;
	return khz;
}

static int
w520_ram_calc_div(struct w520_ram *w)
{
	int khz = w520_ram_src(w, 0x137300, 0x137310, w->freq,
			       &w->dsrc, &w->dctl);
	if (khz < 0)
		return khz;

	w->mode = W520_RAM_DIV;
	return khz;
}

static const struct w520_ram_level *
w520_ram_level(struct w520_ram *w, u32 khz)
{
	u32 mhz = khz / 1000;
	int i;

	for (i = 0; i < w->level_nr; i++) {
		if (mhz >= w->level[i].min && mhz <= w->level[i].max)
			return &w->level[i];
	}

	return NULL;
}

/******************************************************************************
 * Interface
 *****************************************************************************/
int
w520_ram_calc(struct w520_ram *w, u32 freq, u8 source)
{
	struct nvkm_subdev *subdev = &w->ram->fb->subdev;
	struct nvkm_device *device = subdev->device;
	int cur = nvkm_clk_read(device->clk, nv_clk_src_mem);
	const bool stepped = w->stepped;
	int khz, ret;

	w->stepped = false;

	if (cur <= 0)
		return -EINVAL;

	if (nvkm_rd32(device, 0x1373f0) & 0x00000002) {
		nvkm_error(subdev, "memory clock in an unexpected mode\n");
		return -ENOSYS;
	}

	if (nvkm_rd32(device, 0x137360) & 0x00000001) {
		w->from = W520_RAM_DIV;
	} else {
		if (!(nvkm_rd32(device, 0x132000) & 0x00000001)) {
			nvkm_error(subdev, "memory clock source not understood "
					   "(%08x %08x)\n",
				   nvkm_rd32(device, 0x137360),
				   nvkm_rd32(device, 0x132000));
			return -ENOSYS;
		}
		w->from = W520_RAM_PLL;
	}

	/* The VBIOS says which source a level uses; without that, a
	 * divider when one fits.
	 */
	w->freq = freq;
	w->again = false;
	khz = (source != 1) ? w520_ram_calc_div(w) : -EINVAL;
	if (source == 1 ||
	    (source != 2 && (khz < 0 || khz + 2000 < freq || khz > freq + 3000))) {
		/* Off the divider and onto MPLL is done at the current
		 * frequency first, the frequency changes after that.
		 */
		if (w->from == W520_RAM_DIV && abs(cur - (int)freq) > 3000 &&
		    !stepped) {
			w->freq = cur;
			w->again = true;
		}
		khz = w520_ram_calc_pll(w);
	}
	if (khz < 0) {
		nvkm_error(subdev, "no clock source for %d kHz\n", w->freq);
		return khz;
	}

	w->c = w520_ram_level(w, w->freq);
	if (!w->c) {
		nvkm_error(subdev, "no memory config for %d kHz\n", w->freq);
		return -EINVAL;
	}

	nvkm_info(subdev, "W520 mem %d kHz %s -> %d kHz %s (%d kHz)%s\n", cur,
		   w->from == W520_RAM_DIV ? "div" : "pll", w->freq,
		   w->mode == W520_RAM_DIV ? "div" : "pll", khz,
		   w->again ? ", first step" : "");

	/* Memory on the divider: MPLL and its reference PLL are idle, and
	 * the VBIOS leaves both held in reset.  Pulse and release it.
	 */
	if (w->from == W520_RAM_DIV) {
		nvkm_mask(device, 0x132000, 0x00000003, 0x00000002);
		nvkm_mask(device, 0x132000, 0x00000002, 0x00000000);
		nvkm_msec(device, 2000,
			if (nvkm_rd32(device, 0x132000) & 0x00010000)
				break;
		);
		nvkm_mask(device, 0x10fe20, 0x00000007, 0x00000002);
		nvkm_mask(device, 0x10fe20, 0x00000002, 0x00000000);
	}

	ret = nvkm_memx_init(device->pmu, &w->memx);
	if (ret)
		return ret;

	w->words = 0;
	w->reg_nr = 0;
	w->failed = false;
	ret = w520_ram_script(w);
	if (ret)
		nvkm_error(subdev, "transition to %d kHz not built: %d\n",
			   w->freq, ret);
	return ret;
}

static void
w520_ram_prog_10f4xx(struct w520_ram *w, const struct w520_ram_level *c)
{
	struct nvkm_device *device = w->ram->fb->subdev.device;
	const u32 any = get_unaligned_le32(&w->map_any[0x05]);
	const u32 val = get_unaligned_le32(&c->map[0x05]);
	u32 mask, data;

	if (mask = 0, data = 0, any & 0x0003fe00) {
		data |= (val & 0x0003fe00) << 3;
		mask |= 0x001ff000;
	}
	if (any & 0x000001ff) {
		data |= (val & 0x000001ff);
		mask |= 0x000001ff;
	}
	nvkm_mask(device, 0x10f468, mask, data);

	if (any & 0x00040000)
		nvkm_mask(device, 0x10f420, 0x00000001, (val & 0x00040000) >> 18);
	if (any & 0x00080000)
		nvkm_mask(device, 0x10f430, 0x00000001, (val & 0x00080000) >> 19);
	if (any & 0x01f00000)
		nvkm_mask(device, 0x10f400, 0x0000001f, (val & 0x01f00000) >> 20);
	if (any & 0x02000000)
		nvkm_mask(device, 0x10f410, 0x00000200, (val & 0x02000000) >> 16);

	if (mask = 0, data = 0, any & 0x08000000) {
		data |= (val & 0x08000000) >> 20;
		mask |= 0x00000080;
	}
	if (any & 0x04000000) {
		data |= (val & 0x04000000) >> 21;
		mask |= 0x00000020;
	}
	if (w->map_any[0x0a]) {
		data |= c->map[0x0a] << 8;
		mask |= 0x0000ff00;
	}
	nvkm_mask(device, 0x10f444, mask, data);
}

int
w520_ram_prog(struct w520_ram *w)
{
	struct nvkm_subdev *subdev = &w->ram->fb->subdev;
	struct nvkm_device *device = subdev->device;
	const struct w520_ram_level *c = w->c, *low;
	int ret;

	if (!w->memx)
		return -EINVAL;

	if (!nvkm_boolopt(device->cfgopt, "NvMemExec", true)) {
		nvkm_memx_fini(&w->memx, false);
		w->memx = NULL;
		return 0;
	}

	/* Slowest level's host-side settings across the transition. */
	low = w520_ram_level(w, 1000);
	if (!low)
		low = &w->level[0];
	w520_ram_prog_10f4xx(w, low);

	ret = nvkm_memx_fini(&w->memx, true);
	w->memx = NULL;
	if (ret) {
		nvkm_error(subdev, "memory script not run: %d\n", ret);
		return ret;
	}

	if (ANY_MAP(w, 0x04, 0x01) && !MAP(c, 0x04, 0x01)) {
		nvkm_mask(device, 0x132018, 0x00004000, 0x00000000);
		nvkm_mask(device, 0x10f824, 0x00002000, 0x00002000);
	}

	if (ANY_CFG(w, 0x02, 0x10)) {
		if (CFG(c, 0x02, 0x10)) {
			nvkm_mask(device, 0x10f824, 0x00000180, 0x00000180);
			nvkm_mask(device, 0x10f808, 0x40000000, 0x00000000);
		} else {
			nvkm_mask(device, 0x10f824, 0x00000600, 0x00000600);
		}
	}

	if (w->mode == W520_RAM_DIV) {
		nvkm_wr32(device, 0x137370, 0x00000000);
		nvkm_wr32(device, 0x137380, 0x00000000);
		nvkm_mask(device, 0x132000, 0x00000001, 0x00000000);
	}

	w520_ram_prog_10f4xx(w, c);
	nvkm_info(subdev, "W520 mem now %d kHz (%d script words)\n",
		  nvkm_clk_read(device->clk, nv_clk_src_mem), w->words);

	/* A positive return asks for calc() and prog() to run again. */
	w->stepped = w->again;
	return w->again ? 1 : 0;
}

void
w520_ram_tidy(struct w520_ram *w)
{
	if (w->memx) {
		nvkm_memx_fini(&w->memx, false);
		w->memx = NULL;
	}
}

static int
w520_ram_new_level(struct w520_ram *w, u8 strap, int i)
{
	struct nvkm_subdev *subdev = &w->ram->fb->subdev;
	struct nvkm_bios *bios = subdev->device->bios;
	struct w520_ram_level *c = &w->level[w->level_nr];
	u8  ver, hdr, cnt, len, raw[25];
	u32 data, cfg;
	int j;

	memset(c, 0x00, sizeof(*c));

	data = nvbios_rammapEp(bios, i, &ver, &hdr, &cnt, &len, &c->bios);
	if (!data)
		return -ENOENT;
	if (ver != 0x10 || hdr < 0x0e)
		return -ENOSYS;
	if (!c->bios.rammap_min && !c->bios.rammap_max)
		return 1;

	for (j = 0; j < min_t(int, hdr, sizeof(c->map)); j++)
		c->map[j] = nvbios_rd08(bios, data + j);

	cfg = nvbios_rammapSp(bios, data, ver, hdr, cnt, len, strap,
			      &ver, &hdr, &c->bios);
	if (!cfg)
		return -EINVAL;
	if (ver != 0x10 || hdr < 0x0d)
		return -ENOSYS;

	for (j = 0; j < min_t(int, hdr, sizeof(c->cfg)); j++)
		c->cfg[j] = nvbios_rd08(bios, cfg + j);

	if (c->bios.ramcfg_timing == 0xff)
		return -ENOSYS;

	data = nvbios_timingEp(bios, c->bios.ramcfg_timing,
			       &ver, &hdr, &cnt, &len, &c->bios);
	if (!data)
		return -EINVAL;
	if (ver != 0x10 || hdr < 0x19)
		return -ENOSYS;

	c->min = c->bios.rammap_min;
	c->max = c->bios.rammap_max;

	for (j = 0; j < sizeof(c->map); j++) {
		w->map_any[j] |= c->map[j];
		w->cfg_any[j] |= c->cfg[j];
	}
	w->any_RC |= c->bios.timing_10_RC != 0;
	w->any_CWL |= c->bios.timing_10_CWL != 0;
	w->any_DLLoff |= c->bios.ramcfg_DLLoff != 0;
	if (w->level_nr)
		w->fbvddq_differs |= CFG(c, 0x04, 0x08) != CFG(c - 1, 0x04, 0x08);

	nvkm_info(subdev, "W520 mem %4d-%4d MHz: map %*ph\n",
		  c->min, c->max, 14, c->map);
	nvkm_info(subdev, "W520 mem   cfg %*ph\n", 14, c->cfg);
	for (j = 0; j < sizeof(raw); j++)
		raw[j] = nvbios_rd08(bios, data + j);
	nvkm_info(subdev, "W520 mem   timing %02x: %*ph\n",
		  c->bios.ramcfg_timing, (int)sizeof(raw), raw);

	w->level_nr++;
	return 0;
}

int
w520_ram_new(struct nvkm_ram *ram, const struct nvbios_pll *refpll,
	     const struct nvbios_pll *mempll, struct w520_ram **pw)
{
	static const u32 regs[] = {
		0x137300, 0x137310, 0x137320, 0x137330, 0x137360, 0x137370,
		0x137380, 0x137390, 0x1373ec, 0x1373f0, 0x1373f8, 0x132000,
		0x132004, 0x132018, 0x10fe20, 0x10fe24, 0x100b0c, 0x100c00,
		0x10f050, 0x10f200, 0x10f224, 0x10f290, 0x10f294, 0x10f298,
		0x10f29c, 0x10f2a0, 0x10f300, 0x10f304, 0x10f320, 0x10f400,
		0x10f410, 0x10f420, 0x10f430, 0x10f444, 0x10f468, 0x10f604,
		0x10f610, 0x10f614, 0x10f618, 0x10f658, 0x10f660, 0x10f668,
		0x10f808, 0x10f824, 0x10f830, 0x10f870, 0x10f874, 0x10f910,
		0x10f914, 0x611200, 0x10f324, 0x022554, 0x101000, 0x10f090,
		0x10f210, 0x10f310, 0x132020, 0x132024, 0x132100, 0x10f314,
	};
	struct nvkm_subdev *subdev = &ram->fb->subdev;
	struct nvkm_device *device = subdev->device;
	struct w520_ram *w;
	u8 strap;
	int i, ret;

	if (ram->type != NVKM_RAM_TYPE_DDR3) {
		nvkm_warn(subdev, "W520: memory is not DDR3, no memory clock switching\n");
		return -ENODEV;
	}

	if (!(w = kzalloc_obj(*w)))
		return -ENOMEM;
	w->ram = ram;
	w->refpll = *refpll;
	w->mempll = *mempll;

	ram->parts = nvkm_rd32(device, 0x022438);
	ram->part_mask = ~nvkm_rd32(device, 0x022554) & (BIT(ram->parts) - 1);

	strap = nvbios_ramcfg_index(subdev);
	nvkm_info(subdev, "W520 mem strap %d parts %d mask %x refpll %d kHz mempll ref %d kHz\n",
		  strap, ram->parts, ram->part_mask, w->refpll.refclk,
		  w->mempll.refclk);

	for (i = 0, ret = 0; ret >= 0 && w->level_nr < ARRAY_SIZE(w->level); i++) {
		ret = w520_ram_new_level(w, strap, i);
		if (ret < 0 && ret != -ENOENT) {
			nvkm_error(subdev, "W520: memory config %d not usable: %d\n",
				   i, ret);
			kfree(w);
			return ret;
		}
	}

	if (!w->level_nr) {
		kfree(w);
		return -ENODEV;
	}

	for (i = 0; i + 3 < ARRAY_SIZE(regs); i += 4) {
		nvkm_info(subdev, "W520 reg %06x %08x  %06x %08x  %06x %08x  %06x %08x\n",
			  regs[i + 0], nvkm_rd32(device, regs[i + 0]),
			  regs[i + 1], nvkm_rd32(device, regs[i + 1]),
			  regs[i + 2], nvkm_rd32(device, regs[i + 2]),
			  regs[i + 3], nvkm_rd32(device, regs[i + 3]));
	}

	*pw = w;
	return 0;
}
