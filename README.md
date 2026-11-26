# wignix-kernel

The WigNix Kernel for the ThinkPad W520 and MacPro 2013 specifically.
Based on Linux 7.3.x . Notable commits on top are:

## nouveau / W520 (Fermi)

- **drm/nouveau: W520 reclock, low voltage, context-switch firmware,
  copy-engine fix** — memory runs at 900 MHz; the engine level follows
  the power source (0f on mains, 07 on battery); every level runs at the
  lowest level's voltage (0.83 V) by default. Uses NVIDIA's graphics
  context-switch firmware when present and gives each channel its own
  ZCULL save buffer. Fixes the built-in copy-engine image to set the
  control bit that makes the engine honour address high bytes — the fix
  the Vulkan driver (`wignix-nouveau`) depends on.

  Options on the kernel command line, `nouveau.config=...`:
  `NvW520Reclock`, `NvClkMode`, `NvClkModeAC`/`NvClkModeDC`,
  `NvMemExec`, `NvW520EngExec`, `NvW520LowVolt`, `NvW520HoldVolt`,
  `NvGrUseFW`, `NvGrZcullCtxsw`. A read-only `pstate` file reports the
  requested level and the clocks read back.

## amdgpu / Mac Pro 6,1

- **drm/amd/amdgpu: keep SI clocks up with no display** — the SI legacy DPM
  path told the SMC there was no display when there was no CRTC, and the
  SMC held the lowest level; keep the display-change flag set so a
  headless SI GPU can still clock up.
- **drm/amd/amdgpu: optional user caps for SI** — `si_max_sclk`,
  `si_max_mclk` and a throttle point, defaulting to the VBIOS values.
- **drm/amd/amdgpu: guard the virtual-display path on legacy DPM** — guard
  the bandwidth-update call on `adev->mode_info.funcs`, which a virtual
  display never sets on the SI path.

## ALSA / Mac Pro 6,1

- **ALSA: hda/cs4208: fixup for the Mac Pro 6,1 audio GPIOs** — GPIO0 is the
  internal speaker amp enable, GPIO1 the headphone amp enable; jack
  detection on pin 0x10 is not inverted.

## Build fixes

- **kbuild: keep the full version out of the generated autoconf.h** — so
  ccache survives a minor version change.
- **kbuild: fixdep: build with large-file support** — `_FILE_OFFSET_BITS 64`.
- **kbuild: only reference `__clear_pages_unrolled` for built-ins** — wrap
  the KCFI reference in `#ifndef MODULE`.

## Building

```
make defconfig        # or use the Sandy Bridge config from wignix-packages
make -j$(nproc)
```

The packaged form (config, signing, out-of-tree modules) is the
`linux-sandy` template in `wignix-packages`.
