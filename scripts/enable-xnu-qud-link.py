#!/usr/bin/env python3
"""Wire XNU_QCOM_USB_DEBUG=1 and libQcomXnuUsbDebug.a into UniXNU/scripts/build-xnu-4k-baseline.sh.

Runs in CI during 'prepare uefi-pclm10' on the checked-out UniXNU tree without
modifying upstream UniXNU git history permanently. Idempotent.
"""
from __future__ import annotations

import sys
from pathlib import Path

MARKER = "# PCLM10_QCOM_USB_DEBUG_HOOK"

HOOK_CFLAGS_BLOCK = """# PCLM10_QCOM_USB_DEBUG_HOOK
QCOM_USB_DEBUG_MODE=0
if [ "${XNU_BOOTSTRAP:-0}" != 1 ] && [ "${XNU_QCOM_USB_DEBUG:-0}" = 1 ]; then
  QCOM_USB_DEBUG_MODE=1
fi
QUD_STAMP=$ROOT/build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/.qcom-usb-debug-mode
if [ -d "$ROOT/build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837" ]; then
  if [ "$(cat "$QUD_STAMP" 2>/dev/null || echo 0)" != "$QCOM_USB_DEBUG_MODE" ]; then
    find "$ROOT/build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837" -maxdepth 4 -type f \\
      \\( -name pe_kprintf.o -o -name debug.o -o -name printf.o -o -name serial_console.o -o -name serial_general.o -o -name bsd_init.o \\) \\
      -delete
    printf '%s\\n' "$QCOM_USB_DEBUG_MODE" > "$QUD_STAMP"
  fi
fi
QUD_OBJECT_CFLAGS="-DXNU_QCOM_USB_DEBUG=0"
if [ "$QCOM_USB_DEBUG_MODE" = 1 ]; then
  test -d "$ROOT/build/qcom-usb-debug/include"
  QUD_OBJECT_CFLAGS="-DXNU_QCOM_USB_DEBUG=1 -I$ROOT/build/qcom-usb-debug/include"
fi
EXTRA_MAKE_ARGS+=("pe_kprintf.o_CFLAGS_ADD=$QUD_OBJECT_CFLAGS")
EXTRA_MAKE_ARGS+=("debug.o_CFLAGS_ADD=$QUD_OBJECT_CFLAGS")
EXTRA_MAKE_ARGS+=("printf.o_CFLAGS_ADD=$QUD_OBJECT_CFLAGS")
EXTRA_MAKE_ARGS+=("serial_console.o_CFLAGS_ADD=$QUD_OBJECT_CFLAGS")
EXTRA_MAKE_ARGS+=("serial_general.o_CFLAGS_ADD=$QUD_OBJECT_CFLAGS")
"""

BSD_INIT_OLD = (
    'EXTRA_MAKE_ARGS+=("bsd_init.o_CFLAGS_ADD=-DXNU_SHENG_PMGR=$SHENG_PMGR '
    '-DXNU_VERIFIED_ROOT_PROBE=$OWNER_ROOT_PROBE -DXNU_OWNER_ROOT_AUDIT=$OWNER_ROOT_AUDIT '
    '-DXNU_OWNER_ROOT_TRUST=$OWNER_ROOT_TRUST -DXNU_ADRENO_FIRMWARE_PROBE=$ADRENO_FIRMWARE_PROBE '
    '-DXNU_ADRENO_PM4_PROBE=$ADRENO_PM4_PROBE -DXNU_SMMU4K_PROBE=$SMMU4K_PROBE '
    '-DXNU_CONSTANT_UPLOADS_PROBE=$CONSTANT_UPLOADS_PROBE '
    '-DXNU_NATIVE_LEGACY_DISPLAY=${XNU_NATIVE_LEGACY_DISPLAY:-0} '
    '-DXNU_NATIVE_OFFLINE_GPU=$NATIVE_OFFLINE_GPU")'
)
BSD_INIT_NEW = BSD_INIT_OLD[:-2] + ' $QUD_OBJECT_CFLAGS")'

LINK_ANCHOR = '  KERNEL_EXTRA_ARCHIVES="$KERNEL_EXTRA_ARCHIVES ${XNU_EXTRA_LD_ARCHIVES:-}"'
LINK_NEW = """  if [ "$QCOM_USB_DEBUG_MODE" = 1 ]; then
    QUD_ARCHIVE=$ROOT/build/qcom-usb-debug/libQcomXnuUsbDebug.a
    test -f "$QUD_ARCHIVE"
    KERNEL_EXTRA_ARCHIVES="$KERNEL_EXTRA_ARCHIVES -Wl,-force_load,$QUD_ARCHIVE"
    KERNEL_ARCHIVE_DEPENDENCIES="$KERNEL_ARCHIVE_DEPENDENCIES $QUD_ARCHIVE"
  fi
  KERNEL_EXTRA_ARCHIVES="$KERNEL_EXTRA_ARCHIVES ${XNU_EXTRA_LD_ARCHIVES:-}\""""

STAMP_ANCHOR = '  "$SHENG_PMGR" "$ROOT/build/sheng-platform-power/libShengPlatformPower.a" \\'
STAMP_NEW = STAMP_ANCHOR + '\n  "$QCOM_USB_DEBUG_MODE" "$ROOT/build/qcom-usb-debug/libQcomXnuUsbDebug.a" \\'


def main() -> int:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("UniXNU/scripts/build-xnu-4k-baseline.sh")
    text = path.read_text(encoding="utf-8")
    if MARKER in text:
        print(f"{path} already hooked for QcomXnuUsbDebug")
        return 0
    for anchor, label in ((BSD_INIT_OLD, "bsd_init"), (LINK_ANCHOR, "link"), (STAMP_ANCHOR, "stamp")):
        if text.count(anchor) != 1:
            raise SystemExit(f"Expected 1 match for {label} in {path}, found {text.count(anchor)}")
    text = text.replace(BSD_INIT_OLD, HOOK_CFLAGS_BLOCK + BSD_INIT_NEW, 1)
    text = text.replace(LINK_ANCHOR, LINK_NEW, 1)
    text = text.replace(STAMP_ANCHOR, STAMP_NEW, 1)
    path.write_text(text, encoding="utf-8")
    print(f"Patched {path} with QcomXnuUsbDebug build/link hooks")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
