#!/bin/bash
# Build the SWEET iOS root HFS+ (Case-sensitive, Journaled) image natively on macOS.
#
#   sudo scripts/mkiosroot-mac.sh OS.dmg SYSTEMOS.dmg APPOS.dmg OUT.img [SIZE]
#
# Preserves root/system owners (--numeric-ids), setuid bits, hard links, dyld shared cache
# links, _logd directories, plain systembag.kb, and applies scripts/sweet/sweet-root-tweaks.py.
set -euo pipefail

if [[ $# -lt 4 ]]; then
	echo "usage: sudo $0 OS.dmg SYSTEMOS.dmg APPOS.dmg OUT.img [SIZE_GB=16G]" >&2
	exit 2
fi
if [[ $EUID -ne 0 ]]; then
	echo "error: run with sudo so root:wheel / _logd (272) ownership on HFS+ is preserved" >&2
	exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
OS_DMG="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
SYS_DMG="$(cd "$(dirname "$2")" && pwd)/$(basename "$2")"
APP_DMG="$(cd "$(dirname "$3")" && pwd)/$(basename "$3")"
OUT_IMG="$4"
SIZE="${5:-16G}"

[[ ! -e "$OUT_IMG" ]] || { echo "refusing to overwrite $OUT_IMG" >&2; exit 1; }

WORK="$(mktemp -d /tmp/pclm10-iosroot.XXXXXX)"
DEVS=()
cleanup() {
	for ((i=${#DEVS[@]}-1; i>=0; i--)); do
		hdiutil detach "${DEVS[$i]}" -force >/dev/null 2>&1 || true
	done
	rmdir "$WORK"/* "$WORK" 2>/dev/null || true
}
trap cleanup EXIT

attach_ro() {
	local img="$1" raw="${2:-0}" out_mnt_var="$3"
	local mnt
	mnt="$(mktemp -d "$WORK/mnt.XXXXXX")"
	local args=(attach -readonly -nobrowse -mountpoint "$mnt")
	if [[ "$raw" == "1" ]]; then
		args+=(-imagekey diskimage-class=CRawDiskImage)
	fi
	local info dev
	info="$(hdiutil "${args[@]}" "$img")"
	dev="$(printf '%s\n' "$info" | awk 'NR==1{print $1}')"
	DEVS+=("$dev")
	printf -v "$out_mnt_var" '%s' "$mnt"
}

attach_ro "$OS_DMG" 0 OS_MNT
attach_ro "$SYS_DMG" 1 SYS_MNT
attach_ro "$APP_DMG" 1 APP_MNT

size_bytes="$(python3 -c '
import sys
s = sys.argv[1].strip().upper()
mult = {"K": 1024, "M": 1024**2, "G": 1024**3}.get(s[-1:], 1)
n = int(s[:-1]) * mult if mult > 1 else int(s)
print(n)
' "$SIZE")"

echo "Allocating sparse raw image $OUT_IMG ($SIZE = $size_bytes bytes)..."
truncate -s "$size_bytes" "$OUT_IMG"

RAW_DEV="$(hdiutil attach -imagekey diskimage-class=CRawDiskImage -nomount "$OUT_IMG" | awk 'NR==1{print $1}')"
DEVS+=("$RAW_DEV")
newfs_hfs -s -J -v iOSRoot "$RAW_DEV"
hdiutil detach "$RAW_DEV"
unset 'DEVS[${#DEVS[@]}-1]'

ROOT_MNT="$WORK/root"
mkdir -p "$ROOT_MNT"
ROOT_DEV="$(hdiutil attach -imagekey diskimage-class=CRawDiskImage -owners on -nobrowse -mountpoint "$ROOT_MNT" "$OUT_IMG" | awk 'NR==1{print $1}')"
DEVS+=("$ROOT_DEV")

echo "Copying OS rootfs -> $ROOT_MNT ..."
rsync -aH --numeric-ids "$OS_MNT/" "$ROOT_MNT/"

echo "Copying SystemOS cryptex -> $ROOT_MNT/private/preboot/Cryptexes/OS ..."
mkdir -p "$ROOT_MNT/private/preboot/Cryptexes/OS"
rsync -aH --numeric-ids "$SYS_MNT/" "$ROOT_MNT/private/preboot/Cryptexes/OS/"

echo "Copying AppOS cryptex -> $ROOT_MNT/private/preboot/Cryptexes/App ..."
mkdir -p "$ROOT_MNT/private/preboot/Cryptexes/App"
rsync -aH --numeric-ids "$APP_MNT/" "$ROOT_MNT/private/preboot/Cryptexes/App/"

# Shared cache at ignition level 1
DSC_REL="private/preboot/Cryptexes/OS/System/Library/Caches/com.apple.dyld"
mkdir -p "$ROOT_MNT/System/Library/Caches/com.apple.dyld"
for f in "$ROOT_MNT/$DSC_REL"/*; do
	ln -f "$f" "$ROOT_MNT/System/Library/Caches/com.apple.dyld/${f##*/}"
done

# Hard-link remaining cryptex files not present in OS root
for c in OS App; do
	(
		cd "$ROOT_MNT/private/preboot/Cryptexes/$c"
		find . ! -type d ! -path './System/Library/Caches/com.apple.dyld/*'
	) | while IFS= read -r f; do
		f="${f#./}"
		dst="$ROOT_MNT/$f"
		if [[ ! -e "$dst" && ! -L "$dst" ]]; then
			mkdir -p "$(dirname "$dst")"
			src="$ROOT_MNT/private/preboot/Cryptexes/$c/$f"
			if [[ -L "$src" ]]; then
				cp -P "$src" "$dst"
			else
				ln "$src" "$dst"
			fi
		fi
	done
done

# logd directories & markers
DB="$ROOT_MNT/private/var/db"
install -d -o 272 -g 272 -m 0751 "$DB/diagnostics"
install -d -o 272 -g 272 -m 0755 "$DB/uuidtext" "$DB/diagnostics/timesync"
for d in diagnostics uuidtext diagnostics/timesync; do
	install -o 272 -g 272 -m 0644 /dev/null "$DB/$d/.migrated"
done

# Plain keybag for /defaults/no-effaceable-storage=1
install -d -m 0700 -o 0 -g 0 "$ROOT_MNT/private/var/keybags"
printf '%s\n' \
	'<?xml version="1.0" encoding="UTF-8"?>' \
	'<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">' \
	'<plist version="1.0"><dict><key>KeyBagKeys</key><data>U1dFRVQ=</data></dict></plist>' \
	| install -m 0600 -o 0 -g 0 /dev/stdin "$ROOT_MNT/private/var/keybags/systembag.kb"

# Apply ios-sweet bring-up tweaks
python3 "$SCRIPT_DIR/sweet/sweet-root-tweaks.py" "$ROOT_MNT"

df -h "$ROOT_MNT"
hdiutil detach "$ROOT_DEV"
unset 'DEVS[${#DEVS[@]}-1]'

echo "Running fsck_hfs check on $OUT_IMG ..."
CHK_DEV="$(hdiutil attach -imagekey diskimage-class=CRawDiskImage -nomount "$OUT_IMG" | awk 'NR==1{print $1}')"
DEVS+=("$CHK_DEV")
fsck_hfs -fn "$CHK_DEV"
hdiutil detach "$CHK_DEV"
unset 'DEVS[${#DEVS[@]}-1]'

echo "Done: $OUT_IMG"
