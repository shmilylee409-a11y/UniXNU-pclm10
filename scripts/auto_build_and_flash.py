#!/usr/bin/env python3
import os
import sys
import time
import subprocess
import json
import struct
import hashlib

RUN_ID = sys.argv[1]
REPO_DIR = "/Users/renny/Downloads/hakios/UniXNU-pclm10"
BIN_DIR = "/Users/renny/.local/bin:/Users/renny/.local/platform-tools:" + os.environ.get("PATH", "")
ENV = os.environ.copy()
ENV["PATH"] = BIN_DIR

def run(cmd, check=True):
    return subprocess.run(cmd, shell=True, cwd=REPO_DIR, env=ENV, capture_output=True, text=True, check=check)

def log(msg):
    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}", flush=True)

log(f"Starting auto-monitoring for GitHub Actions run {RUN_ID}...")

# 1. Poll until run completes
while True:
    res = run(f"gh run view {RUN_ID} --json status,conclusion", check=False)
    if res.returncode == 0:
        try:
            data = json.loads(res.stdout)
            status = data.get("status")
            conclusion = data.get("conclusion")
            log(f"Run {RUN_ID} status: {status}, conclusion: {conclusion}")
            if status == "completed":
                if conclusion != "success":
                    log(f"ERROR: Build ended with conclusion: {conclusion}")
                    sys.exit(1)
                break
        except Exception as e:
            log(f"Error parsing json: {e}")
    else:
        log(f"gh run view failed: {res.stderr}")
    time.sleep(30)

log("Build completed successfully! Downloading artifacts...")
out_artifacts = os.path.join(REPO_DIR, f"build_artifacts_run_{RUN_ID}")
import shutil
shutil.rmtree(out_artifacts, ignore_errors=True)
run(f"gh run download {RUN_ID} --dir {out_artifacts}")

# Locate boot.img and payload.img
boot_path = None
payload_path = None
for root, dirs, files in os.walk(out_artifacts):
    if "boot.img" in files:
        boot_path = os.path.join(root, "boot.img")
    if "payload.img" in files:
        payload_path = os.path.join(root, "payload.img")

log(f"Found boot: {boot_path}, payload: {payload_path}")

# Repack with Header v2 and DTB
DTB_FILE = "/tmp/stock_boot_dtb.dtb"
TARGET_BOOT = os.path.join(REPO_DIR, "build_artifacts", f"boot_run_{RUN_ID}.img")
UEFI_KERNEL = "/tmp/uefi_kernel_v2.bin"

# Extract uefi kernel from new boot.img
with open(boot_path, "rb") as f:
    hdr = f.read(4096)
    k_sz = struct.unpack_from("<I", hdr, 8)[0]
    r_sz = struct.unpack_from("<I", hdr, 16)[0]
    f.seek(4096)
    kernel_data = f.read(k_sz)
    with open(UEFI_KERNEL, "wb") as kf:
        kf.write(kernel_data)

# Read payload
with open(payload_path, "rb") as f:
    ramdisk_data = f.read()
ramdisk_size = len(ramdisk_data)
kernel_size = len(kernel_data)

with open(DTB_FILE, "rb") as f:
    dtb_data = f.read()
dtb_size = len(dtb_data)

# Compute SHA1 id
h = hashlib.sha1()
h.update(kernel_data)
h.update(struct.pack("<I", kernel_size))
h.update(ramdisk_data)
h.update(struct.pack("<I", ramdisk_size))
h.update(b"") # second
h.update(struct.pack("<I", 0))
h.update(b"") # recovery_dtbo
h.update(struct.pack("<I", 0))
h.update(dtb_data)
h.update(struct.pack("<I", dtb_size))
img_id = h.digest() + b"\x00" * 12

page_size = 4096
header_version = 2
kernel_addr = 0x8000
ramdisk_addr = 0x1000000
second_size = 0
second_addr = 0
tags_addr = 0x100
os_version = 369099109

hdr = bytearray(4096)
hdr[:8] = b"ANDROID!"
struct.pack_into("<10I", hdr, 8,
    kernel_size, kernel_addr,
    ramdisk_size, ramdisk_addr,
    second_size, second_addr,
    tags_addr, page_size,
    header_version, os_version
)
cmdline = b"console=ttyMSM0,115200n8 earlycon=msm_geni_serial,0xa90000 androidboot.hardware=qcom"
hdr[64:64+len(cmdline)] = cmdline
hdr[576:608] = img_id
struct.pack_into("<IQI", hdr, 1632, 0, 0, 1660)
struct.pack_into("<IQ", hdr, 1648, dtb_size, 0x1f00000)

def pad(size):
    return (page_size - (size % page_size)) % page_size

with open(TARGET_BOOT, "wb") as f:
    f.write(hdr)
    f.write(kernel_data)
    f.write(b"\x00" * pad(kernel_size))
    f.write(ramdisk_data)
    f.write(b"\x00" * pad(ramdisk_size))
    f.write(dtb_data)
    f.write(b"\x00" * pad(dtb_size))

total_size = os.path.getsize(TARGET_BOOT)
log(f"Repacked {TARGET_BOOT}: {total_size} bytes ({total_size/1024/1024:.2f} MiB)")

# Check if device is in fastboot, then flash!
res = run("fastboot devices", check=False)
if "fastboot" in res.stdout:
    log("Device detected in fastboot! Flashing boot partition...")
    run(f"fastboot flash boot {TARGET_BOOT}")
    log("Flashing completed! Rebooting device...")
    run("fastboot reboot")
    log("Rebooted! SUCCESS!")
else:
    log("Device not in fastboot. Please connect in fastboot mode to flash.")
