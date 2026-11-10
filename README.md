# UniXNU-pclm10 — OPPO Reno Ace (`PCLM10` / Project `19081`)

Format 2 (`schema_version = 2`) UniXNU device repository for **OPPO Reno Ace** (`PCLM10`, OPPO project `19081`, Qualcomm Snapdragon 855+ / `SM8150-AC`).

## Hardware Summary

| Subsystem | Specification | Physical Address / Resource |
|---|---|---|
| **SoC** | Qualcomm Snapdragon 855+ (`SM8150-AC`, v2) | 4x Kryo 485 Silver + 3x Gold + 1x Prime (`2.96 GHz`) |
| **DRAM / Carveouts** | 8 GB / 12 GB LPDDR4X | Bank 1 `0xc0000000` (`2 GiB` baseline); `tzapps_mem` `0x87a00000` (`0x5d00000`), `pil_region` `0x8dc00000` (`0xf400000`), `qseecom_region` `0xa0000000` (`0x1400000`), `cdsp_sec` `0xa4c00000` (`0xc00000`) |
| **Display / GOP** | Samsung `SOFEF03F-M` 6.5" AMOLED (`1080x2400 @ 90Hz`) | `cont_splash_region` `0x9d000000` (`0x2400000`, `32bpp`, stride `4320`) |
| **UEFI (`Mu-Silicium`)** | `pclm10Pkg` (`HanaPkg`, `SOC_TYPE = 1`) | `UEFI FD` `0x9fc00000` (`0x300000`), `UEFI Stack` `0x9ffb0000` (`0x20000`), `DXE Heap` `0xa5800000` (`0x1a800000`) |
| **Touchscreen** | Synaptics `S3706` (`synaptics,s3706` @ `0x20`) | `qupv3_se17_i2c` (`0x00c80000`), IRQ GPIO `122`, Reset GPIO `54` |
| **Storage** | Samsung UFS 3.0 (128 GB / 256 GB) | `ufshc@1d84000`, GCC `0x00177000`, SMMU `0x15000000` (stream `0x300`) |
| **GPU** | Qualcomm Adreno 640 v2 (`675 MHz`) | `kgsl_3d0` `0x02c00000`, GMU `0x02c6a000`, ZAP PIL `0x99515000` (`0x2000`) |
| **Battery / Fast Charge** | 4000 mAh Dual-Cell + 65W SuperVOOC 2.0 | `qupv3_se1_i2c` (`0x00a80000`): TI `BQ27541` (`0x55`) + `STM8S` (`0x26`) |
| **Haptics** | Awinic `AW8697` LRA | `qupv3_se7_i2c` (`0x0089c000` @ `0x5a`), Reset GPIO `116`, IRQ GPIO `24` |

## Cloud Build via GitHub Actions (`macos-26` Runner)

This repository includes a self-contained GitHub Actions workflow (`.github/workflows/package.yml`) that builds `boot.img`, `payload.img`, and `kernel.macho` in the cloud without requiring a local full Xcode installation:

1. Create a new repository on GitHub (e.g., `UniXNU-pclm10`) and push this repository:
   ```sh
   git remote add origin https://github.com/<your-username>/UniXNU-pclm10.git
   git push -u origin main
   ```
2. *(Optional)* In your GitHub repository **Settings → Secrets and variables → Actions**, add a repository secret `UNIXNU_UNIQUE_CHIP_ID` set to any 8-hex-digit seed (e.g., `19081001`). If omitted, the workflow automatically falls back to `19081001`.
3. Open the **Actions** tab → select **package** → click **Run workflow** (default OS profile: `17`).
4. Once the build finishes, download the **`images-pclm10-17`** artifact containing:
   - `boot.img` (Mu-Silicium UEFI `pclm10Pkg` + `DarwinBoot` + `UniXNU` + `oppo-pclm10-19081.dtb`)
   - `payload.img` (FAT32 volume `PCLM10` with prelinked `kernel.macho`, `kernel.macho.adt`, and `kernel.macho.trustcache`)
   - `kernel.macho`
   - `package.manifest.json`

## First-Boot Verification (`fastboot`)

With an unlocked bootloader on your OPPO Reno Ace (`PCLM10`):

```sh
# 1. Stage payload.img (either as ramdisk in boot.img or on a FAT32 partition / OTG)
# 2. Boot the UEFI + DarwinBoot image in RAM without overwriting stock partitions:
fastboot boot boot.img
```

## Repository Layout

- `board.toml`: Format 2 board specification (`SM8150`, `ipados-17.7.11-21H461`, `[firmware]`, `[bootimg]`, `[payload]`)
- `configs/pclm10_defconfig`: Stage 1 kernel configuration (`CONFIG_XNU_NATIVE_GOP_FRAMEBUFFER=y`, `CONFIG_XNU_DT_PANIC_REGION=y`)
- `dts/pclm10.dts`: Hardware FDT for ADT generation (`memory@c0000000`, `19081` carveouts, `framebuffer0@9d000000`)
- `dts/oppo-pclm10-19081.dtb`: Single-board DTB (`486,447` bytes, `oppo,prjversion = <1 19081>`) for `boot.img`
- `scripts/prepare-pclm10-uefi.py`: Injects `Resources/Configs/pclm10.toml` and `Platforms/OPPO/pclm10Pkg/` into `UniXNU/uefi-sheng` and stages `build/pclm10/pclm10.dtb`
- `porting.toml`: Auditable 13-area porting record verified by `porting-record.py check`
