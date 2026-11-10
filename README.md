# UniXNU-pclm10 — OPPO Reno Ace (`PCLM10`)

Format 2 [UniXNU](https://github.com/UniXNU/UniXNU) device repository for **OPPO Reno Ace** (`PCLM10`, OPPO project `19081`, Qualcomm Snapdragon 855 Plus / `SM8150-AC`).

## Hardware Summary

| Subsystem | Hardware / Address |
|-----------|--------------------|
| **Model / Project** | OPPO Reno Ace (`PCLM10` / `OP4A89`, `oppo,dtsi_no = <19081>`) |
| **SoC** | Qualcomm Snapdragon 855 Plus (`SM8150-AC`, `msm-id = <0x153 0x20000>`, `board-id = <8 0>`) |
| **CPU** | 8x Kryo 485 (`MPIDR` `0x000..0x700`: 4x A55 Silver + 3x A76 Gold + 1x A76 Prime @ 2.96 GHz), PSCI `smc`, 19.2 MHz timer (virtual INTID `19`) |
| **GICv3** | Distributor `0x17a00000` (`0x10000`), Redistributors `0x17a60000` (`0x100000`, stride `0x20000`) |
| **DRAM** | Bank 0 `0x80000000..0xbbb00000` (with `tzapps_region` `0x87a00000` size `0x05d00000`, `pil_region` `0x8d700000` size `0x0f400000`); Contiguous XNU DRAM Bank 1 `0xc0000000` (`0x80000000`, 2 GiB baseline) + Bank 2 `0x140000000` |
| **Display** | Samsung SOFEF03F-M 6.5" Single-DSI Command-Mode AMOLED (`1080x2400` @ 90Hz, DSC), UEFI GOP framebuffer @ `0x9d000000` (`0x02400000`, stride `4320`, 32bpp) |
| **Touchscreen** | Synaptics S3706 (`synaptics-s3706`, I2C `0x20`) on `qupv3_se17_i2c` (`0x00c80000`, GIC SPI `373`, TLMM IRQ GPIO `122`, Reset GPIO `54`) |
| **Storage** | UFS 3.0 `ufshc@1d84000` (`0x01d84000`, size `0x2500`), PHY `0x01d87000`, GCC `0x00177000`, Apps SMMU `0x15000000` (stream `0x300`), 4096B blocks |
| **GPU** | Adreno 640 v2 (`chip-id = 0x06040001`, 1 MiB GMEM) @ `0x02c00000`, GMU `0x02c6a000`, GPU SMMU `0x02ca0000`, GPUCC `0x02c90000`, ZAP PIL `0x99515000` |
| **Wi-Fi / BT** | Qualcomm WCN3990 (`qca,wcn3990`) |
| **Haptics** | Awinic AW8697 (`awinic,aw8697_haptic`, I2C `0x5a`) on `qupv3_se7_i2c` (`0x0089c000`) |
| **Battery / Charge** | 65W SuperVOOC 2.0 dual-cell (`oppo,bq27541-battery` @ `0x55`, `oppo,stm8s-fastcg` @ `0x26` on `qupv3_se1_i2c` `0x00a80000`) |

## Layout

| Path | Role |
|------|------|
| `board.toml` | Format 2 manifest: board metadata, component selection, kernel fragments, DTS paths, UEFI firmware, boot.img & payload recipes |
| `feeds.toml` | Pinned git commit of `UniXNU-sm8150-common` (`f363dcda920101a99e440ce3e8b68849122a3c0a`) |
| `local.toml.example` | Template for pointing `sm8150` at a local checkout (`../UniXNU-sm8150-common`) |
| `configs/pclm10_defconfig` | Board kernel configuration fragment (`CONFIG_BOARD_PCLM10=y`, `CONFIG_SOC_SM8150=y`) |
| `dts/pclm10.dts` | Board device tree source including `sm8150.dtsi` from `UniXNU-sm8150-common` |
| `patches/xnu/series` | Board-specific XNU patches |
| `proprietary-files.txt` | Vendor firmware inventory relative to `/vendor/` |
| `porting.toml` | 13-area structured porting record validated by `scripts/porting-record.py` |
| `scripts/package.sh` | Release packaging entry point (`unixnu release -D .`) |

## Usage

From a `UniXNU` checkout alongside this directory:

```bash
cd ../UniXNU
./unixnu defconfig pclm10_defconfig -D ../UniXNU-pclm10
./unixnu status -D ../UniXNU-pclm10
python3 scripts/porting-record.py check --device ../UniXNU-pclm10
./unixnu build -D ../UniXNU-pclm10
```
