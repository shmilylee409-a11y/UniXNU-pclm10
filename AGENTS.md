# Agent Instructions for `UniXNU-pclm10`

This repository is the **Format 2** UniXNU device repository for **OPPO Reno Ace (`PCLM10`, OPPO project `19081`, Qualcomm Snapdragon 855 Plus / `SM8150-AC`)**.

## Repository Boundaries

- **What belongs here**:
  - `board.toml`, `feeds.toml`, `configs/pclm10_defconfig`, `dts/pclm10.dts`, `patches/xnu/series`, `proprietary-files.txt`, `porting.toml`, and `scripts/package.sh`.
- **What belongs in `UniXNU-sm8150-common`**:
  - Shared SM8150 SoC definitions (`soc.toml`, `dts/sm8150.dtsi`, `configs/sm8150.config`, SoC-wide XNU patches).
- **What belongs in `UniXNU`**:
  - Build orchestration (`./unixnu`), kernel/driver components (`drivers/`, `apple-oss-distributions/`), and OS profiles (`profiles/os/`).

## Hardware Invariants for `pclm10` (OPPO Project `19081`)

- **Model**: `PCLM10` (do not confuse with `PCKM00`, which is OPPO Reno2 SM7150).
- **Display**: Samsung SOFEF03F-M `1080x2400` @ 90Hz single-DSI command-mode AMOLED with DSC. UEFI GOP framebuffer is at `0x9d000000` (`0x02400000`, stride `4320`, 32bpp).
- **Touch**: Synaptics S3706 (`synaptics-s3706`, I2C `0x20`) on `qupv3_se17_i2c` (`0x00c80000`, GIC SPI `373`, TLMM IRQ GPIO `122`, Reset GPIO `54`).
- **Memory**: `tzapps_region` at `0x87a00000` is `0x05d00000` (94 MiB), shifting `pil_region` to `0x8d700000` (`0x0f400000`). A firmware carveout hole exists at `0xbbb00000..0xc0000000`. XNU's contiguous DRAM window starts at `0xc0000000` (`0x80000000` baseline).

## Verification Commands

Always validate changes from the sibling `UniXNU` checkout before committing:

```bash
./unixnu defconfig pclm10_defconfig -D ../UniXNU-pclm10
./unixnu status --json -D ../UniXNU-pclm10
python3 scripts/porting-record.py check --device ../UniXNU-pclm10
```
