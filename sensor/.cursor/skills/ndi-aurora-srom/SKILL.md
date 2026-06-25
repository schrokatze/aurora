---
name: ndi-aurora-srom
description: >-
  Export, program, and verify NDI Aurora physical SROM .rom files over serial
  (PSRCH/PSEL/PPRD/PPWR). Use when the user mentions SROM, .rom burn, ROM dump,
  blank NDI tool cable, Aurora sensor cloning, ndi_srom_dump, ndi_srom_write,
  or copying tool definition to identical hardware.
---

# NDI Aurora 物理 SROM 读写烧录

## 何时使用

用户要：**导出 ROM**、**烧录空白硬件**、**验证烧录**、或询问 SROM/`.rom`/物理芯片读写。

## 必读文档

完整流程、原理、错误码与检查清单：

**[Linux/NDI_SROM_WORKFLOW.md](../../../Linux/NDI_SROM_WORKFLOW.md)**

执行前先读该文档；以下仅为 Agent 操作摘要。

## 核心事实

- **物理 SROM**：`PPRD` 读、`PPWR` 写；**虚拟 SROM**：仅 `PVWR` 写、不可读。
- 必须在 **Setup 模式**（不要 `TSTART`）读写 SROM。
- `PSRCH` 前必须 `PHSR 02`，否则 `ERROR 0x2B`。
- `PSRCH` 正文：`1 位数量 + N×16 位 hex 设备 ID`；`PSEL` 用 **16 位完整 ID**。
- 每块硬件芯片 ID 不同；烧录前对**当前插入**硬件运行 `--list-devices`。
- 烧录写入需 `--confirm`；脚本内置 PPRD 回读校验。

## 标准工作流

### 导出（源硬件）

```bash
python3 Linux/ndi_srom_dump.py -p /dev/ttyUSB0 --sensor 1 -o srom_port1.rom
sha256sum srom_port1.rom
```

### 烧录（空白硬件，同端口）

```bash
# 1. 查 ID
python3 Linux/ndi_srom_write.py -p /dev/ttyUSB0 --sensor 1 --list-devices

# 2. 写入（替换 --device-id 为步骤 1 输出）
python3 Linux/ndi_srom_write.py -p /dev/ttyUSB0 --sensor 1 \
  -i srom_port1.rom --device-id <16位ID> --confirm

# 3. 文件校验
python3 Linux/ndi_srom_dump.py -p /dev/ttyUSB0 --sensor 1 -o verify.rom --skip-reset
sha256sum srom_port1.rom verify.rom
```

### 功能验证

1. **ROM 加载**：`PINIT`/`PENA` 后 `PHINF` 应为 `tool_type=0B000000`、含 `080061`（或源 ROM part number）。
2. **位姿**：`python3 Linux/ndi_sensor_tracker.py -p /dev/ttyUSB0 --sensors 1`
   - TX 有效坐标 → 完全成功
   - TX `MISSING` 但 PHINF 正确 → 烧录成功，传感器需放入测量体积

权限：用户未在 `dialout` 时用 `sg dialout -c '…'`。

## 实现位置

| 组件 | 路径 |
|------|------|
| 导出 CLI | `Linux/ndi_srom_dump.py` |
| 烧录 CLI | `Linux/ndi_srom_write.py` |
| 协议类 | `aurora传感器/sensor_tracker.py` → `dump_physical_srom`, `program_physical_srom` |

## 禁止

- 不要用 `PVWR` 代替 `PPWR` 烧录物理芯片。
- 不要未 `--confirm` 就声称已烧录。
- 不要复用旧硬件的 `--device-id` 给新插入的空白硬件。
- 不要在 `TSTART` 跟踪模式下执行 PSRCH/PPRD/PPWR。

## 验证报告模板

向用户汇报时包含：

- SROM 设备 ID、源/校验 SHA256 是否一致
- PHINF 是否识别为 NDI 工具
- TX 有效帧数或 MISSING 及物理层建议
