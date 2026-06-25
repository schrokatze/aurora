# NDI Aurora 物理 SROM 读写与烧录流程

本文档记录本仓库中 **从已配置传感器导出 `.rom` → 烧录到空白相同硬件 → 验证** 的完整流程。适用于带 **物理 SROM 芯片** 的 NDI Aurora 有源工具线缆（非纯电磁线圈虚拟 SROM 场景）。

相关脚本与实现：

| 路径 | 作用 |
|------|------|
| `Linux/ndi_srom_dump.py` | 从物理 SROM **读出** `.rom` |
| `Linux/ndi_srom_write.py` | 向物理 SROM **写入** `.rom` |
| `Linux/ndi_sensor_tracker.py` | 跟踪验证（位姿） |
| `Linux/ndi_serial_probe.py` | 串口/CRC 探测 |
| `aurora传感器/sensor_tracker.py` | `NDISensorTracker`：`PPRD`/`PPWR`/`PSRCH`/`PSEL` 等 |

串口与权限说明见 [NDI_SERIAL_LINUX.md](./NDI_SERIAL_LINUX.md)。

---

## 1. 原理

### 1.1 物理 SROM vs 虚拟 SROM

```
PC (Python)  ←─串口─→  Aurora SCU  ←─工具线─→  物理 SROM 芯片
```

| 类型 | 存储位置 | 读 | 写 | 本仓库命令 |
|------|----------|----|----|------------|
| **物理 SROM** | 工具线缆上的芯片 | `PPRD` | `PPWR` | `ndi_srom_dump.py` / `ndi_srom_write.py` |
| **虚拟 SROM** | SCU/主机内存 | 不可读 | `PVWR` | Combined API Sample（未封装） |

空白相同硬件烧录使用的是 **物理 SROM** 路径。

### 1.2 命令序列

**读出（导出）：**

```
INIT → PHSR 02 → PSRCH → PSEL → PPRD × 16（每块 64B，共 1024B）
```

**写入（烧录）：**

```
INIT → PHSR 02 → PSRCH → PSEL → PPWR × 16 → PPRD 回读校验
```

**跟踪验证：**

```
INIT → PINIT → PENA → TSTART → TX 循环
```

**约束：**

- 读写 SROM 必须在 **Setup 模式**（不要 `TSTART`）。
- 执行 `PSRCH` 前应先 `PHSR 02`，否则可能 `ERROR 0x2B`（句柄未分配）。
- 波特率默认 **9600**，设备节点常见为 `/dev/ttyUSB0`。

### 1.3 PSRCH 响应格式（重要）

Aurora 物理 SROM 的 `PSRCH` 返回 **非 OKAY 前缀** 的数据帧，正文结构为：

```
[1 位十六进制：设备数量 N] + [N × 16 位十六进制：SROM 设备 ID]
```

示例：

```
响应正文: 10B1182B602000045
         │└────────────────── SROM ID（16 hex）
         └─ 数量 = 1
```

`PSEL` 必须使用 **完整 16 位** ID，不能用 4 位分段猜测。

不同硬件芯片的 SROM ID 不同（正常），例如本项目中出现过：

| 硬件 | SROM 设备 ID |
|------|----------------|
| 原始已配置 | `0B1182B602000045` |
| 空白 #1 | `0B7276B602000066` |
| 空白 #2 | `0B481A9F02000097` |

写入的是同一份 **工具定义内容**（如 part `080061`），与芯片 ID 无关。

---

## 2. 环境准备

```bash
# 依赖
pip install pyserial

# 权限（一次性）
sudo usermod -aG dialout $USER
# 重登后生效；临时测试：
sg dialout -c 'python3 Linux/ndi_serial_probe.py -p /dev/ttyUSB0'
```

确认设备节点：

```bash
ls -la /dev/ttyUSB*
```

---

## 3. 流程 A：从已配置传感器导出 ROM

### 3.1 命令

```bash
cd /path/to/sensor

python3 Linux/ndi_srom_dump.py \
  -p /dev/ttyUSB0 \
  --sensor 1 \
  -o srom_port1.rom
```

可选参数：

| 参数 | 说明 |
|------|------|
| `--handle 0A` | 直接指定句柄（十六进制） |
| `--device-id <16位hex>` | 跳过 PSRCH，手动指定 ID |
| `--init-port` | 读取前先 PINIT |
| `--skip-reset` | 跳过 BREAK 复位 |

### 3.2 导出验证

```bash
# 大小应为 1024
ls -la srom_port1.rom

# 内容特征
strings srom_port1.rom    # 期望含 080061、NDI 相关字段
xxd srom_port1.rom | head

# 重复导出一致性
python3 Linux/ndi_srom_dump.py -p /dev/ttyUSB0 --sensor 1 -o verify.rom --skip-reset
sha256sum srom_port1.rom verify.rom   # 应相同
```

---

## 4. 流程 B：烧录 ROM 到空白相同硬件

### 4.1 插入空白硬件

将空白工具接到 **相同端口**（如 sensor1），上电。

### 4.2 查询空白芯片 SROM ID

```bash
python3 Linux/ndi_srom_write.py \
  -p /dev/ttyUSB0 \
  --sensor 1 \
  --list-devices
```

记录输出的 16 位 ID，例如 `0B481A9F02000097`。

### 4.3 执行烧录（必须 `--confirm`）

```bash
python3 Linux/ndi_srom_write.py \
  -p /dev/ttyUSB0 \
  --sensor 1 \
  -i srom_port1.rom \
  --device-id 0B481A9F02000097 \
  --confirm
```

未加 `--confirm` 时 **仅预览，不会写入**。

脚本会自动：

1. `PPWR` 写入 16 × 64B  
2. `PPRD` 逐块回读并与源文件对比  

### 4.4 烧录后文件验证

```bash
python3 Linux/ndi_srom_dump.py -p /dev/ttyUSB0 --sensor 1 -o verify_after_write.rom --skip-reset
sha256sum srom_port1.rom verify_after_write.rom
```

---

## 5. 流程 C：功能验证

### 5.1 ROM 加载识别（软件层）

烧录成功后，`PINIT` / `PENA` 应能识别工具：

| 阶段 | tool_type | 说明 |
|------|-----------|------|
| 空白 | `00000000` | 未配置 |
| 烧录后 PINIT | `0B000000` | NDI 工具 |
| PHINF | serial 含 `080061` | part number 正确 |

可通过启用 sensor1 时的 `PHINF` 日志确认。

### 5.2 位姿跟踪（物理层）

```bash
python3 Linux/ndi_sensor_tracker.py -p /dev/ttyUSB0 --sensors 1
```

**判定：**

| 现象 | 含义 |
|------|------|
| `PINIT`/`PENA`/`TSTART` 均 OKAY，PHINF 为 NDI 080061 | ROM 烧录与加载 **成功** |
| TX 返回 **MISSING** | 工具已启用，但线圈不在测量体积内或 FG 未就绪 |
| TX 返回有效 X/Y/Z | 全流程 **完全成功** |

MISSING 多为 **传感器未放入 Aurora 测量体积**，不是 ROM 烧录失败。

---

## 6. 一键检查清单

复制用于实验记录：

```markdown
## SROM 烧录记录

- 日期：
- 串口：/dev/ttyUSB0
- 端口：sensor 1 / 句柄 0A
- 源 ROM：srom_port1.rom（SHA256: …）
- 目标芯片 SROM ID：（来自 --list-devices）
- 烧录命令：ndi_srom_write.py … --confirm
- 回读 SHA256：与源文件 [ ] 一致
- PHINF：tool_type=0B000000，080061 [ ] 是
- TX 位姿：[ ] 有效 / [ ] MISSING（物理位置）
```

---

## 7. 常见错误

| 错误码 | 含义 | 处理 |
|--------|------|------|
| `0x2B` | 句柄未分配 | 先 `PHSR 02` 再 `PSRCH` |
| `0x07` | 命令参数数量错误 | `PSEL` 的 ID 长度不对，用 16 位完整 ID |
| `0x20` | 无法选中 SROM | ID 错误或芯片无响应 |
| `0x1E` | 无法读 SROM | 未 `PSEL` 或芯片不可读 |
| `0x1F` / `0xF5` | 无法写 SROM / Flash | 芯片写保护或硬件故障 |
| TX `MISSING` | 体积外/无场 | 将传感器放入 FG 测量区域 |

---

## 8. Agent 执行顺序（自动化）

当用户要求导出、烧录或验证 SROM 时，按序执行：

```
1. ls /dev/ttyUSB* 确认设备
2. [导出] ndi_srom_dump.py -p … --sensor N -o …
3. [烧录] ndi_srom_write.py --list-devices → 记录 ID
4. [烧录] ndi_srom_write.py -i … --device-id … --confirm
5. [验证] ndi_srom_dump.py + sha256sum
6. [验证] 短脚本：PINIT/PENA/TSTART + 若干 TX 帧 + PHINF 检查
```

烧录前 **必须** 用 `--list-devices` 获取**当前插入硬件**的 ID，不可复用上一块芯片的 ID。

---

## 9. 参考

- NDI Combined API：`PSRCH` / `PSEL` / `PPRD` / `PPWR` / `PINIT` / `PENA`
- PlusToolkit ndicapi：`ndiPPRD` / `ndiPPWR` 宏定义
- 本仓库 C++ 示例：`aurora传感器/Source/CommandHandling.cpp`（`PVWR` 为虚拟 SROM，与物理 `PPWR` 不同）
