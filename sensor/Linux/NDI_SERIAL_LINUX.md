# Linux 下 NDI Aurora / USB 串口使用说明

## 插入后是否「每次都要命令才能用」？

**一般不需要单独的「注册」或「启动服务」命令。**

- 插上 **USB 转串口（NDI SCU / Host 转换器等）** 后，内核里的 **USB 串口驱动**（常见为 `ftdi_sio` 或 CDC ACM）会通过 **udev 自动枚举** 出设备节点，例如：
  - `/dev/ttyUSB0`、`/dev/ttyUSB1` …
  - 或 `/dev/ttyACM0` …
- 你的应用或探测脚本只要 **打开对应串口**、按 **NDI 协议**（BREAK / `RESET` / 带 CRC 的命令等）通信即可。

也就是说：**插上 → 等系统出现 `/dev/ttyUSB*`（或 `ttyACM*`）→ 有权限即可用**，没有类似 Windows 里必须先跑一遍「驱动安装向导」那种必经步骤。

---

## 真正「一次性必要」的准备：串口权限

无法打开 `/dev/ttyUSB0` 且提示 **Permission denied** 时，不是设备没注册，而是 **当前用户对设备节点没有读写权限**。

### 1. 把用户加入 `dialout` 组（推荐，做一次即可）

```bash
sudo usermod -aG dialout $USER
```

然后 **注销并重新登录**（或重启）。新开终端执行：

```bash
groups
```

输出里应包含 **`dialout`**。之后在同一用户下，**每次插入 USB** 一般都可执行（**在仓库根目录**下相对路径如下；也可把 `Linux/...` 换成绝对路径）：

```bash
python3 Linux/ndi_serial_probe.py
```

（若有多块 USB 串口，用 `-p` 指定端口，例如 `-p /dev/ttyUSB1`。）

### 2. 尚未重登、又想立刻测试

当前会话里组信息还没刷新时，可临时：

```bash
sg dialout -c 'python3 Linux/ndi_serial_probe.py'
```

或先 `newgrp dialout` 再运行脚本。

### 3. 确认系统已识别用户属组（与当前 shell 无关）

```bash
id $USER
```

若列表里已有 **`dialout`**，说明系统侧已配置好；若 `groups` 仍没有，多半是 **当前终端会话未刷新**，重登即可。

---

## 每次插入时要注意什么？

| 情况 | 说明 |
|------|------|
| 设备节点名变化 | 多串口时枚举顺序可能变，`ttyUSB0` 不一定是 NDI；用 `dmesg \| tail` 或断开再插看新增节点。 |
| 权限 | 已加 `dialout` 并重登后，通常 **无需每次再执行 usermod**。 |
| 无「守护进程」 | 不需要 `systemctl start …` 之类来「启动传感器」；设备上电 + USB 连接即在线。 |
| 旧内核补丁 | 现代发行版内核已内置常见 USB 串口 VID/PID；仓库 `Linux/usb-patch/` 仅针对很老的内核，一般可忽略。 |

---

## 与本仓库脚本的关系

- **`Linux/ndi_serial_probe.py`**：用于验证 **BREAK → RESET → `VER 4`** 与 CRC 是否正常。
- **`Linux/ndi_sensor_tracker.py`**：在 Linux 上打开 `/dev/ttyUSB*`（或 `ttyACM*`），复用 **`aurora传感器/sensor_tracker.py`** 里的 **`NDISensorTracker`**，完成初始化、启用指定物理端口、**`TSTART` 后以 `TX` 循环读取位姿**（终端实时打印；`Ctrl+C` 退出）。
- 日常集成到自己的程序里：用 **pyserial** 或语言自带串口库打开 `/dev/ttyUSBx`，按 NDI API 发命令即可；**无额外「注册命令」**。

这里的「driver」在本仓库中指 **用户态 Python 协议实现**（CRC、命令帧、BREAK、`INIT`、`PHSR`/`PENA`、`TSTART`/`TX` 等），**不是**必须单独加载的内核 `.ko` 模块；内核侧仍是常见的 **USB 串口驱动**（如 `ftdi_sio`、`cdc_acm`）提供 `/dev/ttyUSB*` 设备节点。

---

## 本次测试「都调用了什么」与实现链（从外到内）

下面描述一次典型成功测试时，**从命令到代码**的调用关系（便于写实验记录或交接文档）。

### 1. Shell 层（你实际敲的命令）

| 步骤 | 命令 / 作用 |
|------|----------------|
| 串口权限 | `sg dialout -c '...'`：在未重登前，为**这一条子命令**附加 `dialout` 组，避免 `Permission denied`。已 `usermod -aG dialout` 并重登后，可省略 `sg dialout -c`。 |
| 协议探测（可选但推荐先做） | `python3 Linux/ndi_serial_probe.py -p /dev/ttyUSB0`：发 BREAK、读 `RESET`、发带 CRC 的 `VER 4`，确认链路与 CRC 一致。 |
| 位姿采集 / 「启动跟踪」 | `sg dialout -c 'python3 Linux/ndi_sensor_tracker.py -p /dev/ttyUSB0 --sensors 1,2'`：完整走 NDI 初始化 + 启用端口 1 和 2 + 进入 TX 循环。 |

### 2. Python 入口层（`Linux/ndi_sensor_tracker.py`）

- 把本仓库下的 **`aurora传感器`** 目录加入 **`sys.path`**。
- **`from sensor_tracker import NDISensorTracker`**：不在 Linux 目录里复制一大段协议代码，而是**直接复用** Windows 侧已验证过的同一套类。
- 解析 `--port` / `--sensors` 等参数，按顺序调用：`connect()` → `hardware_reset()`（可用 `--skip-reset` 跳过）→ `initialize_system()` → `enable_sensors([1,2])` → `start_tracking()` → `run_tracking_loop()`；退出时在 `finally` 里 `stop_tracking()`、`disconnect()`。

### 3. 协议实现层（`aurora传感器/sensor_tracker.py` 中的 `NDISensorTracker`）

- **串口**：**pyserial** `serial.Serial(...)` 打开设备节点，波特率等与 NDI 示例一致（默认 9600）。
- **硬件复位**：`send_break()` + 读应答 + CRC 校验 + 等待，与 Combined API 示例思路一致。
- **命令**：`_add_crc_to_command`（首空格改 `:`，CRC-16，追加 4 位 hex + CR）、`_send_command` / `_get_response`。
- **初始化与工具**：`INIT`、`PHSR`、`PHINF`、`PINIT`、`PENA` 等（启用 `--sensors` 指定的物理端口）。
- **跟踪**：`TSTART` 后循环 **`TX 0001`**，在 `get_tx_transforms()` 里解析 ASCII 位姿；`run_tracking_loop()` 里打印单点坐标或两传感器相对位移。

### 4. 内核 / 设备层（系统自带）

- USB 线连接后，**内核 USB 串口驱动** 创建 **`/dev/ttyUSB0`**（或 `ttyACM0` 等）。
- 应用只负责**有权限地打开该字符设备**并按 NDI 文本协议读写。

---

## 如何在 md 里「记录利用 driver」的写法建议

写实验记录或本说明的增补时，建议固定包含下面几块（复制到任意 `*.md` 即可复用结构）：

1. **环境**：发行版、内核大致版本、`python3 --version`、是否已 `dialout`、`pyserial` 版本。  
2. **硬件与节点**：NDI 主机如何接 USB、本次使用的设备路径（如 `/dev/ttyUSB0`）、如何确认（`ls`、`dmesg`）。  
3. **执行的命令**：完整一行，例如 `sg dialout -c 'python3 Linux/ndi_sensor_tracker.py -p /dev/ttyUSB0 --sensors 1,2'`。  
4. **软件栈说明**：写明「**非内核 NDI 专有驱动**，而是 **Linux 通用 USB 串口 + 本仓库 Python（`NDISensorTracker`）**」。  
5. **结果**：probe 是否通过、tracker 是否进入循环打印、异常时贴首尾日志。

**可直接粘贴的示例段落：**

```markdown
## NDI 串口采集记录

- **日期**：
- **设备节点**：`/dev/ttyUSB0`（确认方式：插拔对比 `dmesg` / `ls`）
- **权限**：`sg dialout -c '...'` / 或已加入 `dialout` 并重登
- **依赖**：`pip install pyserial`
- **探测**：`python3 Linux/ndi_serial_probe.py -p /dev/ttyUSB0`
- **采集**：`python3 Linux/ndi_sensor_tracker.py -p /dev/ttyUSB0 --sensors 1,2`
- **实现说明**：使用仓库 `Linux/ndi_sensor_tracker.py` 加载 `aurora传感器/sensor_tracker.py` 的 `NDISensorTracker`；内核为 USB 串口驱动，应用层为 NDI 文本协议 + CRC。
- **结果**：（通过 / 失败现象与日志摘录）
```

---

## 可选：固定设备名（进阶）

若希望无论插拔顺序如何都得到稳定路径（如 `/dev/ndi_aurora`），可自行写 **udev 规则**（按 USB 序列号或 VID/PID 匹配）。这不属于「必须」步骤，按需再做即可。

---

## 简要结论

1. **每次插入**：多数情况下 **插上就能用**（节点自动出现）。  
2. **必要的一次性操作**：把用户加入 **`dialout`** 并 **重新登录**（或每次用 `sg dialout` 临时带组）。  
3. **不需要**：每次插入都执行某种「系统注册 / 服务启动」命令才能用串口。
