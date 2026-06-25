# NDI电磁传感器跟踪脚本

基于Source目录中的NDI CombinedAPI示例代码实现的Python传感器跟踪脚本。

## 功能特性

1. **串口通信**: 通过COM9口，9600波特率连接传感器
2. **系统识别**: 自动识别并初始化传感器系统
3. **实时坐标**: 实时获取并显示传感器坐标数据（位置、旋转、误差）
4. **传感器状态**: 显示1、2、3、4号传感器的开启/关闭状态
5. **硬件复位**: 支持硬件复位功能

## 依赖要求

- Python 3.6+
- pyserial >= 3.5

## 安装

```bash
pip install -r requirements.txt
```

## 使用方法

### 基本使用

直接运行脚本：

```bash
python sensor_tracker.py
```

脚本会自动执行以下步骤：
1. 连接到COM9口（9600波特率）
2. 执行硬件复位
3. 初始化系统
4. 启用所有端口
5. 显示传感器状态
6. 开始跟踪并实时显示坐标数据

### 自定义串口

如果需要使用其他串口或波特率，可以修改脚本中的参数：

```python
tracker = NDISensorTracker(port="COM9", baudrate=9600)
```

### 代码示例

```python
from sensor_tracker import NDISensorTracker

# 创建跟踪器实例
tracker = NDISensorTracker(port="COM9", baudrate=9600)

# 连接
if tracker.connect():
    # 硬件复位
    tracker.hardware_reset()
    
    # 初始化系统
    tracker.initialize_system()
    
    # 启用所有端口
    tracker.enable_all_ports()
    
    # 获取传感器状态
    status = tracker.get_sensor_status()
    print(f"传感器状态: {status}")
    
    # 开始跟踪
    tracker.start_tracking()
    
    # 获取坐标数据
    transforms = tracker.get_tx_transforms()
    for handle, transform in transforms.items():
        if transform.get('valid'):
            pos = transform['translation']
            print(f"句柄 {handle:02X}: ({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f})")
    
    # 停止跟踪
    tracker.stop_tracking()
    tracker.disconnect()
```

## 输出格式

### 传感器状态

```
=== 传感器状态 ===
传感器 1: 开启
传感器 2: 关闭
传感器 3: 开启
传感器 4: 关闭
==================
```

### 坐标数据

```
=== 坐标数据 ===
句柄 01:
  位置: (123.456, 789.012, 345.678) mm
  旋转: q0=0.9239, qx=0.0000, qy=0.3827, qz=0.0000
  误差: 0.0012, 帧号: 12345
================
```

## 实现说明

本脚本基于Source目录中的以下文件实现：

- `CommandHandling.cpp/h`: 命令处理和通信协议
- `Comm32.cpp/h`: 串口通信
- `SystemCRC.cpp`: CRC校验计算
- `Conversions.cpp/h`: 数据转换和解析
- `APIStructures.h`: 数据结构和常量定义

主要功能对应关系：

- `NDISensorTracker._calc_crc16()` ← `SystemCRC.cpp::CalcCrc16()`
- `NDISensorTracker._send_command()` ← `CommandHandling.cpp::nSendMessage()`
- `NDISensorTracker._get_response()` ← `CommandHandling.cpp::nGetResponse()`
- `NDISensorTracker.hardware_reset()` ← `CommandHandling.cpp::nHardWareReset()`
- `NDISensorTracker.initialize_system()` ← `CommandHandling.cpp::nInitializeSystem()`
- `NDISensorTracker.get_tx_transforms()` ← `CommandHandling.cpp::nGetTXTransforms()`

## 注意事项

1. 确保COM9口未被其他程序占用
2. 传感器需要正确连接并上电
3. 硬件复位后需要等待3秒让系统初始化
4. 如果遇到通信错误，检查串口设置和连接

## 故障排除

### 连接失败
- 检查串口名称是否正确（Windows: COM9, Linux: /dev/ttyUSB0等）
- 确认串口未被其他程序占用
- 检查波特率设置（默认9600）

### 初始化失败
- 确认传感器已正确连接并上电
- 尝试执行硬件复位
- 检查串口线是否正常

### 无坐标数据
- 确认传感器已启用（`enable_all_ports()`）
- 检查传感器是否在跟踪范围内
- 查看传感器状态是否显示为"开启"

## 许可证

本脚本基于NDI提供的示例代码实现，遵循原代码的许可证条款。

