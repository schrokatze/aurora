#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
电磁传感器驱动脚本
基于Source目录中的NDI CombinedAPI示例代码实现
功能：
1. 通过COM9口，9600波特率连接传感器
2. 识别传感器并初始化系统
3. 实时获取坐标数据
4. 显示1、2、3、4号传感器状态
5. 提供硬件复位功能
"""

import serial
import time
import struct
import sys
import csv
import os
from datetime import datetime
from typing import Optional, Tuple, Dict, List

# 常量定义（参考APIStructures.h和CommandHandling.h）
CARRIAGE_RETURN = 0x0D
LINE_FEED = 0x0A
MAX_REPLY_MSG = 4096
MAX_COMMAND_MSG = 1024
NO_HANDLES = 0xFF

# 回复类型（参考APIStructures.h）
REPLY_ERROR = 0x00
REPLY_OKAY = 0x01
REPLY_RESET = 0x02
REPLY_OTHER = 0x04
REPLY_BADCRC = 0x08
REPLY_WARNING = 0x10
REPLY_INVALID = 0x20

# 变换状态（参考APIStructures.h）
TRANSFORM_VALID = 0x0000
TRANSFORM_MISSING = 0x1000
TRANSFORM_UNOCCUPIED = 0x2000
TRANSFORM_DISABLED = 0x3000
TRANSFORM_ERROR = 0x4000


class NDISensorTracker:
    """NDI传感器跟踪器类，基于Source代码实现"""
    
    def __init__(self, port: str = "COM9", baudrate: int = 9600, debug: bool = False):
        """
        初始化传感器跟踪器
        
        Args:
            port: 串口名称，默认COM9
            baudrate: 波特率，默认9600
            debug: 是否启用调试模式
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self.crc_table = []
        self.crc_initialized = False
        self.timeout = 3  # 默认超时时间3秒
        self.handle_info = {}  # 存储传感器句柄信息
        self.debug = debug  # 调试模式
        self.record_file = None  # 记录文件对象
        self.csv_writer = None  # CSV写入器
        self.record_enabled = False  # 是否启用记录
        
        # 初始化CRC表
        self._init_crc_table()
    
    def _init_crc_table(self):
        """初始化CRC查找表（参考SystemCRC.cpp的InitCrcTable）"""
        self.crc_table = [0] * 256
        for i in range(256):
            l_crc_table = i
            for j in range(8):
                l_crc_table = (l_crc_table >> 1) ^ ((l_crc_table & 1) and 0xA001 or 0)
            self.crc_table[i] = l_crc_table & 0xFFFF
        self.crc_initialized = True
    
    def _calc_crc16(self, crc: int, data: int) -> int:
        """
        计算CRC16（参考SystemCRC.cpp的CalcCrc16）
        使用多项式 X^16 + X^15 + X^2 + 1
        """
        if not self.crc_initialized:
            self._init_crc_table()
        crc = self.crc_table[(crc ^ data) & 0xFF] ^ (crc >> 8)
        return crc & 0xFFFF
    
    def _add_crc_to_command(self, command: str) -> str:
        """
        为命令添加CRC校验（参考CommandConstruction.cpp的nAddCRCToCommand）
        
        Args:
            command: 原始命令字符串
            
        Returns:
            添加了CRC的命令字符串
        """
        if len(command) >= MAX_COMMAND_MSG - 6:
            raise ValueError("命令过长")
        
        # 将第一个空格替换为冒号
        cmd_list = list(command)
        first_space = False
        for i, char in enumerate(cmd_list):
            if char == ' ' and not first_space:
                cmd_list[i] = ':'
                first_space = True
        
        command_with_colon = ''.join(cmd_list)
        
        # 计算CRC
        u_crc = 0
        for char in command_with_colon:
            u_crc = self._calc_crc16(u_crc, ord(char))
        
        # 添加CRC（4位十六进制）和回车符
        return command_with_colon + f"{u_crc:04X}" + chr(CARRIAGE_RETURN)
    
    def connect(self) -> bool:
        """
        连接串口（参考Comm32.cpp的SerialOpen）
        
        Returns:
            连接是否成功
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                rtscts=False  # 不使用硬件流控
            )
            print(f"成功连接到 {self.port}，波特率 {self.baudrate}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接（参考Comm32.cpp的SerialClose）"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("已断开连接")
    
    def _send_command(self, command: str, add_crc: bool = True) -> bool:
        """
        发送命令（参考CommandHandling.cpp的nSendMessage）
        
        Args:
            command: 命令字符串
            add_crc: 是否添加CRC校验
            
        Returns:
            发送是否成功
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("串口未打开")
            return False
        
        try:
            if add_crc:
                full_command = self._add_crc_to_command(command)
            else:
                full_command = command + chr(CARRIAGE_RETURN)
            
            # 发送命令
            self.serial_conn.write(full_command.encode('latin-1'))
            self.serial_conn.flush()
            print(f"发送命令: {command}")
            return True
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False
    
    def _get_response(self) -> Optional[str]:
        """
        获取响应（参考CommandHandling.cpp的nGetResponse）
        
        Returns:
            响应字符串（包含回车符），失败返回None
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        
        try:
            response = b""
            start_time = time.time()
            
            while time.time() - start_time < self.timeout:
                if self.serial_conn.in_waiting > 0:
                    char = self.serial_conn.read(1)
                    # 将字符添加到响应中（包括回车符，参考源代码第2185行）
                    response += char
                    if char == bytes([CARRIAGE_RETURN]):
                        # 找到回车符，响应完成
                        response_str = response.decode('latin-1', errors='ignore')
                        print(f"收到响应: {response_str.rstrip()}")
                        return response_str
                time.sleep(0.01)
            
            print("响应超时")
            return None
        except Exception as e:
            print(f"获取响应失败: {e}")
            return None
    
    def _check_crc(self, response: str) -> bool:
        """
        检查响应CRC（参考SystemCRC.cpp的SystemCheckCRC）
        
        Args:
            response: 响应字符串（应包含回车符）
            
        Returns:
            CRC校验是否通过
        """
        if len(response) < 4:
            return False
        
        # 检查是否为二进制响应（BX命令）
        if len(response) >= 2:
            byte0 = ord(response[0]) if len(response) > 0 else 0
            byte1 = ord(response[1]) if len(response) > 1 else 0
            if (byte0 & 0xFF) == 0xC4 and (byte1 & 0xFF) == 0xA5:
                # 二进制响应，使用CalcCRCByLen（简化处理）
                return True  # 二进制CRC检查较复杂，这里简化处理
        
        # ASCII响应CRC检查
        # 找到回车符位置（参考SystemCRC.cpp第250行）
        cr_pos = response.find(chr(CARRIAGE_RETURN))
        if cr_pos < 4:
            # 如果没有找到回车符，尝试使用字符串长度
            # 某些情况下响应可能没有回车符
            if len(response) < 4:
                return False
            cr_pos = len(response)
        
        # 确保有足够的字符（至少4位CRC）
        if cr_pos < 4:
            return False
        
        # 计算CRC（不包括最后4位CRC和回车符，参考SystemCRC.cpp第270行）
        u_crc = 0
        for i in range(cr_pos - 4):
            u_crc = self._calc_crc16(u_crc, ord(response[i]))
        
        # 读取响应中的CRC（最后4位十六进制，参考SystemCRC.cpp第276行）
        try:
            reply_crc_str = response[cr_pos-4:cr_pos]
            reply_crc = int(reply_crc_str, 16)
        except ValueError:
            return False
        
        # 调试信息（仅在CRC不匹配时显示）
        if u_crc != reply_crc:
            print(f"  CRC不匹配: 计算值={u_crc:04X}, 收到值={reply_crc_str}, 响应长度={len(response)}, CR位置={cr_pos}")
            print(f"  响应内容: {repr(response[:cr_pos-4])}")
        
        return u_crc == reply_crc
    
    def _verify_response(self, response: str, check_crc: bool = True) -> int:
        """
        验证响应类型（参考CommandHandling.cpp的nVerifyResponse）
        
        Args:
            response: 响应字符串
            check_crc: 是否检查CRC（参考源代码第2532行）
            
        Returns:
            响应类型代码
        """
        if not response:
            return REPLY_OTHER
        
        # 移除回车符进行比较（但保留在字符串中用于CRC检查）
        response_clean = response.rstrip('\r\n')
        response_upper = response_clean.upper()
        
        # 确定响应类型（参考源代码第2519-2530行）
        if response_upper.startswith("RESET"):
            n_response = REPLY_RESET
        elif response_upper.startswith("OKAY"):
            n_response = REPLY_OKAY
        elif response_upper.startswith("ERROR"):
            n_response = REPLY_ERROR
        elif response_upper.startswith("WARNING"):
            n_response = REPLY_WARNING
        elif len(response_clean) > 0:
            n_response = REPLY_OTHER
        else:
            return REPLY_OTHER
        
        # 对于OKAY和OTHER响应，检查CRC（参考源代码第2532行）
        if check_crc and (n_response & REPLY_OKAY or (n_response & REPLY_OTHER)):
            if not self._check_crc(response):
                return REPLY_BADCRC
        
        return n_response
    
    def hardware_reset(self) -> bool:
        """
        硬件复位（参考CommandHandling.cpp的nHardWareReset）
        
        Returns:
            复位是否成功
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("串口未打开")
            return False
        
        try:
            # 发送串口中断（Serial Break）
            self.serial_conn.send_break(duration=0.25)  # 250ms中断
            time.sleep(0.5)  # 等待中断生效
            
            # 读取复位响应
            response = self._get_response()
            if response:
                resp_type = self._verify_response(response, check_crc=True)
                if resp_type & REPLY_RESET:
                    # 检查CRC（参考源代码第264行）
                    if not self._check_crc(response):
                        print("硬件复位响应CRC校验失败")
                        return False
                    print("硬件复位成功")
                    time.sleep(3)  # 等待系统初始化（参考源代码中的3秒延迟）
                    return True
                else:
                    print(f"复位响应异常: {resp_type}")
                    return False
            else:
                print("未收到复位响应")
                return False
        except Exception as e:
            print(f"硬件复位失败: {e}")
            return False
    
    def initialize_system(self) -> bool:
        """
        初始化系统（参考CommandHandling.cpp的nInitializeSystem）
        
        Returns:
            初始化是否成功
        """
        if not self._send_command("INIT "):
            return False
        
        response = self._get_response()
        if not response:
            return False
        
        resp_type = self._verify_response(response, check_crc=True)
        if resp_type == REPLY_OKAY:
            print("系统初始化成功")
            return True
        elif resp_type == REPLY_BADCRC:
            print("系统初始化失败: CRC校验错误")
            # 调试信息：显示计算的CRC
            if len(response) >= 4:
                cr_pos = response.find(chr(CARRIAGE_RETURN))
                if cr_pos < 4:
                    cr_pos = len(response)
                if cr_pos >= 4:
                    u_crc = 0
                    for i in range(cr_pos - 4):
                        u_crc = self._calc_crc16(u_crc, ord(response[i]))
                    print(f"  计算的CRC: {u_crc:04X}, 收到的CRC: {response[cr_pos-4:cr_pos]}")
            return False
        else:
            print(f"系统初始化失败: 响应类型 {resp_type}")
            return False
    
    def get_port_handles(self, mode: int = 0) -> List[int]:
        """
        获取端口句柄列表（参考CommandHandling.cpp的PHSR命令）
        
        Args:
            mode: PHSR模式
                0: 获取所有已初始化的句柄
                1: 获取需要释放的句柄
                2: 获取需要初始化的句柄
                3: 获取需要启用的句柄
        
        Returns:
            句柄列表
        """
        if not self._send_command(f"PHSR {mode:02d}"):
            return []
        
        response = self._get_response()
        if not response:
            return []
        
        resp_type = self._verify_response(response)
        if resp_type != REPLY_OKAY and resp_type != REPLY_OTHER:
            return []
        
        # 解析响应：前2位是句柄数量，后面是句柄列表
        # 格式：数量(2) + [句柄(2) + 状态(2) + 分隔符(1)] * N
        try:
            # 移除CRC和回车符
            cr_pos = response.find(chr(CARRIAGE_RETURN))
            if cr_pos >= 4:
                data = response[:cr_pos-4]
            else:
                data = response
            
            if len(data) < 2:
                return []
            
            # 解析句柄数量（前2位十六进制）
            num_handles = self._ascii_to_hex(data[0:2], 2)
            handles = []
            pos = 2
            
            if self.debug:
                print(f"  [调试] PHSR模式{mode}: 找到{num_handles}个句柄，数据: {repr(data)}")
            
            for i in range(num_handles):
                if pos + 2 <= len(data):
                    # 解析句柄（2位十六进制）
                    handle = self._ascii_to_hex(data[pos:pos+2], 2)
                    handles.append(handle)
                    pos += 2  # 句柄
                    
                    # 跳过状态字节（2位）和分隔符（1位）
                    # 格式：句柄(2) + 状态(2) + 分隔符(1) = 5个字符（参考源代码1241-1242行：n+=5）
                    # 每个条目固定5个字符，所以直接跳过5个字符
                    if pos + 3 <= len(data):
                        # 跳过状态（2位）+ 分隔符（1位）
                        pos += 3
                    elif pos + 2 <= len(data):
                        # 如果没有分隔符，只跳过状态
                        pos += 2
                    
                    if self.debug:
                        remaining = data[pos:min(pos+10, len(data))] if pos < len(data) else ""
                        print(f"  [调试] 句柄 {i+1}: {handle:02X}, 当前位置: {pos}, 剩余: {repr(remaining)}")
                else:
                    if self.debug:
                        print(f"  [调试] 句柄 {i+1}: 数据不足，位置 {pos}, 长度 {len(data)}")
                    break
            
            return handles
        except Exception as e:
            print(f"解析句柄列表失败: {e}")
            import traceback
            traceback.print_exc()
            return []
    
    def get_port_information(self, handle: int) -> Optional[Dict]:
        """
        获取端口信息（参考CommandHandling.cpp的nGetPortInformation）
        
        Args:
            handle: 端口句柄
            
        Returns:
            端口信息字典，失败返回None
        """
        if not self._send_command(f"PHINF {handle:02X}0025"):
            return None
        
        response = self._get_response()
        if not response:
            return None
        
        resp_type = self._verify_response(response)
        if resp_type != REPLY_OKAY and resp_type != REPLY_OTHER:
            return None
        
        try:
            # 移除CRC和回车符
            cr_pos = response.find(chr(CARRIAGE_RETURN))
            if cr_pos >= 4:
                data = response[:cr_pos-4]
            else:
                data = response
            
            # 移除所有空白字符（空格、换行、制表符等）用于解析
            data_clean = ''.join(data.split())
            
            # 同时保留原始数据用于查找物理端口信息
            data_original = data
            
            if len(data_clean) < 33:  # 至少需要33个字符（到status字节）
                print(f"  警告: PHINF响应数据长度不足: {len(data_clean)}")
                # 尝试从原始响应中解析
                data_clean = data_original.replace(' ', '').replace('\n', '').replace('\r', '').replace('\t', '')
            
            # 解析端口信息（参考源代码第1345-1370行）
            # PHINF 0025格式：tool_type(8) + manufact(12) + rev(3) + serial(8) + status(2) + part_number(20) + ...
            info = {
                'handle': handle,
                'tool_type': data_clean[0:8] if len(data_clean) > 8 else "",
                'manufacturer': data_clean[8:20] if len(data_clean) > 20 else "",
                'revision': data_clean[20:23] if len(data_clean) > 23 else "",
                'serial_no': data_clean[23:31] if len(data_clean) > 31 else "",
                'status': 0,
                'physical_port': "",
                'enabled': False,
                'initialized': False,
                'tool_in_port': False,
            }
            
            # 解析状态字节（参考源代码第1357-1365行）
            if len(data_clean) > 33:
                try:
                    status_byte = int(data_clean[31:33], 16)
                    info['status'] = status_byte
                    info['tool_in_port'] = (status_byte & 0x01) != 0
                    info['initialized'] = (status_byte & 0x10) != 0
                    info['enabled'] = (status_byte & 0x20) != 0
                except ValueError:
                    pass
            
            # 解析物理端口号（参考源代码第1376-1377行）
            # 物理端口信息在part_number之后的位置
            import re
            # 方法1: 从原始响应中查找 "Port X" 或类似格式
            port_match = re.search(r'[Pp]ort[\s]*([1-4])', response)
            if port_match:
                info['physical_port'] = port_match.group(1)
            else:
                # 方法2: 从PHINF响应的特定位置提取
                # 根据源代码，物理端口信息在特定偏移位置
                # 对于PHINF 0025，物理端口在偏移约53字节后（8+12+3+8+2+20=53）
                if len(data_clean) > 53:
                    # 尝试从该位置提取2个字符作为物理端口
                    phys_port_str = data_clean[53:55]
                    try:
                        # 尝试解析为十六进制或十进制
                        port_num = int(phys_port_str, 16)
                        if 1 <= port_num <= 4:
                            info['physical_port'] = str(port_num)
                        else:
                            # 尝试十进制
                            port_num = int(phys_port_str)
                            if 1 <= port_num <= 4:
                                info['physical_port'] = str(port_num)
                    except ValueError:
                        pass
                
                # 方法3: 从响应字符串中查找数字1-4
                if not info['physical_port']:
                    num_match = re.search(r'\b([1-4])\b', response)
                    if num_match:
                        info['physical_port'] = num_match.group(1)
            
            return info
        except Exception as e:
            print(f"解析端口信息失败: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def initialize_port(self, handle: int) -> bool:
        """
        初始化端口（参考CommandHandling.cpp的nInitializeHandle）
        
        Args:
            handle: 端口句柄
            
        Returns:
            初始化是否成功
        """
        print(f"  初始化端口句柄 {handle:02X}...")
        if not self._send_command(f"PINIT {handle:02X}"):
            return False
        
        response = self._get_response()
        if not response:
            print(f"  端口 {handle:02X} 初始化失败: 无响应")
            return False
        
        resp_type = self._verify_response(response)
        if resp_type == REPLY_OKAY:
            print(f"  端口 {handle:02X} 初始化成功")
            return True
        else:
            print(f"  端口 {handle:02X} 初始化失败: 响应类型 {resp_type}")
            return False
    
    def enable_port(self, handle: int, check_initialized: bool = True) -> bool:
        """
        启用端口（参考CommandHandling.cpp的nEnableOnePorts）
        
        Args:
            handle: 端口句柄
            check_initialized: 是否检查并初始化端口（如果未初始化）
            
        Returns:
            启用是否成功
        """
        # 检查端口是否已初始化
        if check_initialized:
            info = self.get_port_information(handle)
            if info and not info.get('initialized', False):
                print(f"  端口 {handle:02X} 未初始化，先进行初始化...")
                if not self.initialize_port(handle):
                    print(f"  端口 {handle:02X} 初始化失败，无法启用")
                    return False
        
        print(f"  启用端口句柄 {handle:02X}...")
        if not self._send_command(f"PENA {handle:02X}D"):
            return False
        
        response = self._get_response()
        if not response:
            print(f"  端口 {handle:02X} 启用失败: 无响应")
            return False
        
        resp_type = self._verify_response(response)
        if resp_type == REPLY_OKAY:
            # 重新获取端口信息以验证启用状态
            info = self.get_port_information(handle)
            if info and info.get('enabled', False):
                print(f"  端口 {handle:02X} 启用成功（已验证）")
                self.handle_info[handle] = info
                return True
            else:
                print(f"  端口 {handle:02X} 启用命令成功，但状态未更新")
                return True  # 命令成功，即使状态检查失败也返回成功
        else:
            print(f"  端口 {handle:02X} 启用失败: 响应类型 {resp_type}")
            return False
    
    def enable_all_ports(self) -> bool:
        """
        启用所有端口（参考CommandHandling.cpp的nEnableAllPorts）
        
        Returns:
            是否成功
        """
        handles = self.get_port_handles(3)  # 获取需要启用的句柄
        if not handles:
            print("没有需要启用的端口")
            return True
        
        success = True
        for handle in handles:
            if not self.enable_port(handle):
                success = False
            # 获取端口信息
            info = self.get_port_information(handle)
            if info:
                self.handle_info[handle] = info
        
        return success
    
    def start_tracking(self) -> bool:
        """
        开始跟踪（参考CommandHandling.cpp的nStartTracking）
        
        Returns:
            是否成功
        """
        if not self._send_command("TSTART "):
            return False
        
        response = self._get_response()
        if not response:
            return False
        
        resp_type = self._verify_response(response)
        if resp_type == REPLY_OKAY:
            print("开始跟踪")
            return True
        else:
            print(f"开始跟踪失败: {resp_type}")
            return False
    
    def stop_tracking(self) -> bool:
        """
        停止跟踪（参考CommandHandling.cpp的nStopTracking）
        
        Returns:
            是否成功
        """
        if not self._send_command("TSTOP "):
            return False
        
        response = self._get_response()
        if not response:
            return False
        
        resp_type = self._verify_response(response)
        if resp_type == REPLY_OKAY:
            print("停止跟踪")
            return True
        else:
            return False
    
    def _ascii_to_hex(self, s: str, length: int) -> int:
        """
        ASCII十六进制字符串转整数（参考Conversions.cpp的uASCIIToHex）
        
        Args:
            s: 十六进制字符串
            length: 字符串长度
            
        Returns:
            整数值
        """
        val = 0
        for i in range(length):
            if i >= len(s):
                break
            ch = s[i]
            if '0' <= ch <= '9':
                ch_val = ord(ch) - ord('0')
            elif 'A' <= ch <= 'F':
                ch_val = 10 + (ord(ch) - ord('A'))
            elif 'a' <= ch <= 'f':
                ch_val = 10 + (ord(ch) - ord('a'))
            else:
                return 0
            val |= (ch_val << (4 * (length - 1 - i)))
        return val
    
    def _extract_value(self, s: str, length: int, divisor: float) -> Optional[float]:
        """
        提取数值（参考Conversions.cpp的bExtractValue）
        
        Args:
            s: 数值字符串（格式：+/-后跟数字）
            length: 字符串长度
            divisor: 除数
            
        Returns:
            浮点数值，失败返回None
        """
        if len(s) < length:
            return None
        
        # 检查第一个字符是否为+或-（参考源代码第169行）
        if s[0] not in ['+', '-']:
            return None
        
        try:
            # 提取指定长度的字符串并转换为浮点数（参考源代码第198行）
            value_str = s[:length]
            # 验证剩余字符都是数字（参考源代码第183-191行）
            for i in range(1, length):
                if i >= len(value_str) or value_str[i] < '0' or value_str[i] > '9':
                    return None
            
            value = float(value_str) / divisor
            return value
        except (ValueError, IndexError, TypeError):
            return None
    
    def get_tx_transforms(self) -> Dict[int, Dict]:
        """
        获取TX变换数据（参考CommandHandling.cpp的nGetTXTransforms）
        
        Returns:
            字典，键为句柄，值为变换数据
        """
        # 使用TX命令获取ASCII格式的变换数据
        # 0001 = 返回所有句柄的数据（在体积内和体积外）
        if not self._send_command("TX 0001"):
            return {}
        
        response = self._get_response()
        if not response:
            return {}
        
        resp_type = self._verify_response(response, check_crc=False)  # TX响应可能不需要严格CRC检查
        if resp_type == REPLY_ERROR:
            print(f"  TX命令返回错误")
            return {}
        # 对于TX命令，即使不是OKAY也可能有数据，继续解析
        
        transforms = {}
        try:
            # 移除CRC和回车符
            cr_pos = response.find(chr(CARRIAGE_RETURN))
            if cr_pos >= 4:
                data = response[:cr_pos-4]
            else:
                data = response
            
            # 移除所有换行符和空白字符（TX响应可能包含换行符）
            # 但保留原始数据用于调试
            data_original = data
            data = data.replace('\n', '').replace('\r', '').replace(' ', '').replace('\t', '')
            
            if len(data) < 2:
                return {}
            
            # 调试：打印原始数据和处理后的数据
            if self.debug:
                print(f"  [TX响应] 原始数据: {repr(data_original)}")
                print(f"  [TX响应] 处理后数据: {repr(data)}")
                print(f"  [TX响应] 数据长度: {len(data)}")
            
            # 解析句柄数量（前2位十六进制）
            num_handles = self._ascii_to_hex(data[0:2], 2)
            pos = 2
            
            if self.debug:
                print(f"  [TX响应] 句柄数量: {num_handles}")
            
            for i in range(num_handles):
                if pos + 2 > len(data):
                    break
                
                # 获取句柄（2位十六进制）
                handle = self._ascii_to_hex(data[pos:pos+2], 2)
                pos += 2
                
                if self.debug:
                    print(f"  [TX解析] 句柄 {handle:02X} (第{i+1}个), 当前位置: {pos}, 剩余数据长度: {len(data) - pos}")
                    if len(data) - pos > 0:
                        print(f"  [TX解析] 剩余数据前60字符: {repr(data[pos:min(pos+60, len(data))])}")
                
                transform = {
                    'handle': handle,
                    'valid': False,
                    'status': 'UNKNOWN',
                    'rotation': {'q0': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0},
                    'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'error': 0.0,
                    'frame_number': 0
                }
                
                # 检查状态字符串（参考源代码第1505-1526行）
                if pos + 10 <= len(data) and data[pos:pos+10] == "UNOCCUPIED":
                    transform['status'] = 'UNOCCUPIED'
                    transform['valid'] = False
                    pos += 10
                    # print(f"    句柄 {handle:02X}: UNOCCUPIED")
                elif pos + 8 <= len(data) and data[pos:pos+8] == "DISABLED":
                    transform['status'] = 'DISABLED'
                    transform['valid'] = False
                    pos += 8
                    # print(f"    句柄 {handle:02X}: DISABLED")
                elif pos + 7 <= len(data) and data[pos:pos+7] == "MISSING":
                    transform['status'] = 'MISSING'
                    transform['valid'] = False
                    pos += 7
                    # print(f"    句柄 {handle:02X}: MISSING")
                else:
                    # 解析有效的变换数据（参考源代码第1538-1563行）
                    # 格式：q0(6) qx(6) qy(6) qz(6) x(7) y(7) z(7) error(6) = 51字符
                    if pos + 51 <= len(data):
                        transform['status'] = 'VALID'
                        transform['valid'] = True
                        
                        # 提取数据段用于调试
                        if self.debug:
                            data_segment = data[pos:pos+51]
                            print(f"    [TX解析] 句柄 {handle:02X} 数据段 (51字符): {repr(data_segment)}")
                            print(f"    [TX解析] q0: {repr(data[pos:pos+6])}, qx: {repr(data[pos+6:pos+12])}")
                            print(f"    [TX解析] x: {repr(data[pos+24:pos+31])}, y: {repr(data[pos+31:pos+38])}, z: {repr(data[pos+38:pos+45])}")
                        
                        # 四元数（每个6位，除以10000）
                        q0 = self._extract_value(data[pos:pos+6], 6, 10000.0)
                        qx = self._extract_value(data[pos+6:pos+12], 6, 10000.0)
                        qy = self._extract_value(data[pos+12:pos+18], 6, 10000.0)
                        qz = self._extract_value(data[pos+18:pos+24], 6, 10000.0)
                        
                        # 位置（每个7位，除以100）
                        x = self._extract_value(data[pos+24:pos+31], 7, 100.0)
                        y = self._extract_value(data[pos+31:pos+38], 7, 100.0)
                        z = self._extract_value(data[pos+38:pos+45], 7, 100.0)
                        
                        # 误差（6位，除以10000）
                        error = self._extract_value(data[pos+45:pos+51], 6, 10000.0)
                        
                        # 检查数据是否有效
                        if q0 is None or qx is None or qy is None or qz is None:
                            if self.debug:
                                print(f"    [TX解析] 句柄 {handle:02X}: 四元数解析失败")
                            transform['valid'] = False
                            transform['status'] = 'PARSE_ERROR'
                        else:
                            transform['rotation'] = {
                                'q0': q0,
                                'qx': qx,
                                'qy': qy,
                                'qz': qz
                            }
                            if self.debug:
                                print(f"    [TX解析] 句柄 {handle:02X}: 四元数 q0={q0:.4f}, qx={qx:.4f}, qy={qy:.4f}, qz={qz:.4f}")
                        
                        if x is None or y is None or z is None:
                            if self.debug:
                                print(f"    [TX解析] 句柄 {handle:02X}: 位置解析失败")
                            if transform['valid']:
                                transform['valid'] = False
                                transform['status'] = 'PARSE_ERROR'
                        else:
                            transform['translation'] = {
                                'x': x,
                                'y': y,
                                'z': z
                            }
                            if self.debug:
                                print(f"    [TX解析] 句柄 {handle:02X}: 位置 x={x:.3f}, y={y:.3f}, z={z:.3f}")
                        
                        if error is not None:
                            transform['error'] = error
                        
                        pos += 51
                        
                        # 解析句柄状态（8位十六进制，参考源代码第1568行）
                        if pos + 8 <= len(data):
                            handle_status = self._ascii_to_hex(data[pos:pos+8], 8)
                            transform['handle_status'] = handle_status
                            transform['enabled'] = (handle_status & 0x20) != 0
                            transform['out_of_volume'] = (handle_status & 0x40) != 0
                            pos += 8
                            
                            # 解析帧号（8位十六进制，参考源代码第1585-1587行）
                            if pos + 8 <= len(data):
                                transform['frame_number'] = self._ascii_to_hex(data[pos:pos+8], 8)
                                pos += 8
                        
                        # 跳过回车符（如果有，参考源代码第1589行）
                        if pos < len(data) and data[pos] == chr(CARRIAGE_RETURN):
                            pos += 1
                    else:
                        # print(f"    句柄 {handle:02X}: 数据不足，需要51字符，剩余 {len(data) - pos} 字符")
                        transform['status'] = 'INSUFFICIENT_DATA'
                        break
                
                transforms[handle] = transform
            
            # 调试：打印解析结果
            # for handle, t in transforms.items():
            #     print(f"  句柄 {handle:02X}: {t['status']}, valid={t['valid']}")
            
        except Exception as e:
            print(f"解析变换数据失败: {e}")
            import traceback
            traceback.print_exc()
        
        return transforms
    
    def get_sensor_status(self) -> Dict[int, bool]:
        """
        获取传感器状态（1、2、3、4号传感器）
        
        Returns:
            字典，键为传感器编号(1-4)，值为是否启用
        """
        status = {1: False, 2: False, 3: False, 4: False}
        
        # 获取所有已初始化的句柄
        handles = self.get_port_handles(0)  # 获取所有已初始化的句柄
        
        for handle in handles:
            info = self.get_port_information(handle)
            if info:
                self.handle_info[handle] = info
                # 尝试从物理端口信息中提取端口号
                physical_port = info.get('physical_port', '')
                
                # 方法1: 直接是数字字符串
                try:
                    port_num = int(physical_port)
                    if 1 <= port_num <= 4:
                        status[port_num] = info.get('enabled', False)
                        continue
                except (ValueError, TypeError):
                    pass
                
                # 方法2: 十六进制格式
                try:
                    port_num = int(physical_port, 16)
                    if 1 <= port_num <= 4:
                        status[port_num] = info.get('enabled', False)
                        continue
                except (ValueError, TypeError):
                    pass
                
                # 方法3: 从句柄顺序映射（如果物理端口信息不可用）
                # 按句柄在列表中的顺序映射到端口号
                if not physical_port or physical_port == "":
                    # 根据句柄在列表中的位置推断端口号
                    try:
                        handle_index = handles.index(handle)
                        port_num = handle_index + 1  # 第一个句柄对应端口1
                        if 1 <= port_num <= 4:
                            status[port_num] = info.get('enabled', False)
                            # 更新句柄信息中的物理端口
                            info['physical_port'] = str(port_num)
                            self.handle_info[handle] = info
                            continue
                    except (ValueError, IndexError):
                        pass
        
        return status
    
    def enable_sensor_by_port(self, port_number: int) -> bool:
        """
        根据物理端口号启用传感器（1、2、3、4）
        
        Args:
            port_number: 物理端口号（1-4）
            
        Returns:
            是否成功启用
        """
        if port_number < 1 or port_number > 4:
            print(f"无效的端口号: {port_number}，必须是1-4")
            return False
        
        print(f"\n启用传感器 {port_number}...")
        
        # 获取所有已初始化的句柄（PHSR 00）
        handles = self.get_port_handles(0)
        print(f"  找到 {len(handles)} 个句柄: {[f'{h:02X}' for h in handles]}")
        
        # 查找对应物理端口号的句柄
        target_handle = None
        for handle in handles:
            info = self.get_port_information(handle)
            if info:
                self.handle_info[handle] = info
                physical_port = info.get('physical_port', '')
                enabled = info.get('enabled', False)
                initialized = info.get('initialized', False)
                
                print(f"  句柄 {handle:02X}: 物理端口={physical_port}, 已初始化={initialized}, 已启用={enabled}")
                
                # 尝试匹配端口号
                try:
                    port_num = int(physical_port)
                    if port_num == port_number:
                        target_handle = handle
                        print(f"  找到匹配的句柄 {handle:02X} 对应端口 {port_number}")
                        break
                except (ValueError, TypeError):
                    try:
                        port_num = int(physical_port, 16)
                        if port_num == port_number:
                            target_handle = handle
                            print(f"  找到匹配的句柄 {handle:02X} 对应端口 {port_number}")
                            break
                    except (ValueError, TypeError):
                        pass
        
        if target_handle is None:
            print(f"  未找到端口 {port_number} 对应的句柄，尝试使用顺序映射...")
            # 如果无法通过物理端口找到，使用顺序映射
            # 按顺序映射：第一个句柄->端口1，第二个句柄->端口2，以此类推
            if len(handles) >= port_number:
                target_handle = handles[port_number - 1]
                print(f"  ✓ 使用顺序映射: 端口 {port_number} -> 句柄 {target_handle:02X} (列表中的第{port_number}个)")
            else:
                print(f"  ✗ 无法找到端口 {port_number} 对应的句柄")
                print(f"  可用句柄: {[f'{h:02X}' for h in handles]}")
                return False
        
        # 启用该句柄（会自动检查并初始化）
        success = self.enable_port(target_handle, check_initialized=True)
        
        if success:
            # 等待一下让硬件响应
            time.sleep(0.5)
            # 再次验证状态
            info = self.get_port_information(target_handle)
            if info:
                if info.get('enabled', False):
                    print(f"✓ 传感器 {port_number} (句柄 {target_handle:02X}) 已成功启用")
                    return True
                else:
                    print(f"⚠ 传感器 {port_number} 启用命令已发送，但状态显示未启用")
                    return True  # 命令成功就算成功
            else:
                print(f"⚠ 无法验证传感器 {port_number} 的状态")
                return True  # 命令成功就算成功
        
        return False
    
    def enable_sensors(self, port_numbers: List[int]) -> bool:
        """
        启用多个传感器
        
        Args:
            port_numbers: 端口号列表，例如 [1, 2]
            
        Returns:
            是否全部成功启用
        """
        success = True
        for port_num in port_numbers:
            print(f"\n启用传感器 {port_num}...")
            if self.enable_sensor_by_port(port_num):
                print(f"传感器 {port_num} 启用成功")
            else:
                print(f"传感器 {port_num} 启用失败")
                success = False
        
        return success
    
    def print_sensor_status(self):
        """打印传感器状态"""
        status = self.get_sensor_status()
        print("\n=== 传感器状态 ===")
        for sensor_num in [1, 2, 3, 4]:
            state = "开启" if status[sensor_num] else "关闭"
            print(f"传感器 {sensor_num}: {state}")
        print("==================\n")
    
    def calculate_relative_position(self, pos1: Dict[str, float], pos2: Dict[str, float]) -> Dict[str, float]:
        """
        计算两个传感器之间的相对位置（差值）
        
        Args:
            pos1: 第一个传感器的位置字典 {'x': float, 'y': float, 'z': float}
            pos2: 第二个传感器的位置字典 {'x': float, 'y': float, 'z': float}
            
        Returns:
            相对位置字典 {'x': float, 'y': float, 'z': float, 'distance': float}
        """
        dx = pos2.get('x', 0) - pos1.get('x', 0)
        dy = pos2.get('y', 0) - pos1.get('y', 0)
        dz = pos2.get('z', 0) - pos1.get('z', 0)
        distance = (dx**2 + dy**2 + dz**2)**0.5
        
        return {
            'x': dx,
            'y': dy,
            'z': dz,
            'distance': distance
        }
    
    def start_recording(self, filename: Optional[str] = None) -> bool:
        """
        开始记录坐标数据到文件
        
        Args:
            filename: 记录文件名，如果为None则自动生成带时间戳的文件名
            
        Returns:
            是否成功开始记录
        """
        try:
            if filename is None:
                # 自动生成文件名：track_record_YYYYMMDD_HHMMSS.csv
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"track_record_{timestamp}.csv"
            
            # 确保文件扩展名是.csv
            if not filename.endswith('.csv'):
                filename += '.csv'
            
            # 打开文件并创建CSV写入器
            self.record_file = open(filename, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.record_file)
            
            # 写入CSV表头
            self.csv_writer.writerow([
                '时间戳', '帧号', 
                '传感器1_端口', '传感器1_X', '传感器1_Y', '传感器1_Z',
                '传感器2_端口', '传感器2_X', '传感器2_Y', '传感器2_Z',
                '相对位置_X', '相对位置_Y', '相对位置_Z', '距离'
            ])
            
            self.record_enabled = True
            print(f"开始记录数据到文件: {filename}")
            return True
        except Exception as e:
            print(f"开始记录失败: {e}")
            return False
    
    def stop_recording(self):
        """停止记录并关闭文件"""
        if self.record_file:
            self.record_file.close()
            self.record_file = None
            self.csv_writer = None
            self.record_enabled = False
            print("记录已停止，文件已保存")
    
    def record_frame_data(self, frame_count: int, valid_sensors: List[Dict], relative: Optional[Dict[str, float]] = None):
        """
        记录一帧的数据到文件
        
        Args:
            frame_count: 帧号
            valid_sensors: 有效的传感器数据列表
            relative: 相对位置数据（如果有两个传感器）
        """
        if not self.record_enabled or not self.csv_writer:
            return
        
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 精确到毫秒
            
            # 准备数据行
            row = [timestamp, frame_count]
            
            # 传感器1数据
            if len(valid_sensors) >= 1:
                sensor1 = valid_sensors[0]
                pos1 = sensor1['position']
                row.extend([
                    sensor1['port'],
                    f"{pos1['x']:.6f}",
                    f"{pos1['y']:.6f}",
                    f"{pos1['z']:.6f}"
                ])
            else:
                row.extend(['', '', '', ''])
            
            # 传感器2数据
            if len(valid_sensors) >= 2:
                sensor2 = valid_sensors[1]
                pos2 = sensor2['position']
                row.extend([
                    sensor2['port'],
                    f"{pos2['x']:.6f}",
                    f"{pos2['y']:.6f}",
                    f"{pos2['z']:.6f}"
                ])
            else:
                row.extend(['', '', '', ''])
            
            # 相对位置数据
            if relative:
                row.extend([
                    f"{relative['x']:.6f}",
                    f"{relative['y']:.6f}",
                    f"{relative['z']:.6f}",
                    f"{relative['distance']:.6f}"
                ])
            else:
                row.extend(['', '', '', ''])
            
            # 写入CSV文件
            self.csv_writer.writerow(row)
            self.record_file.flush()  # 立即刷新到磁盘
            
        except Exception as e:
            print(f"记录数据失败: {e}")
    
    def run_tracking_loop(self):
        """运行跟踪循环，实时显示两个传感器的相对位置"""
        print("\n开始跟踪循环，按Ctrl+C停止...")
        print("=" * 70)
        
        frame_count = 0
        
        try:
            while True:
                frame_count += 1
                
                # 获取变换数据
                transforms = self.get_tx_transforms()
                
                # 提取有效的传感器数据
                valid_sensors = []
                for handle, transform in transforms.items():
                    if transform.get('valid', False):
                        # 查找对应的物理端口号
                        port_info = self.handle_info.get(handle, {})
                        physical_port = port_info.get('physical_port', '')
                        
                        # 如果物理端口未知，尝试从句柄列表推断
                        if not physical_port or physical_port == '?':
                            all_handles = list(self.handle_info.keys())
                            if handle in all_handles:
                                try:
                                    port_index = all_handles.index(handle)
                                    physical_port = str(port_index + 1)
                                except (ValueError, IndexError):
                                    physical_port = f"句柄{handle:02X}"
                        
                        valid_sensors.append({
                            'port': physical_port,
                            'handle': handle,
                            'position': transform.get('translation', {'x': 0, 'y': 0, 'z': 0})
                        })
                
                # 计算相对位置（如果有两个或更多传感器）
                relative = None
                if len(valid_sensors) >= 2:
                    pos1 = valid_sensors[0]['position']
                    pos2 = valid_sensors[1]['position']
                    relative = self.calculate_relative_position(pos1, pos2)
                
                # 记录数据到文件
                self.record_frame_data(frame_count, valid_sensors, relative)
                
                # 显示相对位置（如果有两个或更多传感器）
                if len(valid_sensors) >= 2:
                    print(f"[帧 {frame_count:5d}] 传感器{valid_sensors[0]['port']} -> 传感器{valid_sensors[1]['port']}: "
                          f"ΔX={relative['x']:8.3f} mm, "
                          f"ΔY={relative['y']:8.3f} mm, "
                          f"ΔZ={relative['z']:8.3f} mm, "
                          f"距离={relative['distance']:8.3f} mm")
                elif len(valid_sensors) == 1:
                    pos = valid_sensors[0]['position']
                    print(f"[帧 {frame_count:5d}] 传感器{valid_sensors[0]['port']}: "
                          f"X={pos['x']:8.3f} mm, Y={pos['y']:8.3f} mm, Z={pos['z']:8.3f} mm "
                          f"(仅一个传感器，无法计算相对位置)")
                else:
                    if frame_count % 20 == 0:  # 每20帧显示一次提示
                        print(f"[帧 {frame_count:5d}] 等待传感器数据...")
                
                time.sleep(0.05)  # 50ms更新一次（约20Hz）
                
        except KeyboardInterrupt:
            print("\n\n停止跟踪...")
            self.stop_tracking()
            self.stop_recording()


def main():
    """主函数"""
    print("=" * 70)
    print("NDI电磁传感器跟踪器 - 相对位置显示")
    print("=" * 70)
    
    # 创建跟踪器实例（debug=False关闭详细调试信息）
    tracker = NDISensorTracker(port="COM9", baudrate=9600, debug=False)
    
    try:
        # 连接
        if not tracker.connect():
            print("无法连接到传感器，请检查串口设置")
            return
        
        # 硬件复位
        print("执行硬件复位...", end=" ")
        if not tracker.hardware_reset():
            print("警告: 硬件复位可能失败，继续尝试初始化...")
        else:
            print("完成")
        
        # 初始化系统
        print("初始化系统...", end=" ")
        if not tracker.initialize_system():
            print("失败")
            return
        else:
            print("完成")
        
        # 询问用户要启用哪些传感器
        print("\n" + "=" * 70)
        print("请输入要启用的传感器编号（用逗号分隔，例如: 1,2）:")
        try:
            user_input = input().strip()
            if user_input:
                port_numbers = [int(x.strip()) for x in user_input.split(',') if x.strip().isdigit()]
                if port_numbers:
                    print(f"\n启用传感器: {port_numbers}")
                    tracker.enable_sensors(port_numbers)
                else:
                    print("未输入有效的端口号")
                    return
            else:
                print("未输入端口号")
                return
        except (ValueError, KeyboardInterrupt):
            print("输入无效或取消")
            return
        
        # 开始跟踪
        print("开始跟踪...", end=" ")
        if not tracker.start_tracking():
            print("失败")
            return
        else:
            print("完成")
        
        # 开始记录数据
        print("开始记录数据...", end=" ")
        if tracker.start_recording():
            print("完成")
        else:
            print("失败（继续运行但不记录）")
        
        # 运行跟踪循环
        tracker.run_tracking_loop()
        
    except Exception as e:
        print(f"发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理
        tracker.stop_tracking()
        tracker.stop_recording()
        tracker.disconnect()
        print("程序结束")


if __name__ == "__main__":
    main()

