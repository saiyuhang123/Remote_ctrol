# GQ-OTK9633 串口通信和控制

使用 `gq_otk9633.py`，这是可以单独复制运行的Python文件。
本文件按本次提供的型号说明书第6、7页编写。此前的 `gas_detector_v6.py`
针对最初的V6文档，单位、状态与控制寄存器有区别，这台设备请使用新文件。

## 读取

本机为 Ubuntu 22.04、Python 3.10，已安装 pyserial 3.5。
其他 Ubuntu 电脑安装：`sudo apt install python3-serial`。
默认地址1、2400波特率、8N1，完整响应超时0.5秒。
串口选择顺序：显式 `--port`、环境变量 `GQ_OTK9633_PORT`、
固定设备名 `/dev/gq_otk9633`，最后自动识别 USB 串口。
自动识别优先选择本机使用的 CH343（1a86:55d3）；候选不唯一时提示指定串口。
本机转换器目前为 `/dev/ttyACM0`，固定设备名不受 tty 编号变化影响。
`/dev/ttyUSB1` 是正在使用的 IMU，请使用气体检测仪的固定设备名。
Linux 打开串口时启用独占锁，避免多个本脚本实例同时访问。
每条请求之间至少等待210ms，满足型号说明书的采样间隔要求。

```bash
# 查看串口
python3 gq_otk9633.py ports

# 发送说明书的实时值读取指令，输出原始数值
python3 gq_otk9633.py --debug read

# 显式指定Ubuntu固定串口
python3 gq_otk9633.py --port /dev/gq_otk9633 --debug read

# 指定地址读取，如地址2
python3 gq_otk9633.py --address 2 --debug read

# 地址扫描，只读；找到第一个响应便结束
python3 gq_otk9633.py scan --first

# 读取完整参数，或连续采集
python3 gq_otk9633.py snapshot
python3 gq_otk9633.py watch --interval 1

# 有限次数采集，完成后自动退出
python3 gq_otk9633.py watch --interval 1 --count 3
```

`read`的返回值是原始寄存器整数。`snapshot`中的 `raw_value` 是原始检测值，
`value_scaled` 根据小数位高字节计算 `raw_value / 10**decimals`。
照片没有明确给出这个缩放公式，因此该字段是待设备显示值核对的推算值，
`decimal_scaling_verified` 固定为false；不要未经核对就用于报警或控制。
照片也未说明负数编码，因此保留无符号原始值，不套用V6的符号位解析。

## 参数控制

以下命令均会修改设备。数值参数以**原始寄存器整数**输入，与说明书示例一致。
低报、高报和量程设置会先读取关联参数，要求 `0 <= 低报 <= 高报 <= 量程`。

```bash
python3 gq_otk9633.py set low_alarm 40
python3 gq_otk9633.py set high_alarm 80
python3 gq_otk9633.py set range 1000

# 单位：0=%LEL，1=ppm，3=%VOL，5=ug/m3
python3 gq_otk9633.py unit 1

# 小数位：1对应说明书示例的0.0显示
python3 gq_otk9633.py decimals 1

# 将设备地址改为2，成功应答后实例切换到新地址
python3 gq_otk9633.py address 2
```

校零和标定是独立命令，采集、扫描和普通设置都不会触发它们：

```bash
# 正确零气条件下执行，写入0x000D=0x00AA
python3 gq_otk9633.py zero

# 通入与设备量程及气体类型匹配的标准气体后执行
# 示例写入0x0010=60；这不是零点或幅值偏移寄存器
python3 gq_otk9633.py calibrate 60
```

`write REGISTER VALUE` 可对说明书明确的可写寄存器发送原始06指令，
不做低报、高报和量程之间的关联检查。参数支持十进制或 `0x` 前缀的十六进制。

## 回复校验与说明书歧义

CRC按标准Modbus RTU低字节在前发送，读取实时值指令为：
`01 03 00 00 00 01 84 0A`。

写入默认要求标准06回显。照片中多个写入回复将功能码写为03，
却仍沿用06请求的CRC；例如低报请求为 `01 06 00 07 00 28 38 15`，
其03版本的正确CRC应重新计算，不能把印刷示例直接视为有效响应。
若真实设备返回**CRC有效**的03写入回显，可显式启用：

```bash
python3 gq_otk9633.py --allow-write03 set low_alarm 40
```

这个兼容选项仍严格检查CRC、寄存器地址和写入值。
修改地址时允许旧地址或新地址应答，但应答CRC必须与实际内容一致。
写入超时可能已部分生效，程序不自动重试；需读取或检查设备端设置确认。
此前 Windows 测试记录：2026-10-10 已在 COM10、2400/8N1、地址1完成实物读取验证。
连续读取10次、重新打开串口3次和读取全部7个已知参数均成功。
当时读取到：实时值0、状态正常、单位%LEL、小数位1、量程原始值1000、
低报原始值150、高报原始值150。控制指令尚未对设备执行。

修复记录：接收循环不再动态修改 `serial.timeout`。Windows版pyserial修改
该属性时会重新配置串口，在本次CH343设备上导致回复接收失败。
程序在打开串口时一次设置20ms以内的读取等待，整帧超时通过时间期限控制；
实测原接收流程超时，取消接收中的重新配置后连续读取成功。

## Python调用

```python
from gq_otk9633 import GQOTK9633

with GQOTK9633("/dev/gq_otk9633", address=1, timeout=0.5) as detector:
    print(detector.read_registers(0))
    print(detector.snapshot())
    # 按实际需求显式调用：
    # detector.set_parameter("low_alarm", 40)
    # detector.set_unit(1)
```

一个串口只用一个实例，内部串行处理请求。若打开串口失败，请关闭占用串口的软件。
如果发送灯闪而没有任何回复，继续核对实际地址、接线、接口配置和转换器型号。


## Ubuntu 串口权限与固定设备名

随附 `gq_otk9633.rules` 仅匹配本台 485 转换器的 VID/PID 和序列号
`5CDB375485`，不会匹配 IMU。已在本机安装到
`/etc/udev/rules.d/70-gq-otk9633.rules`，并将 `jkbot` 加入 `dialout` 组。
当前设备也已为 `jkbot` 配置读写 ACL，因此当前会话可直接运行，无需 sudo。
组成员变更需注销并重新登录后才对其他新插入的串口生效。
在其他 Ubuntu 电脑上，进入本目录后执行：

```bash
sudo install -m 0644 gq_otk9633.rules /etc/udev/rules.d/70-gq-otk9633.rules
sudo usermod -aG dialout "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty
```

随后注销并重新登录。更换转换器后，应修改规则中的 VID/PID、序列号，
或通过 `--port /dev/serial/by-id/...` 指定实际设备。


## 本次 Ubuntu 实物测试（2026-10-10）

在 `/dev/gq_otk9633`（实际 `/dev/ttyACM0`）、地址1、2400/8N1 上：
连续读取实时值10次成功，重新打开串口3次成功，全部7个已知参数读取成功，
`watch --count 3` 连续完整采集3次成功并自动退出，全部响应均通过CRC检查。
实时原始值0、状态正常、单位%LEL、小数位1；量程原始值1000，
低报和高报原始值均为150。缩放值仍需与仪表显示核对。
本次测试只执行读取，没有修改仪表参数或执行校准。
详细记录见 `gq_otk9633_ubuntu_test_result.json`。
