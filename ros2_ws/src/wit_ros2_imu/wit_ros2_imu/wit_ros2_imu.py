import math
import serial
import struct
import numpy as np
import rclpy
import threading
from enum import Enum
from rclpy.node import Node
from sensor_msgs.msg import Imu
from imu_msg.msg import ImuData


class protocolType(Enum):
    TTL_STD = 1          # TTL-标准精度
    TTL_HIGH = 2         # TTL-高精度
    CAN_STD = 3          # CAN-标准精度
    CAN_HIGH = 4         # CAN-高精度
    RS485_STD = 5        # RS485-标准精度
    RS485_HIGH = 6       # RS485-高精度


class imuDriverNode(Node):
    def __init__(self):
        super().__init__("imuDriverNode")

        # 参数
        self.declare_parameter("port", "/dev/imu_usb")
        self.declare_parameter("baudrate", 230400)
        self.declare_parameter("protocol", "RS485_HIGH")
        self.declare_parameter("modbusID", 0x50)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.modbusID = self.get_parameter("modbusID").value
        protocolStr = self.get_parameter("protocol").value

        self.protocol = protocolType[protocolStr]

        # 串口 - 阻塞模式：独立线程 read() 无数据时休眠，不占 CPU
        self.serialPort = serial.Serial(
            self.port,
            self.baudrate,
            timeout=None,
            write_timeout=0
        )

        self.get_logger().info(f"Serial opened: {self.port} @ {self.baudrate}")

        self.modbusAddrList = [0x34, 0x37, 0x3A, 0x3D]
        self.modbusIndex = 0

        # 接收缓存 (仅 TTL/CAN 使用)
        self.rxBuffer = bytearray()

        # 数据缓存 (读写线程安全，由 dataLock 保护)
        self.dataLock = threading.Lock()
        self.accData = np.zeros(3)
        self.gyroData = np.zeros(3)
        self.angleData = np.zeros(3)
        self.magData = np.zeros(3)
        self.currentRequestAddr = None      # 标记 485 当前询问地址

        # 处理后的数据
        self.acc = np.zeros(3)
        self.gyro = np.zeros(3)
        self.angle = np.zeros(3)
        self.mag = np.zeros(3)

        # 发布器
        self.imuPublisher = self.create_publisher(Imu, "imu/data", 10)
        self.imuRpyPublisher = self.create_publisher(ImuData, "imu/ImuDataWithRPY", 10)

        # 定时器
        self.create_timer(1.0 / 100, self.publishTimerCallback)  # 200Hz 发布
        self.create_timer(0.1, self.printMsg)       # 终端打印

        # 启动串口阻塞读取线程
        self.readerRunning = True
        self.readerThread = threading.Thread(target=self.serialReaderThread, daemon=True)
        self.readerThread.start()
        self.get_logger().info("Serial reader thread started")

    # ==========================================================
    # 串口读取线程 (独立线程，阻塞读取)
    # ==========================================================
    def serialReaderThread(self):
        """独立线程：阻塞读取串口，替代原来的 1kHz 轮询定时器"""
        try:
            if self.protocol in (protocolType.TTL_STD, protocolType.TTL_HIGH,
                                 protocolType.CAN_STD, protocolType.CAN_HIGH):
                self._readTTLCAN()
            else:
                self._readRS485()
        except (serial.SerialException, OSError):
            if self.readerRunning:
                self.get_logger().error("Serial error in reader thread")
        except Exception as e:
            self.get_logger().error(f"Reader thread error: {e}")

    def _readTTLCAN(self):
        """TTL/CAN 模式：阻塞读取连续数据流"""
        while self.readerRunning:
            # 阻塞等待至少 1 字节（无数据时休眠，不消耗 CPU）
            data = self.serialPort.read(1)
            if not data:
                continue

            # 非阻塞读取剩余可用字节，一次系统调用拿走所有数据
            if self.serialPort.in_waiting > 0:
                data += self.serialPort.read(self.serialPort.in_waiting)

            self.rxBuffer.extend(data)
            self.parseBuffer()

    def _readRS485(self):
        """RS485 模式：发送 Modbus 命令 → 阻塞读取响应 → 下一个地址"""
        while self.readerRunning:
            # --- 发送 Modbus 命令 ---
            addr = self.modbusAddrList[self.modbusIndex]
            self.currentRequestAddr = addr

            if self.protocol == protocolType.RS485_HIGH and addr == 0x3D:
                cmd = self.buildModbusReadCmd(addr, 6)
            else:
                cmd = self.buildModbusReadCmd(addr, 3)

            self.serialPort.write(cmd)

            # --- 临时设置超时，等待响应 ---
            self.serialPort.timeout = 0.05  # 50ms

            try:
                # 读取响应头（ID + 功能码 + 字节数）
                header = self.serialPort.read(3)
                if len(header) < 3:
                    # 超时，无响应，跳到下一个地址
                    self.modbusIndex = (self.modbusIndex + 1) % len(self.modbusAddrList)
                    continue

                # 验证 ID 和功能码
                if header[0] != self.modbusID or header[1] != 0x03:
                    self.serialPort.reset_input_buffer()
                    self.modbusIndex = (self.modbusIndex + 1) % len(self.modbusAddrList)
                    continue

                byteCount = header[2]
                remaining = byteCount + 2  # 数据 + CRC

                # 读取数据和 CRC
                tail = self.serialPort.read(remaining)
                if len(tail) < remaining:
                    # 超时，跳到下一个地址
                    self.modbusIndex = (self.modbusIndex + 1) % len(self.modbusAddrList)
                    continue

                frame = header + tail

                # CRC 校验
                recvCRC = (frame[-2] << 8) | frame[-1]
                calcCRC = self.modbusCRC(frame[:-2])
                calcCRC = ((calcCRC & 0xFF) << 8) | ((calcCRC >> 8) & 0xFF)

                if recvCRC == calcCRC:
                    self.handleModbusFrame(frame)
                # CRC 错误：忽略，进入下一个地址
                else:
                    self.modbusIndex = (self.modbusIndex + 1) % len(self.modbusAddrList)

            finally:
                # 恢复阻塞模式
                self.serialPort.timeout = None

    # ==========================================================
    # buffer解析 (仅 TTL/CAN 使用)
    # ==========================================================
    def parseBuffer(self):
        while True:
            if self.protocol in (protocolType.TTL_STD, protocolType.TTL_HIGH):
                if len(self.rxBuffer) < 11:
                    return

                if self.rxBuffer[0] != 0x55:
                    self.rxBuffer.pop(0)
                    continue

                frame = self.rxBuffer[:11]
                del self.rxBuffer[:11]

                if not self.ttlChecksum(frame):
                    continue

                self.handleTTLFrame(frame)

            elif self.protocol in (protocolType.CAN_STD, protocolType.CAN_HIGH):
                if len(self.rxBuffer) < 8:
                    return

                if self.rxBuffer[0] != 0x55:
                    self.rxBuffer.pop(0)
                    continue

                frame = self.rxBuffer[:8]
                del self.rxBuffer[:8]

                self.handleCanFrame(frame)

    # ==========================================================
    # TTL处理
    # ==========================================================
    def handleTTLFrame(self, frame):
        dataType = frame[1]

        if self.protocol == protocolType.TTL_STD:
            values = struct.unpack("<hhh", frame[2:8])

            with self.dataLock:
                if dataType == 0x51:
                    self.accData = np.array(values)
                elif dataType == 0x52:
                    self.gyroData = np.array(values)
                elif dataType == 0x53:
                    self.angleData = np.array(values)
                elif dataType == 0x54:
                    self.magData = np.array(values)
        else:
            # 普通 16bit 数据（acc/gyro/mag）
            if dataType in (0x51, 0x52, 0x54):
                values = struct.unpack("<hhh", frame[2:8])

                with self.dataLock:
                    if dataType == 0x51:
                        self.accData = np.array(values)
                    elif dataType == 0x52:
                        self.gyroData = np.array(values)
                    elif dataType == 0x54:
                        self.magData = np.array(values)

            # 高精度 32bit 数据
            elif dataType == 0x53:
                low16 = struct.unpack("<H", frame[4:6])[0]      # 读取低 16bit
                high16 = struct.unpack("<h", frame[6:8])[0]      # 读取高 16bit
                angle32 = (high16 << 16) | low16

                # 写入对应轴
                axis = frame[2]
                with self.dataLock:
                    if axis == 0x01:
                        self.angleData[0] = angle32
                    elif axis == 0x02:
                        self.angleData[1] = angle32
                    elif axis == 0x03:
                        self.angleData[2] = angle32


    # ==========================================================
    # CAN处理
    # ==========================================================
    def handleCanFrame(self, frame):
        dataType = frame[1]

        if self.protocol == protocolType.CAN_STD:
            values = struct.unpack("<hhh", frame[2:8])

            with self.dataLock:
                if dataType == 0x51:
                    self.accData = np.array(values)
                elif dataType == 0x52:
                    self.gyroData = np.array(values)
                elif dataType == 0x53:
                    self.angleData = np.array(values)
                elif dataType == 0x54:
                    self.magData = np.array(values)
        else:
            # 普通 16bit 数据（acc/gyro/mag）
            if dataType in (0x51, 0x52, 0x54):
                values = struct.unpack("<hhh", frame[2:8])

                with self.dataLock:
                    if dataType == 0x51:
                        self.accData = np.array(values)
                    elif dataType == 0x52:
                        self.gyroData = np.array(values)
                    elif dataType == 0x54:
                        self.magData = np.array(values)

            # 高精度 32bit 数据
            elif dataType == 0x53:
                low16 = struct.unpack("<H", frame[4:6])[0]      # 读取低 16bit
                high16 = struct.unpack("<h", frame[6:8])[0]      # 读取高 16bit
                angle32 = (high16 << 16) | low16

                # 写入对应轴
                axis = frame[2]
                with self.dataLock:
                    if axis == 0x01:
                        self.angleData[0] = angle32
                    elif axis == 0x02:
                        self.angleData[1] = angle32
                    elif axis == 0x03:
                        self.angleData[2] = angle32


    # ==========================================================
    # RS485处理 (在 reader 线程中直接调用)
    # ==========================================================
    def handleModbusFrame(self, frame):
        if len(frame) < 5:
            self.modbusIndex = (self.modbusIndex + 1) % len(self.modbusAddrList)
            return

        byteCount = frame[2]
        data = frame[3:3+byteCount]

        addr = self.currentRequestAddr

        if self.protocol == protocolType.RS485_STD:
            if len(data) != 6:
                self.modbusIndex = (self.modbusIndex + 1) % len(self.modbusAddrList)
                return

            values = struct.unpack(">hhh", data)

            with self.dataLock:
                if addr == 0x34:
                    self.accData = np.array(values)
                elif addr == 0x37:
                    self.gyroData = np.array(values)
                elif addr == 0x3A:
                    self.magData = np.array(values)
                elif addr == 0x3D:
                    self.angleData = np.array(values)
        else:
            if addr == 0x3D:  # 角度
                if len(data) != 12:
                    self.modbusIndex = (self.modbusIndex + 1) % len(self.modbusAddrList)
                    return

                def parse32(offset):
                    raw = (
                        (data[offset+2] << 24) |
                        (data[offset+3] << 16) |
                        (data[offset+0] << 8)  |
                        data[offset+1]
                    )
                    if raw & 0x80000000:
                        raw -= 0x100000000
                    return raw

                angleX = parse32(0)
                angleY = parse32(4)
                angleZ = parse32(8)

                with self.dataLock:
                    self.angleData = np.array([angleX, angleY, angleZ])
            else:
                if len(data) != 6:
                    self.modbusIndex = (self.modbusIndex + 1) % len(self.modbusAddrList)
                    return

                values = struct.unpack(">hhh", data)
                with self.dataLock:
                    if addr == 0x34:
                        self.accData = np.array(values)
                    elif addr == 0x37:
                        self.gyroData = np.array(values)
                    elif addr == 0x3A:
                        self.magData = np.array(values)

        self.modbusIndex = (self.modbusIndex + 1) % len(self.modbusAddrList)

    # ==========================================================
    # 定时发布回调
    # ==========================================================
    def publishTimerCallback(self):
        self.publishImu()

    # ==========================================================
    # 发布IMU
    # ==========================================================
    def publishImu(self):

        accScale = 16.0 / 32768.0 * 9.80665   # g → m/s² 改为 m/s²
        gyroScale = 2000.0 / 32768.0

        with self.dataLock:
            accData = self.accData.copy()
            gyroData = self.gyroData.copy()
            angleData = self.angleData.copy()
            magData = self.magData.copy()

        ax, ay, az = accData * accScale
        gx, gy, gz = np.radians(gyroData * gyroScale)
        mx, my, mz = magData

        if self.protocol in (protocolType.TTL_HIGH, protocolType.CAN_HIGH, protocolType.RS485_HIGH):
            roll, pitch, yaw = np.radians(angleData / 1000.0)
        else:
            angleScale = 180.0 / 32768.0
            roll, pitch, yaw = np.radians(angleData * angleScale)

        self.acc = (ax, ay, az)
        self.gyro = (gx, gy, gz)
        self.angle = (roll, pitch, yaw)
        self.mag = (mx, my, mz)

        quaternion = self.eulerToQuaternion(roll, pitch, yaw)

        imuMsg = Imu()
        imuMsg.header.stamp = self.get_clock().now().to_msg()
        imuMsg.header.frame_id = "imu_link"

        imuMsg.linear_acceleration.x = float(ax)
        imuMsg.linear_acceleration.y = float(ay)
        imuMsg.linear_acceleration.z = float(az)

        imuMsg.angular_velocity.x = float(gx)
        imuMsg.angular_velocity.y = float(gy)
        imuMsg.angular_velocity.z = float(gz)

        imuMsg.orientation.x = quaternion[0]
        imuMsg.orientation.y = quaternion[1]
        imuMsg.orientation.z = quaternion[2]
        imuMsg.orientation.w = quaternion[3]

        # 协方差未知

        imuMsg.angular_velocity_covariance[8]= 0.02


        self.imuPublisher.publish(imuMsg)

        imuRpyMsg = ImuData()
        imuRpyMsg.header = imuMsg.header
        imuRpyMsg.imu = imuMsg
        imuRpyMsg.roll = math.degrees(roll)
        imuRpyMsg.pitch = math.degrees(pitch)
        imuRpyMsg.yaw = math.degrees(yaw)

        self.imuRpyPublisher.publish(imuRpyMsg)

    # ==========================================================
    # 工具函数
    # ==========================================================
    def buildModbusReadCmd(self, addr, length):
        frame = bytearray()
        frame.append(self.modbusID)
        frame.append(0x03)
        frame.append((addr >> 8) & 0xFF)
        frame.append(addr & 0xFF)
        frame.append((length >> 8) & 0xFF)
        frame.append(length & 0xFF)

        crc = self.modbusCRC(frame)
        frame.append(crc & 0xFF)          # CRC_L
        frame.append((crc >> 8) & 0xFF)   # CRC_H

        return bytes(frame)

    @staticmethod
    def ttlChecksum(frame):
        return (sum(frame[0:10]) & 0xFF) == frame[10]

    @staticmethod
    def modbusCRC(data: bytes):
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    @staticmethod
    def eulerToQuaternion(roll, pitch, yaw):

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return [qx, qy, qz, qw]

    def printMsg(self):
        print(f"Accel: x={self.acc[0]:+12.3f} | y={self.acc[1]:+12.3f} | z={self.acc[2]:+12.3f} m/s²")
        print(f"Gyro:  x={math.degrees(self.gyro[0]):+12.3f} | y={math.degrees(self.gyro[1]):+12.3f} | z={math.degrees(self.gyro[2]):+12.3f} °/s")
        print(f"Angle: x={math.degrees(self.angle[0]):+12.3f} | y={math.degrees(self.angle[1]):+12.3f} | z={math.degrees(self.angle[2]):+12.3f} °")
        print(f"Mag:   x={self.mag[0]:+12.3f} | y={self.mag[1]:+12.3f} | z={self.mag[2]:+12.3f}")
        print("\033[H\033[J", end="")       # 清屏

    def shutdown(self):
        self.readerRunning = False
        if self.serialPort and self.serialPort.is_open:
            self.serialPort.close()           # 关闭串口使阻塞的 read() 抛出异常，退出线程
            self.get_logger().info("Serial port closed")
        if hasattr(self, 'readerThread') and self.readerThread.is_alive():
            self.readerThread.join(timeout=1.0)
            self.get_logger().info("Reader thread joined")

def main():

    rclpy.init()
    node = imuDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()      # 关闭串口、停止线程
        rclpy.spin_once(node, timeout_sec=0.1)  # 确保回调退出
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
