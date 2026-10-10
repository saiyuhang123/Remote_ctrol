"""Check Modbus decoding and fallback without connecting a serial device."""
import struct
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from wit_ros2_imu import wit_ros2_imu as driver


def response(payload):
    frame = bytes([0x50, 0x03, len(payload)]) + payload
    return frame + struct.pack('<H', driver.imuDriverNode.modbusCRC(frame))


def high_angles(values):
    words = []
    for value in values:
        words.extend([value & 0xFFFF, (value >> 16) & 0xFFFF])
    return struct.pack('>6H', *words)


class ScriptedSerial:
    def __init__(self, node, replies):
        self.node, self.replies = node, iter(replies)
        self.stop_after = len(replies)
        self.commands, self.pending = [], b''
        self.timeout = None

    def write(self, command):
        self.commands.append(command)
        self.pending = next(self.replies)

    def read(self, count):
        result, self.pending = self.pending[:count], self.pending[count:]
        if not self.pending and len(self.commands) == self.stop_after:
            self.node.readerRunning = False
        return result

    def reset_input_buffer(self):
        self.pending = b''


class ModbusReadTests(unittest.TestCase):
    def make_node(self, protocol=driver.protocolType.RS485_HIGH):
        node = driver.imuDriverNode.__new__(driver.imuDriverNode)
        node.protocol, node.modbusID = protocol, 0x50
        node.modbusAddrList, node.modbusIndex = [0x34, 0x37, 0x3A, 0x3D], 0
        node.currentRequestAddr = 0x34
        node.rs485_batch_read, node.readerRunning = True, True
        node.dataLock = threading.Lock()
        node.accData, node.gyroData = np.zeros(3), np.zeros(3)
        node.magData, node.angleData = np.zeros(3), np.zeros(3)
        node._last_acc_update = node._last_gyro_update = node._last_angle_update = 0.
        node._gyro_stamp = None
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: 'reception-stamp'))
        node.warnings = []
        node.get_logger = lambda: SimpleNamespace(warning=node.warnings.append)
        return node

    def test_high_batch_decodes_signed_low_word_first_angles(self):
        node = self.make_node()
        measurements = [2048, -1024, 0, 100, -200, 32767, 1, 2, -3]
        angles = [123456, -234567, -2147483648]
        payload = struct.pack('>9h', *measurements) + high_angles(angles)
        with patch.object(driver.time, 'monotonic', return_value=10.):
            node.handleModbusFrame(response(payload))
        np.testing.assert_array_equal(node.accData, measurements[:3])
        np.testing.assert_array_equal(node.gyroData, measurements[3:6])
        np.testing.assert_array_equal(node.magData, measurements[6:9])
        np.testing.assert_array_equal(node.angleData, angles)
        self.assertEqual(node._last_acc_update, node._last_gyro_update)
        self.assertEqual(node._last_angle_update, 10.)
        self.assertEqual(node._gyro_stamp, 'reception-stamp')

    def test_standard_batch_keeps_16_bit_angles(self):
        node = self.make_node(driver.protocolType.RS485_STD)
        values = [0, 0, 2048, -12, 34, 56, 7, 8, 9, -16384, 8192, 32767]
        node.handleModbusFrame(response(struct.pack('>12h', *values)))
        np.testing.assert_array_equal(node.angleData, values[9:])
        np.testing.assert_array_equal(node.gyroData, values[3:6])

    def test_high_angle_group_still_works(self):
        node = self.make_node()
        node.currentRequestAddr = 0x3D
        angles = [-1000, 90000, 2147483647]
        node.handleModbusFrame(response(high_angles(angles)))
        np.testing.assert_array_equal(node.angleData, angles)
        self.assertEqual(node.modbusIndex, 1)

    def test_batch_uses_one_request_and_checks_crc(self):
        node = self.make_node()
        node.serialPort = ScriptedSerial(node, [response(bytes(30))])
        node._readRS485()
        command = node.serialPort.commands[0]
        self.assertEqual(command[:6], bytes.fromhex('50 03 00 34 00 0f'))
        self.assertEqual(int.from_bytes(command[-2:], 'little'),
                         node.modbusCRC(command[:-2]))
        self.assertEqual(len(node.serialPort.commands), 1)
        self.assertEqual(node._gyro_stamp, 'reception-stamp')
        self.assertIsNone(node.serialPort.timeout)

    def test_timeout_falls_back_to_legacy_after_three_failures(self):
        node = self.make_node()
        node.serialPort = ScriptedSerial(
            node, [b'', b'', b'', response(struct.pack('>3h', 0, 0, 2048))])
        node._readRS485()
        lengths = [int.from_bytes(cmd[4:6], 'big') for cmd in node.serialPort.commands]
        self.assertEqual(lengths, [15, 15, 15, 3])
        self.assertEqual(len(node.warnings), 1)
        np.testing.assert_array_equal(node.accData, [0, 0, 2048])
        self.assertIsNone(node._gyro_stamp)

    def test_bad_crc_does_not_update_data(self):
        node = self.make_node()
        frame = response(bytes(30))
        node.serialPort = ScriptedSerial(node, [frame[:-1] + bytes([frame[-1] ^ 1])])
        node._readRS485()
        self.assertEqual(node._last_gyro_update, 0.)
        self.assertIsNone(node._gyro_stamp)

    def test_short_response_does_not_update_data(self):
        node = self.make_node()
        node.serialPort = ScriptedSerial(node, [response(bytes(30))[:-4]])
        node._readRS485()
        self.assertEqual(node._last_gyro_update, 0.)

    def test_batch_can_be_disabled(self):
        node = self.make_node()
        node.rs485_batch_read = False
        node.serialPort = ScriptedSerial(node, [response(bytes(6))])
        node._readRS485()
        self.assertEqual(node.serialPort.commands[0][:6], bytes.fromhex('50 03 00 34 00 03'))
        self.assertEqual(node.modbusIndex, 1)


if __name__ == '__main__':
    unittest.main()
