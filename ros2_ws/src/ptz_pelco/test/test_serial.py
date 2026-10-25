"""Exercise real framing and reconnect paths without opening a device."""
import threading
from unittest.mock import patch

import serial

from ptz_pelco.node import PelcoSerial


class SerialFixture:
    is_open = True

    def __init__(self, reply=b''):
        self.reply = reply
        self.buffer = bytearray()
        self.writes = []
        self.fail_read = False

    @property
    def in_waiting(self):
        return len(self.buffer)

    def reset_input_buffer(self):
        self.buffer.clear()

    def write(self, data):
        self.writes.append(data)
        self.buffer.extend(self.reply)
        return len(data)

    def read(self, count):
        if self.fail_read:
            self.fail_read = False
            raise serial.SerialException('device reports readiness but returned no data')
        result = bytes(self.buffer[:count])
        del self.buffer[:count]
        return result

    def fileno(self):
        return 99

    def close(self):
        self.is_open = False


def test_signed_tilt_round_trip():
    for angle in (-90., -33.59, 0., 25.):
        assert PelcoSerial._dec(*PelcoSerial._enc(angle * 100), signed=True) == angle
    assert PelcoSerial._enc(36000) == (0, 0)


def test_coordinate_frames_match_rs485_pdf_and_requested_sweep():
    link = PelcoSerial('unused', 2400, 1)
    assert link._frame(0x51) == bytes.fromhex('FF 01 00 51 00 00 52')
    assert link._frame(0x53) == bytes.fromhex('FF 01 00 53 00 00 54')
    assert link._dec(0x23, 0x28, signed=False) == 90.
    assert link._dec(0x03, 0xE8, signed=True) == 10.
    assert link._frame(0x4D, *link._enc(-900)) == bytes.fromhex('FF 01 00 4D 89 1C F3')
    assert link._frame(0x4D, *link._enc(1500)) == bytes.fromhex('FF 01 00 4D 05 DC 2F')
    assert link._dec(0x89, 0x1C, signed=True) == -9.
    assert link._dec(0x05, 0xDC, signed=True) == 15.


def test_zoom_encoding_and_checksum_match_manufacturer_example():
    link = PelcoSerial('unused', 2400, 1)
    assert link._frame(0x55) == bytes.fromhex('FF 01 00 55 00 00 56')
    assert link._dec(0x00, 0x64, signed=False) == 1.
    assert link._dec(0x07, 0xD0, signed=False) == 20.
    link.ser = SerialFixture(bytes.fromhex('FF 01 00 5D 00 64 C2'))
    assert link._query_once(0x55) == (0x00, 0x64)


def test_reply_resynchronizes_after_noise_echo_bad_checksum_and_other_address():
    link = PelcoSerial('unused', 2400, 1)
    valid = link._frame(0x5B, *link._enc(-3359))
    corrupt = valid[:-1] + bytes([valid[-1] ^ 1])
    other = PelcoSerial('unused', 2400, 2)._frame(0x5B, 0, 1)
    link.ser = SerialFixture(b'noise' + link._frame(0x53) + corrupt + other + valid)
    assert link._query_once(0x53) == link._enc(-3359)
    assert link.stats['bad_frame'] > 0
    assert link.stats['bad_sum'] > 0
    assert link.stats['ok'] == 1


def test_wrong_reply_is_not_accepted_as_axis_position():
    link = PelcoSerial('unused', 2400, 1)
    link.ser = SerialFixture(link._frame(0x59, 0, 9))
    assert link._query_once(0x53, timeout=0.001) is None


def test_reconnect_inside_transaction_does_not_deadlock():
    link = PelcoSerial('unused', 2400, 1, retries=1)
    broken = SerialFixture()
    broken.fail_read = True
    replacement = SerialFixture()
    link.ser = broken
    with patch('ptz_pelco.node.serial.Serial', return_value=replacement), \
            patch('ptz_pelco.node.fcntl.flock'):
        worker = threading.Thread(target=lambda: link.query(0x51), daemon=True)
        worker.start()
        worker.join(1.)
        assert not worker.is_alive(), 'recovery tried to acquire its own serial lock'
    assert not broken.is_open
    assert link.ser is replacement


def test_move_reopens_disconnected_port_without_deadlock():
    link = PelcoSerial('unused', 2400, 1)
    replacement = SerialFixture()
    result = []
    with patch('ptz_pelco.node.serial.Serial', return_value=replacement), \
            patch('ptz_pelco.node.fcntl.flock'):
        worker = threading.Thread(target=lambda: result.append(link.move(0x04, 25, 25)), daemon=True)
        worker.start()
        worker.join(1.)
        assert not worker.is_alive()
    assert result == [True]
    assert replacement.writes == [link._frame(0x04, 25, 25)]


def test_failed_port_lock_closes_new_handle():
    link = PelcoSerial('unused', 2400, 1)
    candidate = SerialFixture()
    with patch('ptz_pelco.node.serial.Serial', return_value=candidate), \
            patch('ptz_pelco.node.fcntl.flock', side_effect=OSError('locked')):
        assert not link.open()[0]
    assert not candidate.is_open
    assert link.ser is None
