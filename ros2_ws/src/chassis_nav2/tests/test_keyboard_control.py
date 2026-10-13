"""Check keyboard deadman and gear-change behavior without commanding a vehicle."""
import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / 'scripts/jd03_keyboard.py'
spec = importlib.util.spec_from_file_location('jd03_keyboard', path)
keyboard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(keyboard)


class KeyboardTests(unittest.TestCase):
    def stopped(self, command):
        self.assertEqual(command[:2], (0., 0.))
        self.assertIs(command[3], True)

    def test_forward_reverse_units_and_steering_sign(self):
        state = keyboard.KeyboardState()
        state.key('w', 0.)
        forward = state.command(.1, True)
        self.assertAlmostEqual(forward[0], .36)
        self.assertEqual(forward[2], keyboard.Ecu.SHIFT_D)
        self.assertIs(forward[3], False)
        state.key('a', .1)
        self.assertEqual(state.command(.2, True)[1], -10.)
        state.key('d', .2)
        self.assertEqual(state.command(.3, True)[1], 10.)
        state.key('x', .3)
        self.assertEqual(state.command(.4, True)[1], 0.)
        state.key('s', .4)
        reverse = state.command(.5, True, feedback_speed=0.)
        self.assertAlmostEqual(reverse[0], .36)
        self.assertEqual(reverse[2], keyboard.Ecu.SHIFT_R)
        self.assertIs(reverse[3], False)

    def test_input_timeout_stops_and_requires_new_motion_key(self):
        state = keyboard.KeyboardState()
        state.key('w', 0.)
        self.stopped(state.command(.7, True))
        state.key('a', .8)
        command = state.command(.9, True)
        self.assertEqual(command[0], 0.)
        self.assertIs(command[3], True)

    def test_feedback_loss_and_manual_mode_do_not_resume_old_input(self):
        for ready, autodrive in ((False, True), (True, False)):
            state = keyboard.KeyboardState()
            state.key('w', 0.)
            self.stopped(state.command(.1, ready, autodrive=autodrive))
            self.stopped(state.command(.2, True, autodrive=True))

    def test_direction_change_brakes_until_stopped_and_pressed_again(self):
        state = keyboard.KeyboardState()
        state.key('w', 0.)
        state.command(.01, True)
        state.key('s', .1)
        command = state.command(.11, True, feedback_speed=.1)
        self.stopped(command)
        self.assertEqual(command[2], keyboard.Ecu.SHIFT_D)
        self.stopped(state.command(.2, True, feedback_speed=0.))
        state.key('s', .3)
        self.assertEqual(state.command(.31, True, feedback_speed=0.)[2], keyboard.Ecu.SHIFT_R)
        state.key('w', .4)
        command = state.command(.41, True, feedback_speed=-.1)
        self.stopped(command)
        self.assertEqual(command[2], keyboard.Ecu.SHIFT_R)

    def test_space_and_exit_stop_immediately(self):
        for key in (' ', 'q', '\x03'):
            state = keyboard.KeyboardState()
            state.key('w', 0.)
            state.key('a', .1)
            running = state.key(key, .2)
            self.stopped(state.command(.2, True))
            self.assertEqual(running, key == ' ')

    def test_stationary_steering_has_zero_motor_and_brake(self):
        state = keyboard.KeyboardState()
        state.key('a', 0.)
        command = state.command(.1, True)
        self.assertEqual(command[:4], (0., -10., keyboard.Ecu.SHIFT_D, True))

    def test_arrow_escape_and_unrecognized_keys_cannot_drive(self):
        state = keyboard.KeyboardState()
        state.key('w', 0.)
        for key in '\x1b[A':
            state.key(key, .1)
        self.stopped(state.command(.2, True))
        state.key('w', .3)
        state.key('z', .8)
        self.stopped(state.command(1., True))


if __name__ == '__main__':
    unittest.main(verbosity=2)
