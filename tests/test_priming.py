# Implements: SW-001 §2.7 — Two-stage water-line priming unit tests
"""
Verifies PrimingSystem + RelayController.prime_flush + AccumulatorManager.charge_pressure
order of operations:
  1. Disarms if armed
  2. Aims gimbal down (90°, 0°)
  3. Stage 1: Solenoid OPEN + Pump ON (with mid-flow auto_detect capture), then Pump OFF + Solenoid CLOSED
  4. Stage 2: Solenoid CLOSED + build pressure to TARGET_PSI (or re-arm if previously armed)
  5. Restores gimbal position
"""

import unittest
from unittest.mock import MagicMock, patch

import hardware as hw


class _StubGimbal:
    def __init__(self, pitch=10.0, yaw=-5.0):
        self.pitch = pitch
        self.yaw = yaw
        self.moves = []

    def get_status(self):
        return {"pitch": self.pitch, "yaw": self.yaw}

    def set_angles(self, pitch, yaw):
        self.pitch = pitch
        self.yaw = yaw
        self.moves.append((pitch, yaw))


class TestTwoStagePriming(unittest.TestCase):
    def test_default_prime_duration_is_10s_and_clamps_to_30s(self):
        relay = MagicMock()
        primer = hw.PrimingSystem(relay)
        self.assertEqual(primer.prime_duration_ms, 10000)

        primer.update_settings({"prime_duration_ms": 25000})
        self.assertEqual(primer.prime_duration_ms, 25000)

        primer.update_settings({"prime_duration_ms": 99999})
        self.assertEqual(primer.prime_duration_ms, 30000)

        primer.update_settings({"prime_duration_ms": 100})
        self.assertEqual(primer.prime_duration_ms, 500)

    def test_prime_opens_solenoid_then_builds_pressure_disarmed(self):
        events = []

        relay = hw.RelayController.__new__(hw.RelayController)
        relay._lock = hw.threading.Lock()
        relay._pulse_busy = False
        relay._pump_state = False
        relay._solenoid_state = False
        relay._solenoid_driver = "pico"
        relay._pico = None  # stub mode
        relay._module_12v_hardwired = False
        relay._module_power_hold = False

        def fake_set_solenoid(state):
            relay._solenoid_state = bool(state)
            events.append(("solenoid", bool(state)))

        relay._set_solenoid = fake_set_solenoid
        relay.recover_solenoid = lambda re_pinmux=False: events.append(("recover", re_pinmux)) or {}

        accum = MagicMock()
        accum.get_status.return_value = {"armed": False}
        accum.charge_pressure.side_effect = lambda label="": (
            events.append(("charge_pressure", relay._solenoid_state, relay._pump_state))
            or {"status": "charged", "psi": 15.2, "target_psi": 15.0, "reached_target": True}
        )

        gimbal = _StubGimbal(pitch=12.0, yaw=-8.0)
        primer = hw.PrimingSystem(relay, accum=accum)
        primer.prime_duration_ms = 500
        primer.auto_detect = False

        with patch("hardware.time.sleep", return_value=None):
            res = primer.prime(gimbal=gimbal)

        self.assertEqual(res["status"], "primed")
        self.assertTrue(res["solenoid_opened"])
        self.assertEqual(res["psi"], 15.2)
        self.assertEqual(res["target_psi"], 15.0)
        self.assertFalse(res["re_armed"])
        # Gimbal aimed down then restored
        self.assertEqual(gimbal.moves, [(90.0, 0.0), (12.0, -8.0)])
        # Stage 1 opened solenoid, then closed solenoid, then recovered, then Stage 2 charged with solenoid closed
        self.assertEqual(
            events,
            [
                ("solenoid", True),
                ("solenoid", False),
                ("recover", False),
                ("charge_pressure", False, False),
            ],
        )
        accum.disarm.assert_not_called()
        accum.arm.assert_not_called()

    def test_prime_when_armed_disarms_flushes_and_rearms(self):
        events = []

        relay = hw.RelayController.__new__(hw.RelayController)
        relay._lock = hw.threading.Lock()
        relay._pulse_busy = False
        relay._pump_state = False
        relay._solenoid_state = False
        relay._solenoid_driver = "pico"
        relay._pico = None

        relay._set_solenoid = lambda state: (
            setattr(relay, "_solenoid_state", bool(state)),
            events.append(("solenoid", bool(state))),
        )
        relay.recover_solenoid = lambda re_pinmux=False: events.append(("recover",)) or {}

        accum = MagicMock()
        accum.get_status.return_value = {"armed": True}
        accum.disarm.side_effect = lambda reason="": events.append(("disarm", reason))
        accum.arm.side_effect = lambda: (
            events.append(("arm",))
            or {"status": "armed", "psi": 15.0, "target_psi": 15.0, "reached_target": True}
        )

        primer = hw.PrimingSystem(relay, accum=accum)
        primer.prime_duration_ms = 500
        primer.auto_detect = False

        with patch("hardware.time.sleep", return_value=None):
            res = primer.prime()

        self.assertEqual(res["status"], "primed")
        self.assertTrue(res["re_armed"])
        self.assertEqual(
            events,
            [
                ("disarm", "priming"),
                ("solenoid", True),
                ("solenoid", False),
                ("recover",),
                ("arm",),
            ],
        )
        accum.charge_pressure.assert_not_called()

    def test_prime_aborts_if_pico_solenoid_fails_to_open(self):
        relay = hw.RelayController.__new__(hw.RelayController)
        relay._lock = hw.threading.Lock()
        relay._pulse_busy = False
        relay._pump_state = False
        relay._solenoid_state = False
        relay._solenoid_driver = "pico"
        relay._pico = MagicMock()
        relay._pico.set_open.return_value = False  # Pico disconnected / failed

        accum = MagicMock()
        accum.get_status.return_value = {"armed": False}
        gimbal = _StubGimbal(pitch=5.0, yaw=3.0)

        primer = hw.PrimingSystem(relay, accum=accum)
        primer.prime_duration_ms = 500
        primer.auto_detect = False

        with patch("hardware.time.sleep", return_value=None):
            res = primer.prime(gimbal=gimbal)

        self.assertEqual(res["status"], "error")
        self.assertIn("Solenoid failed to open", res["error"])
        self.assertFalse(relay._pump_state)
        self.assertFalse(relay._solenoid_state)
        self.assertFalse(relay._pulse_busy)
        accum.charge_pressure.assert_not_called()
        # Gimbal still restored on error
        self.assertEqual(gimbal.moves, [(90.0, 0.0), (5.0, 3.0)])


if __name__ == "__main__":
    unittest.main()
