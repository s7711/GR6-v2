import unittest
from unittest.mock import patch

import app


def _iface(device, iface_type, connection, state="connected", matching=None):
    return {
        "device": device, "type": iface_type, "connection": connection, "state": state,
        "matching_connections": matching or [],
    }


class TestValidateBatch(unittest.TestCase):
    def test_allows_a_genuine_swap(self):
        # wlan0 wants what wlan1 currently has, and vice versa — see
        # network-prd.md's "atomic wifi change": this must be allowed,
        # it's the whole point of a combined batch Apply.
        desired = {"wlan0": "AmundsenHotspot", "wlan1": "CoffeebeanWifi"}
        with patch("nm.connections_in_use", return_value={"CoffeebeanWifi": "wlan0", "AmundsenHotspot": "wlan1"}):
            errors = app._validate_batch(desired)
        self.assertEqual(errors, [])

    def test_blocks_a_profile_still_wanted_by_its_current_holder(self):
        # wlan0 wants CoffeebeanWifi, but wlan1 (who has it) isn't
        # relinquishing it in this same batch — a genuine conflict.
        desired = {"wlan0": "CoffeebeanWifi", "wlan1": "CoffeebeanWifi"}
        with patch("nm.connections_in_use", return_value={"CoffeebeanWifi": "wlan1"}):
            errors = app._validate_batch(desired)
        self.assertTrue(errors)

    def test_blocks_a_profile_not_in_this_batch_at_all(self):
        desired = {"wlan0": "CoffeebeanWifiSpare"}
        with patch("nm.connections_in_use", return_value={"CoffeebeanWifiSpare": "wlan1"}):
            errors = app._validate_batch(desired)
        self.assertTrue(errors)

    def test_two_devices_choosing_the_same_profile_is_an_error(self):
        desired = {"wlan0": "CoffeebeanWifi", "wlan1": "CoffeebeanWifi"}
        with patch("nm.connections_in_use", return_value={}):
            errors = app._validate_batch(desired)
        self.assertTrue(any("both wlan0 and wlan1" in e or "both" in e for e in errors))

    def test_off_and_none_values_are_never_conflicts(self):
        desired = {"wlan0": app.OFF_VALUE, "eth0": app.NONE_VALUE}
        with patch("nm.connections_in_use", return_value={}):
            errors = app._validate_batch(desired)
        self.assertEqual(errors, [])


class TestApplyBatch(unittest.TestCase):
    def test_swap_sets_both_chosen_profiles_to_selected_priority(self):
        iface_by_device = {
            "wlan0": _iface("wlan0", "wifi", "CoffeebeanWifi", matching=["CoffeebeanWifi", "AmundsenHotspot"]),
            "wlan1": _iface("wlan1", "wifi", "AmundsenHotspot", matching=["CoffeebeanWifi", "AmundsenHotspot"]),
        }
        desired = {"wlan0": "AmundsenHotspot", "wlan1": "CoffeebeanWifi"}
        with patch("nm.is_hotspot", side_effect=lambda n: n == "AmundsenHotspot"), \
             patch("nm.set_autoconnect_priority") as mock_priority, \
             patch("nm.activate_connection") as mock_activate:
            app._apply_batch(iface_by_device, desired)

        priorities = dict(call.args for call in mock_priority.call_args_list)
        # Both are "selected" this batch (each by a different device) —
        # neither should end up at the hotspot-is-lowest default.
        self.assertEqual(priorities["AmundsenHotspot"], app.PRIORITY_SELECTED)
        self.assertEqual(priorities["CoffeebeanWifi"], app.PRIORITY_SELECTED)

        activated = {call.kwargs.get("device", call.args[1] if len(call.args) > 1 else None): call.args[0]
                     for call in mock_activate.call_args_list}
        self.assertEqual(activated.get("wlan0"), "AmundsenHotspot")
        self.assertEqual(activated.get("wlan1"), "CoffeebeanWifi")

    def test_untouched_hotspot_still_gets_low_priority(self):
        iface_by_device = {
            "wlan0": _iface("wlan0", "wifi", "CoffeebeanWifi", matching=["CoffeebeanWifi", "AmundsenHotspot"]),
        }
        desired = {"wlan0": "CoffeebeanWifi"}  # unchanged — no-op activation
        with patch("nm.is_hotspot", side_effect=lambda n: n == "AmundsenHotspot"), \
             patch("nm.set_autoconnect_priority") as mock_priority, \
             patch("nm.activate_connection") as mock_activate:
            app._apply_batch(iface_by_device, desired)

        priorities = dict(call.args for call in mock_priority.call_args_list)
        self.assertEqual(priorities["AmundsenHotspot"], app.PRIORITY_HOTSPOT)
        self.assertEqual(priorities["CoffeebeanWifi"], app.PRIORITY_SELECTED)
        mock_activate.assert_not_called()  # already set to this — nothing to do

    def test_off_value_turns_off_managed(self):
        iface_by_device = {"wlan1": _iface("wlan1", "wifi", "AmundsenHotspot", matching=["AmundsenHotspot"])}
        desired = {"wlan1": app.OFF_VALUE}
        with patch("nm.is_hotspot", return_value=True), \
             patch("nm.set_autoconnect_priority"), \
             patch("nm.set_managed") as mock_managed, \
             patch("nm.activate_connection") as mock_activate, \
             patch("app.scanner_state") as mock_scanner_state:
            mock_scanner_state.get_device.return_value = None
            app._apply_batch(iface_by_device, desired)
        mock_managed.assert_called_once_with("wlan1", False)
        mock_activate.assert_not_called()
        mock_scanner_state.set_device.assert_not_called()

    def test_scanner_value_turns_off_managed_and_records_device(self):
        # Same underlying nmcli action as OFF_VALUE — scanner_state.py is
        # the only thing telling them apart, see its own docstring.
        iface_by_device = {"wlan0": _iface("wlan0", "wifi", "CoffeebeanWifi", matching=["CoffeebeanWifi"])}
        desired = {"wlan0": app.SCANNER_VALUE}
        with patch("nm.is_hotspot", return_value=False), \
             patch("nm.set_autoconnect_priority"), \
             patch("nm.set_managed") as mock_managed, \
             patch("nm.activate_connection") as mock_activate, \
             patch("app.scanner_state") as mock_scanner_state:
            mock_scanner_state.get_device.return_value = None
            app._apply_batch(iface_by_device, desired)
        mock_managed.assert_called_once_with("wlan0", False)
        mock_activate.assert_not_called()
        mock_scanner_state.set_device.assert_called_once_with("wlan0")
        mock_scanner_state.clear.assert_not_called()

    def test_moving_a_device_away_from_scanner_clears_it(self):
        # wlan0 was the scanner; this batch picks a real profile for it
        # instead — the recorded scanner choice must not linger stale.
        iface_by_device = {"wlan0": _iface("wlan0", "wifi", None, state="unmanaged", matching=["CoffeebeanWifi"])}
        desired = {"wlan0": "CoffeebeanWifi"}
        with patch("nm.is_hotspot", return_value=False), \
             patch("nm.set_autoconnect_priority"), \
             patch("nm.set_managed"), \
             patch("nm.activate_connection"), \
             patch("app.scanner_state") as mock_scanner_state:
            mock_scanner_state.get_device.return_value = "wlan0"
            app._apply_batch(iface_by_device, desired)
        mock_scanner_state.clear.assert_called_once()

    def test_scanner_on_a_different_device_is_left_alone(self):
        iface_by_device = {
            "wlan0": _iface("wlan0", "wifi", "CoffeebeanWifi", matching=["CoffeebeanWifi"]),
            "wlan1": _iface("wlan1", "wifi", None, state="unmanaged", matching=[]),
        }
        desired = {"wlan0": "CoffeebeanWifi"}  # wlan1 (the real scanner) isn't in this batch at all
        with patch("nm.is_hotspot", return_value=False), \
             patch("nm.set_autoconnect_priority"), \
             patch("nm.set_managed"), \
             patch("nm.activate_connection"), \
             patch("app.scanner_state") as mock_scanner_state:
            mock_scanner_state.get_device.return_value = "wlan1"
            app._apply_batch(iface_by_device, desired)
        mock_scanner_state.clear.assert_not_called()


class TestRestoreScannerState(unittest.TestCase):
    # Regression coverage for the 2026-09-10 fix: `nmcli device set
    # managed no` doesn't survive a Pi reboot (NetworkManager just
    # re-manages/reconnects the device on its own), unlike
    # scanner_state.json - without re-applying it at startup, a device
    # left in Scanner mode before a power-cycle silently comes back as
    # an ordinary managed/connected interface, with no wifi-scan logging
    # and nothing in the log to explain why.
    def test_reapplies_managed_no_for_the_persisted_scanner_device(self):
        with patch("app.scanner_state") as mock_scanner_state, patch("nm.set_managed") as set_managed:
            mock_scanner_state.get_device.return_value = "wlan1"
            app._restore_scanner_state()
        set_managed.assert_called_once_with("wlan1", False)

    def test_does_nothing_when_no_device_is_in_scanner_mode(self):
        with patch("app.scanner_state") as mock_scanner_state, patch("nm.set_managed") as set_managed:
            mock_scanner_state.get_device.return_value = None
            app._restore_scanner_state()
        set_managed.assert_not_called()

    def test_survives_the_device_being_gone(self):
        with patch("app.scanner_state") as mock_scanner_state, \
             patch("nm.set_managed", side_effect=app.nm.NmError("no such device")):
            mock_scanner_state.get_device.return_value = "wlan1"
            app._restore_scanner_state()  # must not raise


if __name__ == "__main__":
    unittest.main()
