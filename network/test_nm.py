import unittest
from unittest.mock import patch

import nm


def _completed(stdout="", returncode=0, stderr=""):
    import subprocess
    return subprocess.CompletedProcess(args=[], returncode=returncode, stdout=stdout, stderr=stderr)


class TestListInterfaces(unittest.TestCase):
    def test_filters_out_loopback_and_p2p(self):
        out = (
            "wlan0:wifi:connected:preconfigured\n"
            "eth0:ethernet:connected:Wired connection 1\n"
            "lo:loopback:connected (externally):lo\n"
            "p2p-dev-wlan0:wifi-p2p:disconnected:\n"
        )
        with patch("subprocess.run", return_value=_completed(out)):
            interfaces = nm.list_interfaces()
        self.assertEqual([i["device"] for i in interfaces], ["wlan0", "eth0"])

    def test_no_connection_is_none_not_empty_string(self):
        with patch("subprocess.run", return_value=_completed("wlan1:wifi:disconnected:\n")):
            interfaces = nm.list_interfaces()
        self.assertIsNone(interfaces[0]["connection"])


class TestInterfaceDetail(unittest.TestCase):
    def test_wifi_client(self):
        out = "connection.type:802-11-wireless\n802-11-wireless.mode:infrastructure\n802-11-wireless.ssid:Coffeebean\nipv4.method:auto\n"
        with patch("subprocess.run", return_value=_completed(out)):
            detail = nm.interface_detail("wlan0", "preconfigured")
        self.assertEqual(detail["mode"], "client")
        self.assertEqual(detail["ssid"], "Coffeebean")
        self.assertEqual(detail["ipv4_method"], "auto")

    def test_wifi_hotspot(self):
        out = "connection.type:802-11-wireless\n802-11-wireless.mode:ap\n802-11-wireless.ssid:amundsen\nipv4.method:shared\n"
        with patch("subprocess.run", return_value=_completed(out)):
            detail = nm.interface_detail("wlan0", "gr6-wlan0")
        self.assertEqual(detail["mode"], "hotspot")

    def test_ethernet_has_no_mode_or_ssid(self):
        out = "connection.type:802-3-ethernet\nipv4.method:manual\nipv4.addresses:192.168.196.22/24\n"
        with patch("subprocess.run", return_value=_completed(out)):
            detail = nm.interface_detail("eth0", "Wired connection 1")
        self.assertIsNone(detail["mode"])
        self.assertIsNone(detail["ssid"])
        self.assertEqual(detail["ipv4_address"], "192.168.196.22/24")

    def test_no_connection_returns_all_none(self):
        detail = nm.interface_detail("wlan1", None)
        self.assertEqual(detail, {"mode": None, "ssid": None, "ipv4_method": None, "ipv4_address": None, "ipv4_gateway": None})


class TestListConnectionNames(unittest.TestCase):
    def test_excludes_loopback(self):
        out = "preconfigured:802-11-wireless\nlo:loopback\nWired connection 1:802-3-ethernet\n"
        with patch("subprocess.run", return_value=_completed(out)):
            names = nm.list_connection_names()
        self.assertEqual(names, ["preconfigured", "Wired connection 1"])


class TestListConnections(unittest.TestCase):
    def test_maps_connection_type_to_device_type(self):
        out = "preconfigured:802-11-wireless\nWired connection 1:802-3-ethernet\n"
        with patch("subprocess.run", return_value=_completed(out)):
            connections = nm.list_connections()
        self.assertEqual(
            connections,
            [
                {"name": "preconfigured", "device_type": "wifi"},
                {"name": "Wired connection 1", "device_type": "ethernet"},
            ],
        )


class TestDescribeConnection(unittest.TestCase):
    def test_wifi_client_dhcp(self):
        out = "connection.type:802-11-wireless\n802-11-wireless.mode:infrastructure\n802-11-wireless.ssid:Coffeebean\nipv4.method:auto\n"
        with patch("subprocess.run", return_value=_completed(out)):
            self.assertEqual(nm.describe_connection("preconfigured"), "wifi client — SSID Coffeebean, DHCP")

    def test_wifi_hotspot(self):
        out = "connection.type:802-11-wireless\n802-11-wireless.mode:ap\n802-11-wireless.ssid:amundsen\nipv4.method:shared\nipv4.addresses:192.168.4.1/24\n"
        with patch("subprocess.run", return_value=_completed(out)):
            self.assertEqual(
                nm.describe_connection("gr6-wlan0"),
                "wifi hotspot — SSID amundsen, broadcasting at 192.168.4.1/24",
            )

    def test_ethernet_static(self):
        out = "connection.type:802-3-ethernet\nipv4.method:manual\nipv4.addresses:192.168.196.22/24\n"
        with patch("subprocess.run", return_value=_completed(out)):
            self.assertEqual(nm.describe_connection("Wired connection 1"), "ethernet — static 192.168.196.22/24")


class TestCreateWifiClient(unittest.TestCase):
    def test_creates_unbound_profile(self):
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            nm.create_wifi_client("MyWifi", "MySSID", "password123")
        add_call = next(c for c in mock_run.call_args_list if "add" in c.args[0])
        args = add_call.args[0]
        self.assertIn("MyWifi", args)
        self.assertIn("MySSID", args)
        self.assertNotIn("ifname", args)  # deliberately device-unbound, see nm.py's create_* docstrings

    def test_refuses_if_name_already_exists(self):
        with patch("subprocess.run", return_value=_completed("MyWifi:802-11-wireless\n")):
            with self.assertRaises(nm.NmError):
                nm.create_wifi_client("MyWifi", "SSID", "pw")


class TestCreateHotspot(unittest.TestCase):
    def test_creates_ap_mode_unbound_profile(self):
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            nm.create_hotspot("MyHotspot", "MySSID", "password123", address="192.168.4.1/24")
        add_call = next(c for c in mock_run.call_args_list if "add" in c.args[0])
        args = add_call.args[0]
        self.assertIn("ap", args)
        self.assertIn("shared", args)
        self.assertNotIn("ifname", args)


class TestCreateEthernet(unittest.TestCase):
    def test_share_sets_shared_method_and_address_only(self):
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            nm.create_ethernet("MyEth", address="192.168.196.22/24", gateway="1.2.3.4", share=True)
        add_call = next(c for c in mock_run.call_args_list if "add" in c.args[0])
        args = add_call.args[0]
        self.assertIn("shared", args)
        self.assertIn("192.168.196.22/24", args)
        # gateway is meaningless for a shared interface (it *is* the
        # gateway) — must never be passed even though one was given.
        self.assertNotIn("1.2.3.4", args)
        self.assertNotIn("ifname", args)

    def test_static_without_share_still_uses_manual(self):
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            nm.create_ethernet("MyEth", ipv4_method="manual", address="192.168.1.5/24", gateway="192.168.1.1")
        add_call = next(c for c in mock_run.call_args_list if "add" in c.args[0])
        args = add_call.args[0]
        self.assertIn("manual", args)
        self.assertIn("192.168.1.1", args)


class TestActivateConnection(unittest.TestCase):
    def test_pins_to_device_when_given(self):
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            nm.activate_connection("CoffeebeanWifi", device="wlan1")
        args = mock_run.call_args[0][0]
        self.assertIn("ifname", args)
        self.assertIn("wlan1", args)

    def test_no_ifname_when_device_omitted(self):
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            nm.activate_connection("CoffeebeanWifi")
        args = mock_run.call_args[0][0]
        self.assertNotIn("ifname", args)


class TestDisconnectDevice(unittest.TestCase):
    def test_calls_device_disconnect(self):
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            nm.disconnect_device("wlan0")
        args = mock_run.call_args[0][0]
        self.assertIn("device", args)
        self.assertIn("disconnect", args)
        self.assertIn("wlan0", args)


class TestActiveConnections(unittest.TestCase):
    def test_maps_name_to_device(self):
        out = "CoffeebeanWifi:wlan0\nCoffeebeanWifiSpare:wlan1\n"
        with patch("subprocess.run", return_value=_completed(out)):
            self.assertEqual(
                nm.active_connections(),
                {"CoffeebeanWifi": "wlan0", "CoffeebeanWifiSpare": "wlan1"},
            )


class TestRun(unittest.TestCase):
    def test_raises_nm_error_on_failure(self):
        with patch("subprocess.run", return_value=_completed(returncode=1, stderr="boom")):
            with self.assertRaises(nm.NmError):
                nm._run(["connection", "up", "nope"])

    def test_prepends_sudo_when_requested(self):
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            nm._run(["connection", "show"], sudo=True)
        self.assertEqual(mock_run.call_args[0][0][0], "sudo")

    def test_no_sudo_by_default(self):
        with patch("subprocess.run", return_value=_completed()) as mock_run:
            nm._run(["connection", "show"])
        self.assertEqual(mock_run.call_args[0][0][0], "nmcli")


if __name__ == "__main__":
    unittest.main()
