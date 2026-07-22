// Header status badges (brown-out, wifi, CPU) — every service's shared
// header watches the manager's /ws/system directly, cross-port, via
// connectWsUrl (same pattern as aruco's Map page reading oxts-nav's
// /ws/nav). See shared/sysstats.py for what the manager is reading.

(function () {
  const wsUrl = MANAGER_URL.replace(/^http/, "ws") + "ws/system";

  // Thresholds for the "Vs" (supply voltage) badge: red if an under-
  // voltage event happened within the last 20s (or is happening now),
  // amber within 60s, green otherwise — colour only, text stays "Vs".
  const BROWNOUT_RED_SECONDS = 20;
  const BROWNOUT_AMBER_SECONDS = 60;

  connectWsUrl(wsUrl, (msg) => {
    const brownout = document.getElementById("sys-brownout");
    brownout.textContent = "Vs";
    const age = msg.brownout.age_seconds;
    if (age !== null && age < BROWNOUT_RED_SECONDS) {
      brownout.className = "badge text-bg-danger";
    } else if (age !== null && age < BROWNOUT_AMBER_SECONDS) {
      brownout.className = "badge text-bg-warning";
    } else {
      brownout.className = "badge text-bg-success";
    }

    const wifi = document.getElementById("sys-wifi");
    if (msg.wifi_bars === null) {
      wifi.textContent = "Wifi —";
      wifi.className = "badge text-bg-secondary";
    } else {
      wifi.textContent = `Wifi ${msg.wifi_bars}/5`;
      wifi.className = "badge " + (
        msg.wifi_bars >= 4 ? "text-bg-success"
        : msg.wifi_bars >= 2 ? "text-bg-warning"
        : "text-bg-danger"
      );
    }

    const cpu = document.getElementById("sys-cpu");
    if (msg.cpu_percent === null) {
      cpu.textContent = "CPU —";
      cpu.className = "badge text-bg-secondary";
    } else {
      cpu.textContent = `CPU ${Math.round(msg.cpu_percent)}%`;
      cpu.className = "badge " + (
        msg.cpu_percent >= 90 ? "text-bg-danger"
        : msg.cpu_percent >= 70 ? "text-bg-warning"
        : "text-bg-success"
      );
    }
  });
})();
