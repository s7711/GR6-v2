// Header status badges (brown-out, wifi, CPU, GPS position mode, Aruco
// marker status) — every service's shared header watches the manager's
// /ws/system, oxts-nav's /ws/nav, and aruco's /ws/aruco directly,
// cross-port, via connectWsUrl (same pattern as aruco's own Map page
// reading oxts-nav's /ws/nav). See shared/sysstats.py for what the
// manager is reading. The oxts-nav/aruco connections are separate,
// independently-retrying websockets (see ws-utils.js) — if either
// service isn't running, its badge just stays at its initial "—"
// default (set in base.html) rather than anything more elaborate.

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

    // Thresholds for wifi quality (%): this Pi's signal is never seen
    // above ~4/5 bars in practice (never 5/5) — so rather than a scale
    // that requires 5/5 for "good", the old bar 3/5's own quality range
    // (35-49 out of 70, i.e. 50-70%) is used as the transition: its
    // midpoint (60%) is green, its low end (50%) is amber, anything
    // below (bar 2 and under) is red. See Ben's spec in the header
    // status badges discussion (2026-07-24).
    const WIFI_GREEN_PERCENT = 60;
    const WIFI_AMBER_PERCENT = 50;

    const wifi = document.getElementById("sys-wifi");
    wifi.textContent = "Wifi";
    if (msg.wifi_percent === null) {
      wifi.className = "badge text-bg-secondary";
    } else {
      wifi.className = "badge " + (
        msg.wifi_percent >= WIFI_GREEN_PERCENT ? "text-bg-success"
        : msg.wifi_percent >= WIFI_AMBER_PERCENT ? "text-bg-warning"
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

  // GPS position mode ("G"): green for an RTK integer fix (any of the
  // "*integer" variants — RTK/gx/ix), red for any SPS-only fix (no
  // corrections applied), grey for None (no fix at all), dark for GAD
  // (OxTS's confusingly-named "GenAid" mode — see the 2026-07-23 UCOM
  // session notes), amber/orange for anything else (float, differential,
  // WAAS, etc — a real fix, just not RTK-quality).
  if (OXTSNAV_WS_URL) {
    connectWsUrl(OXTSNAV_WS_URL, (msg) => {
      const gnss = document.getElementById("sys-gnss");
      gnss.textContent = "G";
      const code = (msg.status || {}).GnssPosMode;
      const name = (code !== null && code !== undefined) ? NCOM_GPS_MODE_STRINGS[code] : undefined;
      gnss.className = "badge " + (
        name === undefined ? "text-bg-secondary"
        : name === "None" ? "text-bg-secondary"
        : name === "GAD" ? "text-bg-dark"
        : /integer/i.test(name) ? "text-bg-success"
        : /SPS/i.test(name) ? "text-bg-danger"
        : "text-bg-warning"
      );
    });
  }

  // Aruco marker status ("A"): red (an unmapped marker is visible —
  // takes priority, since it means something needs surveying) beats
  // green (a mapped marker is visible and being used for GAD aiding),
  // beats grey (nothing visible). No amber yet — see Ben's spec.
  if (ARUCO_WS_URL) {
    connectWsUrl(ARUCO_WS_URL, (msg) => {
      const aruco = document.getElementById("sys-aruco");
      aruco.textContent = "A";
      const hasUnmapped = Object.keys(msg.unmapped || {}).length > 0;
      const hasVisible = (msg.visible_ids || []).length > 0;
      aruco.className = "badge " + (
        hasUnmapped ? "text-bg-danger"
        : hasVisible ? "text-bg-success"
        : "text-bg-secondary"
      );
    });
  }
})();
