// Header status badges (map-manager logging, battery, wifi, CPU, GPS
// position mode, Aruco marker status) — every service's shared header
// watches the manager's /ws/system, oxts-nav's /ws/nav, aruco's
// /ws/aruco, drive's /ws/drive, and map-manager's /ws/map-manager
// directly, cross-port, via connectWsUrl (same pattern as aruco's own
// Map page reading oxts-nav's /ws/nav). See shared/sysstats.py for what
// the manager is reading. Each of these is a separate, independently-
// retrying websocket (see ws-utils.js) — if the owning service isn't
// running, its badge just stays at its initial "—" default (set in
// base.html) rather than anything more elaborate.

(function () {
  const wsUrl = MANAGER_URL.replace(/^http/, "ws") + "ws/system";

  // Logging on/off badge (added 2026-08-18, replacing the previous
  // brown-out "Vs" badge — see map-manager-prd.md's "Accuracy/logging
  // gating"): green while map-manager is actively updating the
  // occupancy grid, red while it's off (either the manual switch, or
  // accuracy/no-fix — same red either way, since from a glance-at-the-
  // header point of view "not currently recording" is the only thing
  // that matters; the home page's own eligibility message has the
  // detail). Icon stays at its neutral default grey if map-manager
  // itself isn't running.
  if (MAP_MANAGER_WS_URL) {
    connectWsUrl(MAP_MANAGER_WS_URL, (msg) => {
      const logging = document.getElementById("sys-logging");
      logging.className = "badge " + (msg.logging_enabled && !msg.eligibility_reason ? "text-bg-success" : "text-bg-danger");
    });
  }

  // Battery voltage badge (added 2026-09-01, replacing the CPU
  // temperature badge - the new electronics has a big fan and the CPU
  // runs comfortably cool, but the battery is worth watching). Red
  // below drive's own configured battery_low_voltage_v, green at/above
  // - sent alongside the reading itself in drive's feed so this page
  // doesn't need its own copy of drive's config. Two states only (no
  // amber) - deliberately simple until there's a reason for more.
  // Unfiltered raw reading for now - drive-prd.md notes filtering (a
  // slow ~60s average) as a possible future addition, once it's known
  // how noisy the real sensor is.
  if (DRIVE_WS_URL) {
    connectWsUrl(DRIVE_WS_URL, (msg) => {
      const battery = document.getElementById("sys-battery");
      if (msg.battery_voltage_v === undefined || msg.battery_voltage_v === null) {
        battery.textContent = "—V";
        battery.className = "badge text-bg-secondary";
        return;
      }
      battery.textContent = `${msg.battery_voltage_v.toFixed(1)}V`;
      const lowThreshold = msg.battery_low_voltage_v;
      battery.className = "badge " + (
        lowThreshold !== undefined && lowThreshold !== null && msg.battery_voltage_v < lowThreshold
          ? "text-bg-danger" : "text-bg-success"
      );
    });
  }

  connectWsUrl(wsUrl, (msg) => {
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
    // "+" whenever a wifi device is set to Scanner mode (network's own
    // signal-strength logging — see network/scanner_state.py) - a plain
    // visible reminder so it's never silently left on, per Ben's own
    // worry 2026-09-08 about "forgetting" a device is dedicated to
    // scanning instead of its normal job.
    wifi.textContent = msg.wifi_scanning ? "Wifi+" : "Wifi";
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

  // Wheelspeed GAD switch ("W"): green if the switch is on, grey if
  // off - deliberately just "whatever the switch says", not a third
  // "on but not actually sending" state (e.g. no GPS time yet) - see
  // Ben's spec 2026-09-08: wheelspeed isn't a priority when there's no
  // valid time, so that gap isn't worth a badge colour of its own.
  if (WHEELSPEED_WS_URL) {
    connectWsUrl(WHEELSPEED_WS_URL, (msg) => {
      const wheelspeed = document.getElementById("sys-wheelspeed");
      wheelspeed.textContent = "W";
      wheelspeed.className = "badge " + (msg.gad_enabled ? "text-bg-success" : "text-bg-secondary");
    });
  }
})();
