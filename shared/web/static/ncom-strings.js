// NCOM status-code -> string translations, ported verbatim from GR6-v1's
// static/messages.js (GPS_MODE_STRINGS / HEADING_QUALITY_STRINGS /
// STRING_MAP / to_string). Shared because more than one service (oxts-nav,
// aruco) displays these codes. Call translateNcomCodes(prefix, data) right
// after fillFields(prefix, data) to overwrite the bare numeric code with
// "String (n)".
const NCOM_GPS_MODE_STRINGS = ["None", "Search", "Doppler", "SPS", "Differential", "RTK float", "RTK integer",  // 0..6
    "WAAS", "OmniSTAR", "OmniSTAR HP", "No data", "Blanked", "Doppler(PP)", "SPS(PP)", "Differential(PP)", // 7..14
    "RTK float(PP)", "RTK integer(PP)", "OmniStar XP", "CDGPS", "Not recognised", "gxDoppler", "gxSPS",    // 15..21
    "gxDifferential", "gxFloat", "gxInteger", "ixDoppler", "ixSPS", "ixDifferential", "ixFloat",           // 22..28
    "ixInteger", "PPP converging", "PPP", "Unknown", "Unknown", "GAD" // 29..34
];
const NCOM_HEADING_QUALITY_STRINGS = ["None", "Poor", "OK", "Good"];

const NCOM_STRING_TABLES = {
  GpsPosMode: NCOM_GPS_MODE_STRINGS,
  GpsVelMode: NCOM_GPS_MODE_STRINGS,
  GpsAttMode: NCOM_GPS_MODE_STRINGS,
  HeadQuality: NCOM_HEADING_QUALITY_STRINGS,
};

function translateNcomCodes(prefix, data) {
  for (const [key, table] of Object.entries(NCOM_STRING_TABLES)) {
    const value = data[key];
    if (value === undefined || value === null) continue;
    const el = document.getElementById(prefix + key);
    if (!el) continue;
    const index = parseInt(value, 10);
    el.textContent = (index >= 0 && index < table.length) ? `${table[index]} (${index})` : String(index);
  }
}
