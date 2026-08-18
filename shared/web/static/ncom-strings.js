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
// NCOM manual, "GPS Differential Ntrip status" (status channel 76 in
// this codebase's own decoder numbering - see ncomrx.py's
// decodeStatus76 comment). WiFiConnectionStatus (the same channel's
// byte 7) has no table here - it's for other OXTS products, the xNAV
// connects over ethernet and doesn't have wifi.
const NCOM_NTRIP_STATUS_STRINGS = [
  "Invalid", "Ready to connect", "Resolving URL to IP", "Getting source table", "Parsing source table", // 0..4
  "Providing authentication", "Running", "Triggering closing of connection", "Closing connection before retrying", // 5..8
  "Reconfiguring and reconnecting", "Disabled", "Error, undefined", "Error, Bad Authentication", // 9..12
  "Error, TCP connection failed", "Error, unrecognised mount point", "Error, unable to resolve IP", // 13..15
  "Error, invalid address string", "Error, invalid port number", // 16..17
];

const NCOM_STRING_TABLES = {
  GnssPosMode: NCOM_GPS_MODE_STRINGS,
  GnssVelMode: NCOM_GPS_MODE_STRINGS,
  GnssAttMode: NCOM_GPS_MODE_STRINGS,
  GnssDiffNtripStatus: NCOM_NTRIP_STATUS_STRINGS,
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
