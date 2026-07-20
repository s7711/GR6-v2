// Shared helpers for uPlot time-series charts (see ui-style.md).

// A uPlot x-axis `values` formatter that always renders HH:mm:ss, never
// a date. uPlot's own default formatter switches to including the date
// (US month/day order) at coarser zoom levels or across a day boundary
// — a genuine point of friction for a European reading it, and these
// short rolling-window graphs (tens of seconds) have no use for a date
// anyway. hour12: false avoids re-introducing an AM/PM ambiguity too.
function hhmmssAxisValues(u, splits) {
  return splits.map((v) => new Date(v * 1000).toLocaleTimeString([], {
    hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false,
  }));
}
