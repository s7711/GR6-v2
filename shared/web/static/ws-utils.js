// Shared helpers for pages that display live data over a websocket.
// See ui-style.md ("no buffering" over flaky wifi).

function connectWs(path, onMessage) {
  function connect() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${proto}://${location.host}${path}`);
    ws.onmessage = (event) => onMessage(JSON.parse(event.data));
    // Flaky wifi: just retry — never buffer/replay, always resume with
    // whatever is current when reconnected.
    ws.onclose = () => setTimeout(connect, 1000);
    ws.onerror = () => ws.close();
  }
  connect();
}

// Fills any element with id `${prefix}${key}` from data's entries.
// Keys with no matching element are simply ignored — a page only shows
// the fields it has a row for.
function fillFields(prefix, data) {
  for (const [key, value] of Object.entries(data)) {
    const el = document.getElementById(prefix + key);
    if (!el) continue;
    if (typeof value === "number") {
      el.textContent = Number.isInteger(value) ? value : value.toFixed(3);
    } else {
      el.textContent = value;
    }
  }
}
