// Reusable jog joystick widget: drag to send {left_mps, right_mps} to
// postUrl at a fixed repeat rate while held, one explicit zero on
// release. Built independently twice already (drive/home.html,
// navigate/create-path.html) before being promoted here once navigate
// needed a second and third consumer (Run, Edit map) — see
// ui-style.md's shared-asset-promotion convention (geomap.js's own
// history is the precedent for this).
//
// Usage:
//   <div id="joystick" style="touch-action: none; width: 200px; height: 200px; ...; position: relative;">
//     <div id="joystick-thumb" style="width: 50px; height: 50px; border-radius: 50%; position: absolute; pointer-events: none;"></div>
//   </div>
//   createJoystick(document.getElementById("joystick"), document.getElementById("joystick-thumb"), {
//     postUrl: "/jog/manual", maxSpeedMps: 0.8,
//   });

function createJoystick(joystickEl, thumbEl, options) {
  const postUrl = options.postUrl;
  const maxSpeedMps = options.maxSpeedMps;
  const repeatMs = options.repeatMs || 300;
  // Usable travel: half the joystick's own size, minus half the thumb's
  // (so the thumb's edge stays inside the joystick at full deflection) —
  // computed from actual element sizes rather than a magic constant per
  // page, so this works regardless of how big a given page draws it.
  const radius = (joystickEl.clientWidth - thumbEl.clientWidth) / 2;

  let dragging = false;
  let vector = { x: 0, y: 0 };
  let repeatTimer = null;

  function sendManual(x, y) {
    const forward = -y * maxSpeedMps;
    // Turning right means the right (inside) wheel slows down and the
    // left (outside) wheel speeds up — see drive/templates/pages/home.html,
    // where the inverted version of this was a real bug confirmed on
    // hardware.
    const turn = x * maxSpeedMps;
    fetch(postUrl, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ left_mps: forward + turn, right_mps: forward - turn }),
    });
  }

  function setThumb(x, y) {
    thumbEl.style.left = `${(joystickEl.clientWidth - thumbEl.clientWidth) / 2 + x * radius}px`;
    thumbEl.style.top = `${(joystickEl.clientHeight - thumbEl.clientHeight) / 2 + y * radius}px`;
  }
  setThumb(0, 0);

  function updateFromEvent(evt) {
    const rect = joystickEl.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let x = (evt.clientX - cx) / radius;
    let y = (evt.clientY - cy) / radius;
    const mag = Math.hypot(x, y);
    if (mag > 1) { x /= mag; y /= mag; }
    vector = { x, y };
    setThumb(x, y);
  }

  joystickEl.addEventListener("pointerdown", (evt) => {
    dragging = true;
    joystickEl.setPointerCapture(evt.pointerId);
    joystickEl.style.cursor = "grabbing";
    updateFromEvent(evt);
    sendManual(vector.x, vector.y); // send once immediately, so the response feels instant
    repeatTimer = setInterval(() => sendManual(vector.x, vector.y), repeatMs);
  });

  joystickEl.addEventListener("pointermove", (evt) => {
    if (dragging) updateFromEvent(evt);
  });

  function stopJogging() {
    if (!dragging) return;
    dragging = false;
    joystickEl.style.cursor = "grab";
    clearInterval(repeatTimer);
    vector = { x: 0, y: 0 };
    setThumb(0, 0);
    sendManual(0, 0); // one explicit stop — releasing the joystick means "stop now," not "coast until the hold lapses"
  }
  joystickEl.addEventListener("pointerup", stopJogging);
  joystickEl.addEventListener("pointercancel", stopJogging);
}
