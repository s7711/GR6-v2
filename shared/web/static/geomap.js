// Reusable canvas-based local-tangent-plane map: gridlines, scale bar,
// zoom presets, and named "layers" of {lat, lon, ...} points drawn as a
// polyline and/or dots, plus a single "current position" marker. First
// built (independently, twice) for aruco's Map page and navigate's Run
// page — promoted here once a third consumer (navigate's Create Path
// page) needed the same thing. See ui-style.md's shared-asset-promotion
// convention.
//
// Usage:
//   const map = createGeoMap(canvasEl, wrapperEl, ".zoom-btn");
//   map.setLayer("path", pointsArray, { color: "#6c757d", dots: false });
//   map.setCurrent(lat, lon);
//   map.resize(); // call once on load, and whenever a layer changes

function createGeoMap(canvasEl, wrapperEl, zoomButtonsSelector) {
  const ctx = canvasEl.getContext("2d");
  let zoomMode = "auto";
  let current = null; // {lat, lon}
  const layers = {}; // name -> {points: [{lat, lon, ...}], style: {...}}

  // Local tangent-plane offset (metres), recomputed fresh every draw
  // from whatever the current reference point is — simple
  // equirectangular approximation, fine at this project's scale, not
  // used for anything aiding-critical (that's shared/geodesy.py's job,
  // server-side).
  function offsetMetres(lat, lon, ref) {
    const metresPerDegLat = 111320;
    const metresPerDegLon = 111320 * Math.cos((ref.lat * Math.PI) / 180);
    return { north: (lat - ref.lat) * metresPerDegLat, east: (lon - ref.lon) * metresPerDegLon };
  }

  function referencePoint() {
    if (current) return current;
    for (const name in layers) {
      if (layers[name].points.length) return layers[name].points[0];
    }
    return null;
  }

  function niceSpacing(metresAcross, targetLines) {
    const raw = metresAcross / targetLines;
    const pow10 = Math.pow(10, Math.floor(Math.log10(raw)));
    const candidates = [1, 2, 5, 10].map((m) => m * pow10);
    return candidates.reduce((best, c) => (Math.abs(c - raw) < Math.abs(best - raw) ? c : best));
  }

  function setLayer(name, points, style) {
    layers[name] = { points: points || [], style: style || {} };
  }

  function setCurrent(lat, lon) {
    current = lat !== undefined && lon !== undefined ? { lat, lon } : null;
  }

  function setZoom(mode) {
    zoomMode = mode;
    if (zoomButtonsSelector) {
      document.querySelectorAll(zoomButtonsSelector).forEach((btn) => {
        const active = btn.dataset.zoom === String(mode);
        btn.classList.toggle("btn-primary", active);
        btn.classList.toggle("btn-outline-secondary", !active);
      });
    }
    draw();
  }

  // Wire up the zoom buttons themselves — passing zoomButtonsSelector
  // both drives this and setZoom's own active-button styling above, so
  // there's one place a caller needs to touch, not two.
  if (zoomButtonsSelector) {
    document.querySelectorAll(zoomButtonsSelector).forEach((btn) => {
      btn.addEventListener("click", () => {
        setZoom(btn.dataset.zoom === "auto" ? "auto" : parseFloat(btn.dataset.zoom));
      });
    });
  }

  function resize() {
    const top = wrapperEl.getBoundingClientRect().top;
    const height = Math.max(200, window.innerHeight - top - 16);
    wrapperEl.style.height = height + "px";
    canvasEl.width = wrapperEl.clientWidth;
    canvasEl.height = wrapperEl.clientHeight;
    draw();
  }

  function draw() {
    ctx.clearRect(0, 0, canvasEl.width, canvasEl.height);
    const ref = referencePoint();
    if (!ref) {
      ctx.fillStyle = "#6c757d";
      ctx.font = "14px sans-serif";
      ctx.fillText("Waiting for a position fix…", 12, 24);
      return;
    }

    const offsetLayers = {};
    for (const name in layers) {
      offsetLayers[name] = layers[name].points.map((p) => ({ ...p, ...offsetMetres(p.lat, p.lon, ref) }));
    }
    const currentOffset = current ? offsetMetres(current.lat, current.lon, ref) : null;

    const shortSide = Math.min(canvasEl.width, canvasEl.height);
    let metresAcross;
    if (zoomMode === "auto") {
      const all = Object.values(offsetLayers).flat();
      if (currentOffset) all.push(currentOffset);
      const extent = all.reduce((m, p) => Math.max(m, Math.abs(p.north), Math.abs(p.east)), 1.0);
      metresAcross = Math.max(2.0, extent * 2 * 1.25);
    } else {
      metresAcross = zoomMode;
    }

    const metresPerPx = metresAcross / shortSide;
    const cx = canvasEl.width / 2;
    const cy = canvasEl.height / 2;
    const toCanvas = (p) => ({ x: cx + p.east / metresPerPx, y: cy - p.north / metresPerPx });

    const spacing = niceSpacing(metresAcross, 7);
    const halfWidthM = cx * metresPerPx;
    const halfHeightM = cy * metresPerPx;
    ctx.strokeStyle = "#dee2e6";
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = Math.ceil(-halfWidthM / spacing); i * spacing <= halfWidthM; i++) {
      const x = cx + (i * spacing) / metresPerPx;
      if (x >= 0 && x <= canvasEl.width) { ctx.moveTo(x, 0); ctx.lineTo(x, canvasEl.height); }
    }
    for (let i = Math.ceil(-halfHeightM / spacing); i * spacing <= halfHeightM; i++) {
      const y = cy - (i * spacing) / metresPerPx;
      if (y >= 0 && y <= canvasEl.height) { ctx.moveTo(0, y); ctx.lineTo(canvasEl.width, y); }
    }
    ctx.stroke();

    for (const name in offsetLayers) {
      const style = layers[name].style;
      const pts = offsetLayers[name];
      const color = style.color || "#6c757d";
      if (style.line !== false && pts.length > 1) {
        ctx.strokeStyle = color;
        ctx.lineWidth = style.lineWidth || 2;
        ctx.beginPath();
        pts.forEach((p, i) => {
          const c = toCanvas(p);
          if (i === 0) ctx.moveTo(c.x, c.y); else ctx.lineTo(c.x, c.y);
        });
        ctx.stroke();
      }
      if (style.dots) {
        pts.forEach((p, i) => {
          const c = toCanvas(p);
          ctx.fillStyle = color;
          ctx.beginPath();
          ctx.arc(c.x, c.y, style.dotRadius || 4, 0, 2 * Math.PI);
          ctx.fill();
          if (style.labels) {
            ctx.fillStyle = "#212529";
            ctx.font = "12px sans-serif";
            ctx.fillText(String(style.labelFn ? style.labelFn(p, i) : i), c.x + 6, c.y - 6);
          }
        });
      }
    }

    if (currentOffset) {
      const c = toCanvas(currentOffset);
      ctx.fillStyle = "#0d6efd";
      ctx.beginPath();
      ctx.arc(c.x, c.y, 5, 0, 2 * Math.PI);
      ctx.fill();
    }

    const barMetres = spacing;
    const barPx = barMetres / metresPerPx;
    const bx = 16, by = canvasEl.height - 16;
    ctx.strokeStyle = "#212529";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(bx, by); ctx.lineTo(bx + barPx, by);
    ctx.moveTo(bx, by - 5); ctx.lineTo(bx, by + 5);
    ctx.moveTo(bx + barPx, by - 5); ctx.lineTo(bx + barPx, by + 5);
    ctx.stroke();
    ctx.fillStyle = "#212529";
    ctx.font = "12px sans-serif";
    ctx.fillText(`${barMetres} m`, bx, by - 8);
  }

  window.addEventListener("resize", resize);

  return { setLayer, setCurrent, setZoom, resize, draw };
}
