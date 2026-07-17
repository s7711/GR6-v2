# UI / Appearance Style Guide: GR6-v2

This is the umbrella document for appearance/UI, the same way `top-prd.md`
is the umbrella document for architecture. Any service PRD that includes a
web UI should reference this document rather than redefining layout/colour/
framework decisions locally.

## Framework

- **Bootstrap** (5.3+, via CDN link or vendored copy — no build step
  needed). Chosen over a classless framework (Pico/Water) because the UI
  in scope is a genuine multi-widget dashboard (header bar, menu, cards,
  status badges, tables, forms) rather than a couple of simple content
  pages — Bootstrap's premade components (navbar, badges, cards,
  list-group, responsive grid) cover most of that directly, whereas a
  classless framework would leave all of it to be hand-rolled anyway.
- Optional: a Bootswatch theme on top of stock Bootstrap for a bit of
  visual identity, applied consistently across every service. Not
  decided yet — stock Bootstrap is a fine default until/unless we pick one.
- Not using Tailwind or hand-written CSS — more setup/effort than this
  project needs.

## Colour

- No custom palette for now — use Bootstrap's default theme (or the
  chosen Bootswatch theme) as-is. Custom brand colours can be revisited
  later if wanted; not worth design effort now.
- Use Bootstrap's contextual colours for status semantics consistently
  across every service, so colour means the same thing everywhere:
  - `success` (green) — running / good / in range
  - `warning` (yellow) — degraded / attention
  - `danger` (red) — stopped / fault / out of range
- Support both light and dark mode via Bootstrap 5.3+'s built-in
  `data-bs-theme`, following the OS/browser preference — the robot gets
  accessed from both a phone and a desktop, at different times of day.

## Layout

- Standard page skeleton on every service: a header bar (Bootstrap
  navbar) + main content area in a responsive grid (`container-fluid` +
  `row`/`col`), reflowing to a single column at phone widths.
- **Header bar:** shows system-wide status at a glance — wifi, battery,
  nav status, robot status — as icons/badges. Shared across every
  service's page so the chrome looks identical everywhere; implemented
  once as a shared template partial (see Shared assets below), not
  reimplemented per service.
- **Menu:** navbar links, or an offcanvas sidebar if a service's own
  navigation gets non-trivial. Each service can add its own items, but
  the container/style comes from the shared partial.
- **Manager home page** stays the icon-grid launcher described in
  `manager/manager-prd.md` — that page is intentionally sparser than a
  per-service dashboard page.

## Component mapping

| Need | Approach |
|---|---|
| Edit boxes, dropdowns, sliders (config) | Bootstrap form controls: `form-control`, `form-select`, `form-range` |
| Tables | Bootstrap `table` classes |
| File-like lists (paths, watered patches) | Bootstrap `list-group` |
| Text "measurement" readouts (speed, heading) | Custom small stat component — label + large number + unit, built once, reused everywhere (e.g. a `card` with large text) |
| Real-time scrolling graphs | **uPlot** — small footprint, built for fast-updating time series. Not Chart.js (works, but heavier/janks more at high update rates); not attached to this choice, can swap later if needed |
| Camera view | Plain `<img>` pointed at the MJPEG stream endpoint — no library needed |
| Robot overview / ultrasonic "parking sensor" diagram | Custom SVG/canvas widget, hand-built (no framework provides this) — define the visual language once (robot outline + sensor arcs, colour = distance via the `success`/`warning`/`danger` scale above) and share it across any service that needs it |
| Service/page icons | Each service supplies its own `icon.<ext>` (see `manager/manager-prd.md`); lookup is format-agnostic (SVG or bitmap), so a starter hand-coded SVG can be swapped for a nicer bitmap later without any code change; reused in that service's own header where useful |

- Graph update rate: target **2Hz** for real-time scrolling graphs,
  regardless of underlying data rate. The xNAV650 can produce up to
  100Hz, but there's no reason to push every raw sample over wifi to a
  browser just to decimate it visually — whichever service owns that
  websocket feed should throttle/decimate before sending, not the
  graph-rendering code.

## Responsive: phone + desktop

- Must look right at both phone widths (~375–430px) and desktop. Use
  Bootstrap's grid breakpoints; multi-panel pages (camera + graphs +
  sensor diagram together) should stack vertically on narrow screens
  rather than being truncated or requiring horizontal scroll.
- Checked manually at common breakpoints for now — no automated visual
  regression tooling planned (consistent with no other test prior art in
  this codebase yet).

## Shared assets

- Because the header/menu chrome, stat-readout style, status colour
  semantics, and graph styling are meant to look identical across
  services, put the shared pieces in `shared/` (the folder `top-prd.md`
  already sets aside for shared code), e.g. `shared/web/`: a base
  template, any CSS overrides on top of Bootstrap, a small uPlot wrapper
  helper, the stat-readout template, and the sensor-diagram helper.
  Services import/extend these rather than re-implementing them.

## Out of scope / future

- Point-cloud or other 3D visualisation — long way off, not designed for.
- Exact icon dimensions/style guidelines beyond "format-agnostic
  `icon.<ext>`" — settle when the first real (non-placeholder) icon set
  is actually made.
- A distinct custom colour palette/branding beyond Bootstrap defaults —
  can revisit later if wanted.
