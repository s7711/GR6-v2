"""Bore-sight service glue: the capture thread, the solve, and writing
the answer back to config.yaml. aruco/app.py holds the HTTP routes; this
holds everything behind them, so the routes stay thin and this stays
testable without a Flask app.
"""

import logging
import math
import re
import shutil
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import survey  # noqa: E402  (aruco/survey.py — marker pose from one detection)

import capture  # noqa: E402
import drive as drive_mod  # noqa: E402
import placement  # noqa: E402

DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "boresight"
# Where manager keeps its config backups — reused rather than inventing a
# second backup location, so every config change on this robot lands in
# one place regardless of which page made it.
CONFIG_BACKUP_DIR = Path(__file__).resolve().parent.parent.parent / "manager" / "config-backup"

MARKER_IDS = (20, 21, 22)
MARKER_SPACING_M = 0.5


def _continuity_warnings_worth_showing(warnings, steps):
    """Drop jobs' continuity warnings that a turn step already answers.

    jobs checks continuity between each run_path and the NEXT run_path,
    skipping whatever sits between — which is right for pause/water steps
    (they don't move the robot) but wrong for turn_to_heading, whose
    entire job is to change the heading. Every leg here reverses
    direction and is followed by a turn, so the raw check reports a ~180
    degree heading error on all twelve joins and buries any real problem.

    A warning is kept if the legs are genuinely far apart (a gap no turn
    can fix), or if no turn step sits between them.
    """
    kept = []
    for warning in warnings:
        after, before = warning.get("after_step"), warning.get("before_step")
        turned = any(step.get("type") == "turn_to_heading"
                     for step in steps[(after or 0) + 1:before or 0])
        if turned and warning.get("distance_m", 0.0) <= 1.0:
            continue
        kept.append(warning)
    return kept


class BoresightService:
    def __init__(self, cfg, nav_client, frame_reader_fn, detect_fn, marker_size,
                 hpr_cb, dxc_b, camera_cal_fn, paths_dir, data_dir=DATA_DIR,
                 navigate_base_url=None, jobs_base_url=None, max_detection_hz=2.0):
        self.cfg = cfg
        self.driver = (drive_mod.JobDriver(navigate_base_url, jobs_base_url)
                       if navigate_base_url and jobs_base_url else None)
        self.run_state = {"state": "idle", "message": "", "passes_done": 0, "passes": 0}
        self.nav_client = nav_client
        self.frame_reader_fn = frame_reader_fn
        self.detect_fn = detect_fn
        self.marker_size = marker_size
        self.hpr_cb = tuple(hpr_cb)
        self.dxc_b = tuple(dxc_b)
        self.camera_cal_fn = camera_cal_fn
        self.paths_dir = Path(paths_dir)
        self.data_dir = Path(data_dir)
        # Same cap aruco's own detection loop uses (config's
        # max_detection_hz), for the same underlying reason plus one of
        # our own:
        #   * statistically it buys nothing — INS attitude error is
        #     correlated over tens of seconds, so frames closer together
        #     than that carry almost no independent information. The
        #     design study found session LENGTH mattered, not frame rate.
        #   * capture runs alongside aruco's own 2Hz detection loop, so
        #     leaving this uncapped had the Pi doing ~7Hz of ArUco
        #     detection during a run (measured 4.2Hz here plus aruco's 2).
        self.min_frame_period_s = 1.0 / max_detection_hz if max_detection_hz else 0.0

        self.nav_buffer = capture.NavBuffer()
        self.gate = capture.Gate()
        self.session = None
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self.last_result = None

    # --- placement ------------------------------------------------------

    def placement_plan(self, box_path_name, height_m, ins_height_m):
        """Validate the boundary path and say where the markers go."""
        file = self.paths_dir / f"{box_path_name}.yaml"
        if not file.exists():
            return {"error": f"no saved path named {box_path_name!r}"}
        points = yaml.safe_load(file.read_text()) or []
        info, targets = placement.marker_targets(
            points, MARKER_IDS, height_m, MARKER_SPACING_M, ins_height_m)
        out = {
            "is_box": info["is_box"],
            "problems": info["problems"],
            "corners_deg": [round(c, 1) for c in info["corners_deg"]],
            "side_lengths_m": [round(s, 2) for s in info["side_lengths_m"]],
            "targets": targets,
            "box": [{"lat": p["lat"], "lon": p["lon"]} for p in points],
            "marker_size_note": (
                f"size is the BLACK SQUARE's edge ({self.marker_size} m), not the "
                "printed sheet including its white border"
            ),
        }
        if info["is_box"]:
            out["face_bearing_deg"] = round(info["face_bearing_deg"], 1)
            out["clearance_m"] = round(info["clearance_m"], 2)
        return out

    def placement_guidance(self, targets):
        """Live placement help for someone standing in the garden.

        Two kinds, because they're useful at different moments:

        * range/bearing FROM THE ROBOT to each target — for finding the
          spot before the marker exists (drive the robot there, watch it
          count to zero);
        * how to NUDGE a marker that's already planted — only available
          while the camera can actually see it, since it needs the
          marker's real pose. The robot can't be parked where the marker
          must go, so range-from-robot is useless at that point.
        """
        payload = self.nav_client.latest()
        nav = payload.get("nav", {})
        if not nav.get("Lat"):
            return {"error": "no nav fix"}
        lat, lon = float(np.degrees(nav["Lat"])), float(np.degrees(nav["Lon"]))
        out = {
            "robot": {"lat": lat, "lon": lon, "heading_deg": nav.get("Heading")},
            "targets": [placement.guidance_from(lat, lon, t) for t in targets],
            "visible": {},
        }
        for marker_id, actual in self._visible_marker_poses(nav).items():
            target = next((t for t in targets if int(t["id"]) == marker_id), None)
            if target is None:
                continue
            out["visible"][marker_id] = placement.move_guidance(
                float(nav["Heading"]), actual, target)
        return out

    def _visible_marker_poses(self, nav):
        """{id: {lat, lon, heading}} for every bore-sight marker the
        camera can currently see, surveyed from a single detection.

        Uses the CURRENT hpr_cb, which is the thing being calibrated — so
        these poses carry whatever the mount error is (a few degrees at
        2.5m is ~10cm). That's irrelevant here: placement tolerance is
        tens of centimetres and every marker pose is a free parameter in
        the solve anyway. It would matter if this were a survey; it isn't.
        """
        frame_reader = self.frame_reader_fn()
        camera_matrix, dist_coeffs = self.camera_cal_fn()
        if frame_reader is None or camera_matrix is None:
            return {}
        read = frame_reader.read()
        if read is None or read["frame"].size == 0:
            return {}
        try:
            detections = self.detect_fn(read["frame"], camera_matrix, dist_coeffs)
        except Exception:
            logging.exception("[boresight] Detection failed while giving placement guidance")
            return {}
        poses = {}
        for d in detections:
            if int(d["id"]) not in MARKER_IDS:
                continue
            try:
                poses[int(d["id"])] = survey.survey_marker(
                    nav, d, self.hpr_cb, self.dxc_b, (0.0, 0.0, 0.0))
            except (KeyError, ZeroDivisionError, ValueError):
                continue
        return poses

    # --- capture --------------------------------------------------------

    def start_capture(self):
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return {"error": "already capturing"}
            stamp = datetime.now().strftime("%y%m%d_%H%M%S")
            self.session = capture.Session(
                self.data_dir / f"{stamp}.jsonl", MARKER_IDS, self.marker_size,
                self.hpr_cb, self.dxc_b)
            self._stop.clear()
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()
        return self.capture_status()

    def stop_capture(self):
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=5.0)
        status = self.capture_status()
        with self._lock:
            if self.session is not None:
                self.session.close()
        return status

    def capture_status(self):
        with self._lock:
            running = self._thread is not None and self._thread.is_alive()
            status = self.session.status() if self.session else None
        return {"running": running, "session": status,
                "run": dict(self.run_state),
                "nav_buffer": {"samples": len(self.nav_buffer),
                               "span_s": round(self.nav_buffer.span_s(), 2)}}

    def _capture_loop(self):
        camera_matrix, dist_coeffs = self.camera_cal_fn()
        frame_reader = self.frame_reader_fn()
        if camera_matrix is None or frame_reader is None:
            logging.error("[boresight] Cannot capture: no camera calibration or frame feed")
            return
        wanted = set(MARKER_IDS)
        last_seq_time = None
        last_detect_at = 0.0
        # Frames wait here until nav catches up to them — see _drain_pending.
        pending = []
        while not self._stop.is_set():
            payload = self.nav_client.latest()
            self.nav_buffer.add(payload.get("nav", {}), payload.get("status", {}),
                                payload.get("connection", {}).get("timeOffset"))
            self._drain_pending(pending)

            now = time.monotonic()
            if now - last_detect_at < self.min_frame_period_s:
                time.sleep(0.02)
                continue

            read = frame_reader.read()
            if read is None or read["frame"].size == 0 or read["timestamp"] == last_seq_time:
                time.sleep(0.02)
                continue
            last_seq_time = read["timestamp"]
            last_detect_at = now

            try:
                detections = self.detect_fn(read["frame"], camera_matrix, dist_coeffs)
            except Exception:
                logging.exception("[boresight] Detection failed on a frame, skipping")
                continue
            wanted_dets = [d for d in detections if int(d["id"]) in wanted]
            if not wanted_dets:
                self.session.reject("no bore-sight marker in view")
                continue
            # Detections are small; the frame itself is not kept.
            pending.append((read["timestamp"], [
                {"id": int(d["id"]), "size": float(d["size"]), "corners": d["corners"]}
                for d in wanted_dets]))
        self._drain_pending(pending, final=True)

    def _drain_pending(self, pending, final=False):
        """Match buffered detections against nav as soon as nav reaches
        their timestamp.

        Nav samples arrive about 0.35s behind real time, while a frame's
        SensorTimestamp is only ~0.05-0.2s old — so a freshly captured
        frame is usually NEWER than the newest nav sample and cannot be
        bracketed for interpolation yet. Resolving each frame immediately
        therefore threw away 47 of 113 frames (42%) in a live test, all
        reported as "no time-matched nav".

        Waiting costs nothing but a moment's latency: the detections are
        already computed, and interpolation needs a sample on each side.
        """
        still_waiting = []
        for gps_s, detections in pending:
            nav = self.nav_buffer.at(gps_s)
            if nav is None:
                # Too new to bracket yet? Keep it. Too old for the buffer
                # to still hold (or shutting down)? It will never resolve.
                if not final and gps_s > self.nav_buffer.newest():
                    still_waiting.append((gps_s, detections))
                else:
                    self.session.reject("no time-matched nav")
                continue
            reason = self.gate.check(nav)
            if reason:
                self.session.reject(reason)
                continue
            self.session.add(gps_s, nav, detections)
        pending[:] = still_waiting

    # --- drive + capture, as one operation ------------------------------

    def build_capture_job(self, box_path_name, height_m, ins_height_m):
        """Generate the capture legs and the job that sequences them, and
        save both — so the run can also be started by hand from the jobs
        page if anything goes wrong here.

        Includes a lead-in from wherever the robot is actually parked:
        without it the chain just starts at leg 0 wherever that happens
        to fall, and Ben had to drive the robot to the start by hand
        (2026-09-16)."""
        plan = self.placement_plan(box_path_name, height_m, ins_height_m)
        if plan.get("error") or not plan["is_box"]:
            return {"error": plan.get("error") or "boundary path is not a box",
                    "problems": plan.get("problems", [])}
        panel = plan["targets"][len(plan["targets"]) // 2]  # middle marker = panel centre
        file = self.paths_dir / f"{box_path_name}.yaml"
        robot_lat = robot_lon = robot_heading_deg = None
        nav = self.nav_client.latest().get("nav", {})
        if "Lat" in nav and "Lon" in nav and "Heading" in nav:
            robot_lat, robot_lon = math.degrees(nav["Lat"]), math.degrees(nav["Lon"])
            robot_heading_deg = nav["Heading"]
        leg_paths, steps, info = drive_mod.build_job(
            yaml.safe_load(file.read_text()) or [],
            plan["face_bearing_deg"], panel["lat"], panel["lon"],
            robot_lat=robot_lat, robot_lon=robot_lon, robot_heading_deg=robot_heading_deg)
        if leg_paths is None:
            return {"error": "boundary path is not a box"}
        warnings = {}
        if self.driver is not None:
            warnings = self.driver.save(leg_paths, steps) or {}
        real_warnings = _continuity_warnings_worth_showing(warnings.get("warnings", []), steps)
        minutes = info["seconds"] / 60.0
        return {
            "job_name": drive_mod.JOB_NAME,
            "legs": info["leg_count"],
            "steps": len(steps),
            "turns": info["turn_count"],
            "turns_left": info["turns_left"],
            "turns_right": info["turns_right"],
            "waypoints_pulled_in": info["waypoints_pulled_in"],
            "length_m": round(info["length_m"], 1),
            "minutes_per_pass": round(minutes, 1),
            "face_bearing_deg": plan["face_bearing_deg"],
            "continuity_warnings": real_warnings,
        }

    def start_run(self, box_path_name, height_m, ins_height_m, passes=4):
        """Build the path, start capturing, then drive the pattern
        `passes` times. ~20 minutes of driving is where the simulator
        showed accuracy plateauing."""
        if self.driver is None:
            return {"error": "navigate URL not configured"}
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return {"error": "already running"}
        built = self.build_capture_job(box_path_name, height_m, ins_height_m)
        if built.get("error"):
            return built
        started = self.start_capture()
        if started.get("error"):
            return started
        self.run_state = {"state": "starting", "message": "", "passes_done": 0, "passes": passes}
        threading.Thread(target=self._run_loop, args=(passes,), daemon=True).start()
        return {"built": built, "capture": started}

    def _run_loop(self, passes):
        try:
            for i in range(passes):
                if self._stop.is_set():
                    break
                result = self.driver.start()
                if result.get("state") not in ("running", None) and result.get("ok") is False:
                    self.run_state = {"state": "blocked", "passes_done": i, "passes": passes,
                                      "message": f"jobs won't start: {result.get('reason')}"}
                    break
                self.run_state = {"state": "driving", "passes_done": i, "passes": passes,
                                  "message": f"pass {i + 1} of {passes}"}
                final = self.driver.wait_until_idle(should_stop=self._stop.is_set)
                if final.get("state") == "aborted":
                    self.run_state = {"state": "blocked", "passes_done": i + 1, "passes": passes,
                                      "message": f"job aborted: {final.get('abort_reason')}"}
                    break
                self.run_state = {"state": "driving", "passes_done": i + 1, "passes": passes,
                                  "message": f"pass {i + 1} of {passes} done"}
            else:
                self.run_state = {"state": "done", "passes_done": passes, "passes": passes,
                                  "message": "all passes complete"}
        except Exception as exc:  # noqa: BLE001 - surfaced to the page, not swallowed
            logging.exception("[boresight] Drive loop failed")
            self.run_state = {"state": "error", "message": str(exc),
                              "passes_done": self.run_state.get("passes_done", 0),
                              "passes": passes}
        finally:
            try:
                self.driver.stop()
            finally:
                self.stop_capture()

    def stop_run(self):
        self._stop.set()
        if self.driver is not None:
            self.driver.stop()
        self.run_state = {**self.run_state, "state": "stopped"}
        return self.stop_capture()

    # --- solve ----------------------------------------------------------

    def solve_session(self, session_path, estimate_size_scale=True, n_groups=4):
        import solve as solve_mod  # local: keeps scipy off the import path until needed

        header, rows = capture.load_session(session_path)
        if not rows:
            return {"error": "no observations in that session"}
        ref = rows[0]
        obs = capture.session_to_observations(
            rows, ref["lat"], ref["lon"], ref["alt"], marker_ids=MARKER_IDS)
        if obs is None or len(obs) < 100:
            return {"error": f"too few observations ({0 if obs is None else len(obs)})"}
        seen = set(np.unique(obs.marker_idx).tolist())
        if len(seen) < len(MARKER_IDS):
            return {"error": "not every marker was seen — the solve would be singular"}

        geometry = solve_mod.geometry_check(obs)
        if not geometry["ok"]:
            # Refuse rather than report: a degenerate solve looks healthy
            # (low residual, success=True) and is completely wrong.
            return {"error": "not enough viewpoint diversity to determine hpr_cb — "
                             "was the capture path actually driven?",
                    "geometry": geometry}

        camera_matrix, dist_coeffs = self.camera_cal_fn()
        out = solve_mod.solve(obs, self.hpr_cb, self.dxc_b, camera_matrix, dist_coeffs,
                              huber=3.0, estimate_size_scale=estimate_size_scale)
        split = solve_mod.split_check(obs, self.hpr_cb, self.dxc_b, camera_matrix,
                                      dist_coeffs, n_groups=n_groups, huber=3.0)

        result = {
            "session": str(session_path),
            "n_obs": out["n_obs"],
            "hpr_cb": [round(float(v), 4) for v in out["hpr_cb"]],
            # The formal sigma assumes independent corner noise and a
            # perfect INS. It reads far too small, because the real error
            # is dominated by INS attitude error correlated over tens of
            # seconds. The split-group scatter is the number to quote.
            "sigma_formal": [round(float(v), 4) for v in out["hpr_cb_sigma_formal"]],
            "sigma_split": [round(float(v), 4) for v in split["scatter"]],
            "split_estimates": [[round(float(v), 4) for v in e] for e in split["estimates"]],
            "rms_px": round(out["rms_px"], 3),
            "chi2_reduced": round(out["chi2_reduced"], 2),
            "size_scale": round(out["size_scale"], 5),
            "implied_marker_size_m": round(out["size_scale"] * self.marker_size, 5),
            "marker_ids": out["marker_ids"],
            "marker_hpr": [[round(float(v), 2) for v in h] for h in out["marker_hpr"]],
            "hpr_cb_at_capture": header.get("hpr_cb_at_capture") if header else None,
            "geometry": geometry,
            "success": out["success"],
        }
        self.last_result = result
        return result

    def list_sessions(self):
        if not self.data_dir.is_dir():
            return []
        out = []
        for f in sorted(self.data_dir.glob("*.jsonl"), reverse=True):
            header, rows = capture.load_session(f)
            out.append({"name": f.name, "observations": len(rows),
                        "started_at": header.get("started_at") if header else None})
        return out

    # --- apply ----------------------------------------------------------

    def apply_to_config(self, config_path, hpr_cb):
        """Write hpr_cb into config.yaml, preserving comments.

        A targeted line edit rather than a YAML round-trip: yaml.safe_dump
        would strip every comment in the file, and config.yaml's comments
        carry a lot of this project's history (tuning dates, why a value
        is what it is). Backed up into manager's own config-backup/ first,
        the same place its config editor puts them.
        """
        config_path = Path(config_path)
        text = config_path.read_text()
        pattern = re.compile(r"^(\s*hpr_cb:\s*)\[[^\]]*\](.*)$", re.MULTILINE)
        matches = pattern.findall(text)
        if len(matches) != 1:
            return {"error": f"expected exactly one hpr_cb line in config.yaml, found {len(matches)}"}

        values = ", ".join(f"{float(v):.3f}" for v in hpr_cb)
        stamp = datetime.now().strftime("%y%m%d_%H%M%S")
        CONFIG_BACKUP_DIR.mkdir(parents=True, exist_ok=True)
        backup = CONFIG_BACKUP_DIR / f"config_{stamp}.yaml"
        shutil.copy2(config_path, backup)

        def _replace(m):
            comment = m.group(2)
            note = f"  # bore-sight calibrated {datetime.now().strftime('%Y-%m-%d')}"
            if "#" in comment:
                comment = note
            return f"{m.group(1)}[{values}]{comment or note}"

        config_path.write_text(pattern.sub(_replace, text, count=1))
        return {"written": [round(float(v), 3) for v in hpr_cb],
                "backup": str(backup),
                "note": "aruco must be restarted to pick this up"}
