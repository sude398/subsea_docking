# Subsea Docking — Change Log

This document records every modification applied across the sessions, the reasoning behind each, and the constraints respected. Sessions 1-7 were all **additive/minimal**: no architectural rewiring, no new FSM states, no PID class changes, no modifications to `updated_vision.py` or `updated_controller.py`. Session 8 relaxed the FSM-topology constraint (by user request) and refactored states; Session 9 added a runnable integration test.

Files touched:
- `updated_config.py` — 2 constants added (Session 1)
- `updated_docking_node.py` — cumulative patches across Sessions 1-8
- `test_fsm_integration.py` — new file, Session 9

Files explicitly **not** touched across any session:
- `updated_vision.py`
- `updated_controller.py` (PID and Kalman class bodies unchanged)

---

## Session 1 — vision_failure_agent

**Problem.** At close range, ArUco markers leave the camera's field of view because of the 45° forward pitch. The FSM treated this as a failure and reverted `APPROACHING -> SEARCHING` after 3 s, undoing docking progress. Pose values also reset to `0,0,999` every frame while blind, so downstream state checks saw impossible readings.

**Reasoning.** Marker loss near the target is a geometric inevitability, not a sensor failure. The FSM needs to (a) retain the last-known pose so transitions that depend on `y_w` / `d_v` keep working, and (b) suppress the `SEARCHING` fallback when we are genuinely close and were seeing markers moments ago — but only temporarily, so a real loss at distance still triggers recovery.

**Changes.**

1. `updated_config.py` — two constants so the blind-mode window is bounded and tunable:
   ```python
   BLIND_DESCENT_DIST = 1.5      # Below this horizontal distance FOV loss is expected (m)
   BLIND_DESCENT_TIMEOUT = 8.0   # Max blind-mode duration before giving up (s)
   ```
2. `updated_docking_node.py` — imports extended to bring both constants into scope.
3. `__init__` — pose memory:
   ```python
   self.last_x_w = 0.0
   self.last_y_w = 0.0
   self.last_d_h = 999.0
   self.last_d_v = 999.0
   ```
4. `loop()` — snapshot pose when visible; fall back to it when blind, replacing the previous zeroing reset:
   ```python
   if visible and target_tvec is not None:
       ...
       self.last_x_w, self.last_y_w = x_w, y_w
       self.last_d_h, self.last_d_v = d_h, d_v
       ...
   else:
       x_w, y_w = self.last_x_w, self.last_y_w
       d_h, d_v = self.last_d_h, self.last_d_v
   ```
5. `APPROACHING` — guarded the SEARCHING fallback:
   ```python
   blind_active = (
       self.last_d_h < BLIND_DESCENT_DIST
       and (time.time() - self.last_seen_t) < BLIND_DESCENT_TIMEOUT
   )
   if not visible and (time.time() - self.last_seen_t > 3.0) and not blind_active:
       self.state = "SEARCHING"
   elif self.confirmed_4_markers and y_w < CAMERA_OFFSET_Y:
       self.state = "DESCENDING"
   ```
   Because `y_w` is now the last-known value when blind, the `y_w < CAMERA_OFFSET_Y` trigger can fire in blind mode.

**Respected constraints.** No FSM states added. No modifications to `ALIGNING` / `DESCENDING` / `FINAL_ALIGN` transitions. `SEARCHING` logic unchanged. Vision module untouched. Behavior outside close-range blind windows is byte-for-byte identical.

---

## Session 2 — docking_debug_agent

**Problem.** Vehicle reaches `DESCENDING` but oscillates and appears to "revert". User report: "goes into DESCENDING then becomes unstable and may revert or oscillate".

**Root causes identified.**

- **Lateral flicker.** The command block zeroed `vx, vy` when visible during descent but restored `last_v * 0.8` when blind. Close-range visibility toggles every few frames due to FOV loss, so lateral commands toggled `0 ↔ non-zero` at 25 Hz — visible motor wobble.
- **No hysteresis on noisy `d_v`.** `d_v` is the raw `solvePnP` output (not Kalman-smoothed). A single noisy frame crossing `FINAL_ALIGN_DIST` (0.45 m) instantly stepped `vz` from `-0.15` to `-0.08`, producing a visible throttle jerk perceived as "reverting".

**Reasoning.** Two fixes are sufficient: make lateral commands unconditionally zero in descent states regardless of visibility, and require sustained threshold crossing (persistence counter + margin) before committing to a state change.

**Changes.**

1. `__init__` — persistence counters:
   ```python
   self.desc_stable_cnt = 0
   self.final_stable_cnt = 0
   ```
2. Command block, not-visible branch — force descent states to pure vertical:
   ```python
   if self.state in ["DESCENDING", "FINAL_ALIGN"]:
       vx, vy, vyaw = 0.0, 0.0, 0.0
   elif self.state in ["APPROACHING", "ALIGNING"]:
       vx, vy, vyaw = self.last_vx * 0.8, self.last_vy * 0.8, self.last_vyaw * 0.8
   ```
3. `DESCENDING` — hysteresis margin + persistence (3 consecutive frames ≈ 120 ms at 25 Hz):
   ```python
   if d_v < (FINAL_ALIGN_DIST - 0.03):
       self.desc_stable_cnt += 1
   else:
       self.desc_stable_cnt = 0
   if self.desc_stable_cnt >= 3:
       self.state = "FINAL_ALIGN"
       self.desc_stable_cnt = 0
   ```
4. `FINAL_ALIGN` — same pattern with a 0.02 m margin for `DOCKED`.

**Respected constraints.** No FSM states added, no PID/Kalman changes, no vision changes, `ALIGNING` / `APPROACHING` transitions untouched.

---

## Session 3 — control_tuning_agent

**Problem.** Oscillation near target; sudden upward movement after descent.

**Root causes.**

- Constant `vz` values (`-0.15 → -0.08 → 0.0`) created discrete throttle steps at state transitions. MAVLink `manual_control` throttle maps `500 + vz * 1000`, so a jump of `0.08` is a `+80` throttle-unit step in one 40 ms tick. On a slightly positively-buoyant vehicle, the `-0.08 → 0.0` step at `DOCKED` releases down-thrust and lets buoyancy push the vehicle up.
- Kalman output jitters at cm scale; with `kp=0.7` the PID amplifies this into lateral buzz even when the error is physically negligible.

**Reasoning.** Four narrow fixes, none touching the PID class: make descent `vz` a continuous function of distance so there are no steps; low-pass filter the final `vz` command before sending so remaining steps are smoothed; add an **input-side** deadband in the node (the PID class's internal output deadband is untouched).

**Changes.**

1. `__init__` — LPF state:
   ```python
   self.last_vz_cmd = 0.0
   ```
2. `loop()` — input deadband on PID calls (PID class still unmodified):
   ```python
   APPROACH_ERR_DB = 0.03
   YAW_ERR_DB = 0.05
   dx_in   = 0.0 if abs(x_w) < APPROACH_ERR_DB else x_w
   dy_in   = 0.0 if abs(y_w) < APPROACH_ERR_DB else y_w
   dyaw_in = 0.0 if abs(target_yaw) < YAW_ERR_DB else -target_yaw

   self.last_vx   = self.pid_x.update(dy_in)
   self.last_vy   = self.pid_y.update(dx_in)
   self.last_vyaw = self.pid_yaw.update(dyaw_in)
   ```
   Under threshold, PID sees `e = 0`, hits its own small-error branch (`last_out *= 0.5`), and decays to zero — motor buzz stops.
3. `DESCENDING` — adaptive `vz`:
   ```python
   vz = max(-0.15, -0.05 - 0.1 * d_v)   # -0.15 far, -0.05 at target
   ```
4. `FINAL_ALIGN` — gentler adaptive `vz`:
   ```python
   vz = max(-0.08, -0.02 - 0.2 * d_v)   # -0.08 far, -0.02 at target
   ```
5. Command block, immediately before `self.send()` — first-order LPF on `vz`:
   ```python
   vz = 0.8 * self.last_vz_cmd + 0.2 * vz
   self.last_vz_cmd = vz
   ```
   α = 0.2 at 25 Hz → ~160 ms time constant. Residual steps between `DESCENDING → FINAL_ALIGN → DOCKED` vanish.

**Respected constraints.** PID class literally unchanged — all tuning is in the call sites. Kalman untouched. FSM unchanged.

---

## Session 4 — search_strategy_agent

**Problem.** `SEARCHING` was a dumb two-step sequence: move forward 5 s, then spin CW forever. Inefficient, gets stuck, no memory of recent sightings.

**Reasoning.** Replace only the `else:` branch inside the `SEARCHING` FSM case. No entry/exit conditions change. Add three tiers:

1. If markers were seen recently and we know their lateral side, yaw toward it first (cheap re-acquisition after blind-descent timeout).
2. A 12-s cyclic pattern with expanding-spiral + alternating sweep so the vehicle covers area and never commits to a single direction forever.
3. The cycle itself is the implicit "stuck breakout" — every 4 s the behavior changes; every 12 s it repeats from a new pose.

**Change.** One localized replacement in `SEARCHING`:

```python
else:
    elapsed = time.time() - self.search_start_t
    recent_lost = (time.time() - self.last_seen_t) < 15.0
    use_hint = recent_lost and abs(self.last_x_w) > 0.05 and elapsed < 3.0

    if use_hint:
        vx = 0.05
        vyaw = 0.25 * float(np.sign(self.last_x_w))
    else:
        base_t = elapsed - 3.0 if recent_lost else elapsed
        cycle_t = base_t % 12.0 if base_t > 0 else 0.0

        if cycle_t < 4.0:
            vx = 0.12
            vyaw = 0.08 + 0.17 * (cycle_t / 4.0)   # spiral: ramping yaw
        elif cycle_t < 8.0:
            vx = 0.0
            vyaw = 0.25                              # + sweep
        else:
            vx = 0.0
            vyaw = -0.25                             # - sweep
```

**Respected constraints.** FSM structure unchanged. `SEARCHING` entry/exit still via `visible` gate. No new state variables (`last_x_w`, `last_seen_t`, `search_start_t` already existed).

---

## Session 5 — integration_test_agent (report only, no code changes)

Tracer test of all 6 state transitions confirmed:

- **Test 1** no marker: cyclic search pattern active; never exits. PASS.
- **Test 2** marker appears: `SEARCHING → ALIGNING` on first frame. PASS.
- **Test 3** close: `ALIGNING → APPROACHING` at `d_h < 3.0`. PASS.
- **Test 4** very close: `APPROACHING → DESCENDING` on `confirmed_4_markers ∧ y_w < -0.35`. PASS.
- **Test 5** (critical) marker loss during descent: `APPROACHING @ close`, `DESCENDING`, and `FINAL_ALIGN` all withstand marker loss without reverting to `SEARCHING`. PASS.
- **Test 6** final: `FINAL_ALIGN → DOCKED` via 3-frame hysteresis. PASS.

Two non-critical observations raised (addressed in Session 6):

- `DOCKED` state stops sending commands (relies on Pixhawk failsafe). Positively-buoyant vehicles may drift up briefly.
- FSM can stall in `DESCENDING` / `FINAL_ALIGN` if vision never returns — `d_v` stays frozen and hysteresis never triggers.

---

## Session 6 — follow-up fixes for non-critical issues

**Issue 1 — DOCKED must actively command zero.**

Reasoning: if we stop transmitting, the autopilot retains the last commanded value until its own failsafe timeout, which for a buoyant vehicle means brief upward drift. Actively sending `(0, 0, 0, 0)` every tick removes the wait. Also decay the LPF state so the first few post-dock ticks taper off any residual `vz` instead of stepping to literal zero.

Change — additive `else:` branch on the existing command block:
```python
else:
    # DOCKED: actively zero-command instead of waiting for Pixhawk failsafe
    self.last_vz_cmd *= 0.8
    self.send(0.0, 0.0, self.last_vz_cmd, 0.0)
```

**Issue 2 — completion fallback when vision is permanently lost.**

Reasoning: during a prolonged FOV loss in `DESCENDING` or `FINAL_ALIGN`, `d_v` is frozen at the last-seen value; if that value is above the transition threshold, hysteresis can never fire. Vehicle keeps physically descending but FSM never advances. The fix is a time-based fallback that progresses **forward** (never to `SEARCHING`) so the mission still completes.

Reused `BLIND_DESCENT_TIMEOUT` (already 8 s) for consistency with the `APPROACHING` blind guard. Both fallbacks are `elif` branches placed after the primary hysteresis path, so any vision reacquisition keeps the hysteresis authoritative:

```python
# DESCENDING
elif (not visible) and (time.time() - self.last_seen_t) > BLIND_DESCENT_TIMEOUT:
    self.state = "FINAL_ALIGN"
    self.desc_stable_cnt = 0
```

```python
# FINAL_ALIGN
elif (not visible) and (time.time() - self.last_seen_t) > BLIND_DESCENT_TIMEOUT:
    self.state = "DOCKED"
    self.final_stable_cnt = 0
```

**Respected constraints.** No architecture changes, no new states, no new config constants. Blind-descent guard in `APPROACHING` untouched. Frozen-pose snapshot untouched. Hysteresis counters untouched. Transitions remain forward-only; `SEARCHING` is never a target of any fallback.

---

## Session 7 — vision_confidence_agent

**Problem.** The perception layer's `visible` flag is binary. Any single-frame marker drop — from partial occlusion, mid-motion blur, or FOV edge — flips the entire system between "trust the measurement fully" and "ignore it, freeze pose." This caused three coupled symptoms:

- Spurious `ALIGNING → SEARCHING` transitions on 2-second flicker.
- PID control discontinuities at reacquire moments (fresh Kalman output vs. frozen pose).
- `last_seen_t` being reset by noisy single-frame detections, lengthening the effective failure timeout.

**Reasoning.** Replace the binary signal with a continuous `vision_confidence ∈ [0, 1]` that rises smoothly on detection and decays smoothly on loss. Use it as (a) a weight for blending current measurement against last-known pose, and (b) the primary gate the FSM reads instead of `visible`. This turns "binary snap" into a first-order filter with predictable timing.

**Constraints this session respected.** No FSM structure changes (yet — that came in Session 8). No PID changes. No MAVLink changes. No vision module changes. Localized to `updated_docking_node.py`.

**Changes.**

1. `__init__` — new continuous state:
   ```python
   self.vision_confidence = 0.0
   ```
2. `loop()` — confidence update (every tick, before pose handling):
   ```python
   markers_seen = visible and target_tvec is not None
   if markers_seen:
       self.vision_confidence = min(1.0, self.vision_confidence + 0.15)
   else:
       self.vision_confidence = max(0.0, self.vision_confidence - 0.08)
   conf = self.vision_confidence
   ```
   Build-up: 0 → 0.5 in ≈ 4 frames (160 ms @ 25 Hz). Decay: 1 → 0.1 in ≈ 12 frames (480 ms).
3. `loop()` — pose blending replaces freezing. When markers are seen:
   ```python
   x_w = conf * cur_x + (1 - conf) * self.last_x_w
   y_w = conf * cur_y + (1 - conf) * self.last_y_w
   d_v = conf * cur_d_v + (1 - conf) * self.last_d_v
   ```
   At `conf > 0.6` the live measurement dominates; at `conf ∈ [0.2, 0.6]` we blend; at `conf < 0.2` we rely mostly on last-known. When markers are not seen we fall back entirely to last-known (no zeroing).
4. `loop()` — gated memory update. `last_seen_t` and the `last_*` pose snapshots update only when `conf > 0.4`, so single-frame noise can't reset the blind-window timer.
5. FSM guards (still the old `ALIGNING` / `APPROACHING` topology at this point) switched from `not visible` to the 3-part predicate:
   ```python
   perception_lost = (
       self.vision_confidence < 0.1
       and (time.time() - self.last_seen_t) > 3.0
       and not blind_active
   )
   ```
6. Log line additive: `Conf:{vision_confidence:.2f}`.

**Behavioral deltas.**

- Lock-acquisition from `SEARCHING` no longer fires on a 1-frame spurious detection.
- PID always sees a blended (continuous) pose → smoother command output without any PID-tuning changes.
- Reacquire after brief loss: the first noisy `solvePnP` return contributes only ~15% weight instead of 100%, eliminating the "kick" at re-lock.

---

## Session 8 — fsm_redesign_agent

**Problem.** With a continuous `vision_confidence` in place, the old 6-state FSM felt mis-shaped:

- `ALIGNING` and `APPROACHING` had identical control code; the only distinction was the `d_h < DIST_ALIGN` threshold used to move between them. Two states for one behavior.
- There was no explicit place for "perception degraded, don't act on stale commands." Session 2's workaround — damp commands by 0.8× when `not visible` in `ALIGNING / APPROACHING` — was a behavioral hack scattered across the command block.
- FSM transitions were `visible`-flavored in spirit even after Session 7 refined them to `conf < 0.1`.

**Reasoning.** With the relaxed constraints on FSM topology granted in this session:

1. Merge `ALIGNING` + `APPROACHING` → single `HOMING` state. One state, one horizontal-approach controller, one set of transition conditions.
2. Introduce `HOLDING` as an explicit hover state for the "I've lost confidence but the task isn't over" case. This replaces the 0.8× damping hack with first-class FSM semantics.
3. Add asymmetric hysteresis between `HOMING` and `HOLDING` (enter at `conf < 0.3`, exit at `conf > 0.6`) to prevent chatter.
4. Strengthen the commit-to-landing gate: `HOMING → DESCENDING` requires `commit_safe = (conf > 0.6) ∨ blind_active`, so we only descend with either live confidence or the planned FOV-loss blind window.
5. Preserve the invariant that `DESCENDING` and `FINAL_ALIGN` only transition forward — never back to `HOMING`, `HOLDING`, or `SEARCHING`.

**State count is still 6** (`SEARCHING`, `HOMING`, `HOLDING`, `DESCENDING`, `FINAL_ALIGN`, `DOCKED`). We removed two states and added two; the graph is topologically cleaner.

**Changes.**

1. `__init__` — new timer for the `HOLDING` state's 3-second timeout:
   ```python
   self.hold_start_t = None
   ```
2. `approach_confidence` drain exception list extended to include `HOLDING`, so the 4-marker lock doesn't bleed during brief losses:
   ```python
   if self.state not in ["DESCENDING", "FINAL_ALIGN", "HOLDING"]:
       self.approach_confidence = max(0, self.approach_confidence - 1)
   ```
3. `SEARCHING` — entry trigger moved to the confidence metric and added a "lock pending" still-hold so the search pattern doesn't rotate the marker back out of FOV:
   ```python
   if self.vision_confidence > 0.5:
       self.state = "HOMING"
   elif markers_seen:
       vx, vy, vyaw = 0.0, 0.0, 0.0   # hold still while conf saturates
   else:
       # existing cyclic spiral + sweep pattern (unchanged)
   ```
4. `HOMING` — new merged state with two exit edges:
   ```python
   blind_active = (self.last_d_h < BLIND_DESCENT_DIST
                   and (time.time() - self.last_seen_t) < BLIND_DESCENT_TIMEOUT)
   commit_safe = (self.vision_confidence > 0.6) or blind_active

   if self.vision_confidence < 0.3:
       self.state = "HOLDING"; self.hold_start_t = None
   elif self.confirmed_4_markers and y_w < CAMERA_OFFSET_Y and commit_safe:
       self.state = "DESCENDING"
   ```
5. `HOLDING` — new state block with three ordered exit paths:
   ```python
   if self.hold_start_t is None:
       self.hold_start_t = time.time()
   time_in_hold = time.time() - self.hold_start_t

   if self.vision_confidence > 0.6:
       self.state = "HOMING"                            # reacquired
   elif (blind_active and self.confirmed_4_markers
         and self.last_y_w < CAMERA_OFFSET_Y):
       self.state = "DESCENDING"                        # close-range commit
   elif time_in_hold > 3.0 and not blind_active:
       self.state = "SEARCHING"                         # genuine loss
   ```
6. Command block simplified to state-driven (no more `if visible:` branching). The 0.8× damping trick from Session 2 was absorbed into the `HOLDING` state:
   ```python
   if self.state == "HOMING":
       vx, vy, vyaw = self.last_vx, self.last_vy, self.last_vyaw
   elif self.state in ["DESCENDING", "FINAL_ALIGN"]:
       vx, vy, vyaw = 0.0, 0.0, 0.0
   elif self.state == "HOLDING":
       vx, vy, vyaw = 0.0, 0.0, 0.0
       vz = 0.0
   # SEARCHING: vx/vyaw set in state block
   ```
7. HUD confidence tier colors (green/yellow/red for high/medium/low) + `CONF:` readout line.

**Respected constraints.** PID class untouched, Kalman untouched, vision module untouched, MAVLink interface untouched, config file untouched. Only `updated_docking_node.py` modified. Landing-safety invariant preserved: `DESCENDING` and `FINAL_ALIGN` have no edges back to `HOMING`/`HOLDING`/`SEARCHING`; `vision_confidence` is not read inside those states.

**Unused import.** `DIST_ALIGN` is now unused (was the `ALIGNING → APPROACHING` distance threshold). Left in the import list for minimal churn; harmless.

---

## Session 9 — integration_test_agent (test artifact added)

**Problem.** The new FSM introduces a `HOLDING` side-state, confidence-gated entry, and a `commit_safe` predicate. We need machine-checkable evidence that all transitions still work and — critically — that the landing-safety invariant holds under the "marker disappears at close range" failure mode.

**Approach.** Real executable integration tests, not logical traces. A harness (`test_fsm_integration.py`) bypasses ROS/MAVLink/camera hardware at the edges and drives the actual `DockingNode.loop()` frame by frame with controllable vision outputs.

**Harness design.**

- `sys.modules`-level stubs for `rclpy`, `rclpy.node`, `sensor_msgs.msg`, `cv_bridge`, `pymavlink`, `pymavlink.mavutil` (not available in the sandbox).
- `updated_config` / `updated_controller` / `updated_vision` aliased as `config` / `controller` / `vision` to match the un-prefixed imports in `updated_docking_node.py`.
- Real `PID` and `Kalman2D` from `updated_controller`.
- `VisionStub` class reimplements `compute_world()` byte-for-byte from `updated_vision.py` (bypassing the ArUco detector, which needs OpenCV ≥ 4.7's `ArucoDetector` API, unavailable in the sandbox's OpenCV 4.5.4).
- `HarnessNode(DockingNode)` subclass skips `DockingNode.__init__` entirely and seeds just the attributes `loop()` reads/writes. `send()` is captured into a list; `get_logger()` returns a silent logger. `loop()` itself is the real implementation.

**Tests executed and results.**

```
[PASS] T1  SEARCHING holds under no-detection
[PASS] T2a Single-frame flicker does NOT exit SEARCHING (conf=0.15)
[PASS] T2b SEARCHING→HOMING after sustained detection (3 frames, conf=0.60)
[PASS] T3  HOMING accumulates 4-marker lock (apr_conf=23)
[PASS] T4  HOMING→DESCENDING on confirmed+y_w<OFFSET+conf>0.6
[PASS] T5a CRITICAL: NEVER returns to SEARCHING during close-range loss
[PASS] T5b C1 stable during short loss (<blind timeout)
[PASS] T5c C2 DESCENDING→FINAL_ALIGN via blind timeout fallback
[PASS] T5d C3 FINAL_ALIGN→DOCKED via blind timeout fallback
[PASS] T6a DESCENDING→FINAL_ALIGN via hysteresis
[PASS] T6b FINAL_ALIGN→DOCKED via hysteresis
[PASS] T7a HOMING→HOLDING on confidence drop (mid-range)
[PASS] T7b HOLDING→HOMING on reacquire
[PASS] T7c HOLDING→SEARCHING after 3s timeout (mid-range, no blind)
[PASS] T8  HOLDING→DESCENDING (close-range blind commit)

RESULT: 15/15 tests PASSED
```

**T5 — the critical scenario — in detail.** Marker disappears at close range while the vehicle is descending. Test runs three phases:

| Phase | Input | Observed | Expected | Verdict |
|-------|-------|----------|----------|---------|
| A / B | Acquire at d_v=3 m, close in to d_v=1 m, y_w=-0.5 | SEARCHING→HOMING→DESCENDING | reach `DESCENDING` | ✓ |
| C1 | Markers gone, ~1 s sim | no transitions | stay in DESCENDING, no SEARCHING | ✓ |
| C2 | `last_seen_t` aged past 8 s blind-timeout | DESCENDING→FINAL_ALIGN | blind fallback fires | ✓ |
| C3 | `last_seen_t` aged again | FINAL_ALIGN→DOCKED | mission completes | ✓ |

All-transitions audit: `[('DESCENDING','FINAL_ALIGN','C2'), ('FINAL_ALIGN','DOCKED','C3')]`. **Zero** `SEARCHING` entries across the entire blind window. The landing-safety invariant holds.

**Notes.**

- The test file uses `time.time()` directly (no fake clock), but manipulates `last_seen_t` to accelerate the 8-second blind timeout path. This keeps total run-time under a second while exercising the real fallback branches.
- Exit code is 0 on full pass / 1 otherwise — suitable for CI.
- No source changes were required to pass the tests. FSM was correct as implemented after Session 8.

---

## Final behavior map (post Session 8)

```mermaid
flowchart TD
    S[SEARCHING] -->|"conf gt 0.5"| H[HOMING]
    H -->|"conf lt 0.3"| W[HOLDING]
    H -->|"confirmed AND y_w lt OFFSET_Y AND commit_safe"| D[DESCENDING]
    W -->|"conf gt 0.6 (reacquired)"| H
    W -->|"blind_active AND confirmed AND last_y_w lt OFFSET_Y"| D
    W -->|"hold gt 3s AND NOT blind_active"| S
    D -->|"d_v lt thr for 3 frames"| F[FINAL_ALIGN]
    D -->|"blind gt 8s (fallback)"| F
    F -->|"d_v lt thr for 3 frames"| DK[DOCKED]
    F -->|"blind gt 8s (fallback)"| DK
    D -.no path.-> S
    D -.no path.-> W
    F -.no path.-> S
    F -.no path.-> W
    DK -->|every tick| ZC["send(0, 0, decay, 0)"]
```

Where `commit_safe = (vision_confidence > 0.6) ∨ blind_active`, and `blind_active = (last_d_h < BLIND_DESCENT_DIST) ∧ (time_since_last_seen < BLIND_DESCENT_TIMEOUT)`.

## State changes across the project

| Session | States added | States removed | Net topology |
|---------|--------------|----------------|--------------|
| 1-6 (baseline) | — | — | `SEARCHING → ALIGNING → APPROACHING → DESCENDING → FINAL_ALIGN → DOCKED` (6 states) |
| 7 | — | — | unchanged; visibility logic replaced with `vision_confidence` blending |
| 8 | `HOMING`, `HOLDING` | `ALIGNING`, `APPROACHING` | `SEARCHING → HOMING ⇄ HOLDING → DESCENDING → FINAL_ALIGN → DOCKED` (still 6 states) |
| 9 | — | — | integration tests only, no code changes |

## Summary of files

| File | Change |
|------|--------|
| `updated_config.py` | +2 constants (`BLIND_DESCENT_DIST`, `BLIND_DESCENT_TIMEOUT`) — Session 1 |
| `updated_docking_node.py` | Cumulative patches across Sessions 1-8. Current shape: confidence-blended pose, merged `HOMING`, new `HOLDING`, state-driven command block, HUD confidence tier. |
| `updated_vision.py` | No changes across any session |
| `updated_controller.py` | No changes across any session (PID class untouched per the original constraint) |
| `test_fsm_integration.py` | New in Session 9 — runnable integration test harness, 15/15 pass |

## Key invariants preserved throughout every session

- **Forward-only landing.** No path from `DESCENDING` or `FINAL_ALIGN` back to `HOMING`, `HOLDING`, or `SEARCHING`. Verified by Test 5.
- **PID class untouched.** All control tuning lives at the call sites (deadband, LPF) or in FSM-level semantics (HOLDING), never inside `controller.py`.
- **Vision module untouched.** `updated_vision.py` is byte-identical to its baseline across all 9 sessions.
- **MAVLink interface untouched.** `manual_control_send(vx, vy, 500 + vz*1000, yaw, 0)` is the only actuator call.
- **Blind-descent design.** `BLIND_DESCENT_DIST = 1.5 m`, `BLIND_DESCENT_TIMEOUT = 8.0 s` — introduced in Session 1 and honored unchanged by every later session.
