# -*- coding: utf-8 -*-
"""Button page for the two-LASF bench.

    py -3.13 python/bench_ui.py

Opens http://127.0.0.1:8766/ . Close the Inspire host program first; it holds COM4.

Two rods, one USB board. Each rod is the driving tendon of one antagonistic
pair. The antagonist is a constant-force structure, not another actuator.
A typed tip load (grams) is stored with the log. NDI search tries the
stationary tracker address, then a chosen COM port from the list.
"""
from __future__ import annotations

import json
import math
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import lasf16
import log_check
import ndi_ascii

HOST = "127.0.0.1"
PORT = 8766
HERE = Path(__file__).resolve().parent
PAGE = HERE / "ui" / "index.html"
LOG_DIR = HERE.parent / "logs"
STROKE_MM = 16.0
STROKE_COUNTS = 2000
NDI_RESET_AFTER_S = 5 * 60
G_TO_N = 0.00980665
LOG_PERIOD_S = 1.0 / 55.0
REST_COUNTS = 2
# A rod already on the curve start often reads a few counts off the knot.
START_TOLERANCE = 30
SPEED_GAIN = 1.12
# Slower rod, measured 2026-11-17. Both saturated near 1620 counts/s
# when commanded at 2000 steps/s. Faster commands did not go faster.
SPEED_LIMIT = 1620
LOG_COLUMNS = [
    "time_s",
    "motor1_target_position",
    "motor1_position",
    "motor1_target_speed",
    "motor1_speed",
    "motor1_target_force",
    "motor1_force",
    "motor2_target_position",
    "motor2_position",
    "motor2_target_speed",
    "motor2_speed",
    "motor2_target_force",
    "motor2_force",
    "x",
    "y",
    "z",
    "ndi_raw",
    "reaction_force",
    "tip_fx",
    "tip_fy",
    "tip_fz",
    "tip_force_raw",
]


def _csv_cell(value: str) -> str:
    text = str(value)
    if any(ch in text for ch in ",\"\n\r"):
        return '"' + text.replace('"', '""') + '"'
    return text


def counts_to_mm(counts: int) -> float:
    return counts * STROKE_MM / STROKE_COUNTS


def fault_text(status: lasf16.Status) -> str:
    names = []
    if status.stall:
        names.append("stall")
    if status.overtemp:
        names.append("overtemp")
    if status.overcurrent:
        names.append("overcurrent")
    if status.motor_fault:
        names.append("motor fault")
    return ", ".join(names)


def _clean_knots(knots, lo: float = 0.0, hi: float = 2000.0):
    pts = sorted((float(t), min(hi, max(lo, float(p)))) for t, p in knots)
    if len(pts) < 2:
        raise ValueError("A curve needs two markers.")
    cleaned = [pts[0]]
    for t, p in pts[1:]:
        if t <= cleaned[-1][0] + 1e-4:
            t = cleaned[-1][0] + 1e-3
        cleaned.append((t, p))
    return cleaned


def _cubic_state(pts):
    """Clamped cubic with zero end speed. Returns position, speed, acceleration."""
    n = len(pts) - 1
    h = [pts[i + 1][0] - pts[i][0] for i in range(n)]
    y = [p[1] for p in pts]
    size = n + 1
    lower = [0.0] * n
    diag = [0.0] * size
    upper = [0.0] * n
    rhs = [0.0] * size
    diag[0] = 2 * h[0]
    upper[0] = h[0]
    rhs[0] = 6 * ((y[1] - y[0]) / h[0])
    for i in range(1, n):
        lower[i - 1] = h[i - 1]
        diag[i] = 2 * (h[i - 1] + h[i])
        upper[i] = h[i]
        rhs[i] = 6 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])
    lower[n - 1] = h[n - 1]
    diag[n] = 2 * h[n - 1]
    rhs[n] = 6 * (0 - (y[n] - y[n - 1]) / h[n - 1])
    width = diag[:]
    slope = rhs[:]
    for i in range(1, size):
        weight = lower[i - 1] / width[i - 1]
        width[i] -= weight * upper[i - 1]
        slope[i] -= weight * slope[i - 1]
    moment = [0.0] * size
    moment[-1] = slope[-1] / width[-1]
    for i in range(size - 2, -1, -1):
        moment[i] = (slope[i] - upper[i] * moment[i + 1]) / width[i]

    def state(time_s: float):
        if time_s <= pts[0][0]:
            return y[0], 0.0, moment[0]
        if time_s >= pts[-1][0]:
            return y[-1], 0.0, moment[-1]
        i = 0
        while i < n - 1 and time_s > pts[i + 1][0]:
            i += 1
        span = h[i]
        dt = time_s - pts[i][0]
        a = y[i]
        b = (y[i + 1] - y[i]) / span - span * (2 * moment[i] + moment[i + 1]) / 6
        c = moment[i] / 2
        d = (moment[i + 1] - moment[i]) / (6 * span)
        p = a + b * dt + c * dt * dt + d * dt * dt * dt
        v = b + 2 * c * dt + 3 * d * dt * dt
        acc = 2 * c + 6 * d * dt
        return p, v, acc

    return state


def _quintic(t0, t1, p0, v0, a0, p1, v1, a1):
    """Fifth-order blend. End speed and acceleration match the request."""
    span = float(t1 - t0)
    if span <= 1e-9:
        return lambda _t: (p1, 0.0, 0.0)
    c0 = p0
    c1 = v0 * span
    c2 = 0.5 * a0 * span * span
    pos = p1 - c0 - c1 - c2
    vel = v1 * span - c1 - 2.0 * c2
    acc = a1 * span * span - 2.0 * c2
    c5 = 6.0 * pos - 3.0 * vel + 0.5 * acc
    c4 = 0.25 * acc - 0.5 * vel - 2.5 * c5
    c3 = pos - c4 - c5

    def at(time_s: float):
        u = (time_s - t0) / span
        if u < 0.0:
            u = 0.0
        elif u > 1.0:
            u = 1.0
        u2 = u * u
        u3 = u2 * u
        p = c0 + c1 * u + c2 * u2 + c3 * u3 + c4 * u2 * u2 + c5 * u2 * u3
        dp = c1 + 2 * c2 * u + 3 * c3 * u2 + 4 * c4 * u3 + 5 * c5 * u2 * u2
        d2 = 2 * c2 + 6 * c3 * u + 12 * c4 * u2 + 20 * c5 * u3
        return p, dp / span, d2 / (span * span)

    return at


def _hermite(t0, t1, p0, v0, p1, v1):
    """Cubic between two positions. Slopes are limited so the arc stays between them."""
    span = float(t1 - t0)
    if span <= 1e-9:
        return lambda _t: (p1, 0.0, 0.0)
    dist = p1 - p0

    def clip(speed: float) -> float:
        if abs(dist) < 1e-9:
            return 0.0
        alpha = speed * span / dist
        if alpha < 0.0:
            alpha = 0.0
        elif alpha > 3.0:
            alpha = 3.0
        return alpha * dist / span

    v0 = clip(v0)
    v1 = clip(v1)

    def at(time_s: float):
        u = (time_s - t0) / span
        if u < 0.0:
            u = 0.0
        elif u > 1.0:
            u = 1.0
        u2 = u * u
        u3 = u2 * u
        h00 = 2 * u3 - 3 * u2 + 1
        h10 = u3 - 2 * u2 + u
        h01 = -2 * u3 + 3 * u2
        h11 = u3 - u2
        p = h00 * p0 + h10 * span * v0 + h01 * p1 + h11 * span * v1
        dh00 = 6 * u2 - 6 * u
        dh10 = 3 * u2 - 4 * u + 1
        dh01 = -6 * u2 + 6 * u
        dh11 = 3 * u2 - 2 * u
        speed = (dh00 * p0 + dh10 * span * v0 + dh01 * p1 + dh11 * span * v1) / span
        d2h00 = 12 * u - 6
        d2h10 = 6 * u - 4
        d2h01 = -12 * u + 6
        d2h11 = 6 * u - 2
        acc = (d2h00 * p0 + d2h10 * span * v0 + d2h01 * p1 + d2h11 * span * v1) / (span * span)
        return p, speed, acc

    return at


def _in_range(fn, t0, t1, lo, hi, n=32) -> bool:
    if t1 < t0 - 1e-9:
        return False
    steps = max(4, n)
    for i in range(steps + 1):
        p = fn(t0 + (t1 - t0) * i / steps)[0]
        if p < lo - 0.05 or p > hi + 0.05:
            return False
    return True


def _in_span(fn, t0, t1, pa, pb, n=24) -> bool:
    low = min(pa, pb) - 0.4
    high = max(pa, pb) + 0.4
    steps = max(4, n)
    for i in range(steps + 1):
        p = fn(t0 + (t1 - t0) * i / steps)[0]
        if p < low or p > high:
            return False
    return True


def _bridge(t0, t1, p0, v0, a0, p1, v1, a1, lo, hi):
    """Join two states without leaving the stroke or the span between them."""
    options = (
        _quintic(t0, t1, p0, v0, a0, p1, v1, a1),
        _quintic(t0, t1, p0, v0, 0.0, p1, v1, 0.0),
        _hermite(t0, t1, p0, v0, p1, v1),
        _quintic(t0, t1, p0, 0.0, 0.0, p1, 0.0, 0.0),
    )
    for fn in options:
        if _in_range(fn, t0, t1, lo, hi) and _in_span(fn, t0, t1, p0, p1):
            return fn
    return options[-1]


def _outside(p, lo, hi):
    if p > hi + 0.25:
        return hi
    if p < lo - 0.25:
        return lo
    return None


def _bisect_level(state, left, right, level):
    sign = state(left)[0] - level
    for _ in range(42):
        mid = 0.5 * (left + right)
        if (state(mid)[0] - level) * sign <= 0.0:
            right = mid
        else:
            left = mid
            sign = state(left)[0] - level
    return 0.5 * (left + right)


def _excursions(state, t0, t1, lo, hi):
    steps = max(60, int(round((t1 - t0) / 0.002)))
    found = []
    active = None
    prev_t = t0
    prev_p = state(t0)[0]
    for i in range(1, steps + 1):
        t = t0 + (t1 - t0) * i / steps
        p = state(t)[0]
        level = _outside(p, lo, hi)
        if active is None:
            if level is not None:
                hit = prev_t if _outside(prev_p, lo, hi) is not None else _bisect_level(state, prev_t, t, level)
                active = [level, hit, t]
        elif level == active[0]:
            active[2] = t
        else:
            active[2] = _bisect_level(state, prev_t, t, active[0])
            found.append(tuple(active))
            active = None
            if level is not None:
                hit = _bisect_level(state, prev_t, t, level)
                active = [level, hit, t]
        prev_t, prev_p = t, p
    if active is not None:
        found.append((active[0], active[1], min(active[2], t1)))
    return found


def _piece_inside(state, t0, t1, lo, hi) -> bool:
    return not _excursions(state, t0, t1, lo, hi)


def _hold(level):
    return lambda _t, L=level: (L, 0.0, 0.0)


def _brake_before(state, cursor, t_hit, level, lo, hi):
    """Ease onto the limit before the cubic would cross it.

    Returns the start of the ease, the contact time, and the ease itself.
    The gentlest ease that still matches the incoming speed wins.
    """
    room = t_hit - cursor
    if room <= 1e-4:
        return None
    fallback = None
    for frac in (1.0, 0.82, 0.64, 0.48, 0.34, 0.22, 0.12, 0.06):
        h = room * frac
        if h <= 1e-4:
            continue
        t_anchor = t_hit - h
        p0, v0, a0 = state(t_anchor)
        if p0 < lo - 0.05 or p0 > hi + 0.05:
            continue
        fn = _bridge(t_anchor, t_hit, p0, v0, a0, level, 0.0, 0.0, lo, hi)
        if abs(fn(t_hit)[0] - level) > 1.0:
            continue
        if not _in_range(fn, t_anchor, t_hit, lo, hi) or not _in_span(fn, t_anchor, t_hit, p0, level):
            continue
        speed = fn(t_anchor)[1]
        matched = abs(v0) < 80.0 or abs(speed - v0) <= max(80.0, 0.35 * abs(v0))
        if matched:
            return t_anchor, t_hit, fn
        fallback = (t_anchor, t_hit, fn)
    return fallback


def _release_after(state, t_off, t_right, level, lo, hi):
    """Leave the limit and meet the cubic again. Returns end time, blend, and whether the cubic tail remains."""
    end_p, end_v, end_a = state(t_right)
    end_p = min(hi, max(lo, end_p))
    gap = t_right - t_off
    if gap <= 1e-5:
        return t_off, _hold(level), False
    if abs(end_p - level) <= 0.5:
        return t_right, _hold(level), False
    for frac in (0.12, 0.22, 0.36, 0.52, 0.74, 1.0):
        h = gap * frac
        if h <= 1e-4:
            continue
        to_end = frac == 1.0
        t_back = t_right if to_end else t_off + h
        if to_end:
            sp, sv, sa = end_p, end_v, end_a
        else:
            sp, sv, sa = state(t_back)
            if sp < lo - 0.05 or sp > hi + 0.05:
                continue
        fn = _bridge(t_off, t_back, level, 0.0, 0.0, sp, sv, sa, lo, hi)
        if not _in_range(fn, t_off, t_back, lo, hi) or not _in_span(fn, t_off, t_back, level, sp):
            continue
        if not to_end and not _piece_inside(state, t_back, t_right, lo, hi):
            continue
        return t_back, fn, not to_end
    return None


def _cap_piece(state, t_left, t_right, lo, hi):
    """Where the cubic leaves the stroke, rest on that limit and ease off it."""
    trips = _excursions(state, t_left, t_right, lo, hi)
    if not trips:
        return lambda t, s=state: s(t)[0]
    segments = []

    def add(t0, t1, fn):
        if t1 > t0 + 1e-8:
            segments.append((t0, t1, fn))

    cursor = t_left
    for level, t_hit, t_leave in trips:
        if t_leave <= cursor + 1e-5:
            continue
        t_hit = min(max(t_hit, cursor), t_right)
        t_leave = min(max(t_leave, t_hit), t_right)
        braked = _brake_before(state, cursor, t_hit, level, lo, hi)
        if braked is None:
            return None
        t_anchor, t_on, arrive = braked
        if t_anchor > cursor + 1e-6:
            add(cursor, t_anchor, state)
        add(t_anchor, t_on, arrive)
        t_off = min(t_right, max(t_on, t_leave))
        add(t_on, t_off, _hold(level))
        if t_off >= t_right - 1e-7:
            cursor = t_right
            break
        released = _release_after(state, t_off, t_right, level, lo, hi)
        if released is None:
            return None
        t_back, depart, _tail = released
        add(t_off, t_back, depart)
        cursor = t_back
        if cursor >= t_right - 1e-7:
            break
    if cursor < t_right - 1e-6:
        add(cursor, t_right, state)
    if not segments:
        return None

    def at(time_s: float) -> float:
        for start, end, fn in segments:
            if time_s <= end + 1e-8:
                return fn(time_s)[0]
        return segments[-1][2](time_s)[0]

    return at


def _clamped_cubic(knots):
    """Position curve inside 0–2000.

    One cubic through every marker, so the slope does not jump when a marker moves.
    """
    pts = _clean_knots(knots)
    state = _cubic_state(pts)
    n = len(pts) - 1
    pieces = []
    for i in range(n):
        t_left, _y_left = pts[i]
        t_right, _y_right = pts[i + 1]
        pieces.append((t_left, t_right, lambda t, s=state: s(t)[0]))

    def at(time_s: float) -> float:
        if time_s <= pts[0][0]:
            return pts[0][1]
        if time_s >= pts[-1][0]:
            return pts[-1][1]
        for _t_left, t_right, fn in pieces:
            if time_s <= t_right + 1e-9:
                return fn(time_s)
        return pts[-1][1]

    return at


def _soft_speed(speed: float, limit: float) -> float:
    """Ease onto the speed limit. A hard cut leaves a corner in the speed curve."""
    knee = limit * 0.28
    mag = abs(speed)
    start = limit - knee
    if mag <= start:
        return speed
    u = (mag - start) / knee
    shaped = limit - knee * math.exp(-u)
    return -shaped if speed < 0.0 else shaped


def _smooth_limited(path, pts):
    """Position whose slope eases up to the speed limit and eases off it.

    The knot positions stay the targets. Where the cubic would have been
    cut flat at the limit, the speed now bends onto that limit.
    """
    limit = float(SPEED_LIMIT)
    ts: list[float] = []
    per = 64
    for k in range(len(pts) - 1):
        t0, _p0 = pts[k]
        t1, _p1 = pts[k + 1]
        start = 0 if k == 0 else 1
        for i in range(start, per + 1):
            ts.append(t0 + (t1 - t0) * (i / per))
    t0 = ts[0]
    t1 = ts[-1]

    def raw_slope(time_s: float) -> float:
        dt = min(0.012, max(1e-4, (t1 - t0) / 80.0))
        left = max(t0, time_s - dt)
        right = min(t1, time_s + dt)
        if right <= left:
            return 0.0
        return (path(right) - path(left)) / (right - left)

    raw = [raw_slope(t) for t in ts]
    if max(abs(item) for item in raw) <= limit:
        return path
    jmax = limit / (0.1 * 0.1)
    speed = [raw[0]]
    value = raw[0]
    accel = 0.0
    for i in range(len(ts) - 1):
        dt = max(1e-4, ts[i + 1] - ts[i])
        target = _soft_speed(raw[i + 1], limit)
        want = (target - value) / dt
        step = jmax * dt
        if want > accel + step:
            want = accel + step
        elif want < accel - step:
            want = accel - step
        accel = want
        value += accel * dt
        speed.append(value)
    pos = [pts[0][1]]
    for i in range(len(ts) - 1):
        pos.append(pos[-1] + 0.5 * (speed[i] + speed[i + 1]) * (ts[i + 1] - ts[i]))

    def at(time_s: float) -> float:
        if time_s <= ts[0]:
            return pos[0]
        if time_s >= ts[-1]:
            return pos[-1]
        lo, hi = 0, len(ts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= time_s:
                lo = mid
            else:
                hi = mid
        span = ts[hi] - ts[lo]
        if span <= 1e-9:
            return pos[lo]
        u = (time_s - ts[lo]) / span
        return pos[lo] + (pos[hi] - pos[lo]) * u

    return at


def _curve_slope(path, time_s: float, duration: float) -> float:
    """Steps per second from the drawn curve. The slope is the speed command."""
    dt = 0.02
    left = max(0.0, time_s - dt)
    right = min(duration, time_s + dt)
    if right <= left:
        return 0.0
    return (path(right) - path(left)) / (right - left)


def _heading(path, time_s: float, duration: float, slope: float) -> int:
    """End of the current sweep. The drive ignores a new speed unless this is in the same frame."""
    if abs(slope) < 1.0:
        return int(round(path(time_s)))
    sign = 1.0 if slope > 0.0 else -1.0
    aim = path(time_s)
    cursor = time_s
    while cursor < duration - 1e-6:
        cursor = min(duration, cursor + 0.025)
        aim = path(cursor)
        ahead = _curve_slope(path, cursor, duration)
        if ahead * sign <= 0.0 and cursor > time_s + 0.04:
            break
    return int(round(aim))


def _speed_for_window(distance: int, seconds: float) -> int:
    """Cruise speed so a move of this length finishes in this time.

    The drive spends time speeding up and slowing down, so the command has
    to be faster than distance/time. From one bench move: 350 counts at a
    commanded 800 steps/s took 0.58 s, which is about 5600 counts/s² if the
    ramp up and the ramp down are symmetric. T = D/V + V/a.
    """
    distance = abs(int(distance))
    accel = 5600.0
    if distance < 2 or seconds <= 0.05:
        return 0
    disc = (accel * seconds) ** 2 - 4.0 * accel * distance
    if disc <= 0:
        return SPEED_LIMIT
    speed = (accel * seconds - math.sqrt(disc)) / 2.0
    if speed < distance / seconds:
        speed = distance / seconds
    return int(round(max(20.0, min(float(SPEED_LIMIT), speed))))


def _sweeps(path, duration: float) -> list[tuple[float, float, int, int]]:
    """One entry per stretch where the curve keeps the same direction.

    A fresh USB speed frame restarts the motor, so a 0.2 s slice is one
    click. A straight run should be one frame, not one frame per slice.
    """
    step = 0.02
    times = [step * i for i in range(int(duration / step) + 1)]
    if times[-1] < duration:
        times.append(duration)
    sign = 0
    start_t = 0.0
    start_p = int(round(path(0.0)))
    found: list[tuple[float, float, int, int]] = []
    for time_s in times[1:]:
        slope = _curve_slope(path, time_s, duration)
        now = 0 if abs(slope) < 1.0 else (1 if slope > 0.0 else -1)
        if sign == 0 and now != 0:
            sign = now
        if sign != 0 and now != 0 and now != sign:
            end_p = int(round(path(time_s)))
            if time_s - start_t > 0.05 and abs(end_p - start_p) >= 2:
                found.append((start_t, time_s, start_p, end_p))
            start_t = time_s
            start_p = end_p
            sign = now
    end_p = int(round(path(duration)))
    if duration - start_t > 0.05 and abs(end_p - start_p) >= 2:
        found.append((start_t, duration, start_p, end_p))
    return found


class Bench:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.bus: lasf16.LasfBus | None = None
        self.ndi: ndi_ascii.NdiLink | None = None
        self.tare = {1: 0, 2: 0}
        self.pairs: dict[int, dict] = {}
        self.message = "Idle. Connect the actuators before a move."
        self.ok = True
        self.cancel = threading.Event()
        self.play_thread: threading.Thread | None = None
        self.paused = False
        self.resume_gate = threading.Event()
        self.resume_gate.set()
        self._pause_started: float | None = None
        self._resend_leg = False
        self.awaiting_resume = False
        self.speed_hz = 10
        self.busy = False
        self.recording = False
        self.record_stop = threading.Event()
        self.log_path = ""
        self.tip_load_g = ""
        self.ndi_last = "No tracker reply yet."
        self.ndi_port = ""
        self.ndi_choices = ndi_ascii.list_endpoints()
        self.ndi_identity = ""
        self.ndi_revision = ""
        self.ndi_firmware = ""
        self.ndi_marker = ""
        self.ndi_retry_at = 0.0
        self.ndi_io = threading.Lock()
        self.ndi_raw = ""
        self.ndi_tx = "TX 1800"
        self.record_deadline = 0.0
        self.ndi_points: list[tuple[float, float, float]] = []
        self.ndi_frame = 0
        self.ndi_count = 0
        self.ndi_tracking = False
        self.ndi_ready_at = 0.0
        self.ndi_resetting = False
        self.ndi_host_i = 0
        self.ndi_wanted = False
        self.ndi_connecting = False
        self.ndi_connect_gen = 0
        self.ndi_opening = None
        self.ndi_open_lock = threading.Lock()
        self.ndi_ignore_until = 0.0
        self.start_ignore = {1: 0.0, 2: 0.0}
        self.play_checks: list[dict] = []
        self.play_note = ""
        self.ndi_gate = False
        self.pending_play = None
        self.pending_record = None
        self.pending_kind = ""
        self.follow = False
        self.bus_reserved = False
        self.follow_targets = {1: 0, 2: 0}
        self.follow_idle_until = 0.0
        self.aim = {1: 0, 2: 0}
        self.move_rate = 2500
        self.drive_mode = "position"
        self.speed_sps = 300
        self.speed_cmd = {1: None, 2: None}
        self._read_n = {1: 0, 2: 0}
        self.marker_warning = False
        self._log_t0 = None
        self.play_id = 0
        self.sample_n = 0
        self._speed_prev = {1: None, 2: None}
        self._held_at = {1: 0.0, 2: 0.0}
        self.rested = {1: False, 2: False}
        self.rest_stop = threading.Event()
        self._read_at = 0.0
        self.follow_seq = 0
        self.follow_stop = threading.Event()
        self._follower: threading.Thread | None = None
        self.ndi_poll_stop = threading.Event()
        threading.Thread(target=self._ndi_poll_loop, name="ndi-poll", daemon=True).start()
        threading.Thread(target=self._rest_loop, name="lasf-rest", daemon=True).start()

    def _ndi_line(self) -> str:
        if self.ndi_resetting:
            return "resetting"
        if self.ndi is not None:
            parts = [part for part in (self.ndi_port, self.ndi_revision, self.ndi_firmware) if part]
            return " · ".join(parts) if parts else "searching"
        return self.ndi_last or "searching"

    def _xyz_text(self) -> str:
        if self.ndi is None or not self.ndi_points:
            return "X: Null, Y: Null, Z: Null"
        x, y, z = self.ndi_points[0]
        return f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}"

    def _ndi_poll_loop(self) -> None:
        while not self.ndi_poll_stop.is_set():
            with self.lock:
                link = self.ndi if self.ndi is not None and self.ndi_tracking else None
            if link is None:
                if self.ndi_poll_stop.wait(0.25):
                    return
                with self.lock:
                    wanted = self.ndi_wanted
                    connecting = self.ndi_connecting
                if not wanted or connecting:
                    continue
                if time.time() < self.ndi_retry_at:
                    continue
                self.ndi_retry_at = time.time() + 2.0
                hosts = ndi_ascii.tracker_hosts()
                if not hosts:
                    self.ndi_last = "no tracker on the link"
                    continue
                with self.lock:
                    if self.ndi_connecting or self.ndi is not None:
                        continue
                host = hosts[self.ndi_host_i % len(hosts)]
                self.ndi_host_i += 1
                self._ndi_open(host)
                continue
            try:
                wait = 0.05 if self.recording else 0.3
                with self.ndi_io:
                    raw = link.command(self.ndi_tx, wait_s=wait)
                body, _, crc_ok = ndi_ascii.split_reply(raw)
                with self.lock:
                    if self.ndi is not link:
                        continue
                    self.ndi_raw = body if body else (raw or "")
                    if body.startswith("WARNING"):
                        self.ndi_points = []
                        self.ndi_count = 0
                        self.ndi_marker = ""
                        self.ndi_last = body
                        continue
                    if body.startswith("ERROR") and self.ndi_tx != "TX 1000":
                        self.ndi_tx = "TX 1000"
                    if not crc_ok or body.startswith("ERROR"):
                        self.ndi_points = []
                        self.ndi_count = 0
                        self.ndi_marker = ""
                        self.ndi_last = self.ndi_raw or "(empty TX reply)"
                    else:
                        points = ndi_ascii.parse_tx_strays(body)
                        self.ndi_points = points
                        self.ndi_count = len(points)
                        if points:
                            self.ndi_frame += 1
                        if not points:
                            self.ndi_marker = ""
                            self.ndi_last = self.ndi_raw
                        else:
                            text = ndi_ascii.format_xyz(points[0])
                            if len(points) > 1:
                                text = f"{len(points)} markers; {text}"
                            self.ndi_marker = text
                            self.ndi_last = text
            except Exception as exc:
                with self.lock:
                    if self.ndi is link:
                        self._drop_ndi(str(exc))
                continue
            if self.ndi_poll_stop.wait(0.1):
                return

    def _set(self, message: str, ok: bool = True) -> None:
        self.message = message
        self.ok = ok

    def _pack(self, status: lasf16.Status) -> dict:
        raw = status.force_g
        shown = raw - self.tare.get(status.actuator_id, 0)
        return {
            "id": status.actuator_id,
            "position": status.position,
            "target": status.target,
            "mm": round(counts_to_mm(status.position), 3),
            "force_raw_g": raw,
            "force_g": shown,
            "force_N": round(shown * G_TO_N, 4),
            "current_ma": status.current_ma,
            "temp_c": status.temp_c,
            "error": status.error,
            "fault": fault_text(status),
        }

    def _read_both(self) -> None:
        if self.bus is None:
            return
        for actuator_id in (1, 2):
            self.pairs[actuator_id] = self._pack(self.bus.status(actuator_id))

    def _arm(self, actuator_id: int) -> None:
        """Power stage on for a new command. Caller holds the lock."""
        assert self.bus is not None
        self.rested[actuator_id] = False
        self.bus.enable(actuator_id)

    def _rest_close_locked(self) -> None:
        """Power stage off once the rod is within 2 counts of its command.

        Caller holds the lock. A later command arms the drive again. Drifting
        after the cutoff does not turn it back on.
        """
        if self.bus is None or self.busy or self.follow:
            return
        if time.perf_counter() < self.follow_idle_until:
            return
        released: list[int] = []
        for actuator_id in (1, 2):
            if self.rested.get(actuator_id):
                continue
            pair = self.pairs.get(actuator_id)
            if not pair:
                continue
            # The status target can still be the old count after a speed
            # command. Cutting on that count stops the move before it starts.
            # Once the rod is on the aim that was sent, the command is done
            # and the power stage has to come off or the rod hunts and steps back.
            position = int(pair["position"])
            if self.speed_cmd.get(actuator_id) is not None and actuator_id in self.aim:
                reference = int(self.aim[actuator_id])
            else:
                reference = int(pair["target"])
            if abs(position - reference) > REST_COUNTS:
                continue
            self.speed_cmd[actuator_id] = None
            self.bus.estop(actuator_id)
            self.rested[actuator_id] = True
            released.append(actuator_id)
        if not released:
            return
        if len(released) == 2:
            self._set("Both rods are within 2 counts of the command. Power stage off.")
        else:
            self._set(
                f"Pair {released[0]} is within 2 counts of the command. Power stage off."
            )

    def _rest_loop(self) -> None:
        while not self.rest_stop.wait(0.2):
            with self.lock:
                if self.bus is None or self.busy or self.follow:
                    continue
                if all(self.rested.get(actuator_id) for actuator_id in (1, 2)):
                    continue
                try:
                    if not self.recording:
                        self._read_both()
                    self._rest_close_locked()
                except Exception as exc:
                    self._set(str(exc), ok=False)

    def connect(self) -> None:
        with self.lock:
            if self.bus is None:
                self.bus = lasf16.LasfBus()
            for actuator_id in (1, 2):
                self._arm(actuator_id)
                self._restore_output_limit(actuator_id)
            self._read_both()
            self._rest_close_locked()
            if not any(self.rested.values()):
                self._set("Actuators connected on COM4. Power stage on.")

    def _restore_output_limit(self, actuator_id: int) -> None:
        """Pair 2 was found with the motor output cap at ±3, so it accepted targets and drew no current."""
        regs = self.bus.read_regs(actuator_id, 0x21, 2)
        if not regs:
            return
        signed = [value - 65536 if value > 32767 else value for value in regs]
        if abs(signed[0]) >= 50 and abs(signed[1]) >= 50:
            return
        self.bus.write_regs(actuator_id, 0x21, [1000, 0xFC18])

    def stop(self) -> None:
        self.cancel.set()
        self.record_stop.set()
        self.paused = False
        self.awaiting_resume = False
        self.resume_gate.set()
        self.follow = False
        self.pending_play = None
        with self.lock:
            self.follow = False
            if self.bus is not None:
                for actuator_id in (1, 2):
                    self.bus.estop(actuator_id)
                    self.rested[actuator_id] = True
                self._read_both()
            self.speed_cmd = {1: None, 2: None}
            self.busy = False
            self._set("Stopped. Power stage off until the next move.")
        self.ndi_disconnect()

    def pause(self) -> None:
        if not self.busy or self.paused:
            return
        self.paused = True
        self._resend_leg = True
        self._pause_started = time.perf_counter()
        self.resume_gate.clear()
        with self.lock:
            if self.bus is not None:
                for actuator_id in (1, 2):
                    self.bus.estop(actuator_id)
                    self.rested[actuator_id] = True
            self.speed_cmd = {1: None, 2: None}
        self.ndi_disconnect()
        self._set("Paused.")

    def resume(self) -> None:
        if not self.paused:
            return
        if not self._ndi_ready() and not self._ndi_ignored():
            # Same open as Play. The 3 s limit aborted INIT and then
            # reported no tracker while the Vega was already answering.
            self.ndi_connect()
            deadline = time.perf_counter() + 2.0
            while (
                time.perf_counter() < deadline
                and not self._ndi_ready()
                and not self._ndi_ignored()
            ):
                time.sleep(0.05)
        if not self._ndi_ready() and not self._ndi_ignored():
            self.awaiting_resume = True
            self.play_checks = [{"id": "ndi", "text": "No tracker reading"}]
            self._set("Not ready.")
            return
        self.awaiting_resume = False
        self.play_checks = []
        if self._pause_started is not None:
            gap = time.perf_counter() - self._pause_started
            self._pause_started = None
            if self._log_t0 is not None:
                self._log_t0 += gap
        with self.lock:
            if self.bus is not None:
                for actuator_id in (1, 2):
                    self._arm(actuator_id)
        self.paused = False
        self.resume_gate.set()
        self._set("Playing.")

    def set_speed_hz(self, hz: float) -> None:
        self.speed_hz = max(1, min(50, int(round(float(hz)))))

    def _hold_if_paused(self, origin: float) -> float:
        """Keep the plan clock still while paused. A stop ends the wait."""
        if not self.paused:
            return origin
        started = time.perf_counter()
        while self.paused and not self.cancel.is_set():
            self.resume_gate.wait(0.05)
        if self.cancel.is_set():
            return origin
        return origin + (time.perf_counter() - started)

    def tare(self) -> None:
        with self.lock:
            self._read_both()
            for actuator_id, pair in self.pairs.items():
                self.tare[actuator_id] = pair["force_raw_g"]
            self._read_both()
            self._set("Force display tared. The actuator zero was not written.")

    def _move_one(self, actuator_id: int, counts: int) -> lasf16.Status:
        assert self.bus is not None
        counts = max(0, min(STROKE_COUNTS, int(counts)))
        with self.lock:
            self._arm(actuator_id)
            self.bus.move_counts(actuator_id, counts)
            last = self.bus.status(actuator_id)
        deadline = time.time() + 2.5
        while time.time() < deadline and not self.cancel.is_set():
            with self.lock:
                last = self.bus.status(actuator_id)
                self.pairs[actuator_id] = self._pack(last)
                if last.motor_fault or last.stall or last.overcurrent or last.overtemp:
                    for other in (1, 2):
                        self.bus.estop(other)
                    break
            if abs(last.position - counts) <= 20 and last.current_ma < 120:
                break
            time.sleep(0.08)
        return last

    def _run_move(self, plan: list[tuple[int, int]], label: str) -> None:
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            if self.busy:
                self._set("A move is already running.", ok=False)
                return
            self.busy = True
            self.cancel.clear()
            self._set("Moving.")

        def work() -> None:
            try:
                for actuator_id, counts in plan:
                    if self.cancel.is_set():
                        break
                    last = self._move_one(actuator_id, counts)
                    if last.motor_fault or last.stall or last.overcurrent:
                        self._set(
                            f"Pair {actuator_id} stopped: {fault_text(last)}.",
                            ok=False,
                        )
                        return
                with self.lock:
                    if self.bus is not None:
                        self._read_both()
                if self.cancel.is_set():
                    self._set("Move cancelled.")
                else:
                    self._set(label)
            except Exception as exc:
                self._set(str(exc), ok=False)
            finally:
                self.busy = False

        threading.Thread(target=work, daemon=True).start()

    def jog(self, actuator_id: int, delta: int) -> None:
        delta = max(-500, min(500, int(delta)))
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            current = self.bus.status(actuator_id).position
        target = max(0, min(STROKE_COUNTS, current + delta))
        self._run_move([(actuator_id, target)], f"Pair {actuator_id} commanded to {target} counts.")

    def _clamp(self, counts: int) -> int:
        return max(0, min(STROKE_COUNTS, int(round(float(counts)))))

    def _ensure_follower(self) -> None:
        if self._follower is not None and self._follower.is_alive():
            return
        self.follow_stop.clear()

        def loop() -> None:
            next_t = time.perf_counter()
            while not self.follow_stop.wait(0):
                next_t += 0.02
                if self.follow and not self.busy and self.bus is not None:
                    try:
                        with self.lock:
                            if self.follow and not self.busy and self.bus is not None:
                                self.bus.broadcast(0xF3, self.follow_targets)
                    except Exception as exc:
                        self.follow = False
                        self._set(str(exc), ok=False)
                delay = next_t - time.perf_counter()
                if delay > 0:
                    self.follow_stop.wait(delay)
                else:
                    next_t = time.perf_counter()

        self._follower = threading.Thread(target=loop, name="lasf-follow", daemon=True)
        self._follower.start()

    def follow_to(self, counts: dict, seq: int, live: bool) -> None:
        """Stream both targets while a slider is down. One frame, 50 Hz."""
        if seq < self.follow_seq:
            return
        self.follow_seq = seq
        if "1" not in counts and 1 not in counts:
            self._set("Slider counts missing.", ok=False)
            return
        if "2" not in counts and 2 not in counts:
            self._set("Slider counts missing.", ok=False)
            return
        cleaned = {
            1: self._clamp(counts.get("1", counts.get(1))),
            2: self._clamp(counts.get("2", counts.get(2))),
        }
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            self.follow_targets = cleaned
            if self.busy:
                self.aim = dict(cleaned)
                self._set("Moving both rods to the latest command.")
                return
            if live:
                if not self.follow:
                    if not self._enter_position_locked():
                        return
                    for actuator_id in (1, 2):
                        self._arm(actuator_id)
                    self.follow = True
                    self._set("Dragging. Both rods follow the sliders.")
                self._ensure_follower()
                return
            self.follow = False
            self.follow_idle_until = time.perf_counter() + 0.5
            self._set("Sliders released. Both rods keep the last follow target.")

    def _parse_counts(self, counts: dict) -> dict[int, int] | None:
        if ("1" not in counts and 1 not in counts) or ("2" not in counts and 2 not in counts):
            self._set("Command counts missing.", ok=False)
            return None
        return {
            1: self._clamp(counts.get("1", counts.get(1))),
            2: self._clamp(counts.get("2", counts.get(2))),
        }

    def aim_to(self, counts: dict) -> None:
        """Go to these counts. A click during a move replaces the target."""
        cleaned = self._parse_counts(counts)
        if cleaned is None:
            return
        start = False
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            self.follow = False
            self.aim = cleaned
            if self.busy:
                self._set("Moving both rods to the latest command.")
                return
            self.busy = True
            self.cancel.clear()
            start = True
            self._set("Moving both rods to the latest command.")
        if start:
            threading.Thread(target=self._aim_worker, daemon=True).start()

    def _aim_worker(self) -> None:
        """Stream a straight ramp in follow mode so the rod does not use the internal two-step plan."""
        period = 0.02
        try:
            with self.lock:
                if self.bus is None:
                    return
                if self.drive_mode != "position":
                    self._write_mode_locked(0)
                    self.drive_mode = "position"
                for actuator_id in (1, 2):
                    self._arm(actuator_id)
                self._read_both()
                start = {i: self.pairs[i]["position"] for i in (1, 2)}
                goal = dict(self.aim)
            began = time.perf_counter()
            next_t = time.perf_counter()
            tick = 0
            while not self.cancel.is_set():
                now = time.perf_counter()
                with self.lock:
                    if self.bus is None:
                        return
                    if self.aim != goal:
                        start = {
                            i: self.pairs.get(i, {}).get("position", start[i]) for i in (1, 2)
                        }
                        goal = dict(self.aim)
                        began = now
                    distance = max(abs(goal[i] - start[i]) for i in (1, 2))
                    duration = max(period, distance / max(200, self.move_rate))
                    fraction = min(1.0, (now - began) / duration)
                    command = {
                        i: int(round(start[i] + (goal[i] - start[i]) * fraction))
                        for i in (1, 2)
                    }
                    self.bus.broadcast(0xF3, command)
                    watch = 1 if tick % 2 == 0 else 2
                    try:
                        self.pairs[watch] = self._pack(self.bus.status(watch))
                    except TimeoutError:
                        pass
                    faulted = [i for i in (1, 2) if self.pairs.get(i, {}).get("fault")]
                    if faulted:
                        for actuator_id in (1, 2):
                            self.bus.estop(actuator_id)
                        self._set(
                            f"Pair {faulted[0]} stopped: {self.pairs[faulted[0]]['fault']}.",
                            ok=False,
                        )
                        return
                    close = all(
                        abs(self.pairs[i]["position"] - goal[i]) <= 30 for i in (1, 2)
                    )
                    done = fraction >= 1.0 and close and self.aim == goal
                tick += 1
                if done:
                    with self.lock:
                        if self.bus is not None and self.aim == goal:
                            self.bus.broadcast(0xF2, goal)
                            self._read_both()
                    self._set("Both rods at the latest command.")
                    return
                if now - began > duration + 2.0:
                    self._set("Move timed out before the latest command.", ok=False)
                    return
                next_t += period
                delay = next_t - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_t = time.perf_counter()
            self._set("Move cancelled.")
        except Exception as exc:
            self._set(str(exc), ok=False)
        finally:
            self.busy = False

    def set_rate(self, rate: int) -> None:
        self.move_rate = max(200, min(8000, int(rate)))
        self._set(f"Move rate {self.move_rate} counts/s.")

    def read_limits(self) -> dict:
        out = {}
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return {"ok": False}
            if self.busy:
                self._set("Wait until the move finishes before reading limits.", ok=False)
                return {"ok": False}
            for actuator_id in (1, 2):
                current_ma = int.from_bytes(self.bus.read_mem(actuator_id, 32, 2), "little")
                hot = int.from_bytes(self.bus.read_mem(actuator_id, 98, 2), "little") / 10
                restart = int.from_bytes(self.bus.read_mem(actuator_id, 100, 2), "little") / 10
                out[str(actuator_id)] = {
                    "overcurrent_mA": current_ma,
                    "overtemp_C": hot,
                    "restart_C": restart,
                }
        self._set("Read the limit registers.")
        out["ok"] = True
        out["move_rate"] = self.move_rate
        return out

    def apply_limits(self, body: dict) -> None:
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            if self.busy:
                self._set("Wait until the move finishes before writing limits.", ok=False)
                return
            for actuator_id in (1, 2):
                row = body.get(str(actuator_id)) or body.get(actuator_id) or {}
                current_ma = int(row["overcurrent_mA"])
                hot = float(row["overtemp_C"])
                restart = float(row["restart_C"])
                if not 300 <= current_ma <= 2000:
                    self._set("Overcurrent must be 300–2000 mA. The manual range is 300–1500; this unit reads 1800.", ok=False)
                    return
                if not 20 <= restart <= hot - 5 or not hot <= 80:
                    self._set("Restart is 20 °C to over-temp minus 5. Over-temp is at most 80 °C.", ok=False)
                    return
                self.bus.write_u16(actuator_id, 32, current_ma)
                self.bus.write_u16(actuator_id, 98, int(round(hot * 10)))
                self.bus.write_u16(actuator_id, 100, int(round(restart * 10)))
        self._set("Limits written in RAM. They clear at power off.")

    def zero_force_sensors(self, actuator_id: int | None = None) -> None:
        ids = (actuator_id,) if actuator_id in (1, 2) else (1, 2)
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            if self.busy:
                self._set("Wait until the move finishes.", ok=False)
                return
            for item in ids:
                self.bus.write_u8(item, 31, 1)
                self.tare[item] = 0
        label = f"Pair {actuator_id}" if actuator_id in (1, 2) else "Both rods"
        self._set(f"{label} load zeroed.")

    def _enter_position_locked(self) -> bool:
        """Position mode, with no leftover speed command. Caller holds the lock."""
        self.speed_cmd = {1: None, 2: None}
        seen = self._write_mode_locked(0)
        if seen != [0, 0]:
            for actuator_id in (1, 2):
                self.bus.estop(actuator_id)
            seen = self._write_mode_locked(0)
        if seen == [0, 0]:
            self.drive_mode = "position"
            self._set("Position mode.")
            return True
        self._set(f"Position mode readback {seen}.", ok=False)
        return False

    def _write_mode_locked(self, mode_value: int) -> list:
        """Set V2.0.3 register 0x25 on both rods. Caller holds the lock. Does not save flash."""
        seen = []
        for actuator_id in (1, 2):
            self.bus.write_regs(actuator_id, 0x25, [mode_value])
            got = self.bus.read_regs(actuator_id, 0x25, 1)
            seen.append(got[0] if got else None)
        return seen

    def set_drive_mode(self, mode: str) -> None:
        want = 2 if mode == "speed" else 0
        label = "speed" if want == 2 else "position"
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            if self.busy:
                self._set("Wait until the move finishes.", ok=False)
                return
            self.follow = False
            if want == 0:
                self._enter_position_locked()
                return
            seen = self._write_mode_locked(want)
        if seen == [want, want]:
            self.drive_mode = label
            self._set("Speed mode." if label == "speed" else "Position mode.")
        else:
            self._set(f"Mode readback {seen}.", ok=False)

    def speed_go(self, speed: int, counts: dict, live: bool = False) -> None:
        """Move both rods toward the slider counts. A live drag updates the destination."""
        speed = max(20, min(SPEED_LIMIT, int(speed)))
        if "1" not in counts and 1 not in counts:
            self._set("Slider counts missing.", ok=False)
            return
        if "2" not in counts and 2 not in counts:
            self._set("Slider counts missing.", ok=False)
            return
        cleaned = {
            1: self._clamp(counts.get("1", counts.get(1))),
            2: self._clamp(counts.get("2", counts.get(2))),
        }
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            if self.busy:
                self._set("Wait until the move finishes.", ok=False)
                return
            self.follow = False
            if self.drive_mode != "speed":
                seen = self._write_mode_locked(2)
                self.drive_mode = "speed" if seen == [2, 2] else self.drive_mode
            sent = []
            for actuator_id in (1, 2):
                previous = self.aim.get(actuator_id)
                if live and previous is not None and abs(cleaned[actuator_id] - previous) < 20:
                    continue
                self._arm(actuator_id)
                pair = self.pairs.get(actuator_id)
                origin = int(pair["position"]) if pair else cleaned[actuator_id]
                self.speed_cmd[actuator_id] = self._signed_speed(speed, origin, cleaned[actuator_id])
                self.bus.write_regs(actuator_id, 0x28, [speed, cleaned[actuator_id]])
                self.aim[actuator_id] = cleaned[actuator_id]
                sent.append(actuator_id)
            self.speed_sps = speed
            if not sent:
                return
        if live:
            return
        self._set(f"Moving toward {cleaned[1]} and {cleaned[2]}.")

    def _ndi_ignored(self) -> bool:
        return time.time() < self.ndi_ignore_until

    def _wait_ndi_reading(self, wait_s: float = 1.0) -> bool:
        with self.lock:
            start = self.ndi_frame
            live = self.ndi is not None and self.ndi_tracking and not self.ndi_resetting
        deadline = time.perf_counter() + wait_s
        if not live:
            self.ndi_connect()
        while time.perf_counter() < deadline:
            with self.lock:
                if self.ndi_frame > start and self.ndi_points:
                    return True
            time.sleep(0.02)
        return False

    def _begin_play(self, duration: float, path1, path2, mode: str, cycles: int) -> None:
        self.ndi_gate = False
        self.pending_play = None
        with self.lock:
            if self.bus is None:
                self.busy = False
                self._set("Connect the actuators first.", ok=False)
                return
            use_speed = mode == "smooth"
            self.speed_cmd = {1: None, 2: None}
            if not use_speed and self.drive_mode != "position":
                self._write_mode_locked(0)
                self.drive_mode = "position"
            self.follow = False
            self.busy = True
            self.play_id += 1
            self.cancel.clear()
            self._set(f"Playing {cycles} cycle{'s' if cycles != 1 else ''}.")
        self.start_record(duration * cycles + 0.6, note=self.play_note, require_ndi=False)
        worker = self._play_speed_worker if mode == "smooth" else self._play_follow_worker
        thread = threading.Thread(
            target=worker,
            args=(duration, path1, path2, cycles),
            daemon=True,
        )
        self.play_thread = thread
        thread.start()

    def _start_ignored(self, actuator_id: int) -> bool:
        return time.time() < self.start_ignore.get(actuator_id, 0.0)

    def _ndi_ready(self) -> bool:
        if self._ndi_ignored():
            return True
        with self.lock:
            return (
                self.ndi is not None
                and self.ndi_tracking
                and not self.ndi_resetting
                and len(self.ndi_points) == 1
            )

    def _positions_ready(self, targets: dict[int, int]) -> dict[int, bool]:
        ready = {}
        with self.lock:
            for actuator_id, aim in targets.items():
                if self._start_ignored(actuator_id):
                    ready[actuator_id] = True
                    continue
                pair = self.pairs.get(actuator_id)
                ready[actuator_id] = bool(pair) and abs(int(pair["position"]) - aim) <= START_TOLERANCE
        return ready

    def _signed_speed(self, magnitude: int, origin: int, aim: int) -> int:
        """Steps/s with a sign. Positive extends. The drive frame stays unsigned."""
        magnitude = abs(int(magnitude))
        if aim > origin:
            return magnitude
        if aim < origin:
            return -magnitude
        return 0

    def _halt_and_settle(self, hold_s: float = 0.5) -> None:
        """Cut the approach, then wait until both rods sit still."""
        with self.lock:
            if self.bus is not None:
                for actuator_id in (1, 2):
                    try:
                        self.bus.estop(actuator_id)
                    except Exception:
                        pass
                    self.rested[actuator_id] = True
                    self.speed_cmd[actuator_id] = None
        previous: dict[int, int] = {}
        still_since: float | None = None
        deadline = time.perf_counter() + 2.0
        while time.perf_counter() < deadline and not self.cancel.is_set():
            with self.lock:
                if self.bus is not None:
                    try:
                        self._read_both()
                    except Exception:
                        pass
                moving = False
                seen = 0
                for actuator_id in (1, 2):
                    if self._start_ignored(actuator_id):
                        continue
                    pair = self.pairs.get(actuator_id)
                    if not pair:
                        moving = True
                        continue
                    pos = int(pair["position"])
                    seen += 1
                    last = previous.get(actuator_id)
                    previous[actuator_id] = pos
                    if last is None or abs(pos - last) > 1:
                        moving = True
            if seen == 0:
                time.sleep(hold_s)
                return
            now = time.perf_counter()
            if moving:
                still_since = None
            elif still_since is None:
                still_since = now
            elif now - still_since >= hold_s:
                return
            time.sleep(0.04)

    def _command_start(self, targets: dict[int, int]) -> None:
        with self.lock:
            if self.bus is None:
                return
            if self.drive_mode != "speed":
                seen = self._write_mode_locked(2)
                self.drive_mode = "speed" if seen == [2, 2] else self.drive_mode
            for actuator_id, aim in targets.items():
                if self._start_ignored(actuator_id):
                    continue
                pair = self.pairs.get(actuator_id)
                if pair and abs(int(pair["position"]) - aim) <= START_TOLERANCE:
                    continue
                self._arm(actuator_id)
                here = int(pair["position"]) if pair else aim
                self.speed_cmd[actuator_id] = self._signed_speed(SPEED_LIMIT, here, aim)
                self.bus.write_regs(actuator_id, 0x28, [SPEED_LIMIT, aim])
                self.aim[actuator_id] = aim

    def _failed_checks(self, targets: dict[int, int]) -> list[dict]:
        checks = []
        if not self._ndi_ready():
            checks.append({"id": "ndi", "text": "No tracker reading"})
        positions = self._positions_ready(targets)
        for actuator_id in (1, 2):
            if not positions[actuator_id]:
                pair = self.pairs.get(actuator_id)
                here = int(pair["position"]) if pair else None
                checks.append({
                    "id": f"pair{actuator_id}",
                    "text": f"Pair {actuator_id} is at {here}, the curve starts at {targets[actuator_id]}",
                })
        return checks

    def _prepare(self, targets: dict[int, int], move: bool) -> bool:
        """Wait up to 3 s for the tracker and the start positions.

        Play then cuts power and waits until the rods are still. A fixed
        0.2 s left the first cycle starting while the rod was still moving.
        """
        self._set("Initializing.")
        if move:
            self.busy = True
            self._command_start(targets)
        if not self._ndi_ready() and not self._ndi_ignored():
            self.ndi_connect()
        travel = 0
        for actuator_id, aim in targets.items():
            pair = self.pairs.get(actuator_id)
            if pair:
                travel = max(travel, abs(int(pair["position"]) - aim))
        deadline = time.perf_counter() + max(4.0, travel / 1200.0 + 1.5)
        while time.perf_counter() < deadline and not self.cancel.is_set():
            with self.lock:
                if self.bus is not None:
                    try:
                        self._read_both()
                    except Exception:
                        pass
            if self.cancel.is_set():
                self.busy = False
                return False
            if not self._failed_checks(targets):
                if move:
                    self._halt_and_settle(0.5)
                else:
                    time.sleep(0.2)
                self.play_checks = []
                return True
            time.sleep(0.05)
        if self.cancel.is_set():
            self.busy = False
            return False
        if move:
            with self.lock:
                if self.bus is not None:
                    for actuator_id in (1, 2):
                        self.bus.estop(actuator_id)
                        self.rested[actuator_id] = True
                        self.speed_cmd[actuator_id] = None
            self.busy = False
        self.play_checks = self._failed_checks(targets)
        self._set("Not ready.")
        return False

    def _prepare_play(self) -> bool:
        """True when the curve has started. False leaves play_checks for the dialog."""
        duration, path1, path2, mode, cycles = self.pending_play
        targets = {1: self._clamp(path1(0.0)), 2: self._clamp(path2(0.0))}
        if not self._prepare(targets, move=True):
            return False
        self._begin_play(duration, path1, path2, mode, cycles)
        return True

    def _wait_play_thread(self) -> None:
        """Let a stopped move leave before the next Play clears cancel."""
        thread = self.play_thread
        if thread is not None and thread.is_alive():
            thread.join(1.0)

    def play_curves(self, duration: float, knots1, knots2, mode: str = "position", cycles: int = 1, note: str = "") -> None:
        duration = max(0.2, min(30.0, float(duration)))
        try:
            path1 = _clamped_cubic(knots1)
            path2 = _clamped_cubic(knots2)
        except ValueError as exc:
            self._set(str(exc), ok=False)
            return
        cycles = max(1, min(20, int(cycles)))
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            if self.busy and not self.cancel.is_set():
                self._set("A move is already running.", ok=False)
                return
        self._wait_play_thread()
        self.cancel.clear()
        self.play_note = (note or "").strip()
        self.pending_kind = "play"
        self.pending_play = (duration, path1, path2, mode, cycles)
        self.play_checks = []
        self._prepare_play()

    def record_only(self, seconds: float, knots1, knots2, note: str = "") -> None:
        """Same setup as Play, then log without moving the rods."""
        seconds = max(0.5, min(120.0, float(seconds)))
        try:
            path1 = _clamped_cubic(knots1) if knots1 else None
            path2 = _clamped_cubic(knots2) if knots2 else None
        except ValueError as exc:
            self._set(str(exc), ok=False)
            return
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            if self.busy or self.recording:
                self._set("A capture is already running.", ok=False)
                return
        self.pending_kind = "record"
        self.pending_record = (seconds, (note or "").strip(), path1, path2)
        self.play_checks = []
        self._prepare_record()

    def _prepare_record(self) -> bool:
        seconds, note, path1, path2 = self.pending_record
        if path1 is None or path2 is None:
            targets = {1: 0, 2: 0}
            positions = {1: True, 2: True}
        else:
            targets = {1: self._clamp(path1(0.0)), 2: self._clamp(path2(0.0))}
            positions = None
        self._set("Initializing.")
        if not self._ndi_ready() and not self._ndi_ignored():
            self.ndi_connect()
        deadline = time.perf_counter() + 3.0
        while time.perf_counter() < deadline and not self.cancel.is_set():
            with self.lock:
                if self.bus is not None:
                    try:
                        self._read_both()
                    except Exception:
                        pass
            if positions is None:
                ready = not self._failed_checks(targets)
            else:
                ready = self._ndi_ready()
            if ready:
                time.sleep(0.2)
                self.play_checks = []
                self.pending_kind = ""
                self.start_record(seconds, note, require_ndi=False)
                return True
            time.sleep(0.05)
        if positions is None:
            self.play_checks = self._failed_checks(targets)
        elif not self._ndi_ready():
            self.play_checks = [{"id": "ndi", "text": "No tracker reading"}]
        else:
            self.play_checks = []
        self._set("Not ready.")
        return False

    def ignore_check(self, check_id: str) -> None:
        if check_id == "ndi":
            self.ndi_ignore_until = time.time() + 30 * 60
        elif check_id in ("pair1", "pair2"):
            self.start_ignore[int(check_id[-1])] = time.time() + 30 * 60
        else:
            self._set("Unknown condition.", ok=False)
            return
        if self.pending_kind == "record" and self.pending_record is not None:
            self._prepare_record()
            return
        if self.awaiting_resume:
            self.resume()
            return
        if self.pending_play is None:
            self._set("Ignored.")
            return
        self._prepare_play()

    def ignore_ndi_warning(self) -> None:
        self.ignore_check("ndi")

    def _play_follow_worker(self, duration: float, path1, path2, cycles: int = 1) -> None:
        """Follow a point a short time ahead on the curve.

        This is the default. The position target is what the rod chases, so
        the measured position stays with the curve. A target on the rod makes
        the drive finish the step and stop.
        """
        period = 0.05
        lead = 0.3
        paths = {1: path1, 2: path2}
        total = duration * max(1, cycles)
        try:
            with self.lock:
                if self.bus is None:
                    return
                self._write_mode_locked(0)
                self.drive_mode = "position"
                for actuator_id in (1, 2):
                    self._arm(actuator_id)
                self.bus_reserved = True
                bus = self.bus
            origin = time.perf_counter()
            nxt = origin
            trace_prev: dict[int, tuple[float, int]] = {}
            while not self.cancel.is_set():
                origin = self._hold_if_paused(origin)
                if self.cancel.is_set():
                    break
                t = time.perf_counter() - origin
                if t >= total - 1e-9:
                    break
                cycle_t = t % duration
                ahead = min(lead, max(0.0, duration - cycle_t))
                aim_t = min(duration, cycle_t + ahead)
                command = {i: self._clamp(paths[i](aim_t)) for i in (1, 2)}
                with self.lock:
                    if self.bus is None:
                        return
                    self.aim = command
                    for actuator_id in (1, 2):
                        slope = _curve_slope(paths[actuator_id], min(cycle_t, duration), duration)
                        self.speed_cmd[actuator_id] = int(round(max(-SPEED_LIMIT, min(SPEED_LIMIT, slope))))
                bus.broadcast(0xF3, command)
                time.sleep(0.01)
                now_s = time.perf_counter()
                readings = bus.read_status((1, 2), 0.02)
                with self.lock:
                    for actuator_id, found in readings.items():
                        packed = self._pack(found)
                        position = int(packed["position"])
                        previous = trace_prev.get(actuator_id)
                        if previous is not None and now_s - previous[0] > 1e-4:
                            packed["speed"] = (position - previous[1]) / (now_s - previous[0])
                        trace_prev[actuator_id] = (now_s, position)
                        self._read_n[actuator_id] = self._read_n.get(actuator_id, 0) + 1
                        packed["read_n"] = self._read_n[actuator_id]
                        self.pairs[actuator_id] = packed
                nxt += period
                delay = nxt - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    nxt = time.perf_counter()
            with self.lock:
                self.speed_cmd = {1: None, 2: None}
                if self.bus is not None:
                    for actuator_id in (1, 2):
                        self.bus.estop(actuator_id)
                        self.rested[actuator_id] = True
            self._set("Curve finished." if not self.cancel.is_set() else "Move cancelled.")
        except Exception as exc:
            self._set(str(exc), ok=False)
        finally:
            self.bus_reserved = False
            with self.lock:
                self.speed_cmd = {1: None, 2: None}
                if self.bus is not None:
                    for actuator_id in (1, 2):
                        try:
                            self.bus.estop(actuator_id)
                        except Exception:
                            pass
                        self.rested[actuator_id] = True
            self.busy = False

    def _play_speed_worker(self, duration: float, path1, path2, cycles: int = 1) -> None:
        """One speed and one destination for each direction.

        A sine wave is two trips: out, then back. The speed is chosen so the
        rod arrives at the end of that trip on time, including the drive's
        own speed-up and slow-down. No further speed frame is sent until the
        direction changes.
        """
        period = 0.05
        paths = {1: path1, 2: path2}
        total = duration * max(1, cycles)
        try:
            with self.lock:
                if self.bus is None:
                    return
                seen = self._write_mode_locked(2)
                self.drive_mode = "speed" if seen == [2, 2] else self.drive_mode
                for actuator_id in (1, 2):
                    self._arm(actuator_id)
                self.bus_reserved = True
                bus = self.bus
            origin = time.perf_counter()
            nxt = origin
            trace_prev: dict[int, tuple[float, int]] = {}
            sent: dict[int, tuple[int, int]] = {}
            sent_leg = {1: None, 2: None}
            sweeps = {1: _sweeps(path1, duration), 2: _sweeps(path2, duration)}
            while not self.cancel.is_set():
                origin = self._hold_if_paused(origin)
                if self.cancel.is_set():
                    break
                t = time.perf_counter() - origin
                if t >= total - 1e-9:
                    break
                cycle_t = t % duration
                with self.lock:
                    if self.bus is None:
                        return
                    measured = {
                        actuator_id: (
                            int(self.pairs[actuator_id]["position"])
                            if actuator_id in self.pairs and self.pairs[actuator_id].get("position") is not None
                            else None
                        )
                        for actuator_id in (1, 2)
                    }
                if self._resend_leg:
                    sent_leg = {1: None, 2: None}
                    self._resend_leg = False
                cycle_index = int(t // duration)
                command = {}
                for actuator_id in (1, 2):
                    legs = sweeps[actuator_id]
                    if not legs:
                        command[actuator_id] = (0, 0, False)
                        continue
                    index = len(legs) - 1
                    for i, (t0, t1, _p0, _p1) in enumerate(legs):
                        if t0 - 1e-9 <= cycle_t < t1:
                            index = i
                            break
                    key = (cycle_index, index)
                    t0, t1, _p0, end_p = legs[index]
                    previous = sent.get(actuator_id)
                    if sent_leg.get(actuator_id) == key and previous is not None:
                        command[actuator_id] = (previous[0], previous[1], False)
                        continue
                    here = measured[actuator_id]
                    if here is None:
                        here = int(round(paths[actuator_id](min(cycle_t, duration))))
                    remain_d = int(end_p) - here
                    remain_t = max(0.05, t1 - cycle_t)
                    speed = _speed_for_window(remain_d, remain_t)
                    signed = speed if remain_d >= 0 else -speed
                    aim = self._clamp(end_p)
                    command[actuator_id] = (signed, aim, True)
                    sent_leg[actuator_id] = key
                with self.lock:
                    if self.bus is None:
                        return
                    for actuator_id, (signed, aim, fresh) in command.items():
                        self.speed_cmd[actuator_id] = signed
                        self.aim[actuator_id] = aim
                        if not fresh:
                            continue
                        bus.write_regs(actuator_id, 0x28, [abs(signed), aim])
                        sent[actuator_id] = (signed, aim)
                time.sleep(0.01)
                now_s = time.perf_counter()
                readings = bus.read_status((1, 2), 0.02)
                with self.lock:
                    for actuator_id, found in readings.items():
                        packed = self._pack(found)
                        position = int(packed["position"])
                        previous = trace_prev.get(actuator_id)
                        if previous is not None and now_s - previous[0] > 1e-4:
                            packed["speed"] = (position - previous[1]) / (now_s - previous[0])
                        trace_prev[actuator_id] = (now_s, position)
                        self._read_n[actuator_id] = self._read_n.get(actuator_id, 0) + 1
                        packed["read_n"] = self._read_n[actuator_id]
                        self.pairs[actuator_id] = packed
                nxt += period
                delay = nxt - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    nxt = time.perf_counter()
            with self.lock:
                self.speed_cmd = {1: None, 2: None}
                if self.bus is not None:
                    for actuator_id in (1, 2):
                        self.bus.estop(actuator_id)
                        self.rested[actuator_id] = True
            self._set("Curve finished." if not self.cancel.is_set() else "Move cancelled.")
        except Exception as exc:
            self._set(str(exc), ok=False)
        finally:
            self.bus_reserved = False
            with self.lock:
                self.speed_cmd = {1: None, 2: None}
                if self.bus is not None:
                    for actuator_id in (1, 2):
                        try:
                            self.bus.estop(actuator_id)
                        except Exception:
                            pass
                        self.rested[actuator_id] = True
            self.busy = False

    def _stream_follow(self, frames: list[dict[int, int]], period: float = 0.015) -> list[float]:
        """Send follow targets on a fixed clock. No position read on this clock.

        The actuator treats a late or uneven follow packet as a new sprint.
        """
        gaps: list[float] = []
        nxt = time.perf_counter()
        previous = nxt
        for command in frames:
            if self.cancel.is_set():
                break
            now = time.perf_counter()
            gaps.append(now - previous)
            previous = now
            with self.lock:
                if self.bus is None:
                    break
                self.aim = command
                self.bus.broadcast(0xF3, command)
            nxt += period
            delay = nxt - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.perf_counter()
        return gaps

    def _play_worker(self, duration: float, path1, path2, cycles: int = 1) -> None:
        period = 0.01
        try:
            frames = []
            t = 0.0
            while t <= duration + 1e-9:
                frames.append({
                    1: self._clamp(path1(t)),
                    2: self._clamp(path2(t)),
                })
                t += period
            cycle = frames
            with self.lock:
                if self.bus is None:
                    return
                for actuator_id in (1, 2):
                    self._arm(actuator_id)
            gaps = []
            for _cycle in range(max(1, cycles)):
                if self.cancel.is_set():
                    break
                gaps = self._stream_follow(cycle, period)
            with self.lock:
                if self.bus is not None:
                    self._read_both()
            if len(gaps) > 2:
                body = sorted(gaps[1:])
                mid = body[len(body) // 2]
                print(
                    f"follow packets={len(gaps)} median={mid*1000:.1f}ms max={body[-1]*1000:.1f}ms",
                    flush=True,
                )
            self._set("Curve finished." if not self.cancel.is_set() else "Move cancelled.")
        except Exception as exc:
            self._set(str(exc), ok=False)
        finally:
            self.busy = False

    def _run_together(self, steps: list[dict[int, int]], label: str) -> None:
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            if self.busy:
                self._set("A move is already running.", ok=False)
                return
            self.follow = False
            self.busy = True
            self.cancel.clear()
            self._set("Moving both rods together.")

        def work() -> None:
            try:
                for targets in steps:
                    if self.cancel.is_set():
                        break
                    with self.lock:
                        if self.bus is None:
                            return
                        for actuator_id in (1, 2):
                            self._arm(actuator_id)
                        self.bus.broadcast(0xF2, targets)
                    deadline = time.time() + 3.0
                    while time.time() < deadline and not self.cancel.is_set():
                        with self.lock:
                            if self.bus is None:
                                return
                            self._read_both()
                            faulted = [
                                i for i in targets
                                if self.pairs.get(i, {}).get("fault")
                            ]
                            if faulted:
                                for actuator_id in (1, 2):
                                    self.bus.estop(actuator_id)
                                self._set(
                                    f"Pair {faulted[0]} stopped: {self.pairs[faulted[0]]['fault']}.",
                                    ok=False,
                                )
                                return
                            done = all(
                                abs(self.pairs[i]["position"] - counts) <= 25
                                and self.pairs[i]["current_ma"] < 120
                                for i, counts in targets.items()
                            )
                        if done:
                            break
                        time.sleep(0.04)
                with self.lock:
                    if self.bus is not None:
                        self._read_both()
                if self.cancel.is_set():
                    self._set("Move cancelled.")
                else:
                    self._set(label)
            except Exception as exc:
                self._set(str(exc), ok=False)
            finally:
                self.busy = False

        threading.Thread(target=work, daemon=True).start()

    def nudge_both(self, delta: int) -> None:
        delta = max(-500, min(500, int(delta)))
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            current = {i: self.bus.status(i).position for i in (1, 2)}
        targets = {i: self._clamp(current[i] + delta) for i in (1, 2)}
        self._run_together([targets], "Both rods commanded in one frame.")

    def cycle(self, delta: int) -> None:
        delta = max(1, min(500, abs(int(delta))))
        with self.lock:
            if self.bus is None:
                self._set("Connect the actuators first.", ok=False)
                return
            home = {i: self.bus.status(i).position for i in (1, 2)}
        out = {i: self._clamp(home[i] - delta) for i in (1, 2)}
        back = {i: self._clamp(home[i]) for i in (1, 2)}
        self._run_together([out, back], f"Cycled both rods together by {delta} counts.")

    def goto(self, actuator_id: int, counts: int) -> None:
        counts = max(0, min(STROKE_COUNTS, int(counts)))
        self._run_move([(actuator_id, counts)], f"Pair {actuator_id} commanded to {counts} counts.")

    def _motor_cells(self) -> list[str]:
        cells: list[str] = []
        for actuator_id in (1, 2):
            pair = self.pairs.get(actuator_id)
            if not pair:
                cells.extend([""] * 8)
                continue
            cells.extend(
                [
                    str(pair["position"]),
                    f"{pair['mm']:.3f}",
                    str(pair["force_raw_g"]),
                    str(pair["force_g"]),
                    f"{pair['force_N']:.4f}",
                    str(pair["current_ma"]),
                    str(pair["temp_c"]),
                    f"{pair['error']:02X}",
                ]
            )
        return cells

    def _take_ndi_sample(self) -> None:
        """One tracker sample. The background poll stays quiet while a record owns this."""
        with self.lock:
            link = self.ndi if self.ndi is not None and self.ndi_tracking else None
            command = self.ndi_tx
        if link is None:
            return
        try:
            began = time.perf_counter()
            with self.ndi_io:
                raw = link.command(command, wait_s=0.012)
            self._last_ndi_s = time.perf_counter() - began
        except Exception as exc:
            with self.lock:
                if self.ndi is link:
                    self._drop_ndi(str(exc))
            return
        body, _, crc_ok = ndi_ascii.split_reply(raw)
        with self.lock:
            if self.ndi is not link:
                return
            if not raw or not crc_ok or body.startswith("ERROR") or body.startswith("WARNING"):
                return
            try:
                points = ndi_ascii.parse_tx_strays(body)
            except (RuntimeError, ValueError):
                return
            self.ndi_raw = body
            self.ndi_points = points
            self.ndi_count = len(points)
            self.marker_warning = len(points) != 1
            if points:
                text = ndi_ascii.format_xyz(points[0])
                if len(points) > 1:
                    text = f"{len(points)} markers; {text}"
                self.ndi_marker = text
                self.ndi_last = text

    def _log_motor_cells(self, actuator_id: int, time_s: float, fresh: bool) -> list[str]:
        """Target and measured position, speed, and force for one rod.

        Position is counts. Speed is steps/s. The drive does not report
        speed, so the measured value is the change in position divided by
        the time since the previous sample. Target speed is the slope of
        the curve while a play is running, positive when the count is
        increasing. Force is newtons. Target force stays empty. Reaction
        force and the tip sensor are not read on this page.
        """
        commanded = self.speed_cmd.get(actuator_id)
        target_speed = ""
        if commanded is not None:
            target_speed = str(int(commanded))
        pair = self.pairs.get(actuator_id)
        target_pos = "" if not pair else str(int(pair["target"]))
        # During a follow stream the play loop owns the bus and stores the
        # reading. Leaving the measured cells blank is what drew a flat line.
        if self.bus_reserved and pair and pair.get("position") is not None:
            speed = pair.get("speed")
            measured_speed = "" if not isinstance(speed, float) else f"{speed:.1f}"
            return [
                target_pos,
                str(int(pair["position"])),
                target_speed,
                measured_speed,
                "",
                f"{float(pair['force_N']):.4f}",
            ]
        if not fresh or not pair:
            return [target_pos, "", target_speed, "", "", ""]
        position = int(pair["position"])
        previous = self._speed_prev.get(actuator_id)
        measured_speed = ""
        if previous is not None:
            dt = time_s - previous[0]
            if dt > 1e-4:
                measured_speed = f"{(position - previous[1]) / dt:.1f}"
                pair["speed"] = float(measured_speed)
        self._speed_prev[actuator_id] = (time_s, position)
        return [
            target_pos,
            str(position),
            target_speed,
            measured_speed,
            "",
            f"{pair['force_N']:.4f}",
        ]

    def _sample_synced(self) -> str:
        """One row at the log clock. Motors and the tracker share that tick."""
        now = time.perf_counter()
        if self._log_t0 is None:
            self._log_t0 = now
        time_s = now - self._log_t0
        self.sample_n += 1

        def grab() -> None:
            self._take_ndi_sample()

        started = time.perf_counter()
        reader = threading.Thread(target=grab, daemon=True)
        reader.start()
        fresh = {1: False, 2: False}
        with self.lock:
            # A status query between follow frames makes the drive drop the
            # stream and sprint the next point. The bus stays on follow only.
            if self.bus is not None and not self.bus_reserved:
                for actuator_id in (1, 2):
                    found = self.bus.status_quick(actuator_id)
                    if found is not None:
                        self.pairs[actuator_id] = self._pack(found)
                        fresh[actuator_id] = True
        remain = LOG_PERIOD_S - (time.perf_counter() - started) - 0.001
        if remain > 0:
            reader.join(remain)
        with self.lock:
            cells = [f"{time_s:.4f}"]
            for actuator_id in (1, 2):
                cells.extend(self._log_motor_cells(actuator_id, time_s, fresh[actuator_id]))
            cells.extend(self._marker_cells())
            cells.extend(["", "", "", "", ""])
            if len(cells) != len(LOG_COLUMNS):
                raise RuntimeError(f"log row has {len(cells)} cells, header has {len(LOG_COLUMNS)}")
        return ",".join(_csv_cell(cell) for cell in cells)

    def _sample_row(self) -> str:
        cells = [str(time.time_ns()), self.tip_load_g]
        if self.bus is None:
            cells.extend([""] * 18)
        else:
            for actuator_id in (1, 2):
                pair = self._pack(self.bus.status(actuator_id))
                self.pairs[actuator_id] = pair
                cells.extend(
                    [
                        str(pair["position"]),
                        f"{pair['mm']:.3f}",
                        str(pair["force_raw_g"]),
                        str(pair["force_g"]),
                        f"{pair['force_N']:.4f}",
                        str(pair["current_ma"]),
                        str(pair["temp_c"]),
                        f"{pair['error']:02X}",
                    ]
                )
        cells.extend(self._marker_cells())
        return ",".join(cells)

    def start_record(self, seconds: float, note: str = "", require_ndi: bool = True) -> None:
        if self.recording:
            self.record_deadline = max(self.record_deadline, time.time() + float(seconds))
            return
        with self.lock:
            if require_ndi and (self.ndi_resetting or self.ndi is None or not self.ndi_tracking):
                self._set("Connect the tracker before recording.", ok=False)
                return
        note = note.strip()
        self.record_note = note
        self.marker_warning = False
        self.ndi_count = len(self.ndi_points)
        self._log_t0 = None
        self._speed_prev = {1: None, 2: None}
        LOG_DIR.mkdir(exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = LOG_DIR / f"{stamp}_bench.csv"
        meta_path = LOG_DIR / f"{stamp}_meta.json"
        self.log_path = str(path)
        self.record_stop.clear()
        duration = max(0.5, min(120.0, float(seconds)))
        self.record_deadline = time.time() + duration
        self.recording = True
        meta_path.write_text(
            json.dumps(
                {
                    "note": note,
                    "duration_s": duration,
                    "csv": path.name,
                },
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

        def work() -> None:
            try:
                with path.open("w", encoding="utf-8", newline="\n") as handle:
                    handle.write(",".join(LOG_COLUMNS) + "\n")
                    next_tick = time.perf_counter()
                    ndi_times = []
                    while time.time() < self.record_deadline and not self.record_stop.is_set():
                        if self.paused:
                            started = time.perf_counter()
                            while self.paused and not self.record_stop.is_set():
                                time.sleep(0.05)
                            gap = time.perf_counter() - started
                            self.record_deadline += gap
                            next_tick += gap
                            continue
                        handle.write(self._sample_synced() + "\n")
                        if getattr(self, "_last_ndi_s", None):
                            ndi_times.append(self._last_ndi_s)
                            self._last_ndi_s = None
                        next_tick += LOG_PERIOD_S
                        delay = next_tick - time.perf_counter()
                        if delay > 0:
                            time.sleep(delay)
                        else:
                            next_tick = time.perf_counter()
                    if ndi_times:
                        ordered = sorted(ndi_times)
                        mid = ordered[len(ordered) // 2]
                        print(
                            f"log ndi samples={len(ndi_times)} median={mid*1000:.1f}ms",
                            flush=True,
                        )
                self._set(f"Record saved {path.name}")
                threading.Thread(
                    target=self._check_log,
                    args=(path,),
                    name="log-check",
                    daemon=True,
                ).start()
            except Exception as exc:
                self._set(str(exc), ok=False)
            finally:
                self.recording = False
                self.ndi_disconnect()

        threading.Thread(target=work, daemon=True).start()
        self._set(f"Recording {duration:.0f} s to {path.name}")

    def _check_log(self, path: Path) -> None:
        """Run after the file is closed so the sample clock is left alone."""
        try:
            report = log_check.run_checks(path)
        except Exception as exc:
            self._set(f"Record saved {path.name}. Check failed: {exc}", ok=False)
            return
        gaps = int(report.get("ndi_not_one") or 0)
        self.marker_warning = False
        if gaps and report.get("ndi_present", True):
            self._set(f"Record saved {path.name}. Filled {gaps} rows that were not one marker.")
        else:
            self._set(f"Record saved {path.name}")

    def stop_record(self) -> None:
        self.record_stop.set()
        self._set("Stopping the record.")

    def _marker_cells(self) -> list[str]:
        if not self.ndi_points:
            xyz = ["", "", ""]
        else:
            xyz = ndi_ascii.format_xyz(self.ndi_points[0]).split(",")
        raw = (self.ndi_raw or "").replace("\r", " ").replace("\n", " ")
        return xyz + [raw]

    def _poll_marker(self) -> None:
        if self.ndi is None:
            return
        raw = self.ndi.command("TX 1000", wait_s=0.6)
        body, _, crc_ok = ndi_ascii.split_reply(raw)
        if not crc_ok:
            self.ndi_points = []
            self.ndi_marker = ""
            self.ndi_last = raw or "(empty TX reply)"
            return
        points = ndi_ascii.parse_tx_strays(body)
        self.ndi_points = points
        if not points:
            self.ndi_marker = "no marker"
            self.ndi_last = self.ndi_marker
            return
        text = ndi_ascii.format_xyz(points[0])
        if len(points) > 1:
            text = f"{len(points)} markers; {text}"
        self.ndi_marker = text
        self.ndi_last = text

    def _drop_ndi(self, reason: str) -> None:
        link = self.ndi
        self.ndi = None
        self.ndi_tracking = False
        self.ndi_points = []
        self.ndi_marker = ""
        self.ndi_revision = ""
        self.ndi_firmware = ""
        self.ndi_last = reason
        self.ndi_ready_at = 0.0
        if link is not None:
            try:
                link.close()
            except Exception:
                pass

    def _release_ndi(self) -> None:
        with self.lock:
            old = self.ndi
            self.ndi = None
            self.ndi_tracking = False
            self.ndi_points = []
        if old is None:
            return
        try:
            old.command("TSTOP ", wait_s=0.5)
        except Exception:
            pass
        try:
            old.close()
        except Exception:
            pass

    def _ndi_open(self, port: str, gen: int | None = None) -> bool:
        with self.ndi_open_lock:
            with self.lock:
                if gen is None and (self.ndi_connecting or self.ndi is not None):
                    return False
                if gen is not None and gen != self.ndi_connect_gen:
                    return False
                if not self.ndi_wanted or self.ndi_poll_stop.is_set():
                    return False
            self._release_ndi()
            if gen is not None and gen != self.ndi_connect_gen:
                return False
            try:
                link = ndi_ascii.NdiLink(port)
            except Exception as exc:
                self.ndi_last = f"{port} timed out" if "timed out" in str(exc).lower() else f"{port}: {exc}"
                return False
            with self.lock:
                if (gen is not None and gen != self.ndi_connect_gen) or not self.ndi_wanted:
                    link.close()
                    return False
                self.ndi_opening = link
            try:
                revision, firmware = ndi_ascii.read_identity(link)
                ndi_ascii.expect_okay(link, "INIT ", wait_s=12)
                rom_path = Path(__file__).resolve().parent / "8700339.rom"
                ndi_ascii.load_passive_rom(link, rom_path.read_bytes())
                ndi_ascii.expect_okay(link, "TSTART ", wait_s=5)
            except Exception as exc:
                link.close()
                with self.lock:
                    if self.ndi_opening is link:
                        self.ndi_opening = None
                self.ndi_last = str(exc)
                return False
            with self.lock:
                if self.ndi_opening is link:
                    self.ndi_opening = None
                if (
                    self.ndi_poll_stop.is_set()
                    or not self.ndi_wanted
                    or (gen is not None and gen != self.ndi_connect_gen)
                ):
                    link.close()
                    return False
                self.ndi = link
                self.ndi_port = port
                self.ndi_tracking = True
                self.ndi_revision = revision
                self.ndi_firmware = firmware
                self.ndi_identity = "  ".join(
                    part for part in (port, revision, firmware) if part
                )
                self.ndi_points = []
                self.ndi_marker = ""
                self.ndi_ready_at = time.time()
                self.ndi_resetting = False
            return True

    def _ndi_reset_due(self) -> bool:
        with self.lock:
            if self.recording or self.ndi_resetting or self.ndi_ready_at <= 0:
                return False
            return time.time() - self.ndi_ready_at >= NDI_RESET_AFTER_S

    def _ndi_reset(self) -> None:
        with self.lock:
            if self.recording:
                return
            link = self.ndi
            self.ndi = None
            self.ndi_tracking = False
            self.ndi_points = []
            self.ndi_marker = ""
            self.ndi_ready_at = 0.0
            self.ndi_resetting = True
        if link is not None:
            try:
                with self.ndi_io:
                    link.command("RESET ", wait_s=3.0)
            except Exception:
                pass
            try:
                link.close()
            except Exception:
                pass
        if self.ndi_poll_stop.wait(2.0):
            return
        hosts = ndi_ascii.tracker_hosts()
        if hosts:
            self._ndi_open(hosts[0])

    def _ndi_connect_once(self, port: str, gen: int | None = None) -> bool:
        if gen is not None and gen != self.ndi_connect_gen:
            return False
        hosts = ndi_ascii.tracker_hosts()
        if port.strip():
            hosts = [port.strip()]
        if gen is not None and gen != self.ndi_connect_gen:
            return False
        if not hosts:
            self.ndi_last = "no tracker on the link"
            return False
        return self._ndi_open(hosts[0], gen)

    def ndi_connect(self, port: str = "", limit_s: float = 0.0) -> None:
        if limit_s > 0:
            self.ndi_connecting = True
        self.ndi_wanted = True
        if limit_s <= 0:
            self._ndi_connect_once(port)
            return
        self.ndi_connect_gen += 1
        gen = self.ndi_connect_gen
        done = threading.Event()
        holder = {"ok": False}

        def work() -> None:
            try:
                holder["ok"] = self._ndi_connect_once(port, gen)
            finally:
                done.set()

        threading.Thread(target=work, name="ndi-connect", daemon=True).start()
        if done.wait(limit_s) and holder["ok"] and gen == self.ndi_connect_gen:
            self.ndi_connecting = False
            return
        with self.lock:
            if gen == self.ndi_connect_gen:
                self.ndi_connect_gen += 1
            opening = self.ndi_opening
            self.ndi_opening = None
        self.ndi_connecting = False
        self.ndi_disconnect()
        if opening is not None:
            try:
                opening.close()
            except Exception:
                pass

    def ndi_disconnect(self) -> None:
        self.ndi_wanted = False
        self.ndi_resetting = False
        self.ndi_ready_at = 0.0
        self._release_ndi()
        self.ndi_points = []
        self.ndi_marker = ""
        self.ndi_raw = ""

    def ndi_command(self, name: str) -> None:
        if name != "beep":
            return
        with self.ndi_io:
            with self.lock:
                link = self.ndi
            if link is None:
                return
            try:
                reply = link.command("BEEP 2")
                self.ndi_last = reply or "(empty reply)"
            except Exception as exc:
                with self.lock:
                    if self.ndi is link:
                        self._drop_ndi(str(exc))

    def state(self) -> dict:
        with self.lock:
            if (
                self.bus is not None
                and not self.recording
                and not self.busy
                and time.time() - self._read_at > 0.03
            ):
                try:
                    self._read_both()
                    self._read_at = time.time()
                    self._rest_close_locked()
                except Exception as exc:
                    self._set(str(exc), ok=False)
        with self.lock:
            return {
                "ok": self.ok,
                "message": self.message,
                "connected": self.bus is not None,
                "busy": self.busy,
                "paused": self.paused,
                "speed_hz": self.speed_hz,
                "recording": self.recording,
                "record_remaining": round(max(0.0, self.record_deadline - time.time()), 1) if self.recording else 0,
                "log_path": self.log_path,
                "tip_load_g": self.tip_load_g,
                "pairs": [self.pairs[i] for i in (1, 2) if i in self.pairs],
                "following": self.follow,
                "move_rate": self.move_rate,
                "drive_mode": self.drive_mode,
                "speed_sps": self.speed_sps,
                "speed_limit": SPEED_LIMIT,
                "play_id": self.play_id,
                "sample_n": self.sample_n,
                "motion_s": None if self._log_t0 is None or not self.busy else round(time.perf_counter() - self._log_t0, 4),
                "marker_warning": self.marker_warning,
                "ndi_gate": self.ndi_gate,
                "play_checks": self.play_checks,
                "ndi": {
                    "connected": self.ndi is not None,
                    "port": self.ndi_port,
                    "choices": self.ndi_choices,
                    "last": self.ndi_last,
                    "identity": self.ndi_identity,
                    "marker": self.ndi_marker,
                    "info": self._ndi_line(),
                    "xyz": self._xyz_text(),
                    "raw": self.ndi_raw,
                    "ready": self.ndi is not None and self.ndi_tracking and not self.ndi_resetting,
                    "resetting": self.ndi_resetting,
                    "wanted": self.ndi_wanted,
                },
            }

    def close(self) -> None:
        self.cancel.set()
        self.record_stop.set()
        self.follow = False
        self.follow_stop.set()
        self.ndi_poll_stop.set()
        self.rest_stop.set()
        with self.lock:
            if self.bus is not None:
                try:
                    for actuator_id in (1, 2):
                        self.bus.estop(actuator_id)
                except Exception:
                    pass
                self.bus.close()
                self.bus = None
            if self.ndi is not None:
                if self.ndi_tracking:
                    try:
                        self.ndi.command("TSTOP ", wait_s=0.5)
                    except Exception:
                        pass
                self.ndi.close()
                self.ndi = None
                self.ndi_tracking = False


BENCH = Bench()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args) -> None:
        return

    def _send(self, raw: bytes, content_type: str, code: int = 200) -> None:
        try:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            return

    def _json(self, payload: dict, code: int = 200) -> None:
        self._send(json.dumps(payload).encode("utf-8"), "application/json", code)

    def _body(self) -> dict:
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/state":
            self._json(BENCH.state())
            return
        if path in ("/", "/index.html"):
            self._send(PAGE.read_bytes(), "text/html; charset=utf-8")
            return
        self.send_error(404)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            body = self._body()
            if path == "/api/connect":
                BENCH.connect()
            elif path == "/api/stop":
                BENCH.stop()
            elif path == "/api/pause":
                BENCH.pause()
            elif path == "/api/resume":
                BENCH.resume()
            elif path == "/api/smooth-hz":
                BENCH.set_speed_hz(body.get("hz") or 10)
            elif path == "/api/tare":
                BENCH.tare()
            elif path == "/api/jog":
                BENCH.jog(int(body["id"]), int(body["delta"]))
            elif path == "/api/aim":
                BENCH.aim_to(body.get("counts") or {})
            elif path == "/api/rate":
                BENCH.set_rate(int(body.get("rate") or 2500))
            elif path == "/api/limits/read":
                limits = BENCH.read_limits()
                payload = BENCH.state()
                payload["limits"] = limits
                payload["ok"] = bool(limits.get("ok")) and BENCH.ok
                payload["message"] = BENCH.message
                if not payload["ok"]:
                    payload["error"] = BENCH.message
                self._json(payload)
                return
            elif path == "/api/limits/apply":
                BENCH.apply_limits(body)
            elif path == "/api/limits/zero":
                raw_id = body.get("id")
                BENCH.zero_force_sensors(int(raw_id) if raw_id else None)
            elif path == "/api/drive-mode":
                BENCH.set_drive_mode(str(body.get("mode") or "position"))
            elif path == "/api/speed-go":
                BENCH.speed_go(
                    int(body.get("speed") or 1200),
                    body.get("counts") or {},
                    bool(body.get("live")),
                )
            elif path == "/api/play":
                if body.get("hz") is not None:
                    BENCH.set_speed_hz(body.get("hz"))
                BENCH.play_curves(
                    float(body.get("duration") or 1.0),
                    body.get("pair1") or [],
                    body.get("pair2") or [],
                    str(body.get("mode") or "position"),
                    int(body.get("cycles") or 1),
                    str(body.get("note") or ""),
                )
            elif path == "/api/play/ignore":
                BENCH.ignore_check(str(body.get("id") or "ndi"))
            elif path == "/api/nudge-both":
                if body.get("counts"):
                    BENCH.aim_to(body["counts"])
                else:
                    BENCH.nudge_both(int(body["delta"]))
            elif path == "/api/cycle":
                BENCH.cycle(int(body["delta"]))
            elif path == "/api/goto":
                BENCH.goto(int(body["id"]), int(body["counts"]))
            elif path in ("/api/follow", "/api/follow/settle"):
                counts = body.get("counts") or {}
                BENCH.follow_to(counts, int(body.get("seq") or 0), path == "/api/follow")
            elif path == "/api/record":
                BENCH.record_only(
                    float(body.get("seconds") or 5),
                    body.get("pair1") or [],
                    body.get("pair2") or [],
                    str(body.get("note") or ""),
                )
            elif path == "/api/record/stop":
                BENCH.stop_record()
            elif path == "/api/ndi/connect":
                BENCH.ndi_connect(str(body.get("port") or ""), limit_s=3.0)
            elif path == "/api/ndi/disconnect":
                BENCH.ndi_disconnect()
            elif path == "/api/ndi/command":
                BENCH.ndi_command(str(body.get("name") or ""))
            else:
                self._json({"ok": False, "error": "Unknown button"}, 404)
                return
            payload = BENCH.state()
            payload["ok"] = BENCH.ok
            payload["message"] = BENCH.message
            if not BENCH.ok:
                payload["error"] = BENCH.message
            self._json(payload)
        except Exception as exc:
            BENCH._set(str(exc), ok=False)
            self._json({"ok": False, "error": str(exc), "message": str(exc)})


def main() -> None:
    if crc16_ok() is False:
        raise SystemExit("NDI CRC self-check failed")
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"CATs bench  http://{HOST}:{PORT}/  ramp", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        BENCH.close()
        server.server_close()


def crc16_ok() -> bool:
    return ndi_ascii.crc16_arc(b"RESET") == 0xBE6F


if __name__ == "__main__":
    main()
