import sys
import time

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

# ---------- Configuration ----------
CSV_PATH = "Samples_with_jacobian.csv"
Q_COLS = [f"q{i}" for i in range(1, 8)]   # joint angle columns (q7 is read but not used)
P_COLS = ["x", "y", "z"]                  # target point: the end effector
J_COLS = [f"j{i}" for i in range(1, 37)]  # "Jacobian EE": 6x6, column-major (j1..j6 = first column)
CHAIN = list(range(6))                    # joints 1-6. Joint 7 does not move this point or the Jacobian.
USE_JACOBIAN = True                       # False = position-only fit (J_COLS are then not needed)
JACOBIAN_ROW_ORDER = "auto"               # "auto", "linear_angular" or "angular_linear"
JACOBIAN_LENGTH_SCALE = 1.0               # multiply the Jacobian's linear rows by this to get the
                                          # units of x,y,z (e.g. 1000 if J is in m but x,y,z in mm)
ANGLES_IN_DEGREES = False                 # applies to q only; the Jacobian is assumed per radian
LOCKED_LAST_ALPHA = 0.0                   # alpha of the last joint never affects position or the Jacobian
MAX_LENGTH = 1.0      # bound on |a| and |d|, in the units of x,y,z (use 1000 if mm)

# Residual weights. Position and the Jacobian's linear rows are lengths; the angular rows are
# dimensionless axis directions, so they get a "lever arm" so that an axis error of e radians
# counts like a position error of e * W_JAC_ANG. Raise W_JAC_* to trust the Jacobian more.
W_POS = 1.0
W_JAC_LIN = 1.0
W_JAC_ANG = 0.3 * MAX_LENGTH

N_STARTS = 20         # random restarts (problem is nonconvex)
MAX_SAMPLES = 1000    # subsample for speed; set to None to use all rows
SEED = 0
CHUNK_ROWS = 100_000  # rows per chunk while reading the CSV (only affects the progress display)

rng = np.random.default_rng(SEED)
n = len(CHAIN)


# ---------- Progress reporting ----------
def fmt_time(seconds):
    s = int(seconds)
    return f"{s // 3600:02d}:{(s % 3600) // 60:02d}:{s % 60:02d}"


class Progress:
    """Live status line: which start, how many evaluations, how many samples consumed so far.

    One 'evaluation' is a full pass over all samples. The optimizer also makes extra
    passes to estimate its own Jacobian by finite differences (about n_params + 1 passes
    per iteration), so these counters climb much faster than the number of iterations.
    RMS values here are of the *weighted* residual vector.
    """

    def __init__(self, n_starts, n_samples):
        self.n_starts = n_starts
        self.n_samples = n_samples
        self.t0 = time.time()
        self.tty = sys.stdout.isatty()
        self.interval = 0.25 if self.tty else 5.0   # seconds between redraws
        self.last_draw = 0.0
        self.maxlen = 0
        self.start_idx = 0
        self.evals = 0            # full passes over the data
        self.samples = 0          # total samples consumed across all passes
        self.cur_rms = float("nan")
        self.best_rms = float("nan")

    def end_eval(self, res):
        self.evals += 1
        self.samples += self.n_samples
        self.cur_rms = float(np.sqrt(np.mean(res ** 2)))
        self.draw()

    def draw(self, force=False):
        now = time.time()
        if not force and now - self.last_draw < self.interval:
            return
        self.last_draw = now
        line = (f"start {self.start_idx}/{self.n_starts} | eval {self.evals:,} | "
                f"{self.samples:,} samples consumed | wRMS now {self.cur_rms:.4g}, "
                f"best {self.best_rms:.4g} | {fmt_time(now - self.t0)}")
        self.maxlen = max(self.maxlen, len(line))
        if self.tty:
            sys.stdout.write("\r" + line.ljust(self.maxlen))
        else:
            sys.stdout.write(line + "\n")
        sys.stdout.flush()

    def message(self, text):
        """Print a permanent line without garbling the status line."""
        if self.tty:
            sys.stdout.write("\r" + " " * self.maxlen + "\r")
        print(text)
        sys.stdout.flush()

    def finish(self):
        if self.tty:
            sys.stdout.write("\r" + " " * self.maxlen + "\r")
        print(f"Done: {self.evals:,} evaluations, {self.samples:,} samples consumed, "
              f"{fmt_time(time.time() - self.t0)} elapsed")


# ---------- Load data ----------
if USE_JACOBIAN and CHAIN != list(range(6)):
    raise ValueError("The Jacobian columns are joints 1-6, so CHAIN must be [0..5] when USE_JACOBIAN is True.")

use_cols = Q_COLS + P_COLS + (J_COLS if USE_JACOBIAN else [])

with open(CSV_PATH, "rb") as f:
    total_rows = max(sum(1 for _ in f) - 1, 0)   # minus header
print(f"Reading {CSV_PATH}: {total_rows:,} rows")

chunks, rows_read = [], 0
for chunk in pd.read_csv(CSV_PATH, chunksize=CHUNK_ROWS):
    chunks.append(chunk[use_cols])
    rows_read += len(chunk)
    print(f"\r  read {rows_read:,} / {total_rows:,} rows "
          f"({100.0 * rows_read / max(total_rows, 1):.0f}%)", end="", flush=True)
print()
df = pd.concat(chunks, ignore_index=True)
n_before = len(df)
df = df.dropna().reset_index(drop=True)
if len(df) < n_before:
    print(f"Dropped {n_before - len(df):,} rows containing NaN")

Q = df[Q_COLS].to_numpy(float)
P = df[P_COLS].to_numpy(float)

if ANGLES_IN_DEGREES:
    Q = np.deg2rad(Q)
elif np.max(np.abs(Q)) > 2 * np.pi + 0.1:
    print("Warning: joint values exceed 2*pi. If these are degrees, set ANGLES_IN_DEGREES = True.")


# ---------- Jacobian preparation ----------
def prepare_jacobian(Jraw):
    """Split the column-major 6x6 Jacobian into linear (N,3,6) and angular (N,3,6) blocks.

    For a base-frame geometric Jacobian, column j is [z_(j-1) x (p_ee - o_(j-1)) ; z_(j-1)], so the
    angular block of column 1 is the (constant) axis of joint 1. That is used to tell which three
    rows are the angular ones.
    """
    J = Jraw.reshape(-1, 6, 6).transpose(0, 2, 1)          # column-major -> J[sample, row, col]
    top_std = J[:, :3, 0].std(axis=0).max()
    bot_std = J[:, 3:, 0].std(axis=0).max()
    top_norm = np.linalg.norm(J[:, :3, 0].mean(axis=0))
    bot_norm = np.linalg.norm(J[:, 3:, 0].mean(axis=0))
    print(f"Jacobian column 1: rows 1-3 vary by {top_std:.3g} (mean norm {top_norm:.3g}), "
          f"rows 4-6 vary by {bot_std:.3g} (mean norm {bot_norm:.3g})")

    order = JACOBIAN_ROW_ORDER
    if order == "auto":
        bot_const = bot_std < 1e-3 and abs(bot_norm - 1) < 0.05
        top_const = top_std < 1e-3 and abs(top_norm - 1) < 0.05
        if bot_const and top_const:
            print("Warning: both row blocks of column 1 look constant (arm barely moved?). "
                  "Assuming linear_angular.")
            order = "linear_angular"
        elif bot_const:
            order = "linear_angular"
        elif top_const:
            order = "angular_linear"
        else:
            raise ValueError(
                "Cannot identify the angular rows: neither block of Jacobian column 1 is a constant unit "
                "vector. The Jacobian may not be in the base frame, or the columns are not in the "
                "order assumed. Set JACOBIAN_ROW_ORDER explicitly to skip this check.")
    elif order not in ("linear_angular", "angular_linear"):
        raise ValueError(f"Unknown JACOBIAN_ROW_ORDER: {order}")
    print(f"Jacobian row order: {order}")

    lin, ang = (J[:, :3, :], J[:, 3:, :]) if order == "linear_angular" else (J[:, 3:, :], J[:, :3, :])
    lin = lin * JACOBIAN_LENGTH_SCALE

    norms = np.linalg.norm(ang, axis=1).mean(axis=0)       # mean |angular part| per column
    if np.any(np.abs(norms - 1) > 0.05):
        print(f"Warning: angular parts are not unit axis vectors (mean norms {np.round(norms, 3)}). "
              "That suggests an analytic / rate-based Jacobian, which this model does not represent.")
    z1 = ang[:, :, 0].mean(axis=0)
    if np.linalg.norm(z1 - np.array([0.0, 0.0, 1.0])) > 0.05:
        print(f"Warning: joint 1 axis in the data frame is {np.round(z1, 3)}, not +z. A DH chain with an "
              "identity base cannot represent that; a fixed base rotation would be needed.")
    return lin, ang


Jlin = Jang = None
if USE_JACOBIAN:
    Jlin, Jang = prepare_jacobian(df[J_COLS].to_numpy(float))

if MAX_SAMPLES and len(Q) > MAX_SAMPLES:
    idx = rng.choice(len(Q), MAX_SAMPLES, replace=False)
    Q, P = Q[idx], P[idx]
    if USE_JACOBIAN:
        Jlin, Jang = Jlin[idx], Jang[idx]
    print(f"Subsampled {len(Q):,} of {len(df):,} rows (MAX_SAMPLES = {MAX_SAMPLES})")

Qc = Q[:, CHAIN]
print(f"Fitting {n} joints to {len(Qc)} samples of {P_COLS}"
      + (" + Jacobian EE" if USE_JACOBIAN else " (position only)"))

# ---------- Parameter vector (last alpha is locked, not fitted) ----------
LOCK_IDX = (n - 1) * 4 + 2                                  # flat index of the last joint's alpha
FREE_IDX = np.array([i for i in range(n * 4) if i != LOCK_IDX])


def expand(x):
    """Free parameters -> full flat [a, d, alpha, offset] per joint, with the last alpha locked."""
    full = np.empty(n * 4)
    full[FREE_IDX] = x
    full[LOCK_IDX] = LOCKED_LAST_ALPHA
    return full


progress = Progress(N_STARTS, len(Qc))


# ---------- Model ----------
def model(params, Qs):
    """Standard-DH forward kinematics for ALL samples at once.

    params is flat, ordered [a, d, alpha, offset] per joint. Per link:
        A = Rz(q + offset) * Tz(d) * Tx(a) * Rx(alpha)   (same as RevoluteDH)
    Returns
        t       (N, 3)    end-effector position
        axes    (N, n, 3) axis of joint j in the base frame (= z of frame j-1)
        origins (N, n, 3) origin of frame j-1 in the base frame (a point on joint j's axis)
    so the base-frame geometric Jacobian is [axis x (t - origin) ; axis] per column.
    """
    p = params.reshape(n, 4)
    N = Qs.shape[0]
    R = np.broadcast_to(np.eye(3), (N, 3, 3)).copy()
    t = np.zeros((N, 3))
    axes = np.empty((N, n, 3))
    origins = np.empty((N, n, 3))
    for j in range(n):
        a, d, al, off = p[j]
        axes[:, j] = R[:, :, 2]
        origins[:, j] = t
        th = Qs[:, j] + off
        ct, st = np.cos(th), np.sin(th)
        ca, sa = np.cos(al), np.sin(al)
        step = np.stack([a * ct, a * st, np.full(N, d)], axis=1)     # link translation, parent frame
        t = t + np.einsum("nij,nj->ni", R, step)
        Rj = np.zeros((N, 3, 3))
        Rj[:, 0, 0], Rj[:, 0, 1], Rj[:, 0, 2] = ct, -st * ca, st * sa
        Rj[:, 1, 0], Rj[:, 1, 1], Rj[:, 1, 2] = st, ct * ca, -ct * sa
        Rj[:, 2, 1], Rj[:, 2, 2] = sa, ca
        R = R @ Rj
    return t, axes, origins


def model_jacobian(t, axes, origins):
    """Model Jacobian blocks, each (N, 3, n), comparable with Jlin / Jang."""
    lin = np.cross(axes, t[:, None, :] - origins).transpose(0, 2, 1)
    ang = axes.transpose(0, 2, 1)
    return lin, ang


def residuals(x):
    t, axes, origins = model(expand(x), Qc)
    parts = [W_POS * (t - P).ravel()]
    if USE_JACOBIAN:
        lin_m, ang_m = model_jacobian(t, axes, origins)
        parts.append(W_JAC_LIN * (lin_m - Jlin).ravel())
        parts.append(W_JAC_ANG * (ang_m - Jang).ravel())
    res = np.concatenate(parts)
    progress.end_eval(res)
    return res


def check_against_rtb():
    """Cross-check position and Jacobian against roboticstoolbox (if installed) on a few samples."""
    try:
        import roboticstoolbox as rtb
    except ImportError:
        print("roboticstoolbox not installed: skipping kinematics cross-check")
        return
    rc = np.random.default_rng(123)
    test = np.column_stack([rc.uniform(-0.3, 0.3, n), rc.uniform(-0.3, 0.3, n),
                            rc.choice([0, np.pi / 2, -np.pi / 2, np.pi], n),
                            rc.uniform(-np.pi, np.pi, n)])
    robot = rtb.DHRobot([rtb.RevoluteDH(a=a, d=d, alpha=al, offset=off) for a, d, al, off in test])
    idx = rc.choice(len(Qc), min(5, len(Qc)), replace=False)
    t, axes, origins = model(test.ravel(), Qc[idx])
    ref = np.array([robot.fkine(Qc[i]).t for i in idx])
    err = np.abs(ref - t).max()
    if err > 1e-9:
        raise RuntimeError(f"Vectorised FK disagrees with roboticstoolbox (max diff {err:.3g}).")
    msg = f"position max diff {err:.1e}"
    if hasattr(robot, "jacob0"):
        lin, ang = model_jacobian(t, axes, origins)
        mine = np.concatenate([lin, ang], axis=1)                    # (k, 6, n)
        theirs = np.array([robot.jacob0(Qc[i]) for i in idx])        # (k, 6, n), [linear; angular]
        jerr = np.abs(mine - theirs).max()
        if jerr > 1e-9:
            raise RuntimeError(f"Model Jacobian disagrees with roboticstoolbox jacob0 (max diff {jerr:.3g}).")
        msg += f", Jacobian max diff {jerr:.1e}"
    print(f"Kinematics cross-check against roboticstoolbox passed ({msg})")


check_against_rtb()

# ---------- Bounds and initial guesses ----------
# Angles get +/-2*pi so starting values never sit on a bound.
lower_full = np.tile([-MAX_LENGTH, -MAX_LENGTH, -2 * np.pi, -2 * np.pi], n)
lower = lower_full[FREE_IDX]
upper = -lower


def random_start():
    a = rng.uniform(-0.3, 0.3, n) * MAX_LENGTH
    d = rng.uniform(-0.3, 0.3, n) * MAX_LENGTH
    alpha = rng.choice([0, np.pi / 2, -np.pi / 2, np.pi], n)
    offset = rng.uniform(-np.pi, np.pi, n)
    return np.column_stack([a, d, alpha, offset]).ravel()[FREE_IDX]


# ---------- Multi-start fit ----------
best, costs = None, []
for s in range(N_STARTS):
    progress.start_idx = s + 1
    t_start = time.time()
    x0 = random_start()
    try:
        r = least_squares(residuals, x0, bounds=(lower, upper), method="trf", max_nfev=200)
    except Exception as e:
        progress.message(f"  start {s + 1}/{N_STARTS}: failed ({e})")
        continue
    costs.append(r.cost)
    rms_s = float(np.sqrt(np.mean(r.fun ** 2)))
    new_best = best is None or r.cost < best.cost
    if new_best:
        best = r
        progress.best_rms = rms_s
    progress.message(f"  start {s + 1}/{N_STARTS} done: wRMS {rms_s:.4g}, nfev {r.nfev}, "
                     f"{fmt_time(time.time() - t_start)}" + ("  <- new best" if new_best else ""))

progress.finish()

if best is None:
    raise RuntimeError("All starts failed. Check the CSV and CHAIN settings.")

# ---------- Report ----------
full_best = expand(best.x)
t_b, axes_b, origins_b = model(full_best, Qc)

pos_err = np.linalg.norm(t_b - P, axis=1)
spread = np.std(P, axis=0).mean()
print(f"\nPosition error (euclidean): avg {pos_err.mean():.6g}, min {pos_err.min():.6g}, "
      f"max {pos_err.max():.6g}  (mean point spread: {spread:.4g})")

if USE_JACOBIAN:
    lin_m, ang_m = model_jacobian(t_b, axes_b, origins_b)
    lin_rms = np.sqrt(np.mean((lin_m - Jlin) ** 2))
    d_ang = np.linalg.norm(ang_m - Jang, axis=1)                       # (N, n) chord length
    ang_deg = np.degrees(2 * np.arcsin(np.clip(d_ang / 2, 0, 1)))      # angle between axes
    print(f"Jacobian linear rows: RMS error {lin_rms:.4g}  (typical magnitude {np.sqrt(np.mean(Jlin ** 2)):.4g})")
    print(f"Jacobian angular rows (joint axis direction error): avg {ang_deg.mean():.4g} deg, "
          f"max {ang_deg.max():.4g} deg")
    print("  per joint avg [deg]: " + ", ".join(f"j{CHAIN[i] + 1} {ang_deg[:, i].mean():.3g}" for i in range(n)))

print("\nDistinct cost levels across starts (lowest first):")
for c in sorted(set(np.round(costs, 8)))[:5]:
    print(f"  {c:.6g}  ({np.sum(np.isclose(costs, c, atol=1e-8))} starts)")

p = full_best.reshape(n, 4)
print(f"\n{'joint':>6} {'a':>10} {'d':>10} {'alpha':>10} {'offset':>10}")
for i, (a, d, al, off) in enumerate(p):
    lock = "  (alpha locked)" if i == n - 1 else ""
    print(f"{CHAIN[i] + 1:>6} {a:10.4f} {d:10.4f} {al:10.4f} {off:10.4f}{lock}")

print("\nFor roboticstoolbox:")
for a, d, al, off in p:
    print(f"    rtb.RevoluteDH(a={a:.6f}, d={d:.6f}, alpha={al:.6f}, offset={off:.6f}),")

# ---------- Identifiability check ----------
# Small singular values mean parameter combinations the data cannot resolve,
# e.g. a and d trading off on parallel axes, or the last offset when the last a is ~0.
names_full = [f"j{j + 1}.{nm}" for j in CHAIN for nm in ("a", "d", "alpha", "offset")]
names = [names_full[i] for i in FREE_IDX]
_, S, Vt = np.linalg.svd(best.jac, full_matrices=False)
rel = S / S[0]
weak = np.where(rel < 1e-4)[0]
if len(weak) == 0:
    print("\nAll parameter combinations are well determined.")
else:
    print(f"\n{len(weak)} weakly determined parameter combination(s):")
    for k in weak:
        top = np.argsort(-np.abs(Vt[k]))[:4]
        print("  " + ", ".join(f"{names[t]} ({Vt[k][t]:+.2f})" for t in top))