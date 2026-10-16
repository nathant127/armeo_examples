import sys
import time

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

# ---------- Configuration ----------
CSV_PATH = "JointAngleVsEndEffector.csv"
Q_COLS = [f"q{i}" for i in range(1, 8)]   # joint angle columns, joint 1 = shoulder
P_COLS = ["x", "y", "z"]                  # target point (e.g. EE, EL, WR)
CHAIN = list(range(7))                    # 0-based indices of joints on the serial path
                                          # from the base to the target point
ANGLES_IN_DEGREES = False
MAX_LENGTH = 1.0      # bound on |a| and |d|, in the units of x,y,z (use 1000 if mm)
N_STARTS = 20         # random restarts (problem is nonconvex)
MAX_SAMPLES = 1000    # subsample for speed; set to None to use all rows
SEED = 0
CHUNK_ROWS = 100_000  # rows per chunk while reading the CSV (only affects the progress display)

rng = np.random.default_rng(SEED)


# ---------- Progress reporting ----------
def fmt_time(seconds):
    s = int(seconds)
    return f"{s // 3600:02d}:{(s % 3600) // 60:02d}:{s % 60:02d}"


class Progress:
    """Live status line: which start, how many evaluations, how many samples consumed so far.

    One 'evaluation' is a full pass over all samples. The optimizer also makes extra
    passes to estimate the Jacobian by finite differences (about n_params + 1 passes
    per iteration), so these counters climb much faster than the number of iterations.
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
                f"{self.samples:,} samples consumed | RMS now {self.cur_rms:.4g}, "
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
with open(CSV_PATH, "rb") as f:
    total_rows = max(sum(1 for _ in f) - 1, 0)   # minus header
print(f"Reading {CSV_PATH}: {total_rows:,} rows")

chunks, rows_read = [], 0
for chunk in pd.read_csv(CSV_PATH, chunksize=CHUNK_ROWS):
    chunks.append(chunk[Q_COLS + P_COLS])
    rows_read += len(chunk)
    print(f"\r  read {rows_read:,} / {total_rows:,} rows "
          f"({100.0 * rows_read / max(total_rows, 1):.0f}%)", end="", flush=True)
print()
df = pd.concat(chunks, ignore_index=True)

Q = df[Q_COLS].to_numpy(float)
P = df[P_COLS].to_numpy(float)

if ANGLES_IN_DEGREES:
    Q = np.deg2rad(Q)
elif np.max(np.abs(Q)) > 2 * np.pi + 0.1:
    print("Warning: joint values exceed 2*pi. If these are degrees, set ANGLES_IN_DEGREES = True.")

if MAX_SAMPLES and len(Q) > MAX_SAMPLES:
    idx = rng.choice(len(Q), MAX_SAMPLES, replace=False)
    Q, P = Q[idx], P[idx]
    print(f"Subsampled {len(Q):,} of {total_rows:,} rows (MAX_SAMPLES = {MAX_SAMPLES})")

Qc = Q[:, CHAIN]
n = len(CHAIN)
print(f"Fitting {n} joints to {len(Qc)} samples of {P_COLS}")

progress = Progress(N_STARTS, len(Qc))


# ---------- Model ----------
def fk_points(params, Qs):
    """Standard-DH forward kinematics for ALL samples at once. Returns (N, 3) positions.

    params is flat, ordered [a, d, alpha, offset] per joint. Per link:
        A = Rz(q + offset) * Tz(d) * Tx(a) * Rx(alpha)   (same as RevoluteDH)
    The end position is the sum over links of (rotation of the previous links) * (link
    translation), so only a 3x3 rotation and a 3-vector are carried along.
    """
    p = params.reshape(n, 4)
    N = Qs.shape[0]
    R = np.broadcast_to(np.eye(3), (N, 3, 3)).copy()
    t = np.zeros((N, 3))
    for j in range(n):
        a, d, al, off = p[j]
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
    return t


def residuals(params):
    res = fk_points(params, Qc) - P
    progress.end_eval(res)
    return res.ravel()


def check_against_rtb():
    """Cross-check the vectorised FK against roboticstoolbox (if installed) on a few samples."""
    try:
        import roboticstoolbox as rtb
    except ImportError:
        print("roboticstoolbox not installed: skipping FK cross-check")
        return
    rc = np.random.default_rng(123)
    test = np.column_stack([rc.uniform(-0.3, 0.3, n), rc.uniform(-0.3, 0.3, n),
                            rc.choice([0, np.pi / 2, -np.pi / 2, np.pi], n),
                            rc.uniform(-np.pi, np.pi, n)])
    robot = rtb.DHRobot([rtb.RevoluteDH(a=a, d=d, alpha=al, offset=off) for a, d, al, off in test])
    idx = rc.choice(len(Qc), min(5, len(Qc)), replace=False)
    ref = np.array([robot.fkine(Qc[i]).t for i in idx])
    err = np.abs(ref - fk_points(test.ravel(), Qc[idx])).max()
    if err > 1e-9:
        raise RuntimeError(f"Vectorised FK disagrees with roboticstoolbox (max diff {err:.3g}).")
    print(f"FK cross-check against roboticstoolbox passed (max diff {err:.1e})")


check_against_rtb()


# ---------- Bounds and initial guesses ----------
# Angles get +/-2*pi so starting values never sit on a bound.
lower = np.tile([-MAX_LENGTH, -MAX_LENGTH, -2 * np.pi, -2 * np.pi], n)
upper = -lower


def random_start():
    a = rng.uniform(-0.3, 0.3, n) * MAX_LENGTH
    d = rng.uniform(-0.3, 0.3, n) * MAX_LENGTH
    alpha = rng.choice([0, np.pi / 2, -np.pi / 2, np.pi], n)
    offset = rng.uniform(-np.pi, np.pi, n)
    return np.column_stack([a, d, alpha, offset]).ravel()


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
    progress.message(f"  start {s + 1}/{N_STARTS} done: RMS {rms_s:.4g}, nfev {r.nfev}, "
                     f"{fmt_time(time.time() - t_start)}" + ("  <- new best" if new_best else ""))

progress.finish()

if best is None:
    raise RuntimeError("All starts failed. Check the CSV and CHAIN settings.")

# ---------- Report ----------
rms = np.sqrt(np.mean(best.fun ** 2))
spread = np.std(P, axis=0).mean()
print(f"\nBest RMS residual: {rms:.6g}  (mean point spread: {spread:.4g})")
print("Distinct cost levels across starts (lowest first):")
for c in sorted(set(np.round(costs, 8)))[:5]:
    print(f"  {c:.6g}  ({np.sum(np.isclose(costs, c, atol=1e-8))} starts)")

p = best.x.reshape(n, 4)
print(f"\n{'joint':>6} {'a':>10} {'d':>10} {'alpha':>10} {'offset':>10}")
for i, (a, d, al, off) in enumerate(p):
    print(f"{CHAIN[i] + 1:>6} {a:10.4f} {d:10.4f} {al:10.4f} {off:10.4f}")

# ---------- Identifiability check ----------
# Small singular values mean parameter combinations the data cannot resolve,
# e.g. a and d trading off on parallel axes, or offsets on joints that barely move the point.
names = [f"j{j + 1}.{nm}" for j in CHAIN for nm in ("a", "d", "alpha", "offset")]
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