import numpy as np
import pandas as pd
import roboticstoolbox as rtb
from scipy.optimize import least_squares

# ---------- Configuration ----------
CSV_PATH = "armeo_joints_ee.csv"
Q_COLS = [f"q{i}" for i in range(1, 8)]   # joint angle columns, joint 1 = shoulder
P_COLS = ["x", "y", "z"]                  # target point (e.g. EE, EL, WR)
CHAIN = list(range(7))                    # 0-based indices of joints on the serial path
                                          # from the base to the target point
ANGLES_IN_DEGREES = False
MAX_LENGTH = 1.0      # bound on |a| and |d|, in the units of x,y,z (use 1000 if mm)
N_STARTS = 20         # random restarts (problem is nonconvex)
MAX_SAMPLES = 1000    # subsample for speed; set to None to use all rows
SEED = 0

rng = np.random.default_rng(SEED)

# ---------- Load data ----------
df = pd.read_csv(CSV_PATH)
Q = df[Q_COLS].to_numpy(float)
P = df[P_COLS].to_numpy(float)

if ANGLES_IN_DEGREES:
    Q = np.deg2rad(Q)
elif np.max(np.abs(Q)) > 2 * np.pi + 0.1:
    print("Warning: joint values exceed 2*pi. If these are degrees, set ANGLES_IN_DEGREES = True.")

if MAX_SAMPLES and len(Q) > MAX_SAMPLES:
    idx = rng.choice(len(Q), MAX_SAMPLES, replace=False)
    Q, P = Q[idx], P[idx]

Qc = Q[:, CHAIN]
n = len(CHAIN)
print(f"Fitting {n} joints to {len(Qc)} samples of {P_COLS}")

# ---------- Model ----------
def make_robot(params):
    """params is flat, ordered [a, d, alpha, offset] per joint."""
    p = params.reshape(n, 4)
    links = [rtb.RevoluteDH(a=a, d=d, alpha=al, offset=off) for a, d, al, off in p]
    return rtb.DHRobot(links, name="fit")

def residuals(params):
    robot = make_robot(params)
    res = np.empty((len(Qc), 3))
    for k, q in enumerate(Qc):
        res[k] = robot.fkine(q).t - P[k]
    return res.ravel()

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
    x0 = random_start()
    try:
        r = least_squares(residuals, x0, bounds=(lower, upper), method="trf", max_nfev=200)
    except Exception as e:
        print(f"  start {s}: failed ({e})")
        continue
    costs.append(r.cost)
    if best is None or r.cost < best.cost:
        best = r

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