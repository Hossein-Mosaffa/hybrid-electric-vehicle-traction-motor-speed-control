"""
Regenerates the figures and the numbers quoted in REPORT.md.

This is an independent Python re-implementation of the MATLAB scripts and of the
Simulink block diagram. It is only meant to make the report reproducible without
a MATLAB licence. The MATLAB files in the repository remain the source of truth.

Usage:  python make_figures.py        (needs numpy, scipy, matplotlib)
"""
from pathlib import Path
import numpy as np
import scipy.signal as sg
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).parent / "figures"
OUT.mkdir(exist_ok=True)
plt.rcParams.update({"figure.dpi": 130, "axes.grid": True, "grid.alpha": 0.35,
                     "axes.spines.top": False, "axes.spines.right": False})


def save(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / name, dpi=150)
    plt.close(fig)


def closed_loop(num, den):
    """Unity negative feedback around num/den."""
    num = np.atleast_1d(num).astype(float)
    return num, np.polyadd(den, num)


def step_metrics(t, y, final):
    i10, i90 = np.argmax(y >= 0.1 * final), np.argmax(y >= 0.9 * final)
    out = np.where(np.abs(y - final) > 0.02 * abs(final))[0]
    return dict(final=final, peak=y.max(), overshoot_pct=(y.max() - final) / final * 100,
                rise_10_90=t[i90] - t[i10], settling_2pct=t[out[-1] + 1] if len(out) else 0.0)


def bode(num, den, name, title):
    w = np.logspace(-3, 3, 4000)
    _, H = sg.freqs(num, den, w)
    mag, ph = 20 * np.log10(np.abs(H)), np.unwrap(np.angle(H)) * 180 / np.pi
    k = np.where(np.diff(np.sign(mag)))[0][0]
    wc, pm = w[k], 180 + ph[k]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(6.5, 5.5), sharex=True)
    a1.semilogx(w, mag); a1.axhline(0, color="k", lw=0.8)
    a1.set_ylabel("Magnitude (dB)"); a1.set_title(title)
    a2.semilogx(w, ph); a2.axhline(-180, color="k", lw=0.8)
    a2.set_ylabel("Phase (deg)"); a2.set_xlabel("Frequency (rad/s)")
    a1.axvline(wc, color="C3", ls="--"); a2.axvline(wc, color="C3", ls="--")
    a2.annotate(f"crossover {wc:.3g} rad/s\nphase margin {pm:.1f}°", (wc, ph[k]),
                xytext=(0.55, 0.75), textcoords="axes fraction", fontsize=9)
    save(fig, name)
    return wc, pm


# ---------------------------------------------------------------- Part 1: PI loop
G1n, G1d = [48.83, 29.3], np.poly([0, -0.5858])
T1n, T1d = closed_loop(G1n, G1d)
print("Part 1  closed-loop poles:", np.roots(T1d).round(4), " zero:", np.roots(T1n).round(4))
t = np.linspace(0, 0.2, 4001)
_, y = sg.step((4 * T1n, T1d), T=t)
print("Part 1  4 V step:", {k: round(float(v), 4) for k, v in step_metrics(t, y, 4.0).items()})
fig, ax = plt.subplots(figsize=(6.5, 3.8))
ax.plot(t, y, label="c(t)"); ax.axhline(4, color="k", ls=":", label="4 V reference")
ax.set(xlabel="Time (s)", ylabel="Speed sensor output (V)", ylim=(0, 5),
       title="PI-compensated loop, 4 V step")
ax.legend(loc="lower right"); save(fig, "pi_step.png")

Kv = 29.3 / 0.5858
print("Part 1  Kv =", round(Kv, 2), " ramp error =", round(1 / Kv, 4))
t = np.linspace(0, 5, 2001)
_, c = sg.step((T1n, np.polymul(T1d, [1, 0])), T=t)
fig, ax = plt.subplots(figsize=(6.5, 3.8))
ax.plot(t, t, "r", label="unit ramp"); ax.plot(t, c, "b", label="output")
ax.plot(t, t - c, "g", label="error")
ax.set(xlabel="Time (s)", ylabel="Volts", title="PI-compensated loop, unit-ramp input")
ax.legend(); save(fig, "pi_ramp.png")

wc, pm = bode([48.82, 48.82 * 0.6], G1d, "pi_bode.png", "Open loop 48.82 (s+0.6) / [s (s+0.5858)]")
print(f"Part 1  Bode: crossover {wc:.2f} rad/s, phase margin {pm:.1f} deg")

# ------------------------------------------------------- Part 2: root locus study
zn, zd = np.poly([-0.4, -0.6]), np.poly([0, -0.0163, -0.5858])
zeta = -np.log(0.10) / np.sqrt(np.pi ** 2 + np.log(0.10) ** 2)
Ks = np.concatenate([np.linspace(0, 3, 6000), np.linspace(3, 60, 4000)])
branches = np.array([np.sort_complex(np.roots(np.polyadd(zd, K * zn))) for K in Ks])
fig, ax = plt.subplots(figsize=(6.5, 4.6))
for b in range(3):
    ax.plot(branches[:, b].real, branches[:, b].imag, ".", ms=1.5, color="C0")
ax.plot(np.roots(zd).real, np.roots(zd).imag, "kx", ms=8, label="open-loop poles")
ax.plot(np.roots(zn).real, np.roots(zn).imag, "ko", mfc="none", ms=8, label="open-loop zeros")
ang = np.arccos(zeta)
for s in (1, -1):
    ax.plot([0, -np.cos(ang)], [0, s * np.sin(ang)], "r--", lw=1,
            label=f"10 % overshoot line (ζ = {zeta:.3f})" if s == 1 else None)
p78 = np.roots(np.polyadd(zd, 0.78 * zn))
ax.plot(p78.real, p78.imag, "s", color="C3", label="closed-loop poles, K = 0.78")
ax.set(xlim=(-1, 0), ylim=(-0.5, 0.5), xlabel="Real axis", ylabel="Imaginary axis",
       title="Root locus of K (s+0.4)(s+0.6) / [s (s+0.0163)(s+0.5858)]")
ax.legend(fontsize=8, loc="upper left"); save(fig, "rootlocus.png")

Kz = Ks[np.argmin([abs(-r[np.argmax(r.imag)].real / abs(r[np.argmax(r.imag)]) - zeta) for r in
                   [np.roots(np.polyadd(zd, K * zn)) for K in Ks]])]
print(f"Part 2  gain that puts the complex pair on the 10 % line: K = {Kz:.3f}")
p = p78[np.argmax(p78.imag)]
print(f"Part 2  K = 0.78 poles: {p78.round(4)}  damping ratio of the pair = {-p.real / abs(p):.3f}")
Tn, Td = closed_loop(0.78 * zn, zd)
t = np.linspace(0, 20, 8001)
_, y = sg.step((Tn, Td), T=t)
print("Part 2  K = 0.78 step:", {k: round(float(v), 3) for k, v in step_metrics(t, y, 1.0).items()})
fig, ax = plt.subplots(figsize=(6.5, 3.8))
ax.plot(t, y); ax.axhline(1, color="k", ls=":")
ax.set(xlabel="Time (s)", ylabel="Output", ylim=(0, 1.5), title="Unit step, K = 0.78")
save(fig, "rl_step.png")
wc, pm = bode(0.78 * zn, zd, "rl_bode.png", "Open loop with K = 0.78")
print(f"Part 2  Bode: crossover {wc:.3f} rad/s, phase margin {pm:.1f} deg")
print("Part 2  velocity constant:", round(0.78 * 0.24 / (0.0163 * 0.5858), 2))

# -------------------------------------------------------------- Part 3: state feedback
A = np.array([[-20, -40], [0.2491, -0.0191]]); B = np.array([[0], [1000.0]]); C = np.array([[0, 0.05154]])
r = np.array([0.9251 - 1.72j, -0.3963 + 0.8417j])          # as typed in the script
K = np.array([[-0.001, -0.001]]) @ np.linalg.inv(np.column_stack([r, r.conj()]))
Ak = (A - B @ K).real
print("Part 3  K =", K.real.round(5), " closed-loop poles:", np.linalg.eigvals(Ak).round(4),
      " open-loop poles:", np.linalg.eigvals(A).round(4))
c_exact = np.linalg.solve((-1 - 1j) * np.eye(2) - A, np.array([0, 1000])) / 1000
Kx = np.array([[-0.001, -0.001]]) @ np.linalg.inv(np.column_stack([c_exact, c_exact.conj()]))
print("Part 3  with the unrounded eigenvector: K =", Kx.real.round(5),
      " poles:", np.linalg.eigvals(A - B @ Kx.real).round(4))
num, den = sg.ss2tf(Ak, B, C, np.zeros((1, 1)))
print("Part 3  closed-loop tf num:", num[0].round(3), " den:", den.round(4))
t = np.linspace(0, 12, 4001)
_, yc = sg.step(sg.StateSpace(Ak, B, C, 0), T=t)
_, yo = sg.step(sg.StateSpace(A, B, C, 0), T=t)
print("Part 3  step: closed-loop dc gain", round(float(yc[-1]), 1), " peak", round(float(yc.max()), 1),
      " | open-loop dc gain", round(float((C @ np.linalg.solve(-A, B))[0, 0]), 1))
fig, ax = plt.subplots(figsize=(6.5, 3.8))
ax.plot(t, yo, label="open loop"); ax.plot(t, yc, label="with state feedback")
ax.set(xlabel="Time (s)", ylabel="Output", title="Unit step, state-space model")
ax.legend(); save(fig, "state_feedback.png")

# -------------------------------------------------------------- Part 4: Simulink model
def rhs(_, s, r=4.0):
    x, i1, i2 = s                                  # plant integrator, PI1 integrator, PI2 integrator
    e1 = r - 0.0443 * x                            # Sum1: step minus Gain6 * x
    u1 = 100 * e1 + i1                             # PID Controller  (P = 100, I = 40)
    v = (10 * u1 + i2 - 2 * x) / (1 + 0.5 * 10)    # Sum3, solved algebraically (algebraic loop through Gain5)
    e2 = u1 - 0.5 * v                              # Sum: PID output minus Gain5 * v
    y = 0.3 / 4.875 * x                            # Gain3
    return [(1.8 * v - 0.6154 * y - 0.1 * x) / 7.226, 40 * e1, 6 * e2]   # PID Controller1 (P = 10, I = 6)

t = np.linspace(0, 10, 10001)
sol = solve_ivp(rhs, [0, 60], [0, 0, 0], max_step=0.002, rtol=1e-10, atol=1e-12, dense_output=True)
y = 0.3 / 4.875 * sol.sol(t)[0]
yf = 4 / 0.0443 * 0.3 / 4.875
tt = np.linspace(0, 60, 60001); yy = 0.3 / 4.875 * sol.sol(tt)[0]
m = step_metrics(tt, yy, yf)
print("Part 4  Simulink replica:", {k: round(float(v), 3) for k, v in m.items()},
      " peak time", round(float(tt[np.argmax(yy)]), 2), " max slope", round(float(np.gradient(y, t).max()), 2))
fig, (a1, a2) = plt.subplots(2, 1, figsize=(6.5, 5.5), sharex=True)
a1.plot(t, y); a1.axhline(yf, color="k", ls=":"); a1.set_ylabel("Output y(t)")
a1.set_title("Re-implementation of the Simulink model, 4 V step")
a2.plot(t, np.gradient(y, t)); a2.set(ylabel="dy/dt", xlabel="Time (s)")
save(fig, "simulink_replica.png")
print("figures written to", OUT)
