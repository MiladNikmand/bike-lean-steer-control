# Bike Lean-Steer Model — Stage 1 (Baseline)

This is the foundation layer of the bike control project: a physically
validated model of how a real bike leans and steers, with **no tire slip
or nonlinear effects yet**. Everything after this builds on top of it.

## Why start here

A bike doesn't turn like a car — it turns by leaning, and leaning requires
countersteering (briefly steer *away* from the turn to make the bike fall
into the lean). Any control strategy we build later has to sit on top of
a model that gets this right, or the "control" will be steering a bike
that doesn't behave like a bike.

The model here is the **Whipple–Carvallo benchmark bicycle model**, the
standard reference model in bicycle/motorcycle dynamics research
(Meijaard, Papadopoulos, Ruina & Schwab, 2007, *Proc. R. Soc. A*). It's a
linearized model with two degrees of freedom — roll angle `phi` and
steer angle `delta` — driven by gravity, forward speed, and the
gyroscopic effect of the spinning wheels.

## Files

| File | Purpose |
|---|---|
| `bike_params.m` | Physical parameters (masses, inertias, geometry) for a typical bicycle + rider |
| `compute_benchmark_matrices.m` | Builds the canonical `M`, `C1`, `K0`, `K2` matrices from the physical parameters |
| `bike_state_space.m` | Converts those into standard state-space `A`, `B` matrices at a given forward speed |
| `simulate_uncontrolled.m` | Simulates the bike with **no rider input** at three speeds, to check self-stability |
| `eigs_vs_speed.m` | Sweeps speed 0–10 m/s and plots eigenvalues, to map out the self-stable speed range |
| `simulate_controlled.m` | Adds a simple rider balance controller (steer torque proportional to roll rate) and stabilizes an otherwise-unstable low speed |

## Validation — this actually matches a real bike

I ran the model (in Octave, MATLAB-compatible) before handing it to you, and it reproduces the textbook bicycle stability behavior exactly:

- **v = 1 m/s**: unstable (falls over) — eigenvalues `3.53 ± 0.81i` (growing oscillation)
- **v = 4.3 m/s**: self-stable — all eigenvalues have non-positive real part; the mildly oscillating "weave" mode decays on its own
- **v = 9 m/s**: unstable again ("capsize" mode) — one real eigenvalue creeps positive (`+0.158`)

This is the real, well-known signature of bicycle dynamics: too slow, you fall; in a mid-speed range, the bike balances itself with zero rider input; too fast, a different (slower, non-oscillatory) instability reappears. Getting this for free out of the equations, with no tuning, is the confirmation that the model is physically sound.

The mass matrix computed here (`M = [80.82, 2.32; 2.32, 0.298]`) also matches the published benchmark values exactly, so the matrix-building code is verified against the paper, not just self-consistent.

## The rider controller

`simulate_controlled.m` stabilizes the naturally-unstable v = 1 m/s case using:

```
T_delta = k * phidot        (k ≈ 42+ N·m·s/rad, chosen from a manual root-locus sweep)
```

This says: steer in the direction the bike is currently falling. That's the
low-speed balancing reflex every cyclist has (different from high-speed
countersteering, where you steer briefly *away* from the turn to roll the
bike into it). I verified the stabilizing sign numerically rather than
assuming it — worth noting since it's easy to get backwards.

## How to run

Open in MATLAB and run any of the three `simulate_*` / `eigs_*` scripts directly — no setup needed, all parameters are self-contained in `bike_params.m`.

## What's next (not yet built)

This stage is **linear** and assumes **no tire slip** — it tells us the bike leans and steers correctly, but it can't yet skid, wheelie, or burn out. The next stages:

1. **Nonlinear tire model** — replace the no-slip assumption with a combined slip-angle + camber-thrust + longitudinal-slip tire model (motorcycle-specific, since camber thrust doesn't exist for car tires). This is where skidding and low-mu sliding come from.
2. **Longitudinal dynamics** — throttle/brake torque → wheel slip ratio → traction force, so we get real wheelspin/burnout behavior.
3. **Path-tracking controller** — outer loop converts desired path curvature into a desired lean angle; inner loop (building on the balance controller above) outputs steering torque to hit that lean angle, i.e. automated countersteering.
4. **Test maneuvers** — chicanes, sharp turns, and low-mu surfaces to show off slip/highside/lowside behavior.

Let me know if you want me to move on to the nonlinear tire model next, or if you'd rather look over this baseline first (e.g. play with different speeds/parameters in `bike_params.m` to build intuition).
