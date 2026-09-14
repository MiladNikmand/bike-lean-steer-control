function [d, v_eff] = bc_disturbance(t, cfg, p, v_nominal)
% BC_DISTURBANCE  Generate a 4x1 disturbance vector in state-space form,
% plus the effective forward speed at time t (for speed-variation runs).
%
% State:  x = [phi; delta; phi_dot; delta_dot]
% Torques enter through the Minv block (rows 3-4 of the state derivative).
%
% Disturbance types:
%   kick        — Gaussian impulse on roll channel (lateral impact)
%   gravel      — band-limited noise on roll + steer (rough road)
%   camber      — constant gravity bias from banked road surface
%   wind_gust   — von Karman lateral gust profile (sustained lateral force)
%   speed_var   — sinusoidal speed variation around nominal (parametric)
%   sine_road   — periodic lateral bump at tunable frequency
%
% Inputs:
%   t           scalar  current time (s)
%   cfg         disturbance config struct (see bike_launch / run_all)
%   p           bike_params struct
%   v_nominal   nominal forward speed (m/s), used for speed_var baseline
%
% Outputs:
%   d           4x1 state-derivative disturbance
%   v_eff       effective forward speed at time t (= v_nominal unless
%               speed_var is enabled, in which case it varies)

if nargin < 4, v_nominal = 1.0; end

mB      = p.mB;
g       = p.g;

% Approximate Minv diagonal for torque-to-acceleration mapping
Mpp_approx = mB * 0.95;
Mdd_approx = 0.30;

% Physical torques on [phi; delta] channels
tau = [0; 0];

% ---- Impulsive kick (lateral impact) ------------------------------------
if cfg.kick_enabled
    sigma  = max(cfg.kick_duration, 0.05);
    amp    = deg2rad(cfg.kick_mag_deg) * mB * g * 0.9;
    tau(1) = tau(1) + amp * exp(-0.5*((t - cfg.kick_time)/sigma)^2);
end

% ---- Gravel / rough road (band-limited noise) ---------------------------
if cfg.gravel_enabled && t >= cfg.gravel_start && t <= cfg.gravel_end
    std_tau = deg2rad(cfg.gravel_std) * mB * g * 0.3;
    tau(1)  = tau(1) + std_tau * randn();
    tau(2)  = tau(2) + std_tau * 0.25 * randn();
end

% ---- Road camber (persistent gravity bias) ------------------------------
% A banked road at angle alpha tilts effective gravity, adding a constant
% roll-axis moment: T_phi_camber = mT * g * zT * sin(alpha) ~ mT*g*zT*alpha
% We approximate zT (total CoM height) from bike_params.
if cfg.camber_enabled
    mT  = p.mR + p.mB + p.mH + p.mF;
    zT  = (-p.rR*p.mR + p.zB*p.mB + p.zH*p.mH - p.rF*p.mF) / mT;
    alpha_rad = deg2rad(cfg.camber_deg);
    tau(1) = tau(1) + mT * g * abs(zT) * sin(alpha_rad);
end

% ---- Wind gust (von Karman profile: ramp up, hold, ramp down) ----------
% Acts as a lateral force at CoM height, producing a roll torque.
% Force: F = 0.5 * rho_air * Cd * A_rider * v_wind^2
% Torque arm: approximate CoM height above ground ~ |zB| + rR
if cfg.wind_enabled
    rho_air   = 1.225;           % kg/m^3
    Cd_rider  = 1.0;             % drag coefficient (upright rider)
    A_rider   = 0.5;             % frontal area m^2
    z_com     = abs(p.zB) + p.rR;  % CoM height above ground (approx)
    t1 = cfg.wind_start;
    t2 = cfg.wind_start + cfg.wind_ramp;
    t3 = cfg.wind_start + cfg.wind_ramp + cfg.wind_hold;
    t4 = t3 + cfg.wind_ramp;

    if t >= t1 && t <= t4
        if     t <= t2, frac = (t-t1) / max(cfg.wind_ramp, 0.01);
        elseif t <= t3, frac = 1.0;
        else,           frac = 1.0 - (t-t3) / max(cfg.wind_ramp, 0.01);
        end
        v_wind   = cfg.wind_speed * frac;
        F_wind   = 0.5 * rho_air * Cd_rider * A_rider * v_wind^2;
        tau(1)   = tau(1) + F_wind * z_com;
    end
end

% ---- Sinusoidal road (periodic lateral bump) ----------------------------
% Models corrugated road surface — produces a periodic steer disturbance
if cfg.sine_road_enabled && t >= cfg.sine_road_start
    amp_tau  = deg2rad(cfg.sine_road_amp) * mB * g * 0.15;
    tau(2)   = tau(2) + amp_tau * sin(2*pi*cfg.sine_road_freq * t);
end

% ---- Speed variation (parametric — handled by caller) ------------------
% Returns an effective speed; caller must rebuild A matrix each step.
if cfg.speed_var_enabled
    % Sinusoidal variation: v(t) = v_nom + amp * sin(2*pi*freq*t)
    v_eff = v_nominal + cfg.speed_var_amp * ...
            sin(2*pi*cfg.speed_var_freq * t);
    v_eff = max(v_eff, 0.1);   % never go below 0.1 m/s
else
    v_eff = v_nominal;
end

% ---- Map torques to state-derivative space (rows 3-4) ------------------
d = [0;
     0;
     tau(1) / Mpp_approx;
     tau(2) / Mdd_approx];

end