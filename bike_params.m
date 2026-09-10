function p = bike_params()
% BIKE_PARAMS  Physical parameters for the Whipple-Carvallo benchmark bicycle.
%
% Returns a struct p with mass, inertia, and geometry values for a "typical"
% bicycle + rider, taken from the benchmark defined in:
%   Meijaard, Papadopoulos, Ruina, Schwab (2007), "Linearized dynamics
%   equations for the balance and steer of a bicycle: a benchmark and
%   review", Proc. R. Soc. A, 463, 1955-1982.
%
% This is the reference case we validate our model against before adding
% any nonlinear tire / slip behavior on top.

% --- Rear frame + rider body (B) ---
p.mB   = 85.0;      % kg
p.IBxx = 9.2;       % kg m^2
p.IBxz = 2.4;       % kg m^2
p.IByy = 11.0;      % kg m^2
p.IBzz = 2.8;       % kg m^2
p.xB   = 0.3;       % m
p.zB   = -0.9;      % m

% --- Handlebar + fork assembly (H) ---
p.mH   = 4.0;       % kg
p.IHxx = 0.05892;   % kg m^2
p.IHxz = -0.00756;  % kg m^2
p.IHyy = 0.06;      % kg m^2
p.IHzz = 0.00708;   % kg m^2
p.xH   = 0.9;       % m
p.zH   = -0.7;      % m

% --- Rear wheel (R) ---
p.mR   = 2.0;       % kg
p.IRxx = 0.0603;    % kg m^2 (radial)
p.IRyy = 0.12;      % kg m^2 (spin)
p.rR   = 0.3;       % m

% --- Front wheel (F) ---
p.mF   = 3.0;       % kg
p.IFxx = 0.1405;    % kg m^2 (radial)
p.IFyy = 0.28;      % kg m^2 (spin)
p.rF   = 0.35;      % m

% --- Geometry ---
p.w      = 1.02;        % wheelbase, m
p.c      = 0.08;        % trail, m
p.lambda = pi/10;       % steer axis tilt from vertical, rad

% --- Environment ---
p.g = 9.81;         % m/s^2

end
