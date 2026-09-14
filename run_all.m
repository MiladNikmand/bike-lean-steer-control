% RUN_ALL
% Non-interactive batch driver for the Bike Lean-Steer Control project.
% For the interactive GUI use:  >> bike_launch
%
% Quick start:
%   >> cd path/to/bike-lean-steer-control
%   >> check_setup
%   >> run_all

% =========================================================================
%  CONFIGURATION
% =========================================================================

% Forward speed (m/s)
%   1.0  -> naturally unstable (tests controller stabilization)
%   4.3  -> self-stable range (tests disturbance rejection)
%   9.0  -> high-speed capsize regime
v_run = 1.0;

% Initial perturbation
phi0_deg_run   = 5;
delta0_deg_run = -2;

% Simulation duration (s)
t_end_run = 30;

% Controllers to run (include any combination):
%   'RateOnly'   'PD'   'PID'   'LQR'   'FullState'   'LQG'   'GainSched'
controllers_run = {'RateOnly', 'PD', 'PID', 'LQR', 'FullState', 'LQG', 'GainSched'};

% Full-state / LQR poles
pp_poles_run = [-4, -5, -6, -7];

% LQR cost weights  Q = diag(lqr_Q),  R = scalar
% Q penalises [phi, delta, phi_dot, delta_dot]
lqr_Q_run = [100, 1, 10, 1];   % penalise roll angle most
lqr_R_run = 0.1;               % penalise steer torque effort

% PD manual gains (0 = auto grid-search)
pd_k1_run = 0;
pd_k2_run = 0;

% PID manual gains (0 = auto grid-search)
pid_k1_run = 0;
pid_k2_run = 0;
pid_k3_run = 0;

% LQG noise covariances
% Qw = process noise  (4x1 diagonal, one per state)
% Rv = measurement noise variance (scalar, on phi_dot measurement)
lqg_Qw_run = [0.1, 0.1, 1.0, 0.1];
lqg_Rv_run = 0.5;

% Physical constraint limits
% phi_limit  — HARD STOP: bike has fallen, run terminates
% delta_limit — ACTUATOR SATURATION: steer is clamped at this angle,
%               run CONTINUES (not terminated). Shows saturation behavior.
phi_limit_run   = 90;   % deg — bike fell
delta_limit_run = 60;   % deg — handlebar mechanical stop (clamp only)

% Actuator slew rate: max steer rate in deg/s
%   Inf  = no limit (ideal actuator, current default)
%   e.g. 200 = 200 deg/s limit (realistic servo)
steer_rate_limit_run = Inf;   % deg/s

% Integration step size
%   0.01 s — default, good for most runs (100 Hz equivalent)
%   0.001 s — use for fast disturbances or LQG with high noise
%   0.005 s — good compromise for slew-rate limited runs
dt_run = 0.01;   % seconds

% ---- Disturbances -------------------------------------------------------

% Impulsive kick
kick_enabled_run  = false;
kick_time_run     = 5.0;    % s
kick_mag_run      = 15;     % deg equivalent

% Gravel / rough road
gravel_enabled_run = false;
gravel_start_run   = 8.0;
gravel_end_run     = 15.0;
gravel_std_run     = 5;

% Road camber (persistent lean bias — motivates integral action)
camber_enabled_run = false;
camber_deg_run     = 3.0;   % road bank angle in degrees

% Wind gust (sustained lateral force — von Karman profile)
wind_enabled_run  = false;
wind_start_run    = 5.0;    % s — gust onset
wind_ramp_run     = 1.0;    % s — ramp-up and ramp-down duration
wind_hold_run     = 4.0;    % s — hold duration at peak
wind_speed_run    = 8.0;    % m/s — peak gust speed

% Sinusoidal road (periodic lateral bumps)
sine_road_enabled_run = false;
sine_road_start_run   = 0.0;    % s
sine_road_freq_run    = 2.0;    % Hz
sine_road_amp_run     = 3.0;    % deg equivalent amplitude

% Speed variation (parametric — changes A matrix each step)
speed_var_enabled_run = false;
speed_var_amp_run     = 1.5;    % m/s amplitude of variation
speed_var_freq_run    = 0.1;    % Hz

% ---- Export options -----------------------------------------------------
make_gifs_run  = true;
gif_fps_run    = 8;
gif_frames_run = 80;
dpi_run        = 150;

% =========================================================================

fprintf(' ========================================================== \n');
fprintf(' ==   Bike Lean-Steer Control  -  Batch Run              == \n');
fprintf(' ==   Whipple-Carvallo Benchmark Model                   == \n');
fprintf(' ========================================================== \n\n');

t_total = tic;

% ---- Build disturbance config -------------------------------------------
disturbance_run = struct( ...
    'kick_enabled',      kick_enabled_run, ...
    'kick_time',         kick_time_run, ...
    'kick_mag_deg',      kick_mag_run, ...
    'kick_duration',     0.3, ...
    'gravel_enabled',    gravel_enabled_run, ...
    'gravel_start',      gravel_start_run, ...
    'gravel_end',        gravel_end_run, ...
    'gravel_std',        gravel_std_run, ...
    'camber_enabled',    camber_enabled_run, ...
    'camber_deg',        camber_deg_run, ...
    'wind_enabled',      wind_enabled_run, ...
    'wind_start',        wind_start_run, ...
    'wind_ramp',         wind_ramp_run, ...
    'wind_hold',         wind_hold_run, ...
    'wind_speed',        wind_speed_run, ...
    'sine_road_enabled', sine_road_enabled_run, ...
    'sine_road_start',   sine_road_start_run, ...
    'sine_road_freq',    sine_road_freq_run, ...
    'sine_road_amp',     sine_road_amp_run, ...
    'speed_var_enabled', speed_var_enabled_run, ...
    'speed_var_amp',     speed_var_amp_run, ...
    'speed_var_freq',    speed_var_freq_run);

% ---- Build main config --------------------------------------------------
p_run = bike_params();

cfg = struct();
cfg.p               = p_run;
cfg.v               = v_run;
cfg.t_end           = t_end_run;
cfg.x0              = [deg2rad(phi0_deg_run); deg2rad(delta0_deg_run); 0; 0];
cfg.controllers     = controllers_run;
cfg.pd_k1           = pd_k1_run;
cfg.pd_k2           = pd_k2_run;
cfg.pid_k1          = pid_k1_run;
cfg.pid_k2          = pid_k2_run;
cfg.pid_k3          = pid_k3_run;
cfg.pp_poles        = pp_poles_run;
cfg.lqr_Q           = lqr_Q_run;
cfg.lqr_R           = lqr_R_run;
cfg.lqg_Qw          = lqg_Qw_run;
cfg.lqg_Rv          = lqg_Rv_run;
cfg.phi_limit_deg           = phi_limit_run;
cfg.delta_limit_deg         = delta_limit_run;
cfg.steer_rate_limit_deg_s  = steer_rate_limit_run;
cfg.dt                      = dt_run;
cfg.disturbance     = disturbance_run;

% ---- Create run folder --------------------------------------------------
existing = dir('Bike_Control_Run_*');
existing = existing([existing.isdir]);
if isempty(existing)
    run_num = 1;
else
    nums = zeros(1,numel(existing));
    for ii = 1:numel(existing)
        tok = regexp(existing(ii).name,'Bike_Control_Run_(\d+)','tokens');
        if ~isempty(tok), nums(ii) = str2double(tok{1}{1}); end
    end
    run_num = max(nums) + 1;
end
run_dir = sprintf('Bike_Control_Run_%03d', run_num);
mkdir(run_dir);
fprintf('Run folder: %s\n\n', run_dir);

% ---- Simulate -----------------------------------------------------------
fprintf('>>> Simulation ...\n');
results = bc_run_analysis(cfg);

% ---- Export -------------------------------------------------------------
fprintf('>>> Exporting figures ...\n');
eopts = struct('make_gifs',make_gifs_run,'gif_fps',gif_fps_run, ...
               'gif_frames',gif_frames_run,'dpi',dpi_run);
bc_export_figures(run_dir, results, eopts);

% ---- Save ---------------------------------------------------------------
save(fullfile(run_dir,'results.mat'), 'results', 'cfg');
bc_write_summary(run_dir, run_num, results, cfg);

% ---- Standalone analysis scripts ----------------------------------------
fprintf('\n>>> Standalone analysis scripts ...\n\n');
stability_analysis;
balance_controller;
param_sensitivity;
response_analysis;

elapsed = toc(t_total);
fprintf('\n ========================================================== \n');
fprintf(' ==  Batch run complete  (%.1f s)\n', elapsed);
fprintf(' ==  Results: %s\n', run_dir);
fprintf(' ==  To animate:  >> animate_bike_v2\n');
fprintf(' ========================================================== \n\n');