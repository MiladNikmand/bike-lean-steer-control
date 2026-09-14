% BIKE_LAUNCH
% Interactive GUI entry point for the Bike Lean-Steer Control project.
%
% Five independently scrollable tabs:
%   Tab 1 — Bike Parameters
%   Tab 2 — Simulation Settings  (IC, dt, constraints, actuator limits)
%   Tab 3 — Controllers          (checkboxes + tuning options)
%   Tab 4 — Disturbances         (checkboxes + per-disturbance parameters)
%   Tab 5 — Export & Run         (GIF/PNG options, file layout info)
%
% Scrolling: each tab contains a uipanel with Scrollable='on' (R2020b+).
% The inner content panel is sized to its actual content so the scroll
% range is always correct.
%
% Usage:  >> bike_launch

function bike_launch()

p_nom = bike_params();

% =========================================================================
%  CONSTANTS
% =========================================================================
BG        = [0.11 0.12 0.15];   % figure background
TAB_BG    = [0.13 0.15 0.18];   % tab / panel background
HEADER_H  = 66;
FOOTER_H  = 54;
FIG_W     = 800;
FIG_H     = 700;
INNER_W   = FIG_W - 24;         % usable width inside each scroll panel

% =========================================================================
%  ROOT FIGURE
% =========================================================================
fig = uifigure( ...
    'Name',     'Bike Lean-Steer Control', ...
    'Position', [80 60 FIG_W FIG_H], ...
    'Color',    BG, ...
    'Resize',   'off');

% ---- Sticky header -------------------------------------------------------
hdr = uipanel(fig, ...
    'Position',        [0 FIG_H-HEADER_H FIG_W HEADER_H], ...
    'BackgroundColor', BG, 'BorderType','none');
uilabel(hdr,'Text','Bike Lean-Steer Control', ...
    'Position',[18 34 520 26],'FontSize',19,'FontWeight','bold', ...
    'FontColor',[0.92 0.92 0.92]);
uilabel(hdr,'Text', ...
    'Whipple-Carvallo Benchmark  |  R2020b+  |  No toolboxes required', ...
    'Position',[20 12 600 18],'FontSize',11,'FontColor',[0.46 0.60 0.74]);
uipanel(hdr,'Position',[0 0 FIG_W 2],'BackgroundColor',[0.25 0.32 0.40],...
    'BorderType','none');

% ---- Sticky footer -------------------------------------------------------
ftr = uipanel(fig, ...
    'Position',        [0 0 FIG_W FOOTER_H], ...
    'BackgroundColor', BG, 'BorderType','none');
uipanel(ftr,'Position',[0 FOOTER_H-2 FIG_W 2], ...
    'BackgroundColor',[0.25 0.32 0.40],'BorderType','none');

status_lbl = uilabel(ftr, ...
    'Text',      'Ready.  Configure across tabs, then press Run.', ...
    'Position',  [12 4 520 20], ...
    'FontSize',  10.5,'FontWeight','bold','FontColor',[0.46 0.80 0.54]);

uibutton(ftr,'Text','Check Setup','Position',[12 28 118 22], ...
    'BackgroundColor',[0.17 0.26 0.37],'FontColor',[0.80 0.86 0.92], ...
    'FontSize',10,'ButtonPushedFcn',@(~,~) check_setup());
uibutton(ftr,'Text','Open Animator','Position',[136 28 128 22], ...
    'BackgroundColor',[0.16 0.20 0.40],'FontColor',[0.80 0.86 0.96], ...
    'FontSize',10,'ButtonPushedFcn',@(~,~) animate_bike_v2());

btn_run = uibutton(ftr,'Text','RUN ANALYSIS', ...
    'Position',[FIG_W-158 6 150 44], ...
    'BackgroundColor',[0.09 0.46 0.26],'FontColor',[0.96 0.98 0.96], ...
    'FontSize',14,'FontWeight','bold','ButtonPushedFcn',@(~,~) do_run());

% ---- Tab group (fills between header and footer) ------------------------
TAB_Y  = FOOTER_H;
TAB_H  = FIG_H - HEADER_H - FOOTER_H;

tg = uitabgroup(fig,'Position',[0 TAB_Y FIG_W TAB_H],'TabLocation','top');

% =========================================================================
%  HELPER: make a scrollable tab
%  Returns the inner content panel to add children to, and a function
%  handle finalise(h) that must be called after all children are added,
%  with h = total content height used.
% =========================================================================
    function [content, finalise] = scrollable_tab(tab)
        % Outer panel fills the tab and is scrollable
        outer = uipanel(tab, ...
            'Position',        [0 0 tab.Position(3) tab.Position(4)], ...
            'Scrollable',      'on', ...
            'BackgroundColor', TAB_BG, ...
            'BorderType',      'none');
        % Inner panel: same width, height set later by finalise()
        content = uipanel(outer, ...
            'Position',        [0 0 INNER_W 2000], ...
            'BackgroundColor', TAB_BG, ...
            'BorderType',      'none');
        finalise = @(h) set(content,'Position',[0 0 INNER_W max(h+30, outer.Position(4))]);
    end

% =========================================================================
%  TAB 1 — BIKE PARAMETERS
% =========================================================================
tab1 = uitab(tg,'Title','  1  Bike Parameters  ','BackgroundColor',TAB_BG);
[sc1, fin1] = scrollable_tab(tab1);
y = 10;

[y, param_ef] = blk_fields(sc1, INNER_W, 'Physical Parameters', {
    'v',      'Forward speed  v  (m/s)',         '4.3', ...
    'Speed at which the bicycle is analysed. Below ~4.3 m/s: naturally unstable. 4.3-6 m/s: self-stable. Above 6 m/s: capsize mode.'
    'mB',     'Rider + frame mass  mB  (kg)',     num2str(p_nom.mB), ...
    'Combined mass of rear frame + rider. Increasing mB lowers the self-stable speed band edges.'
    'w',      'Wheelbase  w  (m)',                num2str(p_nom.w), ...
    'Distance between wheel contact points. Longer wheelbase generally widens the stable band.'
    'c',      'Trail  c  (m)',                    num2str(p_nom.c), ...
    'Horizontal offset from steer axis to front contact point. Trail is the dominant passive-stability parameter — trail = 0 makes the bike fall; large trail makes it sluggish to steer.'
    'lambda', 'Steer axis tilt  lambda  (deg)',   num2str(rad2deg(p_nom.lambda),'%.2f'), ...
    'Head angle from vertical. Larger tilt (shallower head angle) increases the caster / gyroscopic effect.'
    'rR',     'Rear wheel radius  rR  (m)',        num2str(p_nom.rR), ...
    'Rear wheel radius. Affects gyroscopic spin-up torque and CoM height above ground.'
    'rF',     'Front wheel radius  rF  (m)',       num2str(p_nom.rF), ...
    'Front wheel radius. Affects gyroscopic precession which contributes to self-stability at speed.'
}, y);

fin1(y);

% =========================================================================
%  TAB 2 — SIMULATION SETTINGS
% =========================================================================
tab2 = uitab(tg,'Title','  2  Simulation  ','BackgroundColor',TAB_BG);
[sc2, fin2] = scrollable_tab(tab2);
y = 10;

[y, ic_ef] = blk_fields(sc2, INNER_W, 'Initial Conditions', {
    'phi0',   'Initial roll angle  phi_0  (deg)',   '5', ...
    'Starting lean angle from upright. Positive = lean right. 5 deg is a realistic "just tipped" perturbation.'
    'delta0', 'Initial steer angle  delta_0  (deg)', '-2', ...
    'Starting handlebar angle. -2 deg with +5 deg roll gives a realistic combined perturbation.'
}, y);

[y, sim_ef] = blk_fields(sc2, INNER_W, 'Time & Integration', {
    't_end',  'Simulation duration  (s)',   '30', ...
    '30 s shows stabilisation clearly at v = 1 m/s. Use 60+ s for slow disturbances like road camber.'
    'dt',     'Integration step  dt  (s)',  '0.01', ...
    'Fixed-step Euler. 0.01 s = 100 Hz equivalent, suitable for most runs. Use 0.005 or smaller for LQG with high sensor noise or tight slew-rate limits where step accuracy matters.'
}, y);

[y, act_ef] = blk_fields(sc2, INNER_W, 'Physical Constraints & Actuator Limits', {
    'phi_lim',   'Roll hard stop  |phi|  (deg)',            '90', ...
    'HARD STOP — bike has fallen over. Simulation terminates and the run is marked FAILED. Physics requires this; do not relax unless deliberately studying post-fall behaviour.'
    'delta_lim', 'Steer saturation  |delta|  (deg)',        '60', ...
    'Actuator mechanical stop. Steer is CLAMPED here — the run continues. Saturation is logged and highlighted orange on the delta trace. This is a realistic actuator limit, not a failure criterion.'
    'slew',      'Steer slew rate  (deg/s,  Inf = no limit)', 'Inf', ...
    'Max steer rate (deg/s). Inf = ideal instantaneous actuator. Try 200 deg/s for a realistic servo. Tighter slew = controller sees delayed response = harder stabilisation problem, especially for LQR and PID.'
}, y);

fin2(y);

% =========================================================================
%  TAB 3 — CONTROLLERS
% =========================================================================
tab3 = uitab(tg,'Title','  3  Controllers  ','BackgroundColor',TAB_BG);
[sc3, fin3] = scrollable_tab(tab3);
y = 10;

ctrl_defs = {
    'RateOnly',  'Rate-Only  —  T = k  ·  phi_dot', 1, ...
    'Steer torque proportional to roll rate only. Simplest balance controller. Gain k is found automatically (root-locus sweep until max Re(eigenvalue) < -0.5). Cannot eliminate steady-state lean under a camber disturbance.'
    'PD',        'PD  —  T = k1 · phi  +  k2 · phi_dot', 1, ...
    'Adds roll-angle correction to rate feedback. k1 corrects lean; k2 adds damping. Both gains found by grid search. Faster settling than Rate-Only but still cannot reject a persistent bias (road camber).'
    'PID',       'PID  —  T = k1 · phi  +  k2 · phi_dot  +  k3 · integral(phi)', 1, ...
    'Adds integral action to PD. The integral term eliminates steady-state roll error under any constant disturbance (road camber, constant wind). Watch for integrator windup when the actuator saturates.'
    'LQR',       'LQR  —  optimal full-state  (minimises  J = integral( x''Qx + Ru² ) dt)', 1, ...
    'Linear Quadratic Regulator. Finds the globally optimal gain matrix for the Q and R weights you specify. Q trades off roll vs steer vs rate errors; R penalises steer torque effort. No Control Toolbox required (DARE solved analytically).'
    'FullState', 'Full-State  —  Ackermann pole placement', 1, ...
    'Places all four closed-loop eigenvalues at the desired poles. Deterministic and easy to reason about, but requires manual pole selection. Compare with LQR: optimal Q/R tuning usually outperforms arbitrary placement.'
    'LQG',       'LQG  —  LQR gains  +  Kalman observer  (measures phi_dot only)', 1, ...
    'Most realistic controller. Only roll rate (phi_dot) is measured — as would come from a gyroscope/IMU on a real bike. A Kalman filter estimates the full state from this noisy measurement. Observer poles placed at 2× controller poles for fast convergence.'
    'GainSched', 'Gain-Scheduled  —  speed-zone rate gains  (3 zones)', 0, ...
    'Pre-computes the minimum stabilising Rate-Only gain at three representative speeds (1.5, 4.3, 7.5 m/s) and interpolates between them each step. Best combined with the Speed Variation disturbance to show why scheduling outperforms a fixed gain.'
};
[y, ctrl_cb] = blk_checks(sc3, INNER_W, 'Select Controllers', ctrl_defs, y);

[y, lqr_ef] = blk_fields(sc3, INNER_W, 'LQR / LQG Tuning  (used by LQR, FullState, LQG)', {
    'lqr_Q',  'LQR  Q weights  [phi  delta  phi_dot  delta_dot]', '100 1 10 1', ...
    'Four space-separated values penalising each state error. phi weight dominates: 100 means roll error costs 100× more than steer error. Increase to tighten lean; decrease to allow more roll and save torque.'
    'lqr_R',  'LQR  R  (steer torque penalty)', '0.1', ...
    'Single positive number penalising control effort. Larger R = softer, energy-saving control; smaller R = aggressive. Ratio Q(1)/R determines the aggressiveness — start at 0.1, halve/double to tune feel.'
    'poles',  'Desired closed-loop poles  (Full-State only)', '-4 -5 -6 -7', ...
    'Four negative real numbers. More negative = faster, but demands higher torques. Suggested range: -2 to -10 each. The ratio between fastest and slowest pole affects inter-mode coupling.'
    'lqg_Qw', 'LQG  process noise covariance  Qw  [4 values]', '0.1 0.1 1.0 0.1', ...
    'Kalman filter: how much you trust the model vs the measurement. Larger = trust sensor more, faster observer response. The 3rd entry (phi_dot) is the measured state — keep it at 1.0 or above.'
    'lqg_Rv', 'LQG  measurement noise variance  Rv  (scalar)', '0.5', ...
    'Variance of the phi_dot sensor noise. Larger Rv = noisier sensor assumed = more filtering = slower observer. Smaller Rv = trusts sensor directly = faster but sensitive to spikes.'
}, y);

fin3(y);

% =========================================================================
%  TAB 4 — DISTURBANCES
% =========================================================================
tab4 = uitab(tg,'Title','  4  Disturbances  ','BackgroundColor',TAB_BG);
[sc4, fin4] = scrollable_tab(tab4);
y = 10;

dist_defs = {
    'kick',      'Impulsive kick  (sudden lateral impact)', 0, ...
    'A Gaussian-shaped roll torque pulse — like being shoved sideways while riding. Tests impulse recovery speed. Set kick time after initial transients have settled (e.g. t > 3 s).'
    'gravel',    'Gravel / rough road  (band-limited noise)', 0, ...
    'Band-limited white noise on roll and steer channels. Simulates continuous surface roughness. Useful for comparing disturbance-rejection bandwidth: LQR and LQG will outperform PD here.'
    'camber',    'Road camber  (persistent gravity bias)', 0, ...
    'Banked road creates a constant gravitational tipping moment. PD and Rate-Only settle at a nonzero lean angle; PID, LQR, LQG null it out. The clearest demonstration of why integral action matters.'
    'wind',      'Wind gust  (von Karman lateral profile)', 0, ...
    'Sustained lateral wind force computed from aerodynamic drag (ramp-up → hold → ramp-down). Tests sustained disturbance rejection. Models real gust timing with configurable onset, hold, and decay.'
    'sine_road', 'Sinusoidal road  (periodic lateral bumps)', 0, ...
    'Periodic steer disturbance at a set frequency. Probes closed-loop bandwidth: controllers with wider bandwidth attenuate higher bump frequencies. Compare across controllers at the same frequency.'
    'speed_var', 'Speed variation  (parametric — rebuilds A each step)', 0, ...
    'Sinusoidal speed variation around nominal. The linearisation point shifts each step so A changes continuously. Best paired with the Gain-Scheduled controller to illustrate why fixed gains fail outside their design speed.'
};
[y, dist_cb] = blk_checks(sc4, INNER_W, 'Select Disturbances', dist_defs, y);

[y, dist_ef] = blk_fields(sc4, INNER_W, 'Kick Parameters', {
    'kick_time', 'Kick onset time  (s)',          '5',  'When the impulse occurs.'
    'kick_mag',  'Kick magnitude  (deg equiv.)',  '15', '15 deg = strong shove; 5 deg = gentle nudge.'
}, y);

[y, dist_ef2] = blk_fields(sc4, INNER_W, 'Gravel Parameters', {
    'grav_start', 'Start time  (s)',   '8',  'When the rough road section begins.'
    'grav_end',   'End time  (s)',     '15', 'When it ends.'
    'grav_std',   'Intensity  (deg equiv. std)', '5', '5 = moderate gravel; 15 = severe offroad.'
}, y);

[y, dist_ef3] = blk_fields(sc4, INNER_W, 'Road Camber Parameters', {
    'camber_deg', 'Road bank angle  (deg)', '3', '3 deg = typical highway camber; 10 deg = banked corner.'
}, y);

[y, dist_ef4] = blk_fields(sc4, INNER_W, 'Wind Gust Parameters', {
    'wind_start', 'Onset time  (s)',   '5', 'When the gust begins.'
    'wind_ramp',  'Ramp duration  (s)', '1', 'Time to reach full speed. Shorter = more impulsive.'
    'wind_hold',  'Hold duration  (s)', '4', 'Duration at peak strength.'
    'wind_speed', 'Peak speed  (m/s)', '8', '8 m/s = moderate gust; 15 m/s = near-storm.'
}, y);

[y, dist_ef5] = blk_fields(sc4, INNER_W, 'Sinusoidal Road Parameters', {
    'sine_freq', 'Frequency  (Hz)',     '2', '2 Hz = corrugated road; 0.5 Hz = slow weave.'
    'sine_amp',  'Amplitude  (deg equiv.)', '3', 'Steer disturbance magnitude.'
}, y);

[y, dist_ef6] = blk_fields(sc4, INNER_W, 'Speed Variation Parameters', {
    'sv_amp',  'Speed amplitude  (m/s)',  '1.5', 'v(t) = v_nom + amp * sin(2π * freq * t).'
    'sv_freq', 'Frequency  (Hz)',         '0.1', '0.1 Hz = slow acceleration/braking cycle.'
}, y);

fin4(y);

% =========================================================================
%  TAB 5 — EXPORT & RUN
% =========================================================================
tab5 = uitab(tg,'Title','  5  Export & Run  ','BackgroundColor',TAB_BG);
[sc5, fin5] = scrollable_tab(tab5);
y = 10;

[y, exp_ef] = blk_fields(sc5, INNER_W, 'Figure Export Settings', {
    'dpi',        'PNG resolution  (DPI)',         '150', ...
    '150 DPI is adequate for screen review and most presentations. Use 300 for print/publication quality.'
    'gif_fps',    'GIF frame rate  (fps)',          '10',  ...
    '10 fps gives smooth animation without large file sizes. Lower (6-8) for smaller files; higher (15) for silky playback.'
    'gif_frames', 'Max GIF frames  (per scenario)', '120', ...
    '120 frames at 10 fps = 12 s of animation. The simulation is downsampled to this count. Reduce to 60-80 for faster export and smaller files.'
}, y);

exp_cb_defs = {
    'make_gifs', 'Export response GIFs  (4-panel phi / delta / rates / torque)', 1, ...
    'Saves an animated version of each controller''s response plot to run_dir/figures/. One GIF per scenario.'
    'make_anim', 'Export 3D animation GIFs  (generated by animate_bike_v2)', 1, ...
    'When animate_bike_v2 is run, it asks whether to save GIFs. This sets the default answer to Yes.'
};
[y, exp_cb] = blk_checks(sc5, INNER_W, 'Animation Export', exp_cb_defs, y);

% Output layout info box
y = y + 8;
info_h = 220;
info_panel = uipanel(sc5, ...
    'Position',        [12 y INNER_W-24 info_h], ...
    'BackgroundColor', [0.14 0.17 0.22], ...
    'BorderType',      'none', ...
    'Title',           'Output folder layout  (created automatically on Run)', ...
    'ForegroundColor', [0.62 0.74 0.88], ...
    'FontSize',        11, 'FontWeight','bold');

info_str = sprintf([...
    'Bike_Control_Run_NNN/\n'...
    '  figures/\n'...
    '    00_parameters.png           parameter table + eigenvalue plot\n'...
    '    <Ctrl>_response.png/.gif    4-panel: phi, delta, rates, torque\n'...
    '    combined_comparison.png     all controllers overlaid\n'...
    '  animation/\n'...
    '    <Ctrl>_animation.gif        3D stick-figure side-by-side GIF\n'...
    '  results.mat                   full simulation data\n'...
    '  run_summary.json              machine-readable summary\n'...
    '  README.txt                    human-readable summary']);

uilabel(info_panel, ...
    'Text',             info_str, ...
    'Position',         [10 5 INNER_W-50 info_h-34], ...
    'FontSize',         9.5, ...
    'FontName',         'Courier New', ...
    'FontColor',        [0.68 0.78 0.88], ...
    'VerticalAlignment','top', ...
    'WordWrap',         'off');

y = y + info_h + 16;
fin5(y);

% =========================================================================
%  RUN CALLBACK  (reads all five tabs)
% =========================================================================
    function do_run()
        set_status('Collecting parameters...','[0.85 0.80 0.40]');
        try
            % Tab 1 --------------------------------------------------------
            p        = p_nom;
            v_val    = pn(param_ef('v'));
            p.mB     = pn(param_ef('mB'));
            p.w      = pn(param_ef('w'));
            p.c      = pn(param_ef('c'));
            p.lambda = deg2rad(pn(param_ef('lambda')));
            p.rR     = pn(param_ef('rR'));
            p.rF     = pn(param_ef('rF'));

            % Tab 2 --------------------------------------------------------
            phi0   = pn(ic_ef('phi0'));
            delta0 = pn(ic_ef('delta0'));
            t_end  = pn(sim_ef('t_end'));
            dt_val = pn(sim_ef('dt'));
            pl     = pn(act_ef('phi_lim'));
            dl     = pn(act_ef('delta_lim'));
            slew_s = strtrim(act_ef('slew').Value);
            if strcmpi(slew_s,'inf') || strcmpi(slew_s,'infinity')
                slew_val = Inf;
            else
                slew_val = str2double(slew_s);
                if isnan(slew_val)
                    error('Slew rate: enter a number or Inf');
                end
            end

            % Tab 3 --------------------------------------------------------
            ctrl_list = {};
            for ci = 1:size(ctrl_defs,1)
                if ctrl_cb(ci).Value
                    ctrl_list{end+1} = ctrl_defs{ci,1}; %#ok<AGROW>
                end
            end
            if isempty(ctrl_list)
                set_status('Select at least one controller on Tab 3.', ...
                           '[0.90 0.30 0.30]');
                return;
            end
            lqr_Q  = str2num(lqr_ef('lqr_Q').Value);  %#ok<ST2NM>
            lqr_R  = pn(lqr_ef('lqr_R'));
            poles  = str2num(lqr_ef('poles').Value);   %#ok<ST2NM>
            lqg_Qw = str2num(lqr_ef('lqg_Qw').Value); %#ok<ST2NM>
            lqg_Rv = pn(lqr_ef('lqg_Rv'));
            if numel(lqr_Q)~=4
                error('LQR Q: need 4 space-separated values');
            end
            if numel(poles)~=4 || any(poles>=0)
                error('Poles: need 4 negative values e.g.  -4 -5 -6 -7');
            end
            if numel(lqg_Qw)~=4
                error('LQG Qw: need 4 space-separated values');
            end

            % Tab 4 --------------------------------------------------------
            d = zeros(1,6,'logical');
            for di = 1:6, d(di) = dist_cb(di).Value; end

            dcfg = struct( ...
                'kick_enabled',      d(1), ...
                'kick_time',         pn(dist_ef('kick_time')), ...
                'kick_mag_deg',      pn(dist_ef('kick_mag')), ...
                'kick_duration',     0.3, ...
                'gravel_enabled',    d(2), ...
                'gravel_start',      pn(dist_ef2('grav_start')), ...
                'gravel_end',        pn(dist_ef2('grav_end')), ...
                'gravel_std',        pn(dist_ef2('grav_std')), ...
                'camber_enabled',    d(3), ...
                'camber_deg',        pn(dist_ef3('camber_deg')), ...
                'wind_enabled',      d(4), ...
                'wind_start',        pn(dist_ef4('wind_start')), ...
                'wind_ramp',         pn(dist_ef4('wind_ramp')), ...
                'wind_hold',         pn(dist_ef4('wind_hold')), ...
                'wind_speed',        pn(dist_ef4('wind_speed')), ...
                'sine_road_enabled', d(5), ...
                'sine_road_start',   0, ...
                'sine_road_freq',    pn(dist_ef5('sine_freq')), ...
                'sine_road_amp',     pn(dist_ef5('sine_amp')), ...
                'speed_var_enabled', d(6), ...
                'speed_var_amp',     pn(dist_ef6('sv_amp')), ...
                'speed_var_freq',    pn(dist_ef6('sv_freq')));

            % Tab 5 --------------------------------------------------------
            eopts = struct( ...
                'make_gifs',  exp_cb(1).Value, ...
                'gif_fps',    pn(exp_ef('gif_fps')), ...
                'gif_frames', pn(exp_ef('gif_frames')), ...
                'dpi',        pn(exp_ef('dpi')));

        catch ME
            set_status(['Input error: ' ME.message],'[0.90 0.30 0.30]');
            return;
        end

        % Build cfg --------------------------------------------------------
        cfg = struct();
        cfg.p                      = p;
        cfg.v                      = v_val;
        cfg.t_end                  = t_end;
        cfg.dt                     = dt_val;
        cfg.x0                     = [deg2rad(phi0); deg2rad(delta0); 0; 0];
        cfg.controllers            = ctrl_list;
        cfg.pd_k1  = 0; cfg.pd_k2  = 0;
        cfg.pid_k1 = 0; cfg.pid_k2 = 0; cfg.pid_k3 = 0;
        cfg.pp_poles               = poles;
        cfg.lqr_Q                  = lqr_Q;
        cfg.lqr_R                  = lqr_R;
        cfg.lqg_Qw                 = lqg_Qw;
        cfg.lqg_Rv                 = lqg_Rv;
        cfg.phi_limit_deg          = pl;
        cfg.delta_limit_deg        = dl;
        cfg.steer_rate_limit_deg_s = slew_val;
        cfg.disturbance            = dcfg;

        % Run folder -------------------------------------------------------
        run_num = next_run_number();
        run_dir = sprintf('Bike_Control_Run_%03d', run_num);
        mkdir(run_dir);
        set_status(sprintf('Simulating  ->  %s  ...', run_dir), ...
                   '[0.85 0.80 0.40]');
        drawnow;

        try
            results = bc_run_analysis(cfg);
        catch ME
            set_status(['Simulation error: ' ME.message],'[0.90 0.30 0.30]');
            return;
        end

        set_status('Exporting figures...','[0.85 0.80 0.40]'); drawnow;
        try
            bc_export_figures(run_dir, results, eopts);
        catch ME
            set_status(['Export error: ' ME.message],'[0.90 0.30 0.30]');
            return;
        end

        save(fullfile(run_dir,'results.mat'),'results','cfg');
        bc_write_summary(run_dir, run_num, results, cfg);
        set_status(sprintf('Done  ->  %s', run_dir),'[0.36 0.82 0.52]');
        fprintf('\nRun complete: %s\n>> animate_bike_v2\n\n', run_dir);
    end

    function set_status(msg, col)
        status_lbl.Text      = msg;
        status_lbl.FontColor = eval(col);
        drawnow;
    end

end % bike_launch

% =========================================================================
%  LAYOUT PRIMITIVES
% =========================================================================

function [next_y, ef_map] = blk_fields(parent, W, title_str, fields, start_y)
% Draws a titled block of  label | edit-field | hint  rows.
% fields: {key, label, default, hint; ...}  (N x 4 cell)
% Returns: next_y below block, containers.Map key -> edit-field handle.

    ROW_H   = 54;    % total height per field (label+field+hint)
    TITLE_H = 30;
    VSEP    = 6;     % vertical gap above block title
    X       = 14;
    LBL_W   = 248;
    EF_W    = 190;
    HINT_X  = X;
    HINT_W  = W - 28;

    n       = size(fields,1);
    blk_top = start_y + VSEP;

    % Block title
    uilabel(parent,'Text',title_str, ...
        'Position',[X blk_top+n*ROW_H+4 W-28 22], ...
        'FontSize',12,'FontWeight','bold','FontColor',[0.64 0.76 0.92]);
    uipanel(parent,'Position',[X blk_top+n*ROW_H HINT_W 1], ...
        'BackgroundColor',[0.26 0.34 0.46],'BorderType','none');

    ef_map = containers.Map();
    for i = 1:n
        row_bot = blk_top + (n-i)*ROW_H;
        key     = fields{i,1};
        lbl     = fields{i,2};
        def     = fields{i,3};
        hint    = fields{i,4};

        % Hint (bottom of row)
        uilabel(parent,'Text',hint, ...
            'Position',[HINT_X row_bot HINT_W 16], ...
            'FontSize',8.5,'FontColor',[0.50 0.60 0.72],'WordWrap','on');

        % Label
        uilabel(parent,'Text',lbl, ...
            'Position',[X row_bot+18 LBL_W 20], ...
            'FontSize',10,'FontWeight','bold','FontColor',[0.80 0.86 0.92]);

        % Edit field
        ef = uieditfield(parent,'text','Value',def, ...
            'Position',[X+LBL_W+6 row_bot+18 EF_W 22], ...
            'BackgroundColor',[0.17 0.20 0.26], ...
            'FontColor',[0.96 0.96 0.96],'FontSize',10.5);
        ef_map(key) = ef;

        % Row separator
        if i < n
            uipanel(parent,'Position',[X row_bot-1 HINT_W 1], ...
                'BackgroundColor',[0.20 0.25 0.32],'BorderType','none');
        end
    end

    next_y = blk_top + n*ROW_H + TITLE_H + VSEP + 12;
end

function [next_y, cb_arr] = blk_checks(parent, W, title_str, defs, start_y)
% Draws a titled block of checkbox + hint rows.
% defs: {key, label, default, hint; ...}  (N x 4 cell)
% Returns: next_y below block, gobjects array of checkbox handles.

    ROW_H   = 50;
    TITLE_H = 30;
    VSEP    = 6;
    X       = 14;
    HINT_W  = W - 28;

    n       = size(defs,1);
    blk_top = start_y + VSEP;

    uilabel(parent,'Text',title_str, ...
        'Position',[X blk_top+n*ROW_H+4 W-28 22], ...
        'FontSize',12,'FontWeight','bold','FontColor',[0.64 0.76 0.92]);
    uipanel(parent,'Position',[X blk_top+n*ROW_H HINT_W 1], ...
        'BackgroundColor',[0.26 0.34 0.46],'BorderType','none');

    cb_arr = gobjects(n,1);
    for i = 1:n
        row_bot = blk_top + (n-i)*ROW_H;
        lbl     = defs{i,2};
        def     = defs{i,3};
        hint    = defs{i,4};

        uilabel(parent,'Text',hint, ...
            'Position',[X+20 row_bot HINT_W-20 16], ...
            'FontSize',8.5,'FontColor',[0.50 0.60 0.72],'WordWrap','on');

        cb_arr(i) = uicheckbox(parent,'Text',lbl,'Value',def, ...
            'Position',[X row_bot+18 HINT_W 22], ...
            'FontSize',10,'FontWeight','bold','FontColor',[0.84 0.88 0.94]);

        if i < n
            uipanel(parent,'Position',[X row_bot-1 HINT_W 1], ...
                'BackgroundColor',[0.20 0.25 0.32],'BorderType','none');
        end
    end

    next_y = blk_top + n*ROW_H + TITLE_H + VSEP + 12;
end

function v = pn(ef)
    v = str2double(ef.Value);
    if isnan(v), error('Invalid number in field: "%s"', ef.Value); end
end

function n = next_run_number()
    d = dir('Bike_Control_Run_*');
    d = d([d.isdir]);
    if isempty(d), n=1; return; end
    nums = zeros(1,numel(d));
    for i=1:numel(d)
        t = regexp(d(i).name,'Bike_Control_Run_(\d+)','tokens');
        if ~isempty(t), nums(i)=str2double(t{1}{1}); end
    end
    n = max(nums)+1;
end