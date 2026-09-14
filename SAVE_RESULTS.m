% SAVE_RESULTS
% Standalone script that runs balance_controller.m and stability_analysis.m
% at a set of speeds and saves every result to a .mat file.
%
% Each file is named  results_v<speed>.mat  e.g. results_v1p0.mat
% and contains a single struct  bc_results  with fields:
%
%   .v            forward speed (m/s)
%   .k_chosen     stabilising gain (NaN if no controller was computed)
%   .x0           initial condition [phi; delta; phi_dot; delta_dot]
%   .t_unc        time vector  — uncontrolled simulation
%   .x_unc        state matrix — uncontrolled simulation  (N x 4)
%   .t_con        time vector  — controlled simulation  ([] if unavailable)
%   .x_con        state matrix — controlled simulation  ([] if unavailable)
%   .ev_ol        open-loop eigenvalues
%   .ev_cl        closed-loop eigenvalues  ([] if unavailable)
%   .regime       'UNSTABLE' | 'self-stable' | '~ boundary'
%   .t_settle     settling time in seconds (NaN if not reached)
%   .label_unc    display string for the uncontrolled run
%   .label_con    display string for the controlled run
%
% These files are read by animate_bike_v2.m.
%
% =========================================================================
%  USER-TUNABLE PARAMETERS
% =========================================================================

% Speeds at which to generate and save results
save_speeds = [1.0, 4.3, 9.0];   % m/s  — add any speeds you want here

% Shared initial condition used for all speeds
phi0_deg_save   = 5;
delta0_deg_save = -2;

% Simulation duration
t_end_save = 50;   % s

% =========================================================================

fprintf('==========================================================\n');
fprintf('  SAVE_RESULTS  -  Generating .mat files for all speeds\n');
fprintf('==========================================================\n\n');

p = bike_params();
[M, C1, K0, K2] = compute_benchmark_matrices(p);
x0_save  = [deg2rad(phi0_deg_save); deg2rad(delta0_deg_save); 0; 0];
tspan_sv = linspace(0, t_end_save, round(t_end_save * 80));
e3       = [0 0 1 0];

for si = 1:numel(save_speeds)
    v = save_speeds(si);
    fprintf('Processing v = %.1f m/s ...\n', v);

    [A, B] = bike_state_space(M, C1, K0, K2, v, p.g);
    ev_ol  = sort(eig(A), 'ComparisonMethod', 'real');
    max_re = max(real(ev_ol));

    if     max_re >  1e-4, regime = 'UNSTABLE';
    elseif max_re < -1e-4, regime = 'self-stable';
    else,                  regime = '~ boundary';
    end

    % Uncontrolled simulation
    [t_unc, x_unc] = ode45(@(t,x) A*x, tspan_sv, x0_save);

    % Find stabilising gain (only meaningful for unstable speeds)
    k_chosen = NaN;
    ev_cl    = [];
    t_con    = [];
    x_con    = [];
    t_settle = NaN;

    if strcmp(regime, 'UNSTABLE')
        gains_sv = linspace(0, 2000, 8000);
        for gi = 1:numel(gains_sv)
            Acl_test = A + gains_sv(gi) * B(:,2) * e3;
            if max(real(eig(Acl_test))) < -0.5
                k_chosen = gains_sv(gi);
                break;
            end
        end

        if ~isnan(k_chosen)
            Acl_sv = A + k_chosen * B(:,2) * e3;
            ev_cl  = sort(eig(Acl_sv), 'ComparisonMethod', 'real');
            [t_con, x_con] = ode45(@(t,x) Acl_sv*x, tspan_sv, x0_save);

            phi_d = rad2deg(x_con(:,1));
            for ji = 1:numel(tspan_sv)
                if all(abs(phi_d(ji:end)) < 1.0)
                    t_settle = tspan_sv(ji); break;
                end
            end
        end
    elseif strcmp(regime, 'self-stable')
        % Self-stable: controlled = uncontrolled (no gain needed)
        k_chosen = 0;
        ev_cl    = ev_ol;
        t_con    = t_unc;
        x_con    = x_unc;
        t_settle = NaN;
        for ji = 1:numel(tspan_sv)
            if all(abs(rad2deg(x_unc(ji:end,1))) < 1.0)
                t_settle = tspan_sv(ji); break;
            end
        end
    end

    % Build label strings
    label_unc = sprintf('Uncontrolled  v=%.1f m/s  [%s]', v, regime);
    if isnan(k_chosen) || k_chosen == 0
        label_con = sprintf('No controller needed  v=%.1f m/s  [%s]', v, regime);
    else
        label_con = sprintf('Controlled  v=%.1f m/s  k=%.1f', v, k_chosen);
    end

    % Pack and save
    bc_results = struct();
    bc_results.v           = v;
    bc_results.k_chosen    = k_chosen;
    bc_results.x0          = x0_save;
    bc_results.phi0_deg    = phi0_deg_save;
    bc_results.delta0_deg  = delta0_deg_save;
    bc_results.t_unc       = t_unc;
    bc_results.x_unc       = x_unc;
    bc_results.t_con       = t_con;
    bc_results.x_con       = x_con;
    bc_results.ev_ol       = ev_ol;
    bc_results.ev_cl       = ev_cl;
    bc_results.regime      = regime;
    bc_results.t_settle    = t_settle;
    bc_results.label_unc   = label_unc;
    bc_results.label_con   = label_con;

    speed_tag = strrep(sprintf('%.1f', v), '.', 'p');
    fname     = sprintf('results_v%s.mat', speed_tag);
    save(fname, 'bc_results');
    fprintf('  Saved:  %s  (regime: %s', fname, regime);
    if ~isnan(k_chosen) && k_chosen > 0
        fprintf(',  k = %.2f', k_chosen);
    end
    if ~isnan(t_settle)
        fprintf(',  settle = %.2f s', t_settle);
    end
    fprintf(')\n');
end

fprintf('\nDone. Files saved:\n');
files = dir('results_v*.mat');
for fi = 1:numel(files)
    fprintf('  %s\n', files(fi).name);
end
fprintf('\nRun animate_bike_v2.m to visualise any of these.\n\n');