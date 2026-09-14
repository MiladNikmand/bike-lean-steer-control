% STABILITY_ANALYSIS
% Combined stability analysis of the uncontrolled Whipple-Carvallo bicycle.
%   Part A - Eigenvalue sweep across a speed range
%   Part B - Time-domain simulation at selected test speeds
%
% Results are saved to:  results_v<speed>.mat  for each test speed
% (loaded by animate_bike_v2.m and save_results.m)
%
% =========================================================================
%  USER-TUNABLE PARAMETERS
% =========================================================================

speed_min   = 0;
speed_max   = 10;
n_speeds    = 200;
test_speeds = [1, 4.3, 9];   % m/s
phi0_deg    = 5;
delta0_deg  = -2;
t_end       = 50;

% =========================================================================

fprintf('==========================================================\n');
fprintf('  STABILITY ANALYSIS  -  Uncontrolled Bicycle\n');
fprintf('  Whipple-Carvallo Benchmark Model\n');
fprintf('==========================================================\n\n');

p = bike_params();
[M, C1, K0, K2] = compute_benchmark_matrices(p);

% =========================================================================
%  PART A - Eigenvalue sweep
% =========================================================================
fprintf('PART A: Eigenvalue sweep  [%.1f - %.1f m/s, %d points]\n\n', ...
    speed_min, speed_max, n_speeds);

speeds     = linspace(speed_min, speed_max, n_speeds);
real_parts = zeros(n_speeds, 4);
for i = 1:n_speeds
    [A, ~] = bike_state_space(M, C1, K0, K2, speeds(i), p.g);
    ev = eig(A);
    real_parts(i,:) = sort(real(ev))';
end

max_real = max(real_parts, [], 2);
stable   = max_real <= 1e-9;
if any(stable)
    idx  = find(stable);
    v_lo = speeds(idx(1));
    v_hi = speeds(idx(end));
    fprintf('  Self-stable band: %.2f - %.2f m/s  (width %.2f m/s)\n\n', ...
            v_lo, v_hi, v_hi - v_lo);
else
    v_lo = NaN; v_hi = NaN;
    fprintf('  No self-stable band found.\n\n');
end

figure('Name', 'Part A - Eigenvalue Sweep', 'Position', [100 100 850 480]);
plot(speeds, real_parts, '.', 'MarkerSize', 5);
yline(0, 'k--', 'LineWidth', 1.2);
xlabel('Forward speed  v  (m/s)'); ylabel('Re(\lambda)  [1/s]');
title('Uncontrolled bicycle: eigenvalue real parts vs. speed');
legend('\lambda_1','\lambda_2','\lambda_3','\lambda_4','Boundary','Location','northeast');
grid on;

% =========================================================================
%  PART B - Time-domain simulation at selected speeds
% =========================================================================
fprintf('PART B: Time-domain simulation\n');
fprintf('  Initial condition: phi_0 = %g deg,  delta_0 = %g deg\n\n', ...
    phi0_deg, delta0_deg);

x0    = [deg2rad(phi0_deg); deg2rad(delta0_deg); 0; 0];
tspan = linspace(0, t_end, round(t_end * 80));

n_test     = numel(test_speeds);
tbl_eig    = zeros(n_test, 4);
tbl_regime = cell(n_test, 1);

figure('Name', 'Part B - Time-Domain Response', ...
       'Position', [100 620 900 min(300*n_test, 900)]);

for i = 1:n_test
    v = test_speeds(i);
    [A, ~] = bike_state_space(M, C1, K0, K2, v, p.g);
    ev     = sort(eig(A), 'ComparisonMethod', 'real');
    tbl_eig(i,:) = real(ev)';

    max_re = max(real(ev));
    if     max_re >  1e-4, regime = 'UNSTABLE';
    elseif max_re < -1e-4, regime = 'self-stable';
    else,                  regime = '~ boundary';
    end
    tbl_regime{i} = regime;

    [t_sim, x_sim] = ode45(@(t,x) A*x, tspan, x0);

    subplot(n_test, 1, i);
    plot(t_sim, rad2deg(x_sim(:,1)), 'LineWidth', 1.5); hold on;
    plot(t_sim, rad2deg(x_sim(:,2)), 'LineWidth', 1.5);
    yline(0, 'k:', 'LineWidth', 0.8);
    ylim([-90 90]);
    xlabel('Time  (s)'); ylabel('Angle  (deg)');
    title(sprintf('v = %.1f m/s  [%s]', v, regime));
    legend('\phi  (roll)', '\delta  (steer)', 'Location', 'northeast');
    grid on;

    % --- Save this speed's uncontrolled result ---
    speed_tag  = strrep(sprintf('%.1f', v), '.', 'p');
    fname      = sprintf('results_v%s.mat', speed_tag);

    % Load existing file if present (balance_controller may have saved it)
    if isfile(fname)
        saved = load(fname, 'bc_results');
        bc_results = saved.bc_results;
    else
        bc_results = struct();
        bc_results.v          = v;
        bc_results.k_chosen   = NaN;
        bc_results.x0         = x0;
        bc_results.phi0_deg   = phi0_deg;
        bc_results.delta0_deg = delta0_deg;
    end

    % Always update uncontrolled trajectory
    bc_results.t_unc     = t_sim;
    bc_results.x_unc     = x_sim;
    bc_results.ev_ol     = ev;
    bc_results.regime    = regime;
    bc_results.label_unc = sprintf('Uncontrolled  v=%.1f m/s  [%s]', v, regime);

    % Add controlled fields as empty if not yet set by balance_controller
    if ~isfield(bc_results, 't_con') || isempty(bc_results.t_con)
        bc_results.t_con     = [];
        bc_results.x_con     = [];
        bc_results.ev_cl     = [];
        bc_results.t_settle  = NaN;
        bc_results.label_con = '(not yet computed — run balance_controller.m at this speed)';
    end

    save(fname, 'bc_results');
    fprintf('  v = %.1f m/s  [%s]  -> saved to %s\n', v, regime, fname);
end

% =========================================================================
%  SUMMARY TABLE
% =========================================================================
fprintf('\n----------------------------------------------------------\n');
fprintf('  SUMMARY TABLE\n');
fprintf('----------------------------------------------------------\n');
fprintf('  %-8s  %-12s  %-30s\n', 'v (m/s)', 'Regime', 'Eigenvalues (real parts)');
fprintf('  %-8s  %-12s  %-30s\n', '--------', '------------', '------------------------------');
for i = 1:n_test
    ev_str = sprintf('%+.3f  ', tbl_eig(i,:));
    fprintf('  %-8.1f  %-12s  %s\n', test_speeds(i), tbl_regime{i}, ev_str);
end
fprintf('----------------------------------------------------------\n');
if ~isnan(v_lo)
    fprintf('  Self-stable band: %.2f - %.2f m/s\n', v_lo, v_hi);
end
fprintf('\n');