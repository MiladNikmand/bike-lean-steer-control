% BALANCE_CONTROLLER
% Combined analysis of the rate-feedback balance controller.
% Covers three sections:
%   Part A - Root-locus sweep to find a stabilising gain at a target speed
%   Part B - Closed-loop time-domain response at the target speed
%   Part C - Performance of the chosen gain across the full speed range
%
% Results are saved to:  results_v<speed>.mat
% (loaded by animate_bike_v2.m and save_results.m)
%
% =========================================================================
%  USER-TUNABLE PARAMETERS
% =========================================================================

target_speed  = 1.0;    % m/s — speed to stabilise
gain_min      = 0;
gain_max      = 1500;
n_gains       = 4000;
stable_margin = -0.5;   % 1/s — target margin inside stable half-plane
phi0_deg      = 5;
delta0_deg    = -2;
t_end         = 50;     % s
sweep_min     = 0.2;
sweep_max     = 10.0;
n_sweep       = 100;

% =========================================================================

fprintf('==========================================================\n');
fprintf('  BALANCE CONTROLLER  -  Rate Feedback  (T_delta = k * phi_dot)\n');
fprintf('  Whipple-Carvallo Benchmark Model\n');
fprintf('==========================================================\n\n');

p = bike_params();
[M, C1, K0, K2] = compute_benchmark_matrices(p);
[A, B] = bike_state_space(M, C1, K0, K2, target_speed, p.g);
e3 = [0 0 1 0];

fprintf('Target speed: %.1f m/s\n', target_speed);
ev_ol = sort(eig(A), 'ComparisonMethod', 'real');
fprintf('Open-loop eigenvalues: '); fprintf('%+.3f  ', real(ev_ol)); fprintf('\n');
if max(real(ev_ol)) > 0
    fprintf('Status: UNSTABLE  (max Re = %+.4f)\n\n', max(real(ev_ol)));
else
    fprintf('Status: already self-stable  (max Re = %+.4f)\n\n', max(real(ev_ol)));
end

% =========================================================================
%  PART A - Root-locus
% =========================================================================
fprintf('PART A: Root-locus sweep  [k = %.0f - %.0f, %d points]\n\n', ...
    gain_min, gain_max, n_gains);

gains        = linspace(gain_min, gain_max, n_gains);
max_real_eig = zeros(size(gains));
for i = 1:numel(gains)
    Acl = A + gains(i) * B(:,2) * e3;
    max_real_eig(i) = max(real(eig(Acl)));
end

idx_stable = find(max_real_eig < stable_margin, 1);
if isempty(idx_stable)
    error('No stabilising gain found. Try increasing gain_max.');
end
k_chosen = gains(idx_stable);
fprintf('  Chosen gain:  k = %.2f  N*m*s/rad\n\n', k_chosen);

figure('Name', 'Part A - Root-Locus', 'Position', [100 100 800 400]);
plot(gains, max_real_eig, 'LineWidth', 1.5, 'Color', [0.2 0.4 0.8]);
yline(0, 'k--'); yline(stable_margin, 'r--');
xline(k_chosen, 'g--', sprintf('k = %.1f', k_chosen));
xlabel('k  (N*m*s/rad)'); ylabel('max Re(\lambda)  [1/s]');
title(sprintf('Root-locus at v = %.1f m/s', target_speed)); grid on;

% =========================================================================
%  PART B - Closed-loop simulation
% =========================================================================
fprintf('PART B: Closed-loop simulation  (v = %.1f m/s, k = %.2f)\n\n', ...
    target_speed, k_chosen);

Acl   = A + k_chosen * B(:,2) * e3;
ev_cl = sort(eig(Acl), 'ComparisonMethod', 'real');
x0    = [deg2rad(phi0_deg); deg2rad(delta0_deg); 0; 0];
tspan = linspace(0, t_end, round(t_end * 80));

% Uncontrolled at target speed
[t_unc, x_unc] = ode45(@(t,x) A*x,   tspan, x0);
% Controlled
[t_con, x_con] = ode45(@(t,x) Acl*x, tspan, x0);

phi_deg   = rad2deg(x_con(:,1));
t_settle  = NaN;
for i = 1:numel(tspan)
    if all(abs(phi_deg(i:end)) < 1.0)
        t_settle = tspan(i); break;
    end
end

figure('Name', 'Part B - Closed-Loop Response', 'Position', [100 560 800 380]);
plot(t_con, rad2deg(x_unc(:,1)), '--', 'Color', [0.7 0.2 0.1], 'LineWidth', 1.3); hold on;
plot(t_con, rad2deg(x_con(:,1)), 'Color', [0.1 0.45 0.75], 'LineWidth', 1.8);
plot(t_con, rad2deg(x_con(:,2)), 'Color', [0.1 0.45 0.75], 'LineWidth', 1.2, 'LineStyle', ':');
yline(0, 'k:');
if ~isnan(t_settle)
    xline(t_settle, 'r--', sprintf('t_{settle} = %.1f s', t_settle));
end
xlabel('Time  (s)'); ylabel('Angle  (deg)');
title(sprintf('Controlled response at v = %.1f m/s  (k = %.2f)', target_speed, k_chosen));
legend('Uncontrolled \phi', 'Controlled \phi', 'Controlled \delta', 'Location', 'best');
grid on;

% =========================================================================
%  PART C - Generalisation sweep
% =========================================================================
fprintf('PART C: Speed generalisation sweep\n\n');

speeds_sweep = linspace(sweep_min, sweep_max, n_sweep);
ol_max = zeros(size(speeds_sweep));
cl_max = zeros(size(speeds_sweep));
min_stab_k = nan(size(speeds_sweep));
gains_fine = linspace(0, 2000, 6000);

for i = 1:n_sweep
    v = speeds_sweep(i);
    [Av, Bv] = bike_state_space(M, C1, K0, K2, v, p.g);
    ol_max(i) = max(real(eig(Av)));
    cl_max(i) = max(real(eig(Av + k_chosen * Bv(:,2) * e3)));
    if ol_max(i) > 0
        for kg = gains_fine
            if max(real(eig(Av + kg * Bv(:,2) * e3))) < 0
                min_stab_k(i) = kg; break;
            end
        end
    end
end

succ = (ol_max > 0) & (cl_max < 0);
fail = (ol_max > 0) & (cl_max >= 0);

figure('Name', 'Part C - Speed Generalisation', 'Position', [960 100 850 640]);
subplot(2,1,1);
plot(speeds_sweep, ol_max, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5); hold on;
plot(speeds_sweep, cl_max, 'Color', [0.2 0.4 0.8], 'LineWidth', 1.8);
yline(0, 'k--');
xlabel('Speed  (m/s)'); ylabel('max Re(\lambda)  [1/s]');
title(sprintf('Fixed gain k = %.1f across all speeds', k_chosen));
legend('Open-loop', sprintf('k = %.1f', k_chosen), 'Location', 'northeast');
ylim([-1.5 2]); grid on;

subplot(2,1,2);
plot(speeds_sweep, min_stab_k, 'o', 'Color', [0.2 0.7 0.5], 'MarkerSize', 3); hold on;
yline(k_chosen, 'b--', sprintf('k = %.1f', k_chosen));
xlabel('Speed  (m/s)'); ylabel('Min. stabilising k  (N*m*s/rad)');
title('Minimum stabilising gain at each speed'); grid on;

% =========================================================================
%  SUMMARY TABLE
% =========================================================================
fprintf('\n----------------------------------------------------------\n');
fprintf('  SUMMARY TABLE\n');
fprintf('----------------------------------------------------------\n');
fprintf('  Target speed:             %.1f m/s\n', target_speed);
fprintf('  Chosen gain:              k = %.2f  N*m*s/rad\n', k_chosen);
fprintf('  Closed-loop eigenvalues:  '); fprintf('%+.3f  ', real(ev_cl)); fprintf('\n');
fprintf('  Max Re(lambda):           %+.4f\n', max(real(ev_cl)));
if ~isnan(t_settle)
    fprintf('  Settling time (|phi|<1 deg): %.2f s\n', t_settle);
else
    fprintf('  Settling time: not reached within %.0f s\n', t_end);
end
if any(succ)
    fprintf('  Fixed gain stabilises:  %.2f - %.2f m/s\n', ...
            speeds_sweep(find(succ,1)), speeds_sweep(find(succ,1,'last')));
end
if any(fail)
    fprintf('  Fixed gain fails:       %.2f - %.2f m/s\n', ...
            speeds_sweep(find(fail,1)), speeds_sweep(find(fail,1,'last')));
end
fprintf('----------------------------------------------------------\n\n');

% =========================================================================
%  SAVE RESULTS  (read by animate_bike_v2.m)
% =========================================================================
speed_tag  = strrep(sprintf('%.1f', target_speed), '.', 'p');
fname      = sprintf('results_v%s.mat', speed_tag);

bc_results = struct();
bc_results.v           = target_speed;
bc_results.k_chosen    = k_chosen;
bc_results.x0          = x0;
bc_results.t_unc       = t_unc;
bc_results.x_unc       = x_unc;
bc_results.t_con       = t_con;
bc_results.x_con       = x_con;
bc_results.ev_ol       = ev_ol;
bc_results.ev_cl       = ev_cl;
bc_results.t_settle    = t_settle;
bc_results.phi0_deg    = phi0_deg;
bc_results.delta0_deg  = delta0_deg;
bc_results.label_unc   = sprintf('Uncontrolled  v=%.1f m/s', target_speed);
bc_results.label_con   = sprintf('Controlled  v=%.1f m/s  k=%.1f', target_speed, k_chosen);

save(fname, 'bc_results');
fprintf('  Results saved to:  %s\n\n', fname);