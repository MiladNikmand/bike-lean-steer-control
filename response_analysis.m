% RESPONSE_ANALYSIS
% Combined analysis of bicycle responses under different conditions.
%   Part A - Linearity check: multiple initial-condition sizes at a
%            self-stable speed (confirms the model scales correctly)
%   Part B - Controller robustness: different disturbance types on the
%            closed-loop system (roll, steer, rate kicks)
%   Part C - Strategy comparison: rate-only vs PD vs full-state feedback
%
% =========================================================================
%  USER-TUNABLE PARAMETERS  (edit this block to explore different scenarios)
% =========================================================================

% Speed for uncontrolled linearity test (Part A) — use self-stable speed
speed_stable = 4.3;   % m/s

% Speed for controller tests (Parts B & C) — use naturally-unstable speed
speed_control = 1.0;  % m/s

% Rate-feedback gain (copy from balance_controller output, or set manually)
k_rate = 47.64;   % N*m*s/rad

% Desired closed-loop poles for Ackermann placement (Part C)
desired_poles = [-4, -5, -6, -7];

% Simulation time windows
t_end_stable  = 30;   % s  (Part A)
t_end_control = 15;   % s  (Parts B & C)

% Settling threshold for Part B
settle_deg = 1.0;   % |phi| must stay below this for the rest of the run

% =========================================================================

fprintf('==========================================================\n');
fprintf('  RESPONSE ANALYSIS  -  Perturbations & Controller Comparison\n');
fprintf('  Whipple-Carvallo Benchmark Model\n');
fprintf('==========================================================\n\n');

p = bike_params();
[M, C1, K0, K2] = compute_benchmark_matrices(p);

% =========================================================================
%  PART A - Linearity check at self-stable speed
% =========================================================================
fprintf('PART A: Linearity check  (v = %.1f m/s, uncontrolled)\n\n', speed_stable);

[A_st, ~] = bike_state_space(M, C1, K0, K2, speed_stable, p.g);
tspan_a   = linspace(0, t_end_stable, 1500);

ics_a = {
    [deg2rad(5);   0;           0; 0],  'small roll (5 deg)'
    [deg2rad(10);  0;           0; 0],  '2x roll (10 deg)'
    [deg2rad(20);  0;           0; 0],  '4x roll (20 deg)'
    [0;            deg2rad(10); 0; 0],  'steer only (10 deg)'
    [0;            0; deg2rad(60); 0],  'rate kick (60 deg/s)'
    [deg2rad(20); deg2rad(-10); 0; 0],  'combined (20/-10 deg)'
};
n_ica    = size(ics_a, 1);
peak_phi = zeros(n_ica, 1);
col_a    = lines(n_ica);

figure('Name', 'Part A - Linearity Check', 'Position', [100 100 900 420]);
for i = 1:n_ica
    [~, x] = ode45(@(t,x) A_st*x, tspan_a, ics_a{i,1});
    phi_d  = rad2deg(x(:,1));
    peak_phi(i) = max(abs(phi_d));
    plot(tspan_a, phi_d, 'LineWidth', 1.4, 'Color', col_a(i,:)); hold on;
end
yline(0, 'k:', 'LineWidth', 0.8);
xlabel('Time  (s)');  ylabel('\phi  (deg)');
title(sprintf('Uncontrolled response at v = %.1f m/s - six initial conditions', speed_stable));
legend(ics_a{:,2}, 'Location', 'northeast', 'FontSize', 8);
grid on;

ratio_2x = peak_phi(2) / peak_phi(1);
ratio_4x = peak_phi(3) / peak_phi(1);
fprintf('  Linearity check:\n');
fprintf('    2x IC  ->  %.4fx peak roll  (expect 2.0000)\n', ratio_2x);
fprintf('    4x IC  ->  %.4fx peak roll  (expect 4.0000)\n', ratio_4x);
if abs(ratio_2x - 2) < 0.001 && abs(ratio_4x - 4) < 0.001
    fprintf('  OK: Model is linear - scaling confirmed.\n\n');
else
    fprintf('  WARNING: Unexpected deviation - check model or ODE tolerance.\n\n');
end

% =========================================================================
%  PART B - Controller robustness across disturbance types
% =========================================================================
fprintf('PART B: Controller robustness  (v = %.1f m/s, k = %.2f)\n\n', ...
        speed_control, k_rate);

[A_ct, B_ct] = bike_state_space(M, C1, K0, K2, speed_control, p.g);
e3   = [0 0 1 0];
Acl_b = A_ct + k_rate * B_ct(:,2) * e3;
tspan_b = linspace(0, t_end_control, 1500);

ics_b = {
    [deg2rad(5);   deg2rad(-2); 0;          0],  'small  (5/-2 deg)'
    [deg2rad(25);  deg2rad(-15);0;          0],  'large  (25/-15 deg)'
    [0;            0;           deg2rad(90);0],  'rate kick  (90 deg/s)'
};
n_icb    = size(ics_b, 1);
t_settle = nan(n_icb, 1);
peak_T_b = zeros(n_icb, 1);
col_b    = [0.2 0.4 0.8; 0.8 0.3 0.1; 0.2 0.6 0.4];

figure('Name', 'Part B - Robustness', 'Position', [100 580 900 420]);
subplot(2,1,1);
for i = 1:n_icb
    [~, x] = ode45(@(t,x) Acl_b*x, tspan_b, ics_b{i,1});
    phi_d  = rad2deg(x(:,1));
    torque = k_rate * x(:,3);
    peak_T_b(i) = max(abs(torque));
    plot(tspan_b, phi_d, 'LineWidth', 1.6, 'Color', col_b(i,:)); hold on;
    for j = 1:numel(tspan_b)
        if all(abs(phi_d(j:end)) < settle_deg)
            t_settle(i) = tspan_b(j); break;
        end
    end
end
yline(0, 'k:', 'LineWidth', 0.8);
yline( settle_deg, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8);
yline(-settle_deg, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8);
xlabel('Time  (s)');  ylabel('\phi  (deg)');
title(sprintf('Controlled response - different disturbance types  (v = %.1f m/s)', speed_control));
legend(ics_b{:,2}, 'Location', 'northeast');
grid on;

subplot(2,1,2);
for i = 1:n_icb
    [~, x] = ode45(@(t,x) Acl_b*x, tspan_b, ics_b{i,1});
    plot(tspan_b, k_rate * x(:,3), 'LineWidth', 1.4, 'Color', col_b(i,:)); hold on;
end
yline(0, 'k:', 'LineWidth', 0.8);
xlabel('Time  (s)');  ylabel('T_\delta  (N*m)');
title('Steer torque applied by controller');
grid on;

% =========================================================================
%  PART C - Strategy comparison
% =========================================================================
fprintf('PART C: Strategy comparison  (v = %.1f m/s)\n\n', speed_control);

% Strategy A: rate only
fA   = [0, 0, k_rate, 0];
AclA = A_ct + B_ct(:,2) * fA;
evA  = sort(eig(AclA), 'ComparisonMethod', 'real');

% Strategy B: PD grid search
fprintf('  Grid-searching PD gains...\n');
best_m = inf; k1b = 0; k2b = 0;
for k1 = linspace(0, 400, 161)
    for k2 = linspace(0, 150, 151)
        m = max(real(eig(A_ct + B_ct(:,2) * [k1, 0, k2, 0])));
        if m < best_m
            best_m = m; k1b = k1; k2b = k2;
        end
    end
end
fB   = [k1b, 0, k2b, 0];
AclB = A_ct + B_ct(:,2) * fB;
evB  = sort(eig(AclB), 'ComparisonMethod', 'real');

% Strategy C: Ackermann pole placement
fprintf('  Computing Ackermann pole-placement gains...\n');
K_ack = ra_ackermann(A_ct, B_ct(:,2), desired_poles);
fC    = -K_ack;
AclC  = A_ct + B_ct(:,2) * fC;
evC   = sort(eig(AclC), 'ComparisonMethod', 'real');

x0_c  = [deg2rad(5); deg2rad(-2); 0; 0];
tspan_c = linspace(0, t_end_control, 1500);
strats  = {AclA, fA, 'A: Rate-only',      [0.56 0.65 0.73]
           AclB, fB, 'B: PD (phi+phidot)', [0.37 0.72 0.69]
           AclC, fC, 'C: Full-state',      [0.88 0.64 0.35]};
ev_c    = {evA, evB, evC};
settle_c = nan(3,1);
peak_c   = zeros(3,1);

figure('Name', 'Part C - Strategy Comparison', 'Position', [960 560 900 640]);
subplot(2,1,1);
for i = 1:3
    [~, x] = ode45(@(t,x) strats{i,1}*x, tspan_c, x0_c);
    phi_d  = rad2deg(x(:,1));
    torque = strats{i,2} * x';
    peak_c(i) = max(abs(torque));
    plot(tspan_c, phi_d, 'LineWidth', 1.6, 'Color', strats{i,4}); hold on;
    for j = 1:numel(tspan_c)
        if all(abs(phi_d(j:end)) < settle_deg)
            settle_c(i) = tspan_c(j); break;
        end
    end
end
yline(0, 'k:', 'LineWidth', 0.8);
xlim([0 6]);
xlabel('Time  (s)');  ylabel('\phi  (deg)');
title(sprintf('Three feedback strategies - same disturbance  (v = %.1f m/s)', speed_control));
legend(strats{:,3}, 'Location', 'northeast');
grid on;

subplot(2,1,2);
for i = 1:3
    [~, x] = ode45(@(t,x) strats{i,1}*x, tspan_c, x0_c);
    plot(tspan_c, (strats{i,2} * x')', 'LineWidth', 1.4, 'Color', strats{i,4}); hold on;
end
yline(0, 'k:', 'LineWidth', 0.8);
xlim([0 6]);
xlabel('Time  (s)');  ylabel('T_\delta  (N*m)');
title('Steer torque - strategy comparison');
grid on;

% =========================================================================
%  SUMMARY TABLES
% =========================================================================
fprintf('\n----------------------------------------------------------\n');
fprintf('  SUMMARY - Part B: Controller robustness\n');
fprintf('----------------------------------------------------------\n');
fprintf('  %-22s  %-12s  %s\n', 'Disturbance', 'Settle (s)', 'Peak torque (N*m)');
fprintf('  %-22s  %-12s  %s\n', '----------------------', '------------', '-----------------');
for i = 1:n_icb
    if isnan(t_settle(i)), ts = sprintf('> %.0f', t_end_control);
    else, ts = sprintf('%.2f', t_settle(i)); end
    fprintf('  %-22s  %-12s  %.2f\n', ics_b{i,2}, ts, peak_T_b(i));
end

fprintf('\n----------------------------------------------------------\n');
fprintf('  SUMMARY - Part C: Strategy comparison\n');
fprintf('----------------------------------------------------------\n');
fprintf('  %-22s  %-12s  %-18s  %s\n', 'Strategy', 'Settle (s)', 'Peak torque (N*m)', 'Max Re(eig)');
fprintf('  %-22s  %-12s  %-18s  %s\n', '----------------------', '------------', '------------------', '-----------');
labels_c = {'A: Rate-only', 'B: PD', 'C: Full-state'};
for i = 1:3
    if isnan(settle_c(i)), ts = sprintf('> %.0f', t_end_control);
    else, ts = sprintf('%.2f', settle_c(i)); end
    fprintf('  %-22s  %-12s  %-18.2f  %+.4f\n', labels_c{i}, ts, peak_c(i), max(real(ev_c{i})));
end
fprintf('----------------------------------------------------------\n');
fprintf('  PD best gains:  k_phi = %.1f,  k_phidot = %.1f\n', k1b, k2b);
fprintf('  Pole-placement poles: '); fprintf('%d  ', desired_poles); fprintf('\n');
fprintf('  Ackermann gains: '); fprintf('%.2f  ', K_ack); fprintf('\n\n');

% =========================================================================
%  LOCAL FUNCTION - Ackermann pole placement (no Control Toolbox needed)
% =========================================================================
function K = ra_ackermann(A, b, poles)
    n  = size(A, 1);
    Wc = zeros(n);
    Ap = eye(n);
    for i = 1:n
        Wc(:,i) = Ap * b;
        Ap = Ap * A;
    end
    coeffs = poly(poles);
    phiA   = zeros(n);
    pows   = n:-1:0;
    for i = 1:numel(coeffs)
        phiA = phiA + coeffs(i) * mpower(A, pows(i));
    end
    last_row    = zeros(1, n);
    last_row(n) = 1;
    K = last_row * (Wc \ phiA);
end