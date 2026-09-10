% EIGS_VS_SPEED
% Sweeps forward speed v and plots the real part of the closed-loop-free
% (uncontrolled) eigenvalues. Positive real part = growing oscillation or
% divergence = unstable. This reproduces the classic bicycle stability
% chart: unstable at low speed, a self-stable "weave" range, then
% unstable again at high speed ("capsize").

clear; clc;

p = bike_params();
[M, C1, K0, K2] = compute_benchmark_matrices(p);

speeds = linspace(0, 10, 200);
n = numel(speeds);
real_parts = zeros(n, 4);

for i = 1:n
    v = speeds(i);
    [A, ~] = bike_state_space(M, C1, K0, K2, v, p.g);
    ev = eig(A);
    real_parts(i, :) = real(ev)';
end

figure('Position', [100 100 800 500]);
plot(speeds, real_parts, '.', 'MarkerSize', 6);
yline(0, 'k--');
xlabel('Forward speed v (m/s)');
ylabel('Re(\lambda)  [1/s]');
title('Stability of the uncontrolled bicycle vs. forward speed');
grid on;
