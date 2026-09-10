% SIMULATE_CONTROLLED
% Demonstrates a rider stabilization strategy at low speed: apply a steer
% torque proportional to the roll rate,
%
%   Tdelta = +k * phidot   (k > 0)
%
% i.e. steer in the direction the bike is falling, which brings the
% front contact patch back underneath the center of mass -- the
% low-speed balancing strategy every cyclist uses intuitively (distinct
% from high-speed countersteering, where you briefly steer the *other*
% way to roll the bike into a turn). The sign was confirmed numerically:
% it is the direction that actually reduces the growth rate of the
% unstable mode, not assumed from the outset. We sweep gain k to find a
% stabilizing value (a manual root locus, since we don't depend on the
% Control Systems Toolbox), then simulate the controlled response.

clear; clc;

p = bike_params();
[M, C1, K0, K2] = compute_benchmark_matrices(p);

v = 1.0;  % m/s -- naturally UNSTABLE speed with no control (see eigs_vs_speed.m)
[A, B] = bike_state_space(M, C1, K0, K2, v, p.g);

% Closed loop: u = [Tphi; Tdelta] = [0; k*phidot] = [0; k*x(3)]
% => B*u = k * B(:,2) * x(3) = k * B(:,2) * (e3' * x)
e3 = [0 0 1 0];

gains = linspace(0, 1500, 4000);
max_real_eig = zeros(size(gains));
for i = 1:numel(gains)
    k = gains(i);
    Acl = A + k * B(:,2) * e3;
    max_real_eig(i) = max(real(eig(Acl)));
end

figure('Position', [100 100 800 400]);
plot(gains, max_real_eig, 'LineWidth', 1.5);
yline(0, 'k--');
xlabel('Feedback gain k (N m s / rad)');
ylabel('max Re(\lambda)  [1/s]');
title(sprintf('Manual root locus: T_\\delta = k \\cdot \\phidot  at v = %.1f m/s', v));
grid on;

% Pick a gain from the stable region (comfortably inside it, not on the edge)
k_chosen = gains(find(max_real_eig < -0.5, 1));
fprintf('Chosen stabilizing gain: k = %.2f N m s/rad\n', k_chosen);

Acl = A + k_chosen * B(:,2) * e3;

x0 = [deg2rad(5); deg2rad(-2); 0; 0];
tspan = [0 50];
[t, x] = ode45(@(t,x) Acl*x, tspan, x0);

figure('Position', [100 550 800 400]);
plot(t, rad2deg(x(:,1)), 'LineWidth', 1.5); hold on;
plot(t, rad2deg(x(:,2)), 'LineWidth', 1.5);
yline(0, 'k:');
xlabel('Time (s)');
ylabel('Angle (deg)');
title(sprintf('Controlled response at v = %.1f m/s (naturally unstable, now stabilized)', v));
legend('roll \phi', 'steer \delta', 'Location', 'best');
grid on;
