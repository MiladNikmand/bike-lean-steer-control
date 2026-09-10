% SIMULATE_UNCONTROLLED
% Validates the linearized lean-steer model by checking that it reproduces
% the well-known self-stability behavior of a real bicycle: unstable at
% very low speed, self-stabilizing in a mid speed range, unstable again
% ("capsize") above some higher speed -- with NO rider steering input.
%
% This is our sanity check before adding any nonlinear tire/slip behavior:
% if this doesn't match known bicycle behavior, nothing built on top of it
% will be trustworthy either.

clear; clc;

p = bike_params();
[M, C1, K0, K2] = compute_benchmark_matrices(p);

speeds_to_test = [1, 4.3, 9];  % m/s: expect unstable, stable, unstable
labels = {'v = 1 m/s (too slow: unstable)', ...
          'v = 4.3 m/s (self-stable range)', ...
          'v = 9 m/s (too fast: capsize unstable)'};

x0 = [deg2rad(5); deg2rad(-2); 0; 0];  % small initial lean + steer perturbation
tspan = [0 50];

figure('Position', [100 100 900 700]);
for i = 1:numel(speeds_to_test)
    v = speeds_to_test(i);
    [A, B] = bike_state_space(M, C1, K0, K2, v, p.g);

    % No control: Tphi = Tdelta = 0
    odefun = @(t, x) A*x;  % + B*[0;0], omitted since it's zero
    [t, x] = ode45(odefun, tspan, x0);

    subplot(3, 1, i);
    plot(t, rad2deg(x(:,1)), 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(x(:,2)), 'LineWidth', 1.5);
    yline(0, 'k:');
    ylim([-90 90]);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title(labels{i});
    legend('roll \phi', 'steer \delta', 'Location', 'best');
    grid on;
end

sgtitle('Uncontrolled bicycle response to a small perturbation');
