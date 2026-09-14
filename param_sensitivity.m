% PARAM_SENSITIVITY
% Sweeps four physical parameters one at a time and tracks how the
% self-stable speed window [v_lo, v_hi] shifts.
%
% Parameters swept:
%   1. Rider + frame mass   mB      (kg)
%   2. Trail                c       (m)
%   3. Wheelbase            w       (m)
%   4. Steer axis tilt      lambda  (deg)
%
% =========================================================================
%  USER-TUNABLE PARAMETERS  (edit this block to explore different scenarios)
% =========================================================================

% Speed grid for finding the stable band (m/s)
speed_min_sens = 0;
speed_max_sens = 12;
n_speeds_sens  = 400;

% Sweep ranges — [min, max, n_points]
sweep_mB     = [60,   110,  26];   % kg
sweep_c      = [0.02, 0.14, 26];   % m
sweep_w      = [0.85, 1.25, 26];   % m
sweep_lambda = [5,    35,   26];   % deg

% =========================================================================

fprintf('==========================================================\n');
fprintf('  PARAMETER SENSITIVITY  -  Self-Stable Speed Band\n');
fprintf('  Whipple-Carvallo Benchmark Model\n');
fprintf('==========================================================\n\n');

p_nom = bike_params();
speeds_grid = linspace(speed_min_sens, speed_max_sens, n_speeds_sens);

% Nominal stable band
[v_lo_nom, v_hi_nom] = sa_stable_band(p_nom, speeds_grid);
fprintf('Nominal self-stable band: %.2f - %.2f m/s  (width %.2f m/s)\n\n', ...
        v_lo_nom, v_hi_nom, v_hi_nom - v_lo_nom);

sweeps = {
    'mB',     'Rider + frame mass  m_B (kg)', sweep_mB,     p_nom.mB,              'kg'
    'c',      'Trail  c (m)',                 sweep_c,      p_nom.c,               'm'
    'w',      'Wheelbase  w (m)',             sweep_w,      p_nom.w,               'm'
    'lambda', 'Steer axis tilt (deg)',        sweep_lambda, rad2deg(p_nom.lambda),  'deg'
};
n_sw = size(sweeps, 1);
results = cell(n_sw, 1);

for si = 1:n_sw
    pname  = sweeps{si,1};
    cfg    = sweeps{si,3};
    vals   = linspace(cfg(1), cfg(2), cfg(3));
    nv     = numel(vals);
    vlo_arr = nan(1,nv);
    vhi_arr = nan(1,nv);
    for vi = 1:nv
        p = p_nom;
        if strcmp(pname, 'lambda')
            p.(pname) = deg2rad(vals(vi));
        else
            p.(pname) = vals(vi);
        end
        [vl, vh] = sa_stable_band(p, speeds_grid);
        vlo_arr(vi) = vl;
        vhi_arr(vi) = vh;
    end
    results{si} = struct('vals', vals, 'v_lo', vlo_arr, 'v_hi', vhi_arr);
end

figure('Name', 'Parameter Sensitivity', 'Position', [100 100 950 700]);
for si = 1:n_sw
    subplot(2, 2, si);
    r   = results{si};
    bw  = r.v_hi - r.v_lo;
    fill([r.vals, fliplr(r.vals)], [r.v_lo, fliplr(r.v_hi)], ...
         [0.4 0.8 0.6], 'FaceAlpha', 0.35, 'EdgeColor', 'none'); hold on;
    plot(r.vals, r.v_lo, 'Color', [0.1 0.5 0.3], 'LineWidth', 1.4);
    plot(r.vals, r.v_hi, 'Color', [0.1 0.5 0.3], 'LineWidth', 1.4);
    xline(sweeps{si,4}, '--', 'Color', [0.85 0.5 0.1], 'LineWidth', 1.4);
    xlabel(sweeps{si,2});
    ylabel('Speed  (m/s)');
    title(sweeps{si,2});
    grid on;
    fprintf('  %-8s  band width: %.2f - %.2f m/s\n', sweeps{si,1}, min(bw), max(bw));
end
sgtitle('Self-stable speed band vs. physical parameters', 'FontSize', 13);

fprintf('\n----------------------------------------------------------\n');
fprintf('  SUMMARY TABLE\n');
fprintf('----------------------------------------------------------\n');
fprintf('  %-10s  %-22s  %-24s  %s\n', 'Parameter', 'Sweep range', 'Band width range', 'Sensitivity');
fprintf('  %-10s  %-22s  %-24s  %s\n', '----------', '----------------------', '------------------------', '-----------');
for si = 1:n_sw
    r   = results{si};
    bw  = r.v_hi - r.v_lo;
    cfg = sweeps{si,3};
    sens = max(bw) - min(bw);
    if sens > 1.5,    slabel = 'HIGH';
    elseif sens > 0.8, slabel = 'moderate';
    else,              slabel = 'low';
    end
    fprintf('  %-10s  %.2f - %.2f %-5s       %.2f - %.2f m/s          %s (%.2f m/s)\n', ...
            sweeps{si,1}, cfg(1), cfg(2), sweeps{si,5}, min(bw), max(bw), slabel, sens);
end
fprintf('----------------------------------------------------------\n\n');

% =========================================================================
%  LOCAL FUNCTION
% =========================================================================
function [v_lo, v_hi] = sa_stable_band(p, speeds)
    [M, C1, K0, K2] = compute_benchmark_matrices(p);
    max_re = zeros(size(speeds));
    for i = 1:numel(speeds)
        [A, ~] = bike_state_space(M, C1, K0, K2, speeds(i), p.g);
        max_re(i) = max(real(eig(A)));
    end
    stable = max_re <= 1e-9;
    if any(stable)
        idx  = find(stable);
        v_lo = speeds(idx(1));
        v_hi = speeds(idx(end));
    else
        v_lo = NaN; v_hi = NaN;
    end
end