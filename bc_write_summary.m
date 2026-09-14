function bc_write_summary(run_dir, run_num, results, cfg)
% BC_WRITE_SUMMARY  Writes run_summary.json and README.txt into run_dir.
% Called by both run_all.m and bike_launch.m so the output is identical
% regardless of which entry point was used.

bc_write_json(run_dir, run_num, results, cfg);
bc_write_readme(run_dir, run_num, results, cfg);

end

% =========================================================================
function bc_write_json(run_dir, run_num, results, cfg)

S = struct();
S.schema      = 'bike-lean-steer-control/1.0';
S.run_number  = run_num;
S.run_folder  = sprintf('Bike_Control_Run_%03d', run_num);
S.generated   = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
S.speed_m_s   = results.v;
S.regime      = results.regime;
S.max_re_ol   = results.max_re;

S.bike_params = struct( ...
    'mB',        cfg.p.mB, ...
    'w',         cfg.p.w, ...
    'c',         cfg.p.c, ...
    'lambda_deg',rad2deg(cfg.p.lambda), ...
    'rR',        cfg.p.rR, ...
    'rF',        cfg.p.rF);

S.initial_conditions = struct( ...
    'phi0_deg',   rad2deg(cfg.x0(1)), ...
    'delta0_deg', rad2deg(cfg.x0(2)));

S.constraints = struct( ...
    'phi_limit_deg',          cfg.phi_limit_deg, ...
    'delta_sat_deg',          cfg.delta_limit_deg, ...
    'steer_rate_limit_deg_s', cfg.steer_rate_limit_deg_s, ...
    'note', 'phi_limit = hard stop (bike fell). delta_sat = clamp only, run continues.');

S.disturbance = cfg.disturbance;

keys = fieldnames(results.runs);
runs_out = struct();
for ki = 1:numel(keys)
    key = keys{ki};
    run = results.runs.(key);
    runs_out.(key) = struct( ...
        'label',       run.label, ...
        'k_info',      run.k_info, ...
        'failed',      run.failed, ...
        'fail_time',   run.fail_time, ...
        'fail_msg',    run.fail_msg, ...
        'settle_time', run.settle_time, ...
        'max_re_cl',   max(real(run.ev_cl)), ...
        'sat_count',   run.sat_info.count, ...
        'sat_first_t', run.sat_info.first_t, ...
        'saturated',   run.sat_info.saturated);
end
S.runs = runs_out;

fid = fopen(fullfile(run_dir,'run_summary.json'),'w');
fwrite(fid, jsonencode(S,'PrettyPrint',true),'char');
fclose(fid);
fprintf('  run_summary.json written.\n');

end

% =========================================================================
function bc_write_readme(run_dir, run_num, results, cfg)

fid = fopen(fullfile(run_dir,'README.txt'),'w');
fprintf(fid,'Bike Lean-Steer Control -- Run %03d\n', run_num);
fprintf(fid,'Generated: %s\n\n', ...
        char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));

fprintf(fid,'BIKE PARAMETERS\n');
fprintf(fid,'  Forward speed:      %.2f m/s\n',  results.v);
fprintf(fid,'  Rider+frame mass:   %.1f kg\n',   cfg.p.mB);
fprintf(fid,'  Wheelbase:          %.3f m\n',    cfg.p.w);
fprintf(fid,'  Trail:              %.3f m\n',    cfg.p.c);
fprintf(fid,'  Steer axis tilt:    %.2f deg\n',  rad2deg(cfg.p.lambda));
fprintf(fid,'  Rear wheel radius:  %.3f m\n',    cfg.p.rR);
fprintf(fid,'  Open-loop regime:   %s  (max Re = %+.4f)\n\n', ...
        results.regime, results.max_re);

fprintf(fid,'INITIAL CONDITIONS\n');
fprintf(fid,'  phi_0:    %.2f deg\n',  rad2deg(cfg.x0(1)));
fprintf(fid,'  delta_0:  %.2f deg\n',  rad2deg(cfg.x0(2)));
fprintf(fid,'  Duration: %.0f s\n\n', cfg.t_end);

fprintf(fid,'CONSTRAINTS\n');
fprintf(fid,'  Roll limit (HARD STOP): +/- %.0f deg  (bike fell)\n', cfg.phi_limit_deg);
fprintf(fid,'  Steer saturation (clamp, NOT stop): +/- %.0f deg\n', cfg.delta_limit_deg);
if isinf(cfg.steer_rate_limit_deg_s)
    fprintf(fid,'  Steer slew rate: unlimited (ideal actuator)\n');
else
    fprintf(fid,'  Steer slew rate: %.0f deg/s\n', cfg.steer_rate_limit_deg_s);
end
fprintf(fid,'  Integration step dt: %.4f s\n\n', cfg.dt);

if cfg.disturbance.kick_enabled || cfg.disturbance.gravel_enabled
    fprintf(fid,'DISTURBANCES\n');
    if cfg.disturbance.kick_enabled
        fprintf(fid,'  Kick at t=%.1fs  magnitude=%.1f deg\n', ...
            cfg.disturbance.kick_time, cfg.disturbance.kick_mag_deg);
    end
    if cfg.disturbance.gravel_enabled
        fprintf(fid,'  Gravel: t=%.1f to %.1f s  std=%.1f\n', ...
            cfg.disturbance.gravel_start, cfg.disturbance.gravel_end, ...
            cfg.disturbance.gravel_std);
    end
    fprintf(fid,'\n');
end

fprintf(fid,'CONTROLLER RESULTS\n');
keys = fieldnames(results.runs);
for ki = 1:numel(keys)
    key = keys{ki};
    run = results.runs.(key);
    fprintf(fid,'  [%s]\n', key);
    if ~isempty(run.k_info)
        fprintf(fid,'    %s\n', run.k_info);
    end
    if run.failed
        fprintf(fid,'    STATUS: FAILED at t=%.2fs -- %s\n', ...
                run.fail_time, run.fail_msg);
    else
        fprintf(fid,'    STATUS: OK\n');
    end
    if run.sat_info.saturated
        fprintf(fid,'    ACTUATOR SATURATED: %d times, first at t=%.2fs\n', ...
                run.sat_info.count, run.sat_info.first_t);
    end
    if ~isnan(run.settle_time)
        fprintf(fid,'    Settle time (|phi|<1 deg): %.2f s\n', run.settle_time);
    end
    fprintf(fid,'\n');
end

fprintf(fid,'FILES\n');
fprintf(fid,'  figures/00_parameters.png       -- parameter summary\n');
for ki = 1:numel(keys)
    fprintf(fid,'  figures/%s_response.png      -- response plot\n', keys{ki});
    fprintf(fid,'  figures/%s_response.gif      -- animated response\n', keys{ki});
end
fprintf(fid,'  figures/combined_comparison.png  -- all controllers\n');
fprintf(fid,'  results.mat                      -- raw simulation data\n');
fprintf(fid,'  run_summary.json                 -- machine-readable summary\n');
fclose(fid);
fprintf('  README.txt written.\n');

end