% BC_BUILD_REPORT
% Generates report_data.js from all available Bike_Control_Run_NNN folders,
% then opens report.html in the system browser.
%
% Usage:
%   bc_build_report          % scans all run folders automatically
%   bc_build_report('Bike_Control_Run_003')  % specific folder only
%
% Output:
%   report_data.js   — JS bundle read by report.html
%
% Keep report.html and report_data.js in the same project root folder.

function bc_build_report(varargin)

% ---- Find run folders ---------------------------------------------------
if nargin >= 1
    run_dirs = {varargin{1}};
else
    d = dir('Bike_Control_Run_*');
    d = d([d.isdir]);
    if isempty(d)
        error('No Bike_Control_Run_NNN folders found. Run bike_launch or run_all first.');
    end
    run_dirs = {d.name};
end

fprintf('\n[bc_build_report] Building report from %d run(s)...\n\n', numel(run_dirs));

% ---- Collect data from each run -----------------------------------------
runs_json = {};
for ri = 1:numel(run_dirs)
    rdir = run_dirs{ri};
    fprintf('  Processing: %s\n', rdir);

    mat_file  = fullfile(rdir, 'results.mat');
    json_file = fullfile(rdir, 'run_summary.json');

    if ~isfile(mat_file)
        fprintf('    WARNING: results.mat not found, skipping.\n');
        continue;
    end

    try
        S = load(mat_file, 'results', 'cfg');
        r = S.results;
        c = S.cfg;
    catch
        fprintf('    WARNING: could not load results.mat, skipping.\n');
        continue;
    end

    % ---- Build run JSON struct ----
    jrun = struct();
    jrun.id         = rdir;
    jrun.name       = rdir;
    jrun.speed      = r.v;
    jrun.regime     = r.regime;
    jrun.max_re_ol  = r.max_re;
    jrun.dt         = c.dt;
    jrun.t_end      = c.t_end;
    jrun.x0_phi_deg    = rad2deg(c.x0(1));
    jrun.x0_delta_deg  = rad2deg(c.x0(2));
    jrun.phi_limit_deg    = c.phi_limit_deg;
    jrun.delta_sat_deg    = c.delta_limit_deg;
    jrun.slew_rate_deg_s  = c.steer_rate_limit_deg_s;

    % Bike parameters
    jrun.params = struct( ...
        'mB',        c.p.mB, ...
        'w',         c.p.w, ...
        'c',         c.p.c, ...
        'lambda_deg',rad2deg(c.p.lambda), ...
        'rR',        c.p.rR, ...
        'rF',        c.p.rF);

    % Open-loop eigenvalues
    ev_ol = r.ev_ol;
    ev_arr = {};
    for ei = 1:numel(ev_ol)
        ev_arr{end+1} = struct('re', real(ev_ol(ei)), 'im', imag(ev_ol(ei))); %#ok<AGROW>
    end
    jrun.ev_ol = ev_arr;

    % Disturbances
    jrun.disturbance = c.disturbance;

    % Controllers
    keys = fieldnames(r.runs);
    ctrl_arr = {};
    for ki = 1:numel(keys)
        key  = keys{ki};
        run  = r.runs.(key);

        % Downsample trajectory for the chart (max 500 points)
        t_full = run.t;
        x_full = run.x;
        n_pts  = numel(t_full);
        skip   = max(1, floor(n_pts / 500));
        t_ds   = t_full(1:skip:end);
        x_ds   = x_full(1:skip:end, :);

        % Replace NaN with null-safe value for JSON
        phi_ds   = rad2deg(x_ds(:,1));
        delta_ds = rad2deg(x_ds(:,2));
        phi_ds(isnan(phi_ds))     = 999;   % sentinel: indicates failure
        delta_ds(isnan(delta_ds)) = 999;

        % Closed-loop eigenvalues (4 values)
        ev_cl = run.ev_cl;
        ev_cl_arr = {};
        for ei = 1:min(numel(ev_cl),8)
            ev_cl_arr{end+1} = struct('re',real(ev_cl(ei)),'im',imag(ev_cl(ei))); %#ok<AGROW>
        end

        ctrl = struct();
        ctrl.key         = key;
        ctrl.label       = run.label;
        ctrl.k_info      = run.k_info;
        ctrl.failed      = run.failed;
        ctrl.fail_time   = run.fail_time;
        ctrl.fail_msg    = run.fail_msg;
        ctrl.settle_time = run.settle_time;
        ctrl.max_re_cl   = max(real(ev_cl));
        ctrl.saturated   = run.sat_info.saturated;
        ctrl.sat_count   = run.sat_info.count;
        ctrl.sat_first_t = run.sat_info.first_t;
        ctrl.ev_cl       = ev_cl_arr;

        % Downsampled time series (for the inline chart)
        ctrl.t     = t_ds(:)';
        ctrl.phi   = phi_ds(:)';
        ctrl.delta = delta_ds(:)';

        % Asset paths (relative to project root)
        fig_dir  = fullfile(rdir, 'figures');
        anim_dir = fullfile(rdir, 'animation');
        ctrl.assets = struct( ...
            'response_png', bld_path(fig_dir,  [key '_response.png']), ...
            'response_gif', bld_path(fig_dir,  [key '_response.gif']), ...
            'anim_gif',     bld_path(anim_dir, [key '_animation.gif']));

        ctrl_arr{end+1} = ctrl; %#ok<AGROW>
    end
    jrun.controllers = ctrl_arr;

    % Combined figure path
    jrun.combined_png = bld_path(fullfile(rdir,'figures'),'combined_comparison.png');
    jrun.params_png   = bld_path(fullfile(rdir,'figures'),'00_parameters.png');

    runs_json{end+1} = jrun; %#ok<AGROW>
end

if isempty(runs_json)
    error('No valid runs found.');
end

% ---- Serialise to JSON --------------------------------------------------
data = struct();
data.schema        = 'bike-lean-steer-control/1.0';
data.generated_utc = char(datetime('now','TimeZone','UTC','Format', ...
                           'yyyy-MM-dd HH:mm:ss')) + " UTC";
data.runs          = runs_json;

json_str = jsonencode(data, 'PrettyPrint', true);

% Write as a JS module so it works on file:// without CORS issues
fid = fopen('report_data.js', 'w');
fprintf(fid, '/* Auto-generated by bc_build_report.m — do not edit */\n');
fprintf(fid, 'window.BIKE_REPORT_DATA = %s;\n', json_str);
fclose(fid);

fprintf('\n[bc_build_report] Done.\n');
fprintf('  report_data.js written (%d run(s))\n', numel(runs_json));
fprintf('  Open report.html in a browser to view the report.\n\n');

% ---- Try to open the browser (best-effort) ------------------------------
html_path = fullfile(pwd, 'report.html');
if isfile(html_path)
    try
        if ispc
            system(sprintf('start "" "%s"', html_path));
        elseif ismac
            system(sprintf('open "%s"', html_path));
        else
            system(sprintf('xdg-open "%s" &', html_path));
        end
    catch
        fprintf('  (could not auto-open browser — open report.html manually)\n');
    end
end

end

% ---- Helper: relative path, empty string if file doesn't exist ----------
function s = bld_path(folder, fname)
    full = fullfile(folder, fname);
    if isfile(full)
        % Convert to forward-slash relative path for HTML
        s = strrep(full, '\', '/');
        % Make relative to cwd
        cwd = strrep(pwd, '\', '/');
        if startsWith(s, cwd)
            s = s(length(cwd)+2:end);  % strip leading cwd + separator
        end
    else
        s = '';
    end
end
