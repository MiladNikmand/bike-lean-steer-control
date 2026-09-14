% CHECK_SETUP
% Verifies that all required files are present in the current folder
% before committing to a run. Mirrors check_packages.m from the
% wind-turbine-optimal-control-ml project.
%
% Usage:
%   check_setup          % prints a pass/fail report
%   ok = check_setup()   % returns true if everything is present

function ok = check_setup()

required = {
    % Core model
    'bike_params.m',                'Physical parameters (Whipple-Carvallo benchmark)'
    'compute_benchmark_matrices.m', 'Canonical M, C1, K0, K2 matrices'
    'bike_state_space.m',           'State-space A, B at a given speed'
    % Analysis scripts
    'stability_analysis.m',         'Eigenvalue sweep + uncontrolled simulation'
    'balance_controller.m',         'Rate-feedback gain selection + closed-loop'
    'param_sensitivity.m',          'Geometry/mass sensitivity of stable band'
    'response_analysis.m',          'Perturbations + controller strategy comparison'
    'save_results.m',               'Batch result generation (.mat files)'
    % Interactive pipeline
    'bike_launch.m',                'GUI entry point'
    'bc_run_analysis.m',            'Simulation engine'
    'bc_export_figures.m',          'Figure and GIF export'
    'bc_disturbance.m',             'Disturbance generator'
    'bc_write_summary.m',           'JSON + README writer (shared by both entry points)'
    'bc_build_report.m',            'Generates report_data.js for the HTML report'
    'animate_bike_v2.m',            '3D animation viewer'
    % Batch entry point
    'run_all.m',                    'Non-interactive batch driver'
};

n   = size(required, 1);
ok  = true;
w   = 38;   % column width for alignment

fprintf('\n');
fprintf('  %-*s  %s\n', w, 'File', 'Status');
fprintf('  %-*s  %s\n', w, repmat('-',1,w), '--------');

for i = 1:n
    fname  = required{i,1};
    desc   = required{i,2};
    exists = isfile(fname);
    if exists
        tag = 'OK';
    else
        tag = 'MISSING';
        ok  = false;
    end
    fprintf('  %-*s  [%s]  %s\n', w, fname, tag, desc);
end

fprintf('\n');
if ok
    fprintf('  All files present. Ready to run.\n');
    fprintf('  >> bike_launch      (interactive GUI)\n');
    fprintf('  >> run_all          (silent batch)\n');
else
    fprintf('  One or more files are missing.\n');
    fprintf('  Make sure you are running from the project root:\n');
    fprintf('  >> cd path/to/bike-lean-steer-control\n');
end
fprintf('\n');

if nargout == 0
    clear ok;
end

end
