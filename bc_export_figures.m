function bc_export_figures(run_dir, results, opts)
% BC_EXPORT_FIGURES  Write all figures and GIFs for one bike control run.
% Mirrors the asset layout of export_control.m from the wind-turbine project:
%   run_dir/figures/   — static PNGs (full + zoomed)
%   run_dir/animation/ — animated GIFs per controller
%
% Inputs:
%   run_dir  — path to Bike_Control_Run_NNN/
%   results  — struct from bc_run_analysis
%   opts     — export options:
%     .make_gifs    logical  (default true)
%     .gif_fps      fps      (default 8)
%     .gif_frames   max frames per gif  (default 120)
%     .dpi          PNG resolution      (default 150)
%     .gif_px       [width height]      (default [1200 500])

if nargin < 3 || isempty(opts), opts = struct(); end
opts = bef_def(opts, 'make_gifs',   true);
opts = bef_def(opts, 'gif_fps',     8);
opts = bef_def(opts, 'gif_frames',  120);
opts = bef_def(opts, 'dpi',         150);
opts = bef_def(opts, 'gif_px',      [1200 500]);

fig_dir  = fullfile(run_dir, 'figures');
anim_dir = fullfile(run_dir, 'animation');
for d = {fig_dir, anim_dir}
    if ~exist(d{1},'dir'), mkdir(d{1}); end
end

keys     = fieldnames(results.runs);
n_ctrl   = numel(keys);
colors   = lines(n_ctrl);
col_map  = containers.Map(keys, num2cell(colors, 2));

fprintf('[bc_export_figures] Exporting to %s\n', run_dir);

% =========================================================================
%  FIGURE 0 — Parameters overview
% =========================================================================
bef_fig0_params(fig_dir, results, opts);

% =========================================================================
%  Per-controller: static PNG + GIF
% =========================================================================
for ki = 1:n_ctrl
    key = keys{ki};
    run = results.runs.(key);
    clr = col_map(key);

    fprintf('  [%d/%d] %s ...\n', ki, n_ctrl, key);

    % --- Static figure ---
    fig = figure('Visible','off','Color','w', ...
                 'Position',[100 100 opts.gif_px(1) opts.gif_px(2)]);
    bef_draw_response(fig, results, run, clr, false);
    png_path = fullfile(fig_dir, [key '_response.png']);
    exportgraphics(fig, png_path, 'Resolution', opts.dpi);
    close(fig);

    % --- GIF ---
    if opts.make_gifs
        gif_path = fullfile(fig_dir, [key '_response.gif']);
        bef_make_gif(gif_path, results, run, clr, opts);
    end
end

% =========================================================================
%  Combined comparison figure
% =========================================================================
fprintf('  Combined comparison ...\n');
bef_fig_combined(fig_dir, results, keys, col_map, opts);

fprintf('[bc_export_figures] Done.\n\n');
end

% =========================================================================
%  DRAWING HELPERS
% =========================================================================

function bef_fig0_params(fig_dir, results, opts)
% One-page parameter summary drawn entirely with axes (no uitable —
% uitable is incompatible with Visible=off / exportgraphics).
    p   = results.p;
    fig = figure('Visible','off','Color','w','Position',[100 100 960 560]);

    % Left panel: parameter table drawn as text
    ax1 = axes('Parent',fig,'Position',[0.03 0.05 0.46 0.88]);
    axis(ax1,'off');
    xlim(ax1,[0 1]); ylim(ax1,[0 1]);

    params_data = {
        'Forward speed',         sprintf('%.2f m/s',  results.v)
        'Rider + frame mass',    sprintf('%.1f kg',   p.mB)
        'Rear wheel radius',     sprintf('%.3f m',    p.rR)
        'Front wheel radius',    sprintf('%.3f m',    p.rF)
        'Wheelbase',             sprintf('%.3f m',    p.w)
        'Trail',                 sprintf('%.3f m',    p.c)
        'Steer axis tilt',       sprintf('%.2f deg',  rad2deg(p.lambda))
        'Initial phi_0',         sprintf('%.1f deg',  rad2deg(results.x0(1)))
        'Initial delta_0',       sprintf('%.1f deg',  rad2deg(results.x0(2)))
        'Regime',                results.regime
        'Max Re(eig) OL',        sprintf('%+.4f',     results.max_re)
    };
    n_rows = size(params_data,1);

    % Header
    text(ax1, 0.02, 0.97, 'Parameter', 'FontWeight','bold','FontSize',10.5, ...
         'Units','normalized','VerticalAlignment','top');
    text(ax1, 0.60, 0.97, 'Value',     'FontWeight','bold','FontSize',10.5, ...
         'Units','normalized','VerticalAlignment','top');
    line(ax1,[0 1],[0.94 0.94],'Color',[0.4 0.4 0.4],'LineWidth',1.2);

    row_h = 0.88 / n_rows;
    for ri = 1:n_rows
        y = 0.93 - ri*row_h;
        bg = mod(ri,2)==0;
        if bg
            patch(ax1,[0 1 1 0],[y y y+row_h y+row_h],[0.95 0.95 0.95], ...
                  'EdgeColor','none','FaceAlpha',0.6);
        end
        text(ax1, 0.02, y+row_h*0.35, params_data{ri,1}, ...
             'FontSize',10,'Interpreter','none');
        text(ax1, 0.60, y+row_h*0.35, params_data{ri,2}, ...
             'FontSize',10,'FontWeight','bold','Interpreter','none');
    end
    line(ax1,[0 1],[0.93-n_rows*row_h 0.93-n_rows*row_h], ...
         'Color',[0.4 0.4 0.4],'LineWidth',0.8);
    title(ax1,'Run Parameters','FontSize',12,'FontWeight','bold');

    % Right panel: eigenvalue plot
    ax2 = axes('Parent',fig,'Position',[0.56 0.12 0.40 0.74]);
    ev  = results.ev_ol;
    plot(ax2, real(ev), imag(ev), 'rx', 'MarkerSize', 12, 'LineWidth', 2.5);
    hold(ax2,'on');
    xline(ax2, 0, 'k--', 'LineWidth', 1.0);
    grid(ax2,'on'); box(ax2,'on');
    xlabel(ax2,'Re(\lambda)  [1/s]','Interpreter','tex');
    ylabel(ax2,'Im(\lambda)  [1/s]','Interpreter','tex');
    title(ax2, sprintf('Open-loop eigenvalues\nv = %.1f m/s  [%s]', ...
                       results.v, results.regime), 'FontSize',11);

    exportgraphics(fig, fullfile(fig_dir,'00_parameters.png'), ...
                   'Resolution', opts.dpi);
    close(fig);
    fprintf('  00_parameters.png\n');
end

function bef_draw_response(fig, results, run, clr, is_gif_frame)
% Draws the 4-panel response figure for one controller run.
    phi_lim_deg   = results.cfg.phi_limit_deg;
    delta_lim_deg = results.cfg.delta_limit_deg;
    t  = run.t;
    x  = run.x;
    n  = numel(t);

    phi_deg   = rad2deg(x(:,1));
    delta_deg = rad2deg(x(:,2));
    phidot    = x(:,3);
    deltadot  = x(:,4);

    % Steer torque (if gain row available)
    if any(run.f_row ~= 0)
        torque = (run.f_row * x')';
    else
        torque = zeros(n,1);
    end

    % Axis scale helper
    function yl = smart_ylim(vals)
        v = vals(isfinite(vals));
        if isempty(v), yl = [-1 1]; return; end
        lo = min(v); hi = max(v);
        pad = 0.12*max(hi-lo, 1e-3);
        yl = [lo-pad, hi+pad];
    end

    % Build plain-text title (strip TeX sequences for sgtitle)
    plain_label = regexprep(run.label, '\\[a-zA-Z_{}^]+', '');
    title_parts = {plain_label, run.k_info};
    if run.failed
        title_parts{end+1} = sprintf('FAILED at t=%.2fs: %s', ...
                                     run.fail_time, run.fail_msg);
    end
    title_str = strjoin(title_parts(~cellfun(@isempty, title_parts)), '  |  ');

    % Panel 1: phi
    ax1 = subplot(2,2,1,'Parent',fig);
    plot(ax1, t, phi_deg,  'Color', clr, 'LineWidth', 1.5); hold(ax1,'on');
    yline(ax1,  phi_lim_deg, 'r--', 'LineWidth',0.9);
    yline(ax1, -phi_lim_deg, 'r--', 'LineWidth',0.9);
    yline(ax1, 0, 'k:', 'LineWidth',0.8);
    bef_fail_line(ax1, run);
    bef_disturbance_markers(ax1, results.cfg.disturbance, t(end));
    ylim(ax1, smart_ylim([phi_deg; phi_lim_deg; -phi_lim_deg]));
    xlabel(ax1,'Time (s)'); ylabel(ax1,'\phi  (deg)');
    title(ax1,'Roll angle'); grid(ax1,'on');

    % Panel 2: delta — with saturation clamping band
    ax2 = subplot(2,2,2,'Parent',fig);
    plot(ax2, t, delta_deg, 'Color', clr*0.7, 'LineWidth', 1.5); hold(ax2,'on');
    yline(ax2,  delta_lim_deg, 'r--', 'LineWidth', 1.2, ...
          'Label', sprintf('+%.0f deg (sat)',  delta_lim_deg), ...
          'LabelHorizontalAlignment','left');
    yline(ax2, -delta_lim_deg, 'r--', 'LineWidth', 1.2, ...
          'Label', sprintf('-%.0f deg (sat)', delta_lim_deg), ...
          'LabelHorizontalAlignment','left');
    yline(ax2, 0, 'k:', 'LineWidth', 0.8);
    % Shade the saturated region
    yl2 = [min(-delta_lim_deg-5, min(delta_deg)-5), ...
           max( delta_lim_deg+5, max(delta_deg(isfinite(delta_deg)))+5)];
    bef_fail_line(ax2, run);
    bef_disturbance_markers(ax2, results.cfg.disturbance, t(end));
    % Mark saturation periods (where |delta| is at the limit)
    if isfield(run,'sat_info') && run.sat_info.saturated
        sat_mask = abs(delta_deg) >= delta_lim_deg - 0.05;
        if any(sat_mask)
            % Draw a thicker overlay on the clamped sections
            t_sat = t; t_sat(~sat_mask) = NaN;
            d_sat = delta_deg; d_sat(~sat_mask) = NaN;
            plot(ax2, t_sat, d_sat, 'Color', [0.9 0.3 0.1], ...
                 'LineWidth', 3.0, 'DisplayName', 'saturated');
        end
        if ~isnan(run.sat_info.first_t)
            xline(ax2, run.sat_info.first_t, 'm:', 'LineWidth', 1.2, ...
                  'Label', sprintf('sat onset t=%.2fs', run.sat_info.first_t), ...
                  'HandleVisibility','off');
        end
    end
    xlabel(ax2,'Time (s)'); ylabel(ax2,'\delta  (deg)');
    title(ax2,'Steer angle  (dashed = actuator limit)'); grid(ax2,'on');

    % Panel 3: angular rates
    ax3 = subplot(2,2,3,'Parent',fig);
    plot(ax3, t, rad2deg(phidot),  'Color', clr,     'LineWidth',1.3, ...
         'DisplayName','\phi_{dot}'); hold(ax3,'on');
    plot(ax3, t, rad2deg(deltadot),'Color', clr*0.6, 'LineWidth',1.3, ...
         'DisplayName','\delta_{dot}','LineStyle','--');
    yline(ax3, 0,'k:','LineWidth',0.8);
    bef_fail_line(ax3, run);
    xlabel(ax3,'Time (s)'); ylabel(ax3,'Rate  (deg/s)');
    title(ax3,'Angular rates');
    legend(ax3,'Location','northeast'); grid(ax3,'on');

    % Panel 4: steer torque
    ax4 = subplot(2,2,4,'Parent',fig);
    if any(run.f_row ~= 0)
        plot(ax4, t, torque, 'Color', [0.4 0.4 0.4], 'LineWidth',1.4);
    else
        plot(ax4, t, zeros(size(t)), 'Color',[0.6 0.6 0.6],'LineWidth',1.0);
    end
    yline(ax4, 0,'k:','LineWidth',0.8);
    bef_fail_line(ax4, run);
    xlabel(ax4,'Time (s)'); ylabel(ax4,'T_\delta  (N\cdotm)');
    title(ax4,'Steer torque'); grid(ax4,'on');

    % Suptitle with failure watermark
    if run.failed
        annotation(fig,'textbox',[0.15 0.44 0.7 0.12], ...
            'String', sprintf('BIKE FELL -- %s', run.fail_msg), ...
            'FontSize',12,'FontWeight','bold','Color',[0.8 0 0], ...
            'HorizontalAlignment','center','EdgeColor',[0.8 0 0], ...
            'BackgroundColor',[1 0.92 0.92]);
    elseif isfield(run,'sat_info') && run.sat_info.saturated
        annotation(fig,'textbox',[0.15 0.44 0.7 0.06], ...
            'String', sprintf('ACTUATOR SATURATED x%d times (first at t=%.2fs) -- run continued', ...
                              run.sat_info.count, run.sat_info.first_t), ...
            'FontSize',10,'FontWeight','bold','Color',[0.7 0.4 0], ...
            'HorizontalAlignment','center','EdgeColor',[0.7 0.4 0], ...
            'BackgroundColor',[1 0.97 0.88]);
    end

    if ~is_gif_frame
        sgtitle(fig, title_str, 'Interpreter','none','FontSize',11);
    end
end

function bef_fig_combined(fig_dir, results, keys, col_map, opts)
% Combined figure: phi and delta for all controllers on one plot.
    n_ctrl = numel(keys);
    fig    = figure('Visible','off','Color','w', ...
                    'Position',[100 100 opts.gif_px(1) opts.gif_px(2)]);

    ax_phi = subplot(2,1,1,'Parent',fig);
    ax_del = subplot(2,1,2,'Parent',fig);
    hold(ax_phi,'on'); hold(ax_del,'on');

    phi_lim_deg   = results.cfg.phi_limit_deg;
    delta_lim_deg = results.cfg.delta_limit_deg;

    for ki = 1:n_ctrl
        key = keys{ki};
        run = results.runs.(key);
        clr = col_map(key);
        t   = run.t;
        phi_deg   = rad2deg(run.x(:,1));
        delta_deg = rad2deg(run.x(:,2));
        ls = '-';
        if run.failed, ls = '--'; end

        plot(ax_phi, t, phi_deg,   'Color', clr, 'LineWidth', 1.6, ...
             'LineStyle', ls, 'DisplayName', key);
        plot(ax_del, t, delta_deg, 'Color', clr, 'LineWidth', 1.6, ...
             'LineStyle', ls, 'DisplayName', key);
    end

    yline(ax_phi,  phi_lim_deg,   'r:', 'LineWidth',0.9,'HandleVisibility','off');
    yline(ax_phi, -phi_lim_deg,   'r:', 'LineWidth',0.9,'HandleVisibility','off');
    yline(ax_del,  delta_lim_deg, 'r:', 'LineWidth',0.9,'HandleVisibility','off');
    yline(ax_del, -delta_lim_deg, 'r:', 'LineWidth',0.9,'HandleVisibility','off');
    yline(ax_phi, 0, 'k:', 'LineWidth', 0.8, 'HandleVisibility','off');
    yline(ax_del, 0, 'k:', 'LineWidth', 0.8, 'HandleVisibility','off');

    bef_disturbance_markers(ax_phi, results.cfg.disturbance, results.tspan(end));
    bef_disturbance_markers(ax_del, results.cfg.disturbance, results.tspan(end));

    xlabel(ax_phi,'Time (s)'); ylabel(ax_phi,'\phi  (deg)');
    title(ax_phi,'Roll angle - all controllers');
    legend(ax_phi,'Location','northeast'); grid(ax_phi,'on');

    xlabel(ax_del,'Time (s)'); ylabel(ax_del,'\delta  (deg)');
    title(ax_del,'Steer angle - all controllers');
    legend(ax_del,'Location','northeast'); grid(ax_del,'on');

    sgtitle(fig, sprintf('Combined comparison  v = %.1f m/s  [%s]', ...
                         results.v, results.regime), 'Interpreter','none');

    exportgraphics(fig, fullfile(fig_dir,'combined_comparison.png'), ...
                   'Resolution', opts.dpi);
    close(fig);
    fprintf('  combined_comparison.png\n');
end

function bef_make_gif(gif_path, results, run, clr, opts)
% Animates the response figure frame by frame and writes a GIF.
    t       = run.t;
    n       = numel(t);
    skip    = max(1, floor(n / opts.gif_frames));
    frames  = 1:skip:n;
    ref_h   = []; ref_w = [];

    gfig = figure('Visible','off','Color','w', ...
                  'Position',[100 100 opts.gif_px(1) opts.gif_px(2)]);

    for fi = 1:numel(frames)
        k = frames(fi);
        clf(gfig);

        % Draw partial trajectory up to frame k
        run_partial     = run;
        run_partial.t   = run.t(1:k);
        run_partial.x   = run.x(1:k,:);
        bef_draw_response(gfig, results, run_partial, clr, true);
        % Use sgtitle for figure-level title; strip TeX sequences for plain display
        plain_label = regexprep(run.label, '\\[a-zA-Z_{}]+', '');
        sgtitle(gfig, sprintf('%s  (t = %.1f s)', plain_label, t(k)), ...
                'Interpreter','none','FontSize',11);

        cd = getframe(gfig); cd = cd.cdata;
        if isempty(ref_h), ref_h = size(cd,1); ref_w = size(cd,2); end
        cd = bef_lock_frame(cd, ref_h, ref_w);
        [im, cm] = rgb2ind(cd, 256, 'nodither');
        bef_write_gif(gif_path, im, cm, fi==1, opts.gif_fps);
    end
    close(gfig);
    fprintf('    GIF: %s (%d frames)\n', gif_path, numel(frames));
end

function bef_fail_line(ax, run)
    if run.failed && ~isnan(run.fail_time)
        xline(ax, run.fail_time, 'r-', 'LineWidth', 1.5, ...
              'Label','FAIL','HandleVisibility','off');
    end
end

function bef_disturbance_markers(ax, dcfg, t_end)
    if dcfg.kick_enabled && dcfg.kick_time <= t_end
        xline(ax, dcfg.kick_time, 'm--', 'LineWidth', 1.0, ...
              'Label','kick','HandleVisibility','off');
    end
    if dcfg.gravel_enabled && dcfg.gravel_start <= t_end
        x1 = dcfg.gravel_start;
        x2 = min(dcfg.gravel_end, t_end);
        yl = ylim(ax);
        % Use patch instead of xregion — works on all MATLAB versions
        patch(ax, [x1 x2 x2 x1], [yl(1) yl(1) yl(2) yl(2)], ...
              [0.9 0.8 0.5], 'FaceAlpha', 0.25, 'EdgeColor', 'none', ...
              'HandleVisibility', 'off');
        % Restore ylim in case patch rescaled it
        ylim(ax, yl);
    end
end

% =========================================================================
%  GIF UTILITIES  (mirrors export_control.m)
% =========================================================================
function bef_write_gif(path, im, cm, first, fps)
    if first
        imwrite(im,cm,path,'gif','Loopcount',inf,'DelayTime',1/fps);
    else
        imwrite(im,cm,path,'gif','WriteMode','append','DelayTime',1/fps);
    end
end

function out = bef_lock_frame(cd, ref_h, ref_w)
    [h,w,~] = size(cd);
    if h==ref_h && w==ref_w, out=cd; return; end
    out = cd(1:min(h,ref_h), 1:min(w,ref_w), :);
    if size(out,1)<ref_h
        out = cat(1,out,repmat(out(end,:,:),ref_h-size(out,1),1,1)); end
    if size(out,2)<ref_w
        out = cat(2,out,repmat(out(:,end,:),1,ref_w-size(out,2),1)); end
end

function o = bef_def(o, f, v)
    if ~isfield(o,f) || isempty(o.(f)), o.(f) = v; end
end