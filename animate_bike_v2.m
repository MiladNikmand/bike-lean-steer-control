% ANIMATE_BIKE_V2
% 3D stick-figure animation viewer for saved bike control runs.
% Scans for Bike_Control_Run_NNN/ folders, defaults to the latest,
% and animates each scenario (uncontrolled first, then each controller).
%
% At startup you are asked:
%   - Which run to load
%   - Whether to save GIFs  (yes -> writes to run_dir/animation/)
%
% Each saved GIF is the full side-by-side layout:
%   LEFT  — 3D stick-figure bike
%   RIGHT — live phi and delta traces
%
% One GIF per scenario, named  <key>_animation.gif
% e.g.  uncontrolled_animation.gif,  LQR_animation.gif, ...
%
% GIF export runs in a hidden off-screen figure so it does not
% interfere with the live window.

% =========================================================================
%  USER-TUNABLE PARAMETERS
% =========================================================================
anim_speed   = 1.5;    % live playback speed multiplier (1 = real time)
n_circle     = 60;     % wheel circle resolution
max_frames   = 600;    % max frames in the LIVE window (downsampled)
gif_max_frames = 120;  % max frames per saved GIF
gif_fps      = 10;     % frames per second in the saved GIF
gif_px       = [1400 700];  % pixel size of the saved GIF figure
% =========================================================================

% ---- Scan for run folders -----------------------------------------------
run_dirs = dir('Bike_Control_Run_*');
run_dirs = run_dirs([run_dirs.isdir]);

if isempty(run_dirs)
    error('No Bike_Control_Run_NNN folders found. Run bike_launch or run_all first.');
end

fprintf('\n==========================================================\n');
fprintf('  ANIMATE_BIKE_V2  -  Available runs\n');
fprintf('==========================================================\n\n');

for di = 1:numel(run_dirs)
    rdir  = run_dirs(di).name;
    jfile = fullfile(rdir,'run_summary.json');
    if isfile(jfile)
        try
            fid = fopen(jfile,'r'); raw = fread(fid,'*char')'; fclose(fid);
            J   = jsondecode(raw);
            fprintf('  [%d]  %-28s  v=%.1f m/s  [%s]\n', ...
                    di, rdir, J.speed_m_s, J.regime);
        catch
            fprintf('  [%d]  %s  (summary unreadable)\n', di, rdir);
        end
    else
        fprintf('  [%d]  %s  (no summary)\n', di, rdir);
    end
end

default_idx = numel(run_dirs);
fprintf('\nDefault: [%d] %s\n', default_idx, run_dirs(default_idx).name);
fprintf('Enter number (or press Enter for default): ');
raw_in = input('','s');
if isempty(strtrim(raw_in))
    chosen = default_idx;
else
    chosen = str2double(strtrim(raw_in));
    if isnan(chosen) || chosen<1 || chosen>numel(run_dirs)
        error('Invalid selection.');
    end
end

chosen_dir = run_dirs(chosen).name;
mat_file   = fullfile(chosen_dir,'results.mat');
if ~isfile(mat_file)
    error('results.mat not found in %s.', chosen_dir);
end

% ---- Ask about GIF saving -----------------------------------------------
fprintf('\nSave animation GIFs to %s/animation/? [y/n, default=y]: ', chosen_dir);
gif_ans = strtrim(input('','s'));
save_gifs = isempty(gif_ans) || strcmpi(gif_ans,'y') || strcmpi(gif_ans,'yes');

if save_gifs
    anim_dir = fullfile(chosen_dir,'animation');
    if ~exist(anim_dir,'dir'), mkdir(anim_dir); end
    fprintf('  GIFs will be saved to: %s\n', anim_dir);
    fprintf('  Settings: %d fps, max %d frames, %dx%d px\n\n', ...
            gif_fps, gif_max_frames, gif_px(1), gif_px(2));
else
    fprintf('  Skipping GIF export.\n\n');
end

% ---- Load results -------------------------------------------------------
fprintf('Loading %s ...\n', chosen_dir);
S       = load(mat_file,'results','cfg');
results = S.results;
cfg     = S.cfg;
p       = results.p;

fprintf('  v = %.2f m/s   regime: %s\n\n', results.v, results.regime);

% ---- Bike geometry constants --------------------------------------------
rR  = p.rR;  rF = p.rF;
wb  = p.w;   c  = p.c;   lam = p.lambda;
xB  = p.xB;  zB = p.zB;
xH  = p.xH;  zH = p.zH;
sa_dir = [-sin(lam); 0; cos(lam)];
th     = linspace(0, 2*pi, n_circle);

phi_lim_deg   = cfg.phi_limit_deg;
delta_lim_deg = cfg.delta_limit_deg;
dcfg          = cfg.disturbance;
t_end_plot    = results.tspan(end);

% ---- Build ordered scenario list ----------------------------------------
keys_all   = fieldnames(results.runs);
keys_order = [{'uncontrolled'}, keys_all(~strcmp(keys_all,'uncontrolled'))'];
scenarios  = {};
for ki = 1:numel(keys_order)
    key = keys_order{ki};
    if ~isfield(results.runs, key), continue; end
    run  = results.runs.(key);
    t_sc = run.t;
    x_sc = run.x;
    % Live window: downsample to max_frames
    skip_live = max(1, floor(numel(t_sc)/max_frames));
    % GIF: downsample to gif_max_frames
    skip_gif  = max(1, floor(numel(t_sc)/gif_max_frames));
    scenarios{end+1} = struct( ...   %#ok<AGROW>
        'key',       key, ...
        'label',     run.label, ...
        'clr',       abv2_color(ki), ...
        't_live',    t_sc(1:skip_live:end), ...
        'x_live',    x_sc(1:skip_live:end,:), ...
        't_gif',     t_sc(1:skip_gif:end), ...
        'x_gif',     x_sc(1:skip_gif:end,:), ...
        't_full',    t_sc, ...
        'x_full',    x_sc, ...
        'failed',    run.failed, ...
        'fail_time', run.fail_time);
end

% =========================================================================
%  LIVE FIGURE LAYOUT
% =========================================================================
fig = figure('Name', sprintf('Animation -- %s', chosen_dir), ...
             'Position',[50 50 1400 700], 'Color','w');

ax3d = subplot('Position',[0.02 0.05 0.50 0.88]);
hold(ax3d,'on'); grid(ax3d,'on'); axis(ax3d,'equal');
view(ax3d,[-35 18]);
xlim(ax3d,[-0.8 2.0]); ylim(ax3d,[-1.0 1.0]); zlim(ax3d,[-0.05 1.5]);
xlabel(ax3d,'x (forward, m)'); ylabel(ax3d,'y (lateral, m)');
zlabel(ax3d,'z (up, m)');
set(ax3d,'Color',[0.97 0.97 0.97]);

[Xg,Yg] = meshgrid([-0.8 2.0],[-1.0 1.0]);
surf(ax3d,Xg,Yg,zeros(2),'FaceColor',[0.78 0.78 0.78], ...
     'EdgeColor','none','FaceAlpha',0.5,'HandleVisibility','off');

ax_phi = subplot('Position',[0.57 0.55 0.40 0.38]);
hold(ax_phi,'on'); grid(ax_phi,'on');
xlabel(ax_phi,'Time (s)'); ylabel(ax_phi,'\phi (deg)','Interpreter','tex');
title(ax_phi,'Roll angle  \phi','Interpreter','tex');
yline(ax_phi, 0,'k:','LineWidth',0.8,'HandleVisibility','off');
yline(ax_phi,  phi_lim_deg,'r--','LineWidth',0.9,'HandleVisibility','off');
yline(ax_phi, -phi_lim_deg,'r--','LineWidth',0.9,'HandleVisibility','off');

ax_del = subplot('Position',[0.57 0.08 0.40 0.38]);
hold(ax_del,'on'); grid(ax_del,'on');
xlabel(ax_del,'Time (s)'); ylabel(ax_del,'\delta (deg)','Interpreter','tex');
title(ax_del,'Steer angle  \delta','Interpreter','tex');
yline(ax_del, 0,'k:','LineWidth',0.8,'HandleVisibility','off');
yline(ax_del,  delta_lim_deg,'r--','LineWidth',0.9,'HandleVisibility','off');
yline(ax_del, -delta_lim_deg,'r--','LineWidth',0.9,'HandleVisibility','off');

% Pre-draw faint full traces for all scenarios
for sc = 1:numel(scenarios)
    s = scenarios{sc};
    plot(ax_phi, s.t_full, rad2deg(s.x_full(:,1)), ...
         'Color',[s.clr 0.15],'LineWidth',0.8);
    plot(ax_del, s.t_full, rad2deg(s.x_full(:,2)), ...
         'Color',[s.clr*0.7 0.15],'LineWidth',0.8);
end

% Disturbance markers
abv2_draw_dist_markers(ax_phi, ax_del, dcfg, t_end_plot);

% =========================================================================
%  LOOP OVER SCENARIOS — LIVE PLAYBACK + GIF CAPTURE
% =========================================================================
for sc = 1:numel(scenarios)
    s     = scenarios{sc};
    clr   = s.clr;
    clr_d = clr * 0.65;

    plain_label = regexprep(s.label,'\\[a-zA-Z_{}^]+','');

    % ---- (A) GIF CAPTURE PASS (hidden figure, full resolution) ----------
    if save_gifs
        gif_path = fullfile(anim_dir, [s.key '_animation.gif']);
        fprintf('  Capturing GIF: %s  (%d frames) ...\n', ...
                [s.key '_animation.gif'], size(s.x_gif,1));

        gfig = figure('Visible','off','Color','w', ...
                      'Position',[50 50 gif_px(1) gif_px(2)]);

        gax3d = subplot('Position',[0.02 0.05 0.50 0.88],'Parent',gfig);
        hold(gax3d,'on'); grid(gax3d,'on'); axis(gax3d,'equal');
        view(gax3d,[-35 18]);
        xlim(gax3d,[-0.8 2.0]); ylim(gax3d,[-1.0 1.0]); zlim(gax3d,[-0.05 1.5]);
        xlabel(gax3d,'x (m)'); ylabel(gax3d,'y (m)'); zlabel(gax3d,'z (m)');
        set(gax3d,'Color',[0.97 0.97 0.97]);
        surf(gax3d,Xg,Yg,zeros(2),'FaceColor',[0.78 0.78 0.78], ...
             'EdgeColor','none','FaceAlpha',0.5,'HandleVisibility','off');

        gax_phi = subplot('Position',[0.57 0.55 0.40 0.38],'Parent',gfig);
        hold(gax_phi,'on'); grid(gax_phi,'on');
        xlabel(gax_phi,'Time (s)'); ylabel(gax_phi,'\phi (deg)','Interpreter','tex');
        title(gax_phi,'Roll angle  \phi','Interpreter','tex');
        yline(gax_phi, 0,'k:','LineWidth',0.8,'HandleVisibility','off');
        yline(gax_phi,  phi_lim_deg,'r--','LineWidth',1.0,'HandleVisibility','off');
        yline(gax_phi, -phi_lim_deg,'r--','LineWidth',1.0,'HandleVisibility','off');

        gax_del = subplot('Position',[0.57 0.08 0.40 0.38],'Parent',gfig);
        hold(gax_del,'on'); grid(gax_del,'on');
        xlabel(gax_del,'Time (s)'); ylabel(gax_del,'\delta (deg)','Interpreter','tex');
        title(gax_del,'Steer angle  \delta','Interpreter','tex');
        yline(gax_del, 0,'k:','LineWidth',0.8,'HandleVisibility','off');
        yline(gax_del,  delta_lim_deg,'r--','LineWidth',1.0,'HandleVisibility','off');
        yline(gax_del, -delta_lim_deg,'r--','LineWidth',1.0,'HandleVisibility','off');

        % Faint full trace in GIF figure
        plot(gax_phi, s.t_full, rad2deg(s.x_full(:,1)), ...
             'Color',[clr 0.15],'LineWidth',0.8);
        plot(gax_del, s.t_full, rad2deg(s.x_full(:,2)), ...
             'Color',[clr_d 0.15],'LineWidth',0.8);
        abv2_draw_dist_markers(gax_phi, gax_del, dcfg, t_end_plot);

        % GIF 3D handles
        gh = abv2_init_handles(gax3d, clr);
        gh_phi_line = plot(gax_phi,nan,nan,'Color',clr,  'LineWidth',2.0);
        gh_del_line = plot(gax_del,nan,nan,'Color',clr_d,'LineWidth',2.0);
        gh_phi_dot  = plot(gax_phi,nan,nan,'o','MarkerSize',7, ...
                           'MarkerFaceColor',clr,'MarkerEdgeColor','k');
        gh_del_dot  = plot(gax_del,nan,nan,'o','MarkerSize',7, ...
                           'MarkerFaceColor',clr_d,'MarkerEdgeColor','k');

        ref_h = []; ref_w = [];
        phi_all_g = rad2deg(s.x_gif(:,1));
        del_all_g = rad2deg(s.x_gif(:,2));
        n_gif     = size(s.x_gif,1);

        for k = 1:n_gif
            phi_k   = s.x_gif(k,1);
            delta_k = s.x_gif(k,2);
            t_k     = s.t_gif(k);

            if isnan(phi_k)
                % FAILED frame — stamp watermark and hold for a few frames
                text(gax3d, 0.5, 0, 1.3, 'CONTROLLER FAILED', ...
                     'Color',[0.85 0.1 0.1],'FontSize',16, ...
                     'FontWeight','bold','HorizontalAlignment','center');
                sgtitle(gfig, sprintf('%s | %s | FAILED', ...
                        chosen_dir, plain_label), ...
                        'Interpreter','none','FontSize',10);
                cd = getframe(gfig); cd = cd.cdata;
                if isempty(ref_h), ref_h=size(cd,1); ref_w=size(cd,2); end
                cd = abv2_lock(cd,ref_h,ref_w);
                [im,cm] = rgb2ind(cd,256,'nodither');
                for rep = 1:5   % hold failed frame for 0.5s
                    abv2_write_gif(gif_path,im,cm,k==1,gif_fps);
                end
                break;
            end

            abv2_update_geom(gax3d,gh,phi_k,delta_k, ...
                             rR,rF,wb,c,sa_dir,th,n_circle, ...
                             xB,zB,xH,zH,p);

            set(gh_phi_line,'XData',s.t_gif(1:k),'YData',phi_all_g(1:k));
            set(gh_del_line,'XData',s.t_gif(1:k),'YData',del_all_g(1:k));
            set(gh_phi_dot,'XData',t_k,'YData',phi_all_g(k));
            set(gh_del_dot,'XData',t_k,'YData',del_all_g(k));

            sgtitle(gfig, sprintf('%s | %s | t = %.1f s', ...
                    chosen_dir, plain_label, t_k), ...
                    'Interpreter','none','FontSize',10);

            cd = getframe(gfig); cd = cd.cdata;
            if isempty(ref_h), ref_h=size(cd,1); ref_w=size(cd,2); end
            cd = abv2_lock(cd,ref_h,ref_w);
            [im,cm] = rgb2ind(cd,256,'nodither');
            abv2_write_gif(gif_path,im,cm,k==1,gif_fps);
        end
        close(gfig);
        fprintf('    -> saved: %s\n', gif_path);
    end

    % ---- (B) LIVE PLAYBACK PASS -----------------------------------------
    n_live    = size(s.x_live,1);
    phi_all_l = rad2deg(s.x_live(:,1));
    del_all_l = rad2deg(s.x_live(:,2));
    dt_real   = (s.t_live(end)-s.t_live(1)) / n_live / anim_speed;

    cla(ax3d);
    surf(ax3d,Xg,Yg,zeros(2),'FaceColor',[0.78 0.78 0.78], ...
         'EdgeColor','none','FaceAlpha',0.5,'HandleVisibility','off');
    title(ax3d, sprintf('%s  |  %s', chosen_dir, plain_label), ...
          'FontSize',11,'Interpreter','none');

    lh = abv2_init_handles(ax3d, clr);
    h_fail_txt = text(ax3d,0.5,0,1.3,'','Color',[0.85 0.1 0.1], ...
                      'FontSize',14,'FontWeight','bold', ...
                      'HorizontalAlignment','center','Visible','off');
    lh_phi_line = plot(ax_phi,nan,nan,'Color',clr,  'LineWidth',2.2);
    lh_del_line = plot(ax_del,nan,nan,'Color',clr_d,'LineWidth',2.2);
    lh_phi_dot  = plot(ax_phi,nan,nan,'o','MarkerSize',8, ...
                       'MarkerFaceColor',clr,'MarkerEdgeColor','k');
    lh_del_dot  = plot(ax_del,nan,nan,'o','MarkerSize',8, ...
                       'MarkerFaceColor',clr_d,'MarkerEdgeColor','k');

    failed_shown = false;
    for k = 1:n_live
        phi_k   = s.x_live(k,1);
        delta_k = s.x_live(k,2);

        if isnan(phi_k)
            if ~failed_shown
                set(h_fail_txt,'String','CONTROLLER FAILED','Visible','on');
                failed_shown = true;
                drawnow; pause(1.0);
            end
            break;
        end

        abv2_update_geom(ax3d,lh,phi_k,delta_k, ...
                         rR,rF,wb,c,sa_dir,th,n_circle, ...
                         xB,zB,xH,zH,p);

        set(lh_phi_line,'XData',s.t_live(1:k),'YData',phi_all_l(1:k));
        set(lh_del_line,'XData',s.t_live(1:k),'YData',del_all_l(1:k));
        set(lh_phi_dot, 'XData',s.t_live(k),  'YData',phi_all_l(k));
        set(lh_del_dot, 'XData',s.t_live(k),  'YData',del_all_l(k));

        drawnow limitrate;
        pause(dt_real);
    end

    % Pause between scenarios
    if sc < numel(scenarios)
        title(ax3d, sprintf('%s  |  %s  -- next up: %s', ...
              chosen_dir, plain_label, ...
              regexprep(scenarios{sc+1}.label,'\\[a-zA-Z_{}^]+','')), ...
              'FontSize',10,'Interpreter','none');
        drawnow; pause(1.5);
    end
end

title(ax3d, sprintf('%s  |  Animation complete', chosen_dir),'FontSize',12);
if save_gifs
    fprintf('\nAll GIFs saved to: %s\n', fullfile(chosen_dir,'animation'));
end
fprintf('Animation complete.\n\n');

% =========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

function h = abv2_init_handles(ax, clr)
% Create all 3D line/marker handles initialised to NaN
    h.rw    = plot3(ax,nan,nan,nan,'Color',[0.15 0.15 0.15],'LineWidth',2.5);
    h.fw    = plot3(ax,nan,nan,nan,'Color',[0.15 0.15 0.15],'LineWidth',2.5);
    h.raxle = plot3(ax,nan,nan,nan,'Color',[0.40 0.40 0.40],'LineWidth',1.2);
    h.faxle = plot3(ax,nan,nan,nan,'Color',[0.40 0.40 0.40],'LineWidth',1.2);
    h.frame = plot3(ax,nan,nan,nan,'Color',clr,'LineWidth',4.0);
    h.fork  = plot3(ax,nan,nan,nan,'Color',clr,'LineWidth',2.8);
    h.seat  = plot3(ax,nan,nan,nan,'Color',[0.30 0.30 0.30],'LineWidth',2.0);
    h.hbar  = plot3(ax,nan,nan,nan,'Color',[0.25 0.25 0.25],'LineWidth',3.5);
    h.sa    = plot3(ax,nan,nan,nan,'--','Color',[0.65 0.65 0.65],'LineWidth',1.0);
    h.com   = plot3(ax,nan,nan,nan,'o','MarkerSize',16, ...
                    'MarkerFaceColor',clr,'MarkerEdgeColor','k','LineWidth',1.5);
end

function abv2_update_geom(ax,h,phi_k,delta_k, ...
                           rR,rF,wb,c,sa_dir,th,n_circle, ...
                           xB,zB,xH,zH,p)
% Update all 3D handles for the current phi and delta.
    Rphi   = [1,0,0; 0,cos(phi_k),-sin(phi_k); 0,sin(phi_k),cos(phi_k)];
    Rdelta = abv2_rodrigues(sa_dir, delta_k);

    P_rw      = [0;  0;  rR];
    P_com_loc = [xB; 0; -zB];
    P_head    = [xH; 0; -zH];
    P_sa_bot  = [wb+c; 0; 0];
    P_sa_top  = P_sa_bot + 1.15*sa_dir;
    P_fw_loc  = [wb; 0; rF];
    P_fw_s    = P_sa_bot + Rdelta*(P_fw_loc - P_sa_bot);
    P_hbar_c  = [xH; 0; -zH+0.12];
    P_hbar_s  = P_sa_bot + Rdelta*(P_hbar_c - P_sa_bot);
    P_hbar_L  = P_hbar_s + Rdelta*[0; -0.20; 0];
    P_hbar_R  = P_hbar_s + Rdelta*[0;  0.20; 0];
    P_seat_lo = P_com_loc + [0;0;-0.22];
    P_seat_hi = P_com_loc + [0;0; 0.15];

    pts = {P_rw,P_com_loc,P_head,P_sa_bot,P_sa_top, ...
           P_fw_s,P_hbar_s,P_hbar_L,P_hbar_R,P_seat_lo,P_seat_hi};
    pts_r = cellfun(@(q) Rphi*q, pts, 'UniformOutput',false);
    [P_rw_r,P_com_r,P_head_r,P_sa_bot_r,P_sa_top_r, ...
     P_fw_r,P_hbar_r,P_hbar_L_r,P_hbar_R_r, ...
     P_seat_lo_r,P_seat_hi_r] = pts_r{:};

    % Rear wheel circle
    rw_circ = rR*[cos(th);zeros(1,n_circle);sin(th)];
    rw_circ = Rphi*rw_circ + P_rw_r;

    % Front wheel circle (steered plane)
    lat_s   = Rdelta*[0;1;0];
    fwd_s   = cross(lat_s,[0;0;1]); fwd_s = fwd_s/norm(fwd_s);
    fw_circ = rF*(fwd_s*cos(th)+[0;0;1]*sin(th)) + P_fw_loc;
    fw_circ = Rphi*fw_circ;

    lat_r   = Rphi*[0;1;0];
    rw_axle = P_rw_r + 0.12*lat_r*[-1,1];
    fw_axle = P_fw_r + 0.12*lat_r*[-1,1];

    set(h.rw,   'XData',rw_circ(1,:),'YData',rw_circ(2,:),'ZData',rw_circ(3,:));
    set(h.fw,   'XData',fw_circ(1,:),'YData',fw_circ(2,:),'ZData',fw_circ(3,:));
    set(h.raxle,'XData',rw_axle(1,:),'YData',rw_axle(2,:),'ZData',rw_axle(3,:));
    set(h.faxle,'XData',fw_axle(1,:),'YData',fw_axle(2,:),'ZData',fw_axle(3,:));
    set(h.frame,'XData',[P_rw_r(1),P_com_r(1),P_head_r(1)], ...
                'YData',[P_rw_r(2),P_com_r(2),P_head_r(2)], ...
                'ZData',[P_rw_r(3),P_com_r(3),P_head_r(3)]);
    set(h.fork, 'XData',[P_head_r(1),P_fw_r(1)], ...
                'YData',[P_head_r(2),P_fw_r(2)], ...
                'ZData',[P_head_r(3),P_fw_r(3)]);
    set(h.seat, 'XData',[P_seat_lo_r(1),P_seat_hi_r(1)], ...
                'YData',[P_seat_lo_r(2),P_seat_hi_r(2)], ...
                'ZData',[P_seat_lo_r(3),P_seat_hi_r(3)]);
    set(h.hbar, 'XData',[P_hbar_L_r(1),P_hbar_R_r(1)], ...
                'YData',[P_hbar_L_r(2),P_hbar_R_r(2)], ...
                'ZData',[P_hbar_L_r(3),P_hbar_R_r(3)]);
    set(h.sa,   'XData',[P_sa_bot_r(1),P_sa_top_r(1)], ...
                'YData',[P_sa_bot_r(2),P_sa_top_r(2)], ...
                'ZData',[P_sa_bot_r(3),P_sa_top_r(3)]);
    set(h.com,  'XData',P_com_r(1),'YData',P_com_r(2),'ZData',P_com_r(3));
end

function abv2_draw_dist_markers(ax_phi, ax_del, dcfg, t_end)
    if isfield(dcfg,'kick_enabled') && dcfg.kick_enabled && dcfg.kick_time<=t_end
        xline(ax_phi,dcfg.kick_time,'m--','LineWidth',1.0,'Label','kick', ...
              'HandleVisibility','off');
        xline(ax_del,dcfg.kick_time,'m--','LineWidth',1.0,'HandleVisibility','off');
    end
    if isfield(dcfg,'gravel_enabled') && dcfg.gravel_enabled
        x1 = dcfg.gravel_start; x2 = min(dcfg.gravel_end,t_end);
        for ax_g = [ax_phi, ax_del]
            yl = ylim(ax_g);
            patch(ax_g,[x1 x2 x2 x1],[yl(1) yl(1) yl(2) yl(2)], ...
                  [0.9 0.8 0.5],'FaceAlpha',0.22,'EdgeColor','none', ...
                  'HandleVisibility','off');
            ylim(ax_g,yl);
        end
    end
    if isfield(dcfg,'wind_enabled') && dcfg.wind_enabled
        x1 = dcfg.wind_start;
        x2 = min(dcfg.wind_start+dcfg.wind_ramp+dcfg.wind_hold+dcfg.wind_ramp,t_end);
        for ax_g = [ax_phi, ax_del]
            yl = ylim(ax_g);
            patch(ax_g,[x1 x2 x2 x1],[yl(1) yl(1) yl(2) yl(2)], ...
                  [0.6 0.8 0.9],'FaceAlpha',0.20,'EdgeColor','none', ...
                  'HandleVisibility','off');
            ylim(ax_g,yl);
        end
    end
    if isfield(dcfg,'camber_enabled') && dcfg.camber_enabled
        xline(ax_phi,0,'b:','LineWidth',1.0,'Label', ...
              sprintf('camber %.1f deg',dcfg.camber_deg),'HandleVisibility','off');
    end
end

function abv2_write_gif(path, im, cm, first, fps)
    if first
        imwrite(im,cm,path,'gif','Loopcount',inf,'DelayTime',1/fps);
    else
        imwrite(im,cm,path,'gif','WriteMode','append','DelayTime',1/fps);
    end
end

function out = abv2_lock(cd, ref_h, ref_w)
    [h,w,~] = size(cd);
    if h==ref_h && w==ref_w, out=cd; return; end
    out = cd(1:min(h,ref_h), 1:min(w,ref_w), :);
    if size(out,1)<ref_h
        out=cat(1,out,repmat(out(end,:,:),ref_h-size(out,1),1,1)); end
    if size(out,2)<ref_w
        out=cat(2,out,repmat(out(:,end,:),1,ref_w-size(out,2),1)); end
end

function R = abv2_rodrigues(axis, theta)
    ax = axis(:)/norm(axis);
    K  = [0,-ax(3),ax(2); ax(3),0,-ax(1); -ax(2),ax(1),0];
    R  = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);
end

function c = abv2_color(idx)
    palette = [0.80 0.15 0.10;
               0.10 0.45 0.75;
               0.15 0.60 0.35;
               0.75 0.45 0.10;
               0.55 0.15 0.70;
               0.10 0.65 0.65;
               0.70 0.20 0.50];
    c = palette(mod(idx-1,size(palette,1))+1,:);
end