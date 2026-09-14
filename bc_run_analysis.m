function results = bc_run_analysis(cfg)
% BC_RUN_ANALYSIS  Simulation engine called by bike_launch and run_all.
%
% Three physical concepts now correctly separated:
%
%   cfg.dt                   — integration step size (s). Smaller = more
%                              accurate but slower. Default 0.01 s.
%
%   cfg.phi_limit_deg        — HARD STOP. |phi| > this means the bike is
%                              on the ground. Simulation terminates. (90 deg)
%
%   cfg.delta_limit_deg      — ACTUATOR SATURATION. |delta| > this means
%                              the handlebar is at its mechanical stop.
%                              Steer angle is CLAMPED, not used to kill the
%                              run. Simulation continues. A saturation flag
%                              is recorded. (60 deg)
%
%   cfg.steer_rate_limit_deg_s — SLEW RATE. Max steer angle change per
%                              second. Models actuator speed. Inf = no
%                              limit (current behavior). (deg/s)
%
% Controllers:
%   RateOnly    PD    PID    LQR    FullState    LQG    GainSched

fprintf('\n[bc_run_analysis] Building model at v = %.2f m/s ...\n', cfg.v);

p = cfg.p;
[M_mat, C1, K0, K2] = compute_benchmark_matrices(p);
[A, B]               = bike_state_space(M_mat, C1, K0, K2, cfg.v, p.g);
b                    = B(:,2);
e3                   = [0 0 1 0];

phi_lim       = deg2rad(cfg.phi_limit_deg);
delta_sat     = deg2rad(cfg.delta_limit_deg);     % saturation clamp, NOT kill
slew_rad_s    = deg2rad(cfg.steer_rate_limit_deg_s);  % Inf = no limit

% dt: use cfg.dt if present, otherwise default to 0.01 s
if isfield(cfg,'dt') && ~isempty(cfg.dt) && cfg.dt > 0
    dt_sim = cfg.dt;
else
    dt_sim = 0.01;
end
tspan = 0 : dt_sim : cfg.t_end;
if tspan(end) < cfg.t_end, tspan(end+1) = cfg.t_end; end

ev_ol  = eig(A);
max_re = max(real(ev_ol));
if     max_re >  1e-4, regime = 'UNSTABLE';
elseif max_re < -1e-4, regime = 'self-stable';
else,                  regime = '~ boundary';
end

results.v       = cfg.v;
results.regime  = regime;
results.ev_ol   = ev_ol;
results.max_re  = max_re;
results.x0      = cfg.x0;
results.tspan   = tspan;
results.p       = p;
results.cfg     = cfg;
results.dt      = dt_sim;
results.runs    = struct();

fprintf('  dt = %.4f s   phi_lim = %.0f deg   delta_sat = %.0f deg', ...
        dt_sim, cfg.phi_limit_deg, cfg.delta_limit_deg);
if ~isinf(slew_rad_s)
    fprintf('   slew = %.0f deg/s', cfg.steer_rate_limit_deg_s);
end
fprintf('\n  Open-loop regime: %s  (max Re = %+.4f)\n', regime, max_re);

% Pack actuator params into a single struct for passing to sim cores
act = struct('phi_lim',phi_lim,'delta_sat',delta_sat,'slew',slew_rad_s);

% =========================================================================
%  UNCONTROLLED
% =========================================================================
fprintf('  Simulating: Uncontrolled ...\n');
[t_u,x_u,fail_u,ft_u,fm_u,sat_u] = ...
    bc_sim(A, tspan, cfg.x0, act, cfg.disturbance, p, cfg.v, M_mat,C1,K0,K2);
results.runs.uncontrolled = mk_run( ...
    'Uncontrolled','',A,zeros(1,4),ev_ol, t_u,x_u,fail_u,ft_u,fm_u,sat_u);
log_run(results.runs.uncontrolled, cfg.t_end);

% =========================================================================
%  RATE-ONLY
% =========================================================================
if any(strcmp(cfg.controllers,'RateOnly'))
    fprintf('  Simulating: Rate-Only ...\n');
    k_rate = find_rate_gain(A,b,e3);
    f_ro   = [0,0,k_rate,0];
    Acl_ro = A + b*f_ro;
    [t_ro,x_ro,fail_ro,ft_ro,fm_ro,sat_ro] = ...
        bc_sim(Acl_ro, tspan, cfg.x0, act, cfg.disturbance, p, cfg.v, M_mat,C1,K0,K2);
    results.runs.RateOnly = mk_run( ...
        'Rate-Only  (T=k*phi_dot)', sprintf('k=%.2f N*m*s/rad',k_rate), ...
        Acl_ro,f_ro,eig(Acl_ro), t_ro,x_ro,fail_ro,ft_ro,fm_ro,sat_ro);
    log_run(results.runs.RateOnly, cfg.t_end);
end

% =========================================================================
%  PD
% =========================================================================
if any(strcmp(cfg.controllers,'PD'))
    fprintf('  Simulating: PD ...\n');
    if cfg.pd_k1==0 && cfg.pd_k2==0
        fprintf('    Grid-searching PD gains ...\n');
        [k1b,k2b] = grid_search_pd(A,b);
    else
        k1b=cfg.pd_k1; k2b=cfg.pd_k2;
    end
    f_pd   = [k1b,0,k2b,0];
    Acl_pd = A + b*f_pd;
    [t_pd,x_pd,fail_pd,ft_pd,fm_pd,sat_pd] = ...
        bc_sim(Acl_pd, tspan, cfg.x0, act, cfg.disturbance, p, cfg.v, M_mat,C1,K0,K2);
    results.runs.PD = mk_run( ...
        'PD  (T=k1*phi+k2*phi_dot)', sprintf('k1=%.1f  k2=%.1f',k1b,k2b), ...
        Acl_pd,f_pd,eig(Acl_pd), t_pd,x_pd,fail_pd,ft_pd,fm_pd,sat_pd);
    log_run(results.runs.PD, cfg.t_end);
end

% =========================================================================
%  PID
% =========================================================================
if any(strcmp(cfg.controllers,'PID'))
    fprintf('  Simulating: PID ...\n');
    Aa = [A,zeros(4,1); 1,0,0,0,0];
    Ba = [b;0];
    if cfg.pid_k1==0 && cfg.pid_k2==0 && cfg.pid_k3==0
        fprintf('    Grid-searching PID gains ...\n');
        [k1p,k2p,k3p] = grid_search_pid(Aa,Ba);
    else
        k1p=cfg.pid_k1; k2p=cfg.pid_k2; k3p=cfg.pid_k3;
    end
    f_pid    = [k1p,0,k2p,0,k3p];
    Acl_pid  = Aa + Ba*f_pid;
    x0_aug   = [cfg.x0;0];
    [t_pid,x_pid,fail_pid,ft_pid,fm_pid,sat_pid] = ...
        bc_sim_aug(Acl_pid, tspan, x0_aug, act, cfg.disturbance, p, cfg.v, M_mat,C1,K0,K2);
    results.runs.PID = mk_run( ...
        'PID  (phi+phi_dot+int_phi)', ...
        sprintf('k1=%.1f  k2=%.1f  k3=%.1f',k1p,k2p,k3p), ...
        Acl_pid(1:4,1:4),f_pid(1:4),eig(Acl_pid), ...
        t_pid,x_pid(:,1:4),fail_pid,ft_pid,fm_pid,sat_pid);
    results.runs.PID.f_row_full = f_pid;
    log_run(results.runs.PID, cfg.t_end);
end

% =========================================================================
%  LQR
% =========================================================================
if any(strcmp(cfg.controllers,'LQR'))
    fprintf('  Simulating: LQR ...\n');
    Q_lqr = diag(cfg.lqr_Q);
    R_lqr = cfg.lqr_R;
    try
        [~,~,K_lqr] = care(A,b,Q_lqr,R_lqr);
        K_lqr = K_lqr(:)';
    catch
        fprintf('    care() failed, using iterative solver ...\n');
        K_lqr = dare_iterative(A,b,Q_lqr,R_lqr);
    end
    f_lqr   = -K_lqr;
    Acl_lqr = A + b*f_lqr;
    [t_lqr,x_lqr,fail_lqr,ft_lqr,fm_lqr,sat_lqr] = ...
        bc_sim(Acl_lqr, tspan, cfg.x0, act, cfg.disturbance, p, cfg.v, M_mat,C1,K0,K2);
    results.runs.LQR = mk_run( ...
        'LQR  (optimal full-state)', ...
        sprintf('Q=diag[%.0f %.0f %.0f %.0f]  R=%.2f', ...
                cfg.lqr_Q(1),cfg.lqr_Q(2),cfg.lqr_Q(3),cfg.lqr_Q(4),R_lqr), ...
        Acl_lqr,f_lqr,eig(Acl_lqr), t_lqr,x_lqr,fail_lqr,ft_lqr,fm_lqr,sat_lqr);
    results.runs.LQR.K_lqr = K_lqr;
    log_run(results.runs.LQR, cfg.t_end);
end

% =========================================================================
%  FULL-STATE (Ackermann)
% =========================================================================
if any(strcmp(cfg.controllers,'FullState'))
    fprintf('  Simulating: Full-State (Ackermann) ...\n');
    K_ack   = bc_ackermann(A,b,cfg.pp_poles);
    f_fs    = -K_ack;
    Acl_fs  = A + b*f_fs;
    [t_fs,x_fs,fail_fs,ft_fs,fm_fs,sat_fs] = ...
        bc_sim(Acl_fs, tspan, cfg.x0, act, cfg.disturbance, p, cfg.v, M_mat,C1,K0,K2);
    results.runs.FullState = mk_run( ...
        'Full-State  (pole placement)', ...
        sprintf('poles: [%s]',num2str(cfg.pp_poles,'%d ')), ...
        Acl_fs,f_fs,eig(Acl_fs), t_fs,x_fs,fail_fs,ft_fs,fm_fs,sat_fs);
    log_run(results.runs.FullState, cfg.t_end);
end

% =========================================================================
%  LQG  (LQR + Kalman observer)
% =========================================================================
if any(strcmp(cfg.controllers,'LQG'))
    fprintf('  Simulating: LQG (LQR + Kalman observer) ...\n');
    Q_lqr = diag(cfg.lqr_Q);
    R_lqr = cfg.lqr_R;
    try
        [~,~,K_lqg] = care(A,b,Q_lqr,R_lqr);
        K_lqg = K_lqg(:)';
    catch
        K_lqg = dare_iterative(A,b,Q_lqr,R_lqr);
    end
    C_obs = [0 0 1 0];
    Qw    = diag(cfg.lqg_Qw);
    Rv    = cfg.lqg_Rv;
    try
        [~,~,L_T] = care(A',C_obs',Qw,Rv);
        L_obs = L_T(:)';
        L_obs = L_obs';
    catch
        L_obs = place(A',C_obs',cfg.pp_poles*2)';
        fprintf('    observer care() failed, used pole placement.\n');
    end
    f_lqg_gain = -K_lqg;
    [t_lqg,x_lqg,x_hat_lqg,fail_lqg,ft_lqg,fm_lqg,sat_lqg] = ...
        bc_sim_lqg(A,b,C_obs,K_lqg,L_obs, tspan, cfg.x0, ...
                   act, cfg.disturbance, p, cfg.v, M_mat,C1,K0,K2,Rv);
    Acl_lqg = A + b*f_lqg_gain;
    results.runs.LQG = mk_run( ...
        'LQG  (LQR + Kalman observer)', ...
        'observes phi_dot only; estimates full state', ...
        Acl_lqg,f_lqg_gain,eig(Acl_lqg), ...
        t_lqg,x_lqg,fail_lqg,ft_lqg,fm_lqg,sat_lqg);
    results.runs.LQG.x_hat = x_hat_lqg;
    results.runs.LQG.C_obs = C_obs;
    results.runs.LQG.L_obs = L_obs;
    results.runs.LQG.K_lqr = K_lqg;
    log_run(results.runs.LQG, cfg.t_end);
end

% =========================================================================
%  GAIN-SCHEDULED
% =========================================================================
if any(strcmp(cfg.controllers,'GainSched'))
    fprintf('  Simulating: Gain-Scheduled ...\n');
    zone_speeds = [1.5, 4.3, 7.5];
    zone_gains  = zeros(1,3);
    for zi = 1:3
        [Av,Bv] = bike_state_space(M_mat,C1,K0,K2,zone_speeds(zi),p.g);
        zone_gains(zi) = find_rate_gain(Av,Bv(:,2),e3);
    end
    fprintf('    Zone gains: low=%.1f  mid=%.1f  high=%.1f\n',zone_gains);
    [t_gs,x_gs,fail_gs,ft_gs,fm_gs,sat_gs] = ...
        bc_sim_gs(M_mat,C1,K0,K2,p, tspan, cfg.x0, act, cfg.disturbance, ...
                  cfg.v, zone_speeds,zone_gains);
    k_nom   = interp1(zone_speeds,zone_gains, ...
                      min(max(cfg.v,zone_speeds(1)),zone_speeds(end)));
    Acl_gs  = A + b*[0,0,k_nom,0];
    results.runs.GainSched = mk_run( ...
        'Gain-Scheduled  (speed-zone rate gains)', ...
        sprintf('zones: v<3->k=%.1f | 3-5.5->k=%.1f | v>5.5->k=%.1f',zone_gains), ...
        Acl_gs,[0,0,k_nom,0],eig(Acl_gs), ...
        t_gs,x_gs,fail_gs,ft_gs,fm_gs,sat_gs);
    log_run(results.runs.GainSched, cfg.t_end);
end

fprintf('[bc_run_analysis] Done.\n\n');
end

% =========================================================================
%  CORE: ACTUATOR ENFORCEMENT
%  Applied inside every simulation loop.
%  Returns the clamped steer angle and updates saturation flag.
% =========================================================================
function [delta_out, delta_dot_out, saturated] = ...
         bc_actuator(delta_cmd, delta_dot_cmd, delta_prev, dt, act)
% Enforce steer saturation (clamp) and slew rate limit.
%   delta_cmd      — unconstrained steer angle from state evolution (rad)
%   delta_dot_cmd  — unconstrained steer rate from state evolution (rad/s)
%   delta_prev     — steer angle at previous step (rad)
%   dt             — integration step (s)
%   act            — actuator struct: .delta_sat, .slew
%
% Returns corrected delta and delta_dot, and a saturation flag.

    saturated = false;

    % 1. Slew rate limit: cap how much delta can change per step
    if ~isinf(act.slew)
        max_change = act.slew * dt;
        actual_change = delta_cmd - delta_prev;
        if abs(actual_change) > max_change
            delta_cmd = delta_prev + sign(actual_change)*max_change;
            delta_dot_cmd = sign(actual_change)*act.slew;
            saturated = true;
        end
    end

    % 2. Position saturation: clamp at mechanical stop
    if abs(delta_cmd) > act.delta_sat
        delta_cmd     = sign(delta_cmd) * act.delta_sat;
        delta_dot_cmd = 0;   % velocity zeroed at the hard stop
        saturated = true;
    end

    delta_out     = delta_cmd;
    delta_dot_out = delta_dot_cmd;
end

% =========================================================================
%  SIMULATION CORES
%  All share the same actuator enforcement and constraint logic.
%  Only phi > phi_lim terminates the run.
%  delta is clamped at delta_sat; saturation is flagged but not fatal.
% =========================================================================

function [t_out,x_out,failed,fail_time,fail_msg,sat_info] = ...
         bc_sim(Acl, tspan, x0, act, dcfg, p, v_nom, M_mat,C1,K0,K2)
    dt = tspan(2)-tspan(1); n = numel(tspan);
    x  = zeros(n,4); x(1,:) = x0(:)';
    failed=false; fail_time=NaN; fail_msg='';
    sat_count=0; first_sat_t=NaN;
    rng(42);

    use_sv  = isfield(dcfg,'speed_var_enabled') && dcfg.speed_var_enabled;
    [A_nom,~] = bike_state_space(M_mat,C1,K0,K2,v_nom,p.g);
    dA_gain   = Acl - A_nom;

    for k = 1:n-1
        t_k = tspan(k); xk = x(k,:)';
        [d,v_eff] = bc_disturbance(t_k,dcfg,p,v_nom);

        if use_sv && abs(v_eff-v_nom)>1e-6
            [A_cur,~] = bike_state_space(M_mat,C1,K0,K2,v_eff,p.g);
            Acl_cur   = A_cur + dA_gain;
        else
            Acl_cur = Acl;
        end

        % Euler step (unconstrained)
        xk1 = xk + dt*(Acl_cur*xk + d);

        % Enforce actuator limits on delta (x(2)) and delta_dot (x(4))
        [xk1(2),xk1(4),sat] = bc_actuator(xk1(2),xk1(4),xk(2),dt,act);
        if sat
            sat_count = sat_count+1;
            if isnan(first_sat_t), first_sat_t = tspan(k+1); end
        end

        x(k+1,:) = xk1';

        % Only phi breach terminates the run
        if abs(xk1(1)) > act.phi_lim
            failed    = true;
            fail_time = tspan(k+1);
            fail_msg  = sprintf('|phi|=%.1f deg > %.0f deg (bike fell)', ...
                                rad2deg(abs(xk1(1))), rad2deg(act.phi_lim));
            x(k+2:end,:) = NaN;
            break;
        end
    end
    t_out = tspan(:); x_out = x;
    sat_info = struct('count',sat_count,'first_t',first_sat_t, ...
                      'saturated', sat_count>0);
end

function [t_out,x_out,failed,fail_time,fail_msg,sat_info] = ...
         bc_sim_aug(Acl_aug, tspan, x0_aug, act, dcfg, p, v_nom, M_mat,C1,K0,K2)
    dt = tspan(2)-tspan(1); n = numel(tspan);
    na = size(Acl_aug,1);
    x  = zeros(n,na); x(1,:) = x0_aug(:)';
    failed=false; fail_time=NaN; fail_msg='';
    sat_count=0; first_sat_t=NaN;
    rng(42);
    d_aug = zeros(na,1);

    for k = 1:n-1
        t_k = tspan(k); xk = x(k,:)';
        [d4,~] = bc_disturbance(t_k,dcfg,p,v_nom);
        d_aug(1:4) = d4;

        xk1 = xk + dt*(Acl_aug*xk + d_aug);

        % Actuator enforcement on delta (idx 2) and delta_dot (idx 4)
        [xk1(2),xk1(4),sat] = bc_actuator(xk1(2),xk1(4),xk(2),dt,act);
        if sat
            sat_count=sat_count+1;
            if isnan(first_sat_t), first_sat_t=tspan(k+1); end
        end

        x(k+1,:) = xk1';

        if abs(xk1(1)) > act.phi_lim
            failed=true; fail_time=tspan(k+1);
            fail_msg=sprintf('|phi|=%.1f deg > %.0f deg (bike fell)', ...
                             rad2deg(abs(xk1(1))),rad2deg(act.phi_lim));
            x(k+2:end,:) = NaN; break;
        end
    end
    t_out=tspan(:); x_out=x;
    sat_info=struct('count',sat_count,'first_t',first_sat_t,'saturated',sat_count>0);
end

function [t_out,x_out,x_hat_out,failed,fail_time,fail_msg,sat_info] = ...
         bc_sim_lqg(A,b,C_obs,K_lqg,L_obs, tspan, x0, act, dcfg, p, v_nom, ...
                    M_mat,C1,K0,K2,Rv)
    dt=tspan(2)-tspan(1); n=numel(tspan);
    x    =zeros(n,4); x(1,:)    =x0(:)';
    x_hat=zeros(n,4); x_hat(1,:)=x0(:)';
    failed=false; fail_time=NaN; fail_msg='';
    sat_count=0; first_sat_t=NaN;
    rng(42);
    meas_noise_std=sqrt(Rv);

    for k = 1:n-1
        t_k=tspan(k); xk=x(k,:)'; xhk=x_hat(k,:)';
        [d,~]=bc_disturbance(t_k,dcfg,p,v_nom);

        u_k  = -K_lqg*xhk;
        b_u  = b*u_k;

        xk1  = xk + dt*(A*xk + b_u + d);

        % Actuator enforcement
        [xk1(2),xk1(4),sat] = bc_actuator(xk1(2),xk1(4),xk(2),dt,act);
        if sat
            sat_count=sat_count+1;
            if isnan(first_sat_t), first_sat_t=tspan(k+1); end
        end
        x(k+1,:) = xk1';

        y_k  = C_obs*xk + meas_noise_std*randn();
        xhk1 = xhk + dt*(A*xhk + b_u + L_obs*(y_k - C_obs*xhk));
        % Clamp observer delta estimate too, so observer stays consistent
        xhk1(2) = max(-act.delta_sat, min(act.delta_sat, xhk1(2)));
        x_hat(k+1,:) = xhk1';

        if abs(xk1(1)) > act.phi_lim
            failed=true; fail_time=tspan(k+1);
            fail_msg=sprintf('|phi|=%.1f deg > %.0f deg (bike fell)', ...
                             rad2deg(abs(xk1(1))),rad2deg(act.phi_lim));
            x(k+2:end,:)    =NaN;
            x_hat(k+2:end,:)=NaN;
            break;
        end
    end
    t_out=tspan(:); x_out=x; x_hat_out=x_hat;
    sat_info=struct('count',sat_count,'first_t',first_sat_t,'saturated',sat_count>0);
end

function [t_out,x_out,failed,fail_time,fail_msg,sat_info] = ...
         bc_sim_gs(M_mat,C1,K0,K2,p, tspan, x0, act, dcfg, v_nom, ...
                   zone_speeds,zone_gains)
    dt=tspan(2)-tspan(1); n=numel(tspan);
    x=zeros(n,4); x(1,:)=x0(:)';
    failed=false; fail_time=NaN; fail_msg='';
    sat_count=0; first_sat_t=NaN;
    rng(42);
    e3=[0 0 1 0];

    for k = 1:n-1
        t_k=tspan(k); xk=x(k,:)';
        [d,v_eff]=bc_disturbance(t_k,dcfg,p,v_nom);

        [Ak,Bk]=bike_state_space(M_mat,C1,K0,K2,v_eff,p.g);
        k_s=interp1(zone_speeds,zone_gains, ...
                    min(max(v_eff,zone_speeds(1)),zone_speeds(end)),'linear');
        Acl_k=Ak+Bk(:,2)*(k_s*e3);

        xk1=xk+dt*(Acl_k*xk+d);

        % Actuator enforcement
        [xk1(2),xk1(4),sat]=bc_actuator(xk1(2),xk1(4),xk(2),dt,act);
        if sat
            sat_count=sat_count+1;
            if isnan(first_sat_t), first_sat_t=tspan(k+1); end
        end
        x(k+1,:)=xk1';

        if abs(xk1(1)) > act.phi_lim
            failed=true; fail_time=tspan(k+1);
            fail_msg=sprintf('|phi|=%.1f deg > %.0f deg (bike fell)', ...
                             rad2deg(abs(xk1(1))),rad2deg(act.phi_lim));
            x(k+2:end,:)=NaN; break;
        end
    end
    t_out=tspan(:); x_out=x;
    sat_info=struct('count',sat_count,'first_t',first_sat_t,'saturated',sat_count>0);
end

% =========================================================================
%  GAIN SEARCH HELPERS
% =========================================================================
function k = find_rate_gain(A,b,e3)
    gains=linspace(0,2000,8000); k=0;
    for gi=1:numel(gains)
        if max(real(eig(A+gains(gi)*b*e3)))<-0.5
            k=gains(gi); return;
        end
    end
    fprintf('    WARNING: no stabilising rate gain found.\n');
end

function [k1,k2]=grid_search_pd(A,b)
    best=inf; k1=0; k2=0;
    for g1=linspace(0,500,101)
        for g2=linspace(0,200,101)
            m=max(real(eig(A+b*[g1,0,g2,0])));
            if m<best, best=m; k1=g1; k2=g2; end
        end
    end
end

function [k1,k2,k3]=grid_search_pid(Aa,Ba)
    best=inf; k1=0; k2=0; k3=0;
    for g1=linspace(0,500,51)
        for g2=linspace(0,200,51)
            for g3=linspace(0,50,21)
                m=max(real(eig(Aa+Ba*[g1,0,g2,0,g3])));
                if m<best, best=m; k1=g1; k2=g2; k3=g3; end
            end
        end
    end
    fprintf('    k1=%.1f  k2=%.1f  k3=%.2f\n',k1,k2,k3);
end

function K=dare_iterative(A,b,Q,R)
    P=Q;
    for i=1:1000
        Pn=Q+A'*P*A-A'*P*b*inv(R+b'*P*b)*b'*P*A;
        if norm(Pn-P,'fro')<1e-10, break; end
        P=Pn;
    end
    K=(R+b'*P*b)\(b'*P*A);
end

% =========================================================================
%  STRUCT BUILDERS
% =========================================================================
function r=mk_run(label,k_info,Acl,f_row,ev_cl,t,x,failed,ft,fm,sat_info)
    r.label       = label;
    r.key         = matlab.lang.makeValidName(label);
    r.k_info      = k_info;
    r.A_cl        = Acl;
    r.f_row       = f_row(:)';
    r.ev_cl       = ev_cl;
    r.t           = t;
    r.x           = x;
    r.failed      = failed;
    r.fail_time   = ft;
    r.fail_msg    = fm;
    r.sat_info    = sat_info;   % actuator saturation record
    r.settle_time = bc_settle(t,x);
end

function ts=bc_settle(t,x)
    ts=NaN; phi=x(:,1);
    for i=1:numel(t)
        v=phi(i:end); v=v(~isnan(v));
        if isempty(v), break; end
        if all(abs(v)<deg2rad(1.0)), ts=t(i); return; end
    end
end

function log_run(r,t_end)
    sat_str = '';
    if r.sat_info.saturated
        sat_str = sprintf('  [SAT x%d from t=%.2fs]', ...
                          r.sat_info.count, r.sat_info.first_t);
    end
    if r.failed
        fprintf('    -> FAILED at t=%.2fs: %s%s\n', ...
                r.fail_time, r.fail_msg, sat_str);
    else
        if isnan(r.settle_time)
            fprintf('    max Re(cl)=%+.4f  settle=>%.0fs%s\n', ...
                    max(real(r.ev_cl)), t_end, sat_str);
        else
            fprintf('    max Re(cl)=%+.4f  settle=%.2fs%s\n', ...
                    max(real(r.ev_cl)), r.settle_time, sat_str);
        end
    end
end

function K=bc_ackermann(A,b,poles)
    n=size(A,1); Wc=zeros(n); Ap=eye(n);
    for i=1:n, Wc(:,i)=Ap*b; Ap=Ap*A; end
    coeffs=poly(poles); phiA=zeros(n); pows=n:-1:0;
    for i=1:numel(coeffs), phiA=phiA+coeffs(i)*mpower(A,pows(i)); end
    last=zeros(1,n); last(n)=1;
    K=last*(Wc\phiA);
end