% ANIMATE_BIKE
% Creates a simple 3D animation of the bicycle's lean and steer response
% using the linearized Whipple-Carvallo model.
%
% Usage:
%   animate_bike(speed, control_gain, x0, tspan)
%
%   speed   - forward speed (m/s), e.g. 1, 4.3, 9
%   control_gain - if 0, use uncontrolled; if >0, apply Tdelta = k*phidot
%   x0      - initial state [phi; delta; phidot; deltadot] (rad, rad/s)
%   tspan   - time vector for simulation (optional)
%
% Example: animate_bike(1, 0, [deg2rad(5); deg2rad(-2); 0; 0], [0 20])

function animate_bike(speed, control_gain, x0, tspan)
    if nargin < 4, tspan = [0 20]; end
    if nargin < 3, x0 = [deg2rad(5); deg2rad(-2); 0; 0]; end
    if nargin < 2, control_gain = 0; end

    % Load parameters and build matrices
    p = bike_params();
    [M, C1, K0, K2] = compute_benchmark_matrices(p);
    [A, B] = bike_state_space(M, C1, K0, K2, speed, p.g);

    % Closed‑loop if gain > 0
    if control_gain > 0
        e3 = [0 0 1 0];
        A = A + control_gain * B(:,2) * e3;
    end

    % Simulate
    odefun = @(t, x) A*x + B * [0; 0];   % no external torque
    [t, x] = ode45(odefun, tspan, x0);
    phi = x(:,1);
    delta = x(:,2);

    % ---- Geometry (in the bike's local frame, no roll, delta=0) ----
    % Rear wheel centre (ground contact at origin)
    rR = p.rR; rF = p.rF;
    w = p.w; c = p.c; lambda = p.lambda;
    R = [0, 0, rR];                     % rear wheel centre

    % Front wheel centre (when delta = 0)
    F0 = [w, 0, rF];                    % front wheel centre

    % Steer axis: passes through ground point P0 and points upward/backward
    P0 = [w + c, 0, 0];                 % steer axis intersection with ground
    s_axis = [-sin(lambda), 0, cos(lambda)];  % unit vector along steer axis

    % Some extra points for the frame (just for visual interest)
    seat = [0.5, 0, 0.8];               % approximate seat position
    handlebar = [w + c, 0, 0.9];        % handlebar centre (near top of steer axis)

    % ---- Prepare figure ----
    fig = figure('Position', [100 100 900 700]);
    ax = axes('Parent', fig, 'DataAspectRatio', [1 1 1], ...
              'XLim', [-1.5, 2.5], 'YLim', [-1.5, 1.5], 'ZLim', [-0.1, 1.5]);
    view(ax, 3);
    grid(ax, 'on');
    xlabel('x (forward)'); ylabel('y (lateral)'); zlabel('z (up)');
    title(ax, sprintf('v = %.1f m/s, gain = %.2f', speed, control_gain));

    % Draw ground plane (semi‑transparent)
    [Xg, Yg] = meshgrid([-1.5, 2.5], [-1.5, 1.5]);
    Zg = zeros(size(Xg));
    surf(ax, Xg, Yg, Zg, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none', ...
         'FaceAlpha', 0.3);

    % ---- Initialise graphic objects ----
    % We'll store handles to update positions each frame.
    % Rear wheel: a circle (patch) in the x‑z plane
    [rw_x, rw_y, rw_z] = circle3d(R(1), R(2), R(3), rR, 'xz');
    rearWheel = patch(ax, rw_x, rw_y, rw_z, 'r', 'EdgeColor', 'k');

    % Front wheel (will be updated per frame)
    [fw_x, fw_y, fw_z] = circle3d(F0(1), F0(2), F0(3), rF, 'xz');
    frontWheel = patch(ax, fw_x, fw_y, fw_z, 'b', 'EdgeColor', 'k');

    % Frame and fork as line segments
    % Rear frame: lines from rear wheel centre to seat, seat to handlebar? 
    % We'll draw a few lines: rear wheel -> seat, seat -> front pivot, etc.
    rearFrame = line(ax, [R(1), seat(1)], [R(2), seat(2)], [R(3), seat(3)], ...
                     'Color', 'k', 'LineWidth', 2);
    seatToPivot = line(ax, [seat(1), P0(1)], [seat(2), P0(2)], [seat(3), P0(3)], ...
                       'Color', 'k', 'LineWidth', 1.5);
    % Fork: pivot to front wheel centre
    fork = line(ax, [P0(1), F0(1)], [P0(2), F0(2)], [P0(3), F0(3)], ...
                'Color', [0.5 0.5 0.5], 'LineWidth', 2);

    % Handlebar (simple cross bar)
    hBar = line(ax, [handlebar(1)-0.1, handlebar(1)+0.1], ...
                [handlebar(2), handlebar(2)], ...
                [handlebar(3), handlebar(3)], ...
                'Color', 'm', 'LineWidth', 3);

    % --- Animation loop ---
    dt = 0.05;  % frame interval (seconds)
    for i = 1:length(t)
        phi_i = phi(i);
        delta_i = delta(i);

        % 1. Compute positions of front assembly after steer rotation
        %    Rotate points about steer axis (through P0, axis s_axis) by delta_i
        %    Points to rotate: F0, handlebar (and maybe fork ends)
        %    Use Rodrigues' rotation formula
        R_delta = rodrigues(s_axis, delta_i);
        F_rot = P0 + R_delta * (F0 - P0)';
        F_rot = F_rot';
        H_rot = P0 + R_delta * (handlebar - P0)';
        H_rot = H_rot';

        % 2. Apply roll rotation (about global x‑axis) to ALL points
        R_phi = [1, 0, 0; 0, cos(phi_i), -sin(phi_i); 0, sin(phi_i), cos(phi_i)];
        % Rear points (no steer): R, seat
        R_roll = (R_phi * R')';
        seat_roll = (R_phi * seat')';
        % Front points (after steer): F_rot, H_rot, and also pivot P0 (same for all)
        P0_roll = (R_phi * P0')';
        F_roll = (R_phi * F_rot')';
        H_roll = (R_phi * H_rot')';

        % 3. Update graphic objects
        % Rear wheel: centre at R_roll, still in x‑z plane (but rotated)
        [rwx, rwy, rwz] = circle3d(R_roll(1), R_roll(2), R_roll(3), rR, 'xz');
        set(rearWheel, 'XData', rwx, 'YData', rwy, 'ZData', rwz);

        % Front wheel: centre at F_roll
        [fwx, fwy, fwz] = circle3d(F_roll(1), F_roll(2), F_roll(3), rF, 'xz');
        set(frontWheel, 'XData', fwx, 'YData', fwy, 'ZData', fwz);

        % Lines
        set(rearFrame, 'XData', [R_roll(1), seat_roll(1)], ...
                       'YData', [R_roll(2), seat_roll(2)], ...
                       'ZData', [R_roll(3), seat_roll(3)]);
        set(seatToPivot, 'XData', [seat_roll(1), P0_roll(1)], ...
                         'YData', [seat_roll(2), P0_roll(2)], ...
                         'ZData', [seat_roll(3), P0_roll(3)]);
        set(fork, 'XData', [P0_roll(1), F_roll(1)], ...
                  'YData', [P0_roll(2), F_roll(2)], ...
                  'ZData', [P0_roll(3), F_roll(3)]);
        set(hBar, 'XData', [H_roll(1)-0.1, H_roll(1)+0.1], ...
                  'YData', [H_roll(2), H_roll(2)], ...
                  'ZData', [H_roll(3), H_roll(3)]);

        drawnow;
        pause(0.01);   % adjust for speed

        % (Optional: capture frames for GIF)
    end
end

% ---- Helper functions ----
function pts = rodrigues(axis, theta)
    % Rodrigues' rotation formula: rotate a point (3x1) about axis (unit)
    % Returns rotation matrix R (3x3)
    ax = axis(:);
    K = [0, -ax(3), ax(2); ax(3), 0, -ax(1); -ax(2), ax(1), 0];
    R = eye(3) + sin(theta)*K + (1 - cos(theta))*(K*K);
    pts = R;
end

function [x, y, z] = circle3d(cx, cy, cz, r, plane)
    % Generate points of a circle in a given plane, centred at (cx,cy,cz)
    % plane: 'xz' (default) or 'xy' or 'yz'
    n = 40;
    th = linspace(0, 2*pi, n);
    switch plane
        case 'xz'
            x = cx + r*cos(th);
            y = cy + zeros(size(th));
            z = cz + r*sin(th);
        case 'xy'
            x = cx + r*cos(th);
            y = cy + r*sin(th);
            z = cz + zeros(size(th));
        case 'yz'
            x = cx + zeros(size(th));
            y = cy + r*cos(th);
            z = cz + r*sin(th);
        otherwise
            error('Unknown plane');
    end
end