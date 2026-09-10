% ANIMATE_BIKE  Visualizes the bicycle state trajectory with a 3D stick-figure model.
% Run this after running simulate_controlled.m (expects t and x to be in the workspace).

if ~exist('t', 'var') || ~exist('x', 'var')
    error('Please run simulate_controlled.m first so t and x are available in the workspace.');
end

p = bike_params();

figure('Position', [100, 100, 900, 700], 'Color', 'w');
ax = axes;
axis equal;
grid on;
hold on;
view(3);

xlim([-1.5, 2.0]);
ylim([-1.5, 1.5]);
zlim([-1.0, 1.0]);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Bicycle Controller Animation');

% Create graphical handles for parts
h_frame = plot3([0 0], [0 0], [0 0], 'LineWidth', 4, 'Color', [0 0.447 0.741]); % Blue frame
h_fork  = plot3([0 0], [0 0], [0 0], 'LineWidth', 3, 'Color', [0.85 0.325 0.098]); % Orange fork
h_rw    = plot3(0,0,0, 'LineWidth', 2, 'Color', 'k'); % Rear wheel placeholder
h_fw    = plot3(0,0,0, 'LineWidth', 2, 'Color', 'k'); % Front wheel placeholder

% Animation loop (downsample slightly for smooth playback speed)
skip = max(1, floor(length(t)/200));

for k = 1:skip:length(t)
    phi   = x(k, 1);   % roll angle
    delta = x(k, 2);   % steer angle
    
    % --- Coordinate calculations ---
    % Rear wheel center at origin (fixed contact point assumption for relative motion)
    p_rw_center = [0, 0, p.rR];
    
    % Rear frame / rider center of mass position (rolls with phi)
    % Using parameters from bike_params.m: xB, zB
    % Apply roll rotation matrix around X axis
    R_roll = [1, 0, 0; 
              0, cos(phi), -sin(phi); 
              0, sin(phi),  cos(phi)];
          
    % Frame top / seat / CoM point relative to rear hub
    frame_top_local = [p.xB; 0; p.zB];
    frame_top_world = p_rw_center' + R_roll * frame_top_local;
    
    % Front hub position (approximate wheelbase 'w' forward along X)
    front_hub_local = [p.w; 0; -p.rF]; 
    % Rotate front hub position by roll, plus steering offset effects
    front_hub_world = p_rw_center' + R_roll * [p.w; 0; 0] + [0; 0; -p.rF];
    
    % Handlebar/fork top position (steers by delta around steering axis)
    % Steering axis tilt angle is lambda from vertical
    steer_axis_tilt = p.lambda; 
    
    % Approximate handle position in world coordinates
    h_local = [p.xH; 0; p.zH];
    h_world = p_rw_center' + R_roll * h_local;
    
    % Update Frame lines (Rear Hub -> Frame Top -> Front Hub area)
    set(h_frame, 'XData', [p_rw_center(1), frame_top_world(1), front_hub_world(1)], ...
                 'YData', [p_rw_center(2), frame_top_world(2), front_hub_world(2)], ...
                 'ZData', [p_rw_center(3), frame_top_world(3), front_hub_world(3)]);
             
    % Update Fork / Handlebar lines (Front Hub -> Handlebar top, tilted by delta)
    % Applying a simple yaw/steer rotation matrix for delta around the steering axis
    steer_rot = [cos(delta), -sin(delta), 0;
                 sin(delta),  cos(delta), 0;
                 0,           0,          1];
                 
    fork_top_world = front_hub_world + R_roll * steer_rot * [0; 0; 0.4];
    set(h_fork, 'XData', [front_hub_world(1), fork_top_world(1)], ...
                'YData', [front_hub_world(2), fork_top_world(2)], ...
                'ZData', [front_hub_world(3), fork_top_world(3)]);

    drawnow limitrate;
    pause(0.1)
end