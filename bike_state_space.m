function [A, B] = bike_state_space(M, C1, K0, K2, v, g)
% BIKE_STATE_SPACE  Build first-order state-space form of the linearized
% bicycle model at forward speed v.
%
% State:  x = [phi; delta; phidot; deltadot]
% Input:  u = [Tphi; Tdelta]   (roll torque, steer torque)
%
%   xdot = A*x + B*u
%
% Derived from  M*qdd + v*C1*qd + (g*K0 + v^2*K2)*q = F
%   =>  qdd = Minv*( F - v*C1*qd - (g*K0 + v^2*K2)*q )

Minv = inv(M);
Ktot = g*K0 + v^2*K2;

A = [zeros(2,2),      eye(2)          ;
     -Minv*Ktot,      -v*Minv*C1      ];

B = [zeros(2,2); Minv];

end
