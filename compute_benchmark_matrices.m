function [M, C1, K0, K2] = compute_benchmark_matrices(p)
% COMPUTE_BENCHMARK_MATRICES  Canonical matrices of the linearized
% lean-steer (Whipple-Carvallo) bicycle model, evaluated about the
% straight-ahead, upright equilibrium.
%
% The equations of motion take the form:
%
%   M*qdd + v*C1*qd + (g*K0 + v^2*K2)*q = F
%
%   q = [phi; delta]   (roll angle, steer angle)
%   F = [Tphi; Tdelta] (roll torque, steer torque)
%
% This packages the mass properties of the four rigid bodies (rear
% frame+rider, fork+handlebar, rear wheel, front wheel) into the
% composite inertial quantities used by the linearized model, following
% Meijaard, Papadopoulos, Ruina & Schwab (2007), Proc. R. Soc. A 463.
%
% Input:
%   p  - struct from bike_params()
% Outputs:
%   M, C1, K0, K2 - 2x2 matrices

mT = p.mR + p.mB + p.mH + p.mF;
xT = (p.xB*p.mB + p.xH*p.mH + p.w*p.mF) / mT;
zT = (-p.rR*p.mR + p.zB*p.mB + p.zH*p.mH - p.rF*p.mF) / mT;

ITxx = p.IRxx + p.IBxx + p.IHxx + p.IFxx + p.mR*p.rR^2 + p.mB*p.zB^2 + ...
       p.mH*p.zH^2 + p.mF*p.rF^2;
ITxz = p.IBxz + p.IHxz - p.mB*p.xB*p.zB - p.mH*p.xH*p.zH + p.mF*p.w*p.rF;
IRzz = p.IRxx;   % wheels are axisymmetric: radial inertia = "vertical" inertia
IFzz = p.IFxx;
ITzz = IRzz + p.IBzz + p.IHzz + IFzz + p.mB*p.xB^2 + p.mH*p.xH^2 + p.mF*p.w^2;

% Front assembly (fork + handlebar + front wheel), body "A"
mA = p.mH + p.mF;
xA = (p.xH*p.mH + p.w*p.mF) / mA;
zA = (p.zH*p.mH - p.rF*p.mF) / mA;

IAxx = p.IHxx + p.IFxx + p.mH*(p.zH - zA)^2 + p.mF*(p.rF + zA)^2;
IAxz = p.IHxz - p.mH*(p.xH - xA)*(p.zH - zA) + p.mF*(p.w - xA)*(p.rF + zA);
IAzz = p.IHzz + IFzz + p.mH*(p.xH - xA)^2 + p.mF*(p.w - xA)^2;

uA = (xA - p.w - p.c)*cos(p.lambda) - zA*sin(p.lambda);

IAll = mA*uA^2 + IAxx*sin(p.lambda)^2 + 2*IAxz*sin(p.lambda)*cos(p.lambda) + ...
       IAzz*cos(p.lambda)^2;
IAlx = -mA*uA*zA + IAxx*sin(p.lambda) + IAxz*cos(p.lambda);
IAlz =  mA*uA*xA + IAxz*sin(p.lambda) + IAzz*cos(p.lambda);

mu = (p.c/p.w) * cos(p.lambda);

% Gyrostatic/spin coefficients (wheel angular momentum / radius)
SR = p.IRyy / p.rR;
SF = p.IFyy / p.rF;
ST = SR + SF;
SA = mA*uA + mu*mT*xT;

% --- Mass matrix ---
Mpp = ITxx;
Mpd = IAlx + mu*ITxz;
Mdd = IAll + 2*mu*IAlz + mu^2*ITzz;
M = [Mpp, Mpd; Mpd, Mdd];

% --- Gravity stiffness ---
K0pp = mT*zT;
K0pd = -SA;
K0dd = -SA*sin(p.lambda);
K0 = [K0pp, K0pd; K0pd, K0dd];

% --- Speed-squared stiffness ---
K2pp = 0.0;
K2pd = (ST - mT*zT)/p.w * cos(p.lambda);
K2dd = (SA + SF*sin(p.lambda))/p.w * cos(p.lambda);
K2 = [K2pp, K2pd; 0.0, K2dd];

% --- Speed-proportional (gyroscopic/damping) matrix ---
C1pd = mu*ST + SF*cos(p.lambda) + ITxz/p.w*cos(p.lambda) - mu*mT*zT;
C1dp = -(mu*ST + SF*cos(p.lambda));
C1dd = IAlz/p.w*cos(p.lambda) + mu*(SA + ITzz/p.w*cos(p.lambda));
C1 = [0.0, C1pd; C1dp, C1dd];

end
