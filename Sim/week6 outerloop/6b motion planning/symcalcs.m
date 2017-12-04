
clear all
params = struct();

params.g = 9.81;
params.mr = 0.25;
params.ir = 0.0001;
params.d = 0.1;
params.r = 0.02;

% 1. Learn how to use the symbolic toolbox in MATLAB
% you will need the state variables, and control input to be declared as symbolic
syms th dth phi dphi u ddth ddphi tau ddx
% 2. call your "eom" function to get \ddot{q} symbolically
u = 0;
qdd = eom(params, th, phi, dth, dphi, 0);
% 3. Linearize the system at 0 (as shown in lecture)

ddth  = subs(expand(qdd(1)),[sin(phi),cos(phi)],[phi,1]);
ddphi = subs(expand(qdd(2)),[sin(phi),cos(phi)],[phi,1]);

eq1 = ddx == params.r*(ddth+ddphi);

%solve ddx to get equation for phi
phides = vpa(solve(eq1,phi),5)

%subs(qdd(1)+qdd(2),[tau],[0])
%x dot
%qd = [dth,dphi,qdd(1)- tau,qdd(2)];


%A = subs(jacobian(qd, [th, phi,dth,dphi]), [th, phi, dth, dphi, tau], [0 0 0 0 0] )  
%b = subs(jacobian(qd, tau), [th, phi, dth, dphi, tau], [0 0 0 0 0] )




