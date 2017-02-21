clc
clear all
close all


par.gamma = 0;
par.g = 9.81;
par.m1 = 2;
par.m2 = 2;
% par.m3 = 5;
% par.m4 = 2;
par.a1 = 1;
par.a2 = 1;
% par.a3 = 1;
% par.a4 = 0.7;
par.c1 = 0.2;
par.c2 = 0.2;
% par.c3 = 0.2;
% par.c4 = 0.2;
par.I1 = (1/15)*par.m1*par.a1^2;
par.I2 = (1/15)*par.m2*par.a2^2;
% par.I3 = (1/15)*par.m3*par.a3^2;
% par.I4 = (1/15)*par.m4*par.a4^2;

x0 = [0.3, -0.3, -0.2, 0.2,...
    0, 0, 0, 0];
%Question 4
% create trajectory
te = 2;
dt = 0.1;
xye = [-0.8;1.4];
tref = 0:dt:te;
[t_traj, phi_traj,xy_ref,xy_desired] = body_trajectory(x0,par,xye,te,dt);
phi_traj35 = interp1(tref,phi_traj,0.35)

% phi_traj451 = interp1(tref(2:end),phi_traj(1:end-1,1),0.35)
% phi_traj452 = interp1(tref(2:end),phi_traj(1:end-1,2),0.35)

% add (constant) leg angles to full state, required for animation
x12 = ones(length(phi_traj),1)*[0.3, -0.3];
x = [x12, phi_traj];

% animate
body_animate(t_traj,x, par)


%Question 5
%PD control
phi_body =  phi_traj(:,1)';
phi_arm = phi_traj(:,2)';
phi_body0 = x0(3);
phi_arm0 = x0(4);
options = odeset('AbsTol',1e-10,'RelTol',1e-8);
tend = 0.45;
ref = [[x0(3);phi_body(1:end-1)'],[x0(4);phi_arm(1:end-1)']];
[t5,x5] = ode45(@(t,y)body_eom_control(t,y,par,ref),[0 tend],x0,options);
% plot(t5,x5(:,3),t_traj)
x5(end,:)

% % animate
% close all
% x12 = ones(length(x5),1)*[0.3, -0.3];
% x = [x12, [x5(:,3),x5(:,4)]];
% x = interp1(t5,x,0:0.01:tend);
% body_animate(0:0.01:tend,x, par)

%Question 6
%PD Control
ref = [-0.5; -1;0;0];
tend = 10;
[t6,x6] = ode45(@(t,y)body_eom_control(t,y,par,ref),[0 tend],x0,options);
error = abs(ref - [x6(end,3);x6(end,4);x6(end,7);x6(end,8)])

% close all
% x12 = ones(length(x6),1)*[0.3, -0.3];
% x = [x12, [x6(:,3),x6(:,4)]];
% x = interp1(t6,x,0:0.01:tend);
% body_animate(0:0.01:tend,x, par)


