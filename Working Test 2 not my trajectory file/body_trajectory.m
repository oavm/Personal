function [t, phi34_ref, xy_ref,xy_desired] = body_trajectory(x0,par,xye,te,dt)

% Initial xye
phi3(1) = x0(3);
phi4(1) = x0(4);

R_phi3 = [cos(phi3+pi/2) -sin(phi3+pi/2);
          sin(phi3+pi/2) cos(phi3+pi/2)];
R_phi4 = [cos(phi4-pi/2) -sin(phi4-pi/2);
           sin(phi4-pi/2) cos(phi4-pi/2)];
       
a3 = par.a3;
a4 = par.a4;

xy_initial = R_phi3*[a3;0] + R_phi4*[a4;0];

t = 0:dt:te;
n = length(t);

trajectory_position_x = linspace(xy_initial(1),xye(1),n);
trajectory_position_y = linspace(xy_initial(2),xye(2),n);
trajectory_position_x(22) = trajectory_position_x(end);
trajectory_position_y(22) = trajectory_position_y(end);

% Jacobian

syms phi3 phi4 a3 a4 phi3d phi4d
phi = [phi3; phi4];
R_phi3 = [cos(phi3+pi/2) -sin(phi3+pi/2);
          sin(phi3+pi/2) cos(phi3+pi/2)];
R_phi4 = [cos(phi4-pi/2) -sin(phi4-pi/2);
           sin(phi4-pi/2) cos(phi4-pi/2)];

E = R_phi3*[a3;0] + R_phi4*[a4;0];

jacobian_E = jacobian(E,phi);

a3 = par.a3;
a4 = par.a4;
phi3 = -0.200;
phi4 = 0.200;

for i = 1:n
    phi3 = phi3(i);
    phi4 = phi4(i);
    xy_initial_new = eval(E); 
    delta_xy = [trajectory_position_x(i+1)-xy_initial_new(1);trajectory_position_y(i+1)-xy_initial_new(2)];
    deltaphi = eval(inv(jacobian_E)*delta_xy);
    phi_new = [phi3;phi4]+deltaphi;
    phi3(i+1) = phi_new(1);
    phi4(i+1) = phi_new(2);
    phi34_ref(i,1) = phi_new(1);
    phi34_ref(i,2) = phi_new(2);
    xy_ref(i,1) = xy_initial_new(1);
    xy_ref(i,2) = xy_initial_new(2);
end

xy_desired = [trajectory_position_x;trajectory_position_y];
