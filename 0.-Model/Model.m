function [qdot] = Model(q,u)                          


%% Initialization of the variables
q1=q(1);
q2=q(2);
q1d=q(3);
q2d=q(4);
Tau=q(5);


%% System parameters
% l1=0.1;
% l2=0.1;
% m1=0.1713;
% m2=0.0613;
% c1=0.0431;
% c2=0.0482;
% I1=0.0369;
% I2=0.00011082;
% g=9.81;
% d1=3.7943;
% d2=0.00057407;
% G=-18.5760; %Data1
Taue=0.03;

l1=0.1;
l2=0.1;
m1=0.18;
m2=0.06;
c1=0.06;
c2=0.045;
I1=0.037;
I2=0.00011;
g=9.81;
d1=4;
d2=0.00062;
G=50;


%% Equation of motion of 

q1dd=(I2*G*u - I2*d1*q1d + I2*d2*q2d + G*u*c2^2*m2 - c2^2*d1*m2*q1d + c2^3*g*m2^2*sin(q1 - q2) + c1*c2^3*m2^2*q2d^2*sin(q2) + c2^2*g*l1*m2^2*sin(q1) + I2*c1*g*m1*sin(q1) + I2*g*l1*m2*sin(q1) + 2*I2*c2*g*m2*sin(q1 - q2) + c1*c2*d2*m2*q2d*cos(q2) + I2*c1*c2*m2*q2d^2*sin(q2) + c1*c2^2*g*m2^2*sin(q1 - q2)*cos(q2) + c1*c2^2*g*m1*m2*sin(q1) - c1^2*c2^2*m2^2*q1d*q2d*cos(q2)*sin(q2) - I2*c1*c2*m2*q1d*q2d*sin(q2) + I2*c2*l1*m2*q1d*q2d*sin(q2) + c1*c2^2*l1*m2^2*q1d*q2d*cos(q2)*sin(q2))/(I1*I2 + c1^2*c2^2*m2^2 + I2*c1^2*m1 + I1*c2^2*m2 + I2*c1^2*m2 + I2*c2^2*m2 + c1^2*c2^2*m1*m2 - c1^2*c2^2*m2^2*cos(q2)^2 - 2*I2*c1*c2*m2*cos(q2));
q2dd=-(I2*G*u - I2*d1*q1d + I1*d2*q2d + I2*d2*q2d + c1^2*d2*m1*q2d + c1^2*d2*m2*q2d + G*u*c1*c2*m2*cos(q2) + I2*c1*g*m1*sin(q1) + c1^2*c2*g*m2^2*sin(q1 - q2) + I2*g*l1*m2*sin(q1) + I1*c2*g*m2*sin(q1 - q2) + 2*I2*c2*g*m2*sin(q1 - q2) - c1^3*c2*m2^2*q1d*q2d*sin(q2) + c1^2*c2^2*m2^2*q2d^2*cos(q2)*sin(q2) - c1*c2*d1*m2*q1d*cos(q2) + I2*c1*c2*m2*q2d^2*sin(q2) + c1*c2^2*g*m2^2*sin(q1 - q2)*cos(q2) + c1^2*c2*g*m1*m2*sin(q1 - q2) - c1^3*c2*m1*m2*q1d*q2d*sin(q2) + c1*c2*g*l1*m2^2*cos(q2)*sin(q1) + c1^2*c2*g*m1*m2*cos(q2)*sin(q1) + c1^2*c2*l1*m2^2*q1d*q2d*sin(q2) - I1*c1*c2*m2*q1d*q2d*sin(q2) - I2*c1*c2*m2*q1d*q2d*sin(q2) + I1*c2*l1*m2*q1d*q2d*sin(q2) + I2*c2*l1*m2*q1d*q2d*sin(q2) + c1^2*c2*l1*m1*m2*q1d*q2d*sin(q2))/(I1*I2 + c1^2*c2^2*m2^2 + I2*c1^2*m1 + I1*c2^2*m2 + I2*c1^2*m2 + I2*c2^2*m2 + c1^2*c2^2*m1*m2 - c1^2*c2^2*m2^2*cos(q2)^2 - 2*I2*c1*c2*m2*cos(q2));
Taud=G*u/Taue-u/Taue;



qdot =[q1d;q2d;q1dd;q2dd;Taud];



end