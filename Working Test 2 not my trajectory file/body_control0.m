function [TA,TB,TC,TD] = body_control0(state)


%Definition of the states
q = state(1:4);
qd = state(5:8);

%Control constants for phi3
Kpq3 = 500;
Kdq3 = 100;

%Reference angle phi3
q3r = -0.15;
% q3r = 4*sin(t) + pi/2;

%Control constants phi4
Kpq4 = 500;
Kdq4 = 70;

%Reference phi4
q4r = -1.75*pi;
% q4r = 1/2*sin(t) + 0.1;

%Vector of references
qr = [q(1);
      q(2);
      q3r;
      q4r];
qdr = [0;
       0;
       0;
        0];

%Defining matrices for control
Kp = [0,0,0,0;
      0,0,0,0;
      0,0,Kpq3,0;
      0,0,0,Kpq4];
Kd = [0,0,0,0;
      0,0,0,0;
      0,0,Kdq3,0;
      0,0,0,Kdq4];
 
%Control input
u = -Kp*(qr-q)-(Kd*(qdr-qd));

TA = u(1);
TB = u(2);
TC = u(3);
TD = u(4);