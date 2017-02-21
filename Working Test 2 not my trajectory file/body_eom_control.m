function [statederivative,TA,TB,TC,TD] = body_eom_control(t, state, par,ref)

m1 = par.m1;
m2 = par.m2;
m3 = par.m3;
m4 = par.m4;
I1 = par.I1;
I2 = par.I2;
I3 = par.I3;
I4 = par.I4;
c1 = par.c1;
c2 = par.c2;
c3 = par.c3;
c4 = par.c4;
a1 = par.a1;
a2 = par.a2;
a3 = par.a3;
a4 = par.a4;
g = par.g;

%Controller


[TA,TB,TC,TD] = body_control(t,state,ref);

[positions,velocities] = bodyq2x(state,par);

x1 = positions(1);
y1 = positions(2);
phi1 = positions(3);
x2 = positions(4);
y2 = positions(5);
phi2 = positions(6);
x3 = positions(7);
y3 = positions(8);
phi3 = positions(9);
x4 = positions(10);
y4 = positions(11);
phi4 = positions(12);

x1d = velocities(1);
y1d = velocities(2);
phi1d = velocities(3);
x2d = velocities(4);
y2d = velocities(5);
phi2d = velocities(6);
x3d = velocities(7);
y3d = velocities(8);
phi3d = velocities(9);
x4d = velocities(10);
y4d = velocities(11);
phi4d = velocities(12);



theta1 = phi1;
theta2 = phi2;
theta3 = phi3;
theta4 = phi4;

theta1d = phi1d;
theta2d = phi2d;
theta3d = phi3d;
theta4d = phi4d;


S = [...
 I1 + a1^2*m2*cos(theta1)^2 + a1^2*m3*cos(theta1)^2 + a1^2*m4*cos(theta1)^2 + a1^2*m2*sin(theta1)^2 + a1^2*m3*sin(theta1)^2 + a1^2*m4*sin(theta1)^2 + m1*cos(theta1)^2*(a1 - c1)^2 + m1*sin(theta1)^2*(a1 - c1)^2, - a1*c2*m2*cos(theta1)*cos(theta2) - a1*c2*m2*sin(theta1)*sin(theta2), a1*a3*m4*cos(theta1)*cos(theta3) + a1*c3*m3*cos(theta1)*cos(theta3) + a1*a3*m4*sin(theta1)*sin(theta3) + a1*c3*m3*sin(theta1)*sin(theta3), - a1*c4*m4*cos(theta1)*cos(theta4) - a1*c4*m4*sin(theta1)*sin(theta4);
                                                                                                                                            - a1*c2*m2*cos(theta1)*cos(theta2) - a1*c2*m2*sin(theta1)*sin(theta2),                    m2*c2^2*cos(theta2)^2 + m2*c2^2*sin(theta2)^2 + I2,                                                                                                                                         0,                                                                     0;
                                                                        a1*a3*m4*cos(theta1)*cos(theta3) + a1*c3*m3*cos(theta1)*cos(theta3) + a1*a3*m4*sin(theta1)*sin(theta3) + a1*c3*m3*sin(theta1)*sin(theta3),                                                                     0,                                        m4*a3^2*cos(theta3)^2 + m4*a3^2*sin(theta3)^2 + m3*c3^2*cos(theta3)^2 + m3*c3^2*sin(theta3)^2 + I3, - a3*c4*m4*cos(theta3)*cos(theta4) - a3*c4*m4*sin(theta3)*sin(theta4);
                                                                                                                                            - a1*c4*m4*cos(theta1)*cos(theta4) - a1*c4*m4*sin(theta1)*sin(theta4),                                                                     0,                                                                     - a3*c4*m4*cos(theta3)*cos(theta4) - a3*c4*m4*sin(theta3)*sin(theta4),                    m4*c4^2*cos(theta4)^2 + m4*c4^2*sin(theta4)^2 + I4];
C = [1,0,0,0;
     0,1,0,0];
        
 
b = [...
 TB - a1*m2*sin(theta1)*(a1*cos(theta1)*theta1d^2 - c2*cos(theta2)*theta2d^2) - a1*m3*sin(theta1)*(a1*cos(theta1)*theta1d^2 + c3*cos(theta3)*theta3d^2) + a1*m2*cos(theta1)*(a1*sin(theta1)*theta1d^2 - c2*sin(theta2)*theta2d^2) + a1*m3*cos(theta1)*(a1*sin(theta1)*theta1d^2 + c3*sin(theta3)*theta3d^2) + a1*g*m2*sin(theta1) + a1*g*m3*sin(theta1) + a1*g*m4*sin(theta1) + g*m1*sin(theta1)*(a1 - c1) - a1*m4*sin(theta1)*(a1*cos(theta1)*theta1d^2 + a3*cos(theta3)*theta3d^2 - c4*cos(theta4)*theta4d^2) + a1*m4*cos(theta1)*(a1*sin(theta1)*theta1d^2 + a3*sin(theta3)*theta3d^2 - c4*sin(theta4)*theta4d^2);
                                                                                                                                                                                                                                                                                                                                                                                                                                        TC + c2*m2*sin(theta2)*(a1*cos(theta1)*theta1d^2 - c2*cos(theta2)*theta2d^2) - c2*m2*cos(theta2)*(a1*sin(theta1)*theta1d^2 - c2*sin(theta2)*theta2d^2) - c2*g*m2*sin(theta2);
                                                                                                                                                                                          TD - TC - TB - c3*m3*sin(theta3)*(a1*cos(theta1)*theta1d^2 + c3*cos(theta3)*theta3d^2) + c3*m3*cos(theta3)*(a1*sin(theta1)*theta1d^2 + c3*sin(theta3)*theta3d^2) + a3*g*m4*sin(theta3) + c3*g*m3*sin(theta3) - a3*m4*sin(theta3)*(a1*cos(theta1)*theta1d^2 + a3*cos(theta3)*theta3d^2 - c4*cos(theta4)*theta4d^2) + a3*m4*cos(theta3)*(a1*sin(theta1)*theta1d^2 + a3*sin(theta3)*theta3d^2 - c4*sin(theta4)*theta4d^2);
                                                                                                                                                                                                                                                                                                                                                                                  c4*m4*sin(theta4)*(a1*cos(theta1)*theta1d^2 + a3*cos(theta3)*theta3d^2 - c4*cos(theta4)*theta4d^2) - c4*g*m4*sin(theta4) - TD - c4*m4*cos(theta4)*(a1*sin(theta1)*theta1d^2 + a3*sin(theta3)*theta3d^2 - c4*sin(theta4)*theta4d^2);
];


unknowns = [S, C'; C, zeros(2)]\[b;0;0];

accelerations = unknowns(1:4);

statederivative = [velocities([3,6,9,12]); accelerations];

% statederivative = bodyx2q(velocities, accelerations);



end

