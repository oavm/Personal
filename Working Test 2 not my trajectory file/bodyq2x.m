function [positions,velocities] = bodyq2x(state,par);
m1 = par.m1; I1 = par.I1; c1 = par.c1; a1 = par.a1;
m2 = par.m2; I2 = par.I2; c2 = par.c2; a2 = par.a2;
m3 = par.m3; I3 = par.I3; c3 = par.c3; a3 = par.a3;
m4 = par.m4; I4 = par.I4; c4 = par.c4; a4 = par.a4;
g  = par.g;

p1 = state(1); p2 = state(2); p3 = state(3); p4 = state(4);
qd = [state(5); state(6); state(7); state(8)];

xh = -a1*sin(p1);
yh =  a1*cos(p1);

positions = [ xh + c1*sin(p1);
      yh - c1*cos(p1);
      p1;
      xh + c2*sin(p2);
      yh - c2*cos(p2);
      p2;
      xh - c3*sin(p3);
      yh + c3*cos(p3);
      p3;
      xh - a3*sin(p3)+c4*sin(p4);
      yh + a3*cos(p3)-c4*cos(p4);
      p4];
 
T=[cos(p1)*(-a1+c1),             0,             0,             0
   sin(p1)*(-a1+c1),             0,             0,             0
                  1,             0,             0,             0
        -a1*cos(p1),    c2*cos(p2),             0,             0
        -a1*sin(p1),    c2*sin(p2),             0,             0
                  0,             1,             0,             0
        -a1*cos(p1),             0,   -c3*cos(p3),             0
        -a1*sin(p1),             0,   -c3*sin(p3),             0
                  0,             0,             1,             0
        -a1*cos(p1),             0,   -a3*cos(p3),    c4*cos(p4)
        -a1*sin(p1),             0,   -a3*sin(p3),    c4*sin(p4)
                  0,             0,             0,             1];
       
velocities = T*qd;