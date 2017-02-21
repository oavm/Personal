% clear all
clc
close all

h=0.05;

%Symbolic parameter variables
l1=sym('l1');
l2=sym('l2');
m1=sym('m1');
m2=sym('m2');
c1=sym('c1');
c2=sym('c2');
I1=sym('I1');
I2=sym('I2');
g=sym('g');
d1=sym('d1');
d2=sym('d2');
Taue=sym('Taue');

%Symbolic variables for states
q1=sym('q1');
q2=sym('q2');
q1d=sym('q1d');
q2d=sym('q2d');
q1dd=sym('q1dd');
q2dd=sym('q2dd');
Tau=sym('Tau');
Taud=sym('Taud');
Taudd=sym('Taudd');
u=sym('u');
G=sym('G');

%Definition of parameters
l1=0.1;
l2=0.1;
m1=0.1713;
m2=0.0613;
c1=0.0431;
c2=0.0482;
I1=0.0369;
I2=0.00011082;
g=9.81;
d1=3.7943;
d2=0.00057407;
Taue=0.03;
% G=18.5760; %Data1
G=50;

%Kinectic Energy
T=0.5*m1*c1^2*q1d^2 + 0.5*I1*q1d^2 + 0.5*I2*(q1d+q2d)^2 + 0.5*m2*(l1^2*q1d^2 + c2^2*q2d^2 + 2*l1*c2*q1d*q2d*cos(q2));

%Potential Energy
V=(1 + cos(q1))*c1*m1*g + (1+(cos(q1))*l1 + (1+cos(q2-q1))*c2)*m2*g;

%Dissipation terms
D=0.5*d1*q1d^2 + 0.5*d2*q2d^2;

%Lagrangian
L=T-V;

%Derivatives
dLdq1=diff(L,q1);
dLdq2=diff(L,q2);
dLdq1d=diff(L,q1d);
dLdq2d=diff(L,q2d);
dDdq1d=diff(D,q1d);
dDdq2d=diff(D,q2d);

%Time derivatives
dt1=(m1*c1^2 + I1 + m2*c1^2 + I2)*q1dd + I2*q2dd + m2*c1*c2*(q2dd*cos(q2)-q2d^2*sin(q2));
dt2=m2*c2^2*q2dd + m2*c1*c2*(q1dd*cos(q2)-q1d*q2d*sin(q2)) + I2*q1dd +I2*q2dd;

%Equations of motion
eq1=dt1-dLdq1+dDdq1d==G*u-Taue*Taudd;
eq2=dt2-dLdq2+dDdq2d==0;

%Solving for q1dd and q2dd
P=solve(eq1,eq2,q1dd,q2dd);

%Linearization
c1q1=diff(P.q1dd,q1);
c1q2=diff(P.q1dd,q2);
c1q1d=diff(P.q1dd,q1d);
c1q2d=diff(P.q1dd,q2d);
c1Tau=diff(P.q1dd,u);

c2q1=diff(P.q2dd,q1);
c2q2=diff(P.q2dd,q2);
c2q1d=diff(P.q2dd,q1d);
c2q2d=diff(P.q2dd,q2d);
c2Tau=diff(P.q2dd,u);

workpoint = [0,0,0,0,0];

C1q1=double(subs(c1q1,[q1,q2,q1d,q2d,Tau],workpoint));
C1q2=double(subs(c1q2,[q1,q2,q1d,q2d,Tau],workpoint));
C1q1d=double(subs(c1q1d,[q1,q2,q1d,q2d,Tau],workpoint));
C1q2d=double(subs(c1q2d,[q1,q2,q1d,q2d,Tau],workpoint));
C1Tau=double(subs(c1Tau,[q1,q2,q1d,q2d,Tau],workpoint));

C2q1=double(subs(c2q1,[q1,q2,q1d,q2d,Tau],workpoint));
C2q2=double(subs(c2q2,[q1,q2,q1d,q2d,Tau],workpoint));
C2q1d=double(subs(c2q1d,[q1,q2,q1d,q2d,Tau],workpoint));
C2q2d=double(subs(c2q2d,[q1,q2,q1d,q2d,Tau],workpoint));
C2Tau=double(subs(c2Tau,[q1,q2,q1d,q2d,Tau],workpoint));

A=[0 0 1 0 0;0 0 0 1 0;C1q1 C1q2 C1q1d C1q2d 1;C2q1 C2q2 C2q1d C2q2d 0;0 0 0 0 -1/Taue];
B=[0;0;C1Tau;C2Tau;G/Taue];
C=[0 1 0 0 0];
D=0;


%State space controller
prompt1 = 'Input desired overshoot percentage: ';
prompt2 = 'Input desired settling time: ';
Ts=input(prompt2);
OS=input(prompt1)/100;
zeta=(-log(OS))/(sqrt(pi^2+(log(OS))^2));
Wn=5/(zeta*Ts);

pd=[1 2*zeta*Wn Wn^2];
Cpoles=roots(pd);

System=ss(A,B,C,D);
ssd=c2d(System,h);
[Ad,Bd,Cd,Dd]=ssdata(ssd);

NewCpole=real(Cpoles(1))*5;
Pp=[Cpoles(1) Cpoles(2) NewCpole NewCpole+0.01 NewCpole+0.02];
K=place(A,B,Pp);
K0=inv(C*inv(-A+B*K)*B);

PpL=[real(Pp(1))*5 real(Pp(2))*5+0.011 real(Pp(3))*5+0.02 real(Pp(4))*5+0.03 real(Pp(5))*5+0.04];
L=(place(A',C',PpL))';

Dpoles=[exp(h*Cpoles(1)) exp(h*Cpoles(2)) exp(h*Pp(3)) exp(h*Pp(4)) exp(h*Pp(5))];
Ppd=[Dpoles(1) Dpoles(2) Dpoles(3) Dpoles(4) Dpoles(5)];
Kd=place(Ad,Bd,Ppd);
K0d=inv(Cd*inv(eye(5)-Ad+Bd*Kd)*Bd);

PpLD=[exp(h*PpL(1)) exp(h*PpL(2)) exp(h*PpL(3)) exp(h*PpL(4)) exp(h*PpL(5))];
Ld=(place(Ad',Cd',PpLD))';

%Best values 2.5,5


