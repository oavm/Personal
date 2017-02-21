
% Copyright (c), Gabriel Lopes, Delft University of Technology, 2015
%Model and Control
clear all;
global Q0 Q1 Q2 Q3 Q4;

for h=0:5
% initialize parameters

modelinit

% initial state: position and velocity
% q=[-0.5966   -0.5701   -0.1615    0.8877   -0.0407    0.5620    0.0311    0.1274]';
 q=[0 0 0 0 0 0 0 0]';
 qd=[0 0 0 0 0 0 0 0]';
 qdd=[0 0 0 0 0 0 0 0]';

% sampling time
ts=0.01;

% parameters for variable step 1st order Newton integration
tsmax=0.1;
tsmin=0.0001;
gdes=0.1;

t=0;
k=1;

eold = 0;
eqold = 0;

% main loop
counter=0;

while t<=10
kf=500;
for k=1:kf


% compute system matrices from state information
[M Gamma G J HLocal HGlobal]=ComputeModel(q',parameters,gravity,mass,inertias);

% increment time
t=t+ts;
time(k+(counter*kf))=t;


% extract griper position and Jacobian
EndEffectorPosition=HGlobal(1:3,end);
EndEffectorJacobian=J(:,end-7:end);


% compute the analytical Jacobian from the geometrical Jacobian
for i=1:8
GlobalEndEffectorJacobian(:,i)=UnTilde(Tilde(EndEffectorJacobian(:,i))*HGlobal(1:4,end-3:end));
end



% the desired gripper position in time

%First position
if (t>=0) && (t<3) 
desiredEndEffectorPosition = [0.5;0.8;-0.4];
desiredEndEffectorPositionq = [-0.1044   -0.8809   -0.8134    0.3912   -1.6406    1.0098 -0.0755    0.0874]';
end

%Second Position

if (t>=3) && (t<6)
desiredEndEffectorPosition = [0.5;-0.8;0.4];
desiredEndEffectorPositionq = [-1.65,-0.34,-1.04,0.06,-0.96,1.02,0.01,0.18;]';
end



%Third Position
if (t>=6) && (t<9)
desiredEndEffectorPosition = [0.5;0.8;0.4];
desiredEndEffectorPositionq = [-0.26,-1.15,-0.76,1.54,-0.35,0.45,-0.38,0.12;]';
end


% compute the control laws

if h==0
%For no controller to show the system dynamics
u = 0;
end

if h==1
%Proportional Workspace control
u = -50*GlobalEndEffectorJacobian'*([0;0;0;EndEffectorPosition]-[0;0;0; desiredEndEffectorPosition ]);
end

if h==2
%PD Workspace control
Kd = 14; Kp = 65;
e=[0;0;0;EndEffectorPosition]-[0;0;0;desiredEndEffectorPosition];
e_dot=(e-eold)/ts;
u = -Kd*GlobalEndEffectorJacobian'*e_dot - Kp*GlobalEndEffectorJacobian'*e+G; 
eold = e;
end

if h==3
%PD Configuration control
% Kdq = 15*diag([1,1,1,1,1,1,1,1],0); 
% Kpq =70*diag([1,1,1,1,1,1,1,1],0);
Kdq=15;
Kpq=70;
eq=q-desiredEndEffectorPositionq;
eq_dot=(eq-eqold)/ts;
u = -Kdq*eq_dot - Kpq*eq+G; 
eqold = eq;
end

if h==4
%Computed Torque Configuration control
Kd=15*diag([1 1 1 1 1 1 1 0.5]);
Kp=70*diag([1 1 1 1 1 10 1 1]);
eq=q-desiredEndEffectorPositionq;
eq_dot=(eq-eqold)/ts;
u=M*(zeros(8,1)-Kd*eq_dot-Kp*eq)+Coriolis(Gamma,qd')'+G;
eqold = eq;
end



% compute gradients for 1st order integration
g1=(inv(M)*(-Coriolis(Gamma,qd')'-G-1*qd+u));
g2=qd;

% find the best sampling rate
ts=max(min(gdes/max(abs([g1 ; g2])),tsmax),tsmin);

% solve dynamics
qdnext = qd + ts*g1;
qnext  = q + ts*g2;

% update state
qd = qdnext;
q = qnext;

% save state information
qres((k+(counter*kf)),:)=q;
endeffector((k+(counter*kf)),:)=EndEffectorPosition;
desiredendeffector((k+(counter*kf)),:)=desiredEndEffectorPosition;

%Homogeneus transformation matrices
for l=1:8
    assignin('base',['a',num2str(l)],HGlobal(1:3,4*l,1));
end

%Save vectors of x
x1((k+(counter*kf)),1)=a1(1);
x2((k+(counter*kf)),1)=a2(1);
x3((k+(counter*kf)),1)=a3(1);
x4((k+(counter*kf)),1)=a4(1);
x5((k+(counter*kf)),1)=a5(1);
x6((k+(counter*kf)),1)=a6(1);
x7((k+(counter*kf)),1)=a7(1);
x8((k+(counter*kf)),1)=a8(1);

%Save vectors of y
y1((k+(counter*kf)),1)=a1(2);
y2((k+(counter*kf)),1)=a2(2);
y3((k+(counter*kf)),1)=a3(2);
y4((k+(counter*kf)),1)=a4(2);
y5((k+(counter*kf)),1)=a5(2);
y6((k+(counter*kf)),1)=a6(2);
y7((k+(counter*kf)),1)=a7(2);
y8((k+(counter*kf)),1)=a8(2);

%Save vectors of z
z1((k+(counter*kf)),1)=a1(3);
z2((k+(counter*kf)),1)=a2(3);
z3((k+(counter*kf)),1)=a3(3);
z4((k+(counter*kf)),1)=a4(3);
z5((k+(counter*kf)),1)=a5(3);
z6((k+(counter*kf)),1)=a6(3);
z7((k+(counter*kf)),1)=a7(3);
z8((k+(counter*kf)),1)=a8(3);

end
counter=counter+1;
end

% %Resampling
timeq=0:0.03:10;
time=time(1:length(x1));
x1=interp1(time,x1,timeq)';
x2=interp1(time,x2,timeq)';
x3=interp1(time,x3,timeq)';
x4=interp1(time,x4,timeq)';
x5=interp1(time,x5,timeq)';
x6=interp1(time,x6,timeq)';
x7=interp1(time,x7,timeq)';
x8=interp1(time,x8,timeq)';

y1=interp1(time,y1,timeq)';
y2=interp1(time,y2,timeq)';
y3=interp1(time,y3,timeq)';
y4=interp1(time,y4,timeq)';
y5=interp1(time,y5,timeq)';
y6=interp1(time,y6,timeq)';
y7=interp1(time,y7,timeq)';
y8=interp1(time,y8,timeq)';

z1=interp1(time,z1,timeq)';
z2=interp1(time,z2,timeq)';
z3=interp1(time,z3,timeq)';
z4=interp1(time,z4,timeq)';
z5=interp1(time,z5,timeq)';
z6=interp1(time,z6,timeq)';
z7=interp1(time,z7,timeq)';
z8=interp1(time,z8,timeq)';

qres=interp1(time,qres,timeq);

assignin('base',['X',num2str(h)],[x1 x2 x3 x4 x5 x6 x7 x8]);
assignin('base',['Y',num2str(h)],[y1 y2 y3 y4 y5 y6 y7 y8]);
assignin('base',['Z',num2str(h)],[z1 z2 z3 z4 z5 z6 z7 z8]);
assignin('base',['Q',num2str(h)],qres);
end
% 




%Parameters for animation
%Size of the axis for the animation
xmin=-1;
xmax=1;
ymin=-1;
ymax=1;
zmin=-2;
zmax=1.5;

%Colors for the different links
c1=rand(1,3);
c2=rand(1,3);
c3=rand(1,3);
c4=rand(1,3);
c5=rand(1,3);
c6=rand(1,3);
c7=rand(1,3);
c8=rand(1,3);

%% Animation
for j=0:4
for i=1:length(x1-1)
     subplot(2,3,j+1)
     if j==0
         title('No controller');
     end
     if j==1
         title('Proportional');
     end
     if j==2
         title('PD Workspace');
     end
     if j==3
         title('PD Configuration');
     end
     if j==4
         title('Computed Torque Con.');
     end
     assignin('base',['h',num2str(j+1)],subplot(2,3,j+1));
    axis([xmin xmax ymin ymax zmin zmax])                                                                   %Define axis
    grid on
    line([0,0],[0,0],[zmin,0],'LineWidth',3.5,'Color',[0,0,0])                                          %Plot robot "base"
    x=evalin('base',['X',int2str(j)]);
    y=evalin('base',['Y',int2str(j)]);
    z=evalin('base',['Z',int2str(j)]);
    line([0 x(i,1)],[0 y1(i,1)],[0 z1(i,1)],'LineWidth',4,'Color',c8);                              %Plot link 1 
    line([x(i,1) x(i,2)],[y(i,1) y(i,2)],[z(i,1) z(i,2)],'LineWidth',2,'Color',c1);                      %Plot link 2 
    line([x(i,2) x(i,3)],[y(i,2) y(i,3)],[z(i,2) z(i,3)],'LineWidth',4,'Color',c2);                      %Plot link 3
    line([x(i,3) x(i,4)],[y(i,3) y(i,4)],[z(i,3) z(i,4)],'LineWidth',2,'Color',c3);                      %Plot link 4 
    line([x(i,4) x(i,5)],[y(i,4) y(i,5)],[z(i,4) z(i,5)],'LineWidth',4,'Color',c4);                      %Plot link 5
    line([x(i,5) x(i,6)],[y(i,5) y(i,6)],[z(i,5) z(i,6)],'LineWidth',2,'Color',c5);                      %Plot link 6
    line([x(i,6) x(i,7)],[y(i,6) y(i,7)],[z(i,6) z(i,7)],'LineWidth',4,'Color',c6);                      %Plot link 7 
    line([x(i,7) x(i,8)],[y(i,7) y(i,8)],[z(i,7) z(i,8)],'LineWidth',2,'Color',c7);                      %Plot link 8 
    
    hold on
    hold off
    
    
    drawnow
    %pause(0.00005)
   if i~=(length(x1-1))
    cla(evalin('base',['h',int2str(j+1)]))
   end
end

end


%% note that you are using a vairable time integrator, so when you plot (or
% animate) don't forget to include the "time" vector.
figure(2);
subplot(2,3,1)
plot(timeq,Q0)
title('No controller')

subplot(2,3,2)
plot(timeq,Q1)
title('Proportional')

subplot(2,3,3)
plot(timeq,Q2)
title('PD Workspace')

subplot(2,3,4)
plot(timeq,Q3)
title('PD Configuration')

subplot(2,3,5)
plot(timeq,Q4)
title('Computed Torque Con.')



figure(3);
subplot(2,3,1)
plot(timeq,X0(:,8),timeq,Y0(:,8),timeq,Z0(:,8));
legend('x','y','z');
title('No controller')

subplot(2,3,2)
plot(timeq,X1(:,8),timeq,Y1(:,8),timeq,Z1(:,8));
legend('x','y','z');
title('Proportional')

subplot(2,3,3)
plot(timeq,X2(:,8),timeq,Y2(:,8),timeq,Z2(:,8));
legend('x','y','z');
title('PD Workspace')

subplot(2,3,4)
plot(timeq,X3(:,8),timeq,Y3(:,8),timeq,Z3(:,8));
legend('x','y','z');
title('PD Configuration')

subplot(2,3,5)
plot(timeq,X4(:,8),timeq,Y4(:,8),timeq,Z4(:,8));
legend('x','y','z');
title('Computed Torque Con.')


save('time','time')
save('qres','qres')
save('endeffector','endeffector')
save('desiredendeffector','desiredendeffector')

% %%V-REP
% anim(Q3) % Run the code to send the data to V-rep