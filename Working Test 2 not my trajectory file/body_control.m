function [TA,TB,TC,TD] = body_control(t,state,ref)
time = t;

tf = isa(ref,'double');

if tf == 0;
    ref = eval(ref);
end

%Defintion of reference
q3r = ref(1);
q4r = ref(2);
q3dr = ref(3);
q4dr = ref(4);

if length(ref) > 4
    te = 2;
    dt = 0.1;
    tref = 0:dt:te;
    q3r = interp1(tref,ref(:,1),t);
    q4r = interp1(tref,ref(:,2),t);
    
    for i=1:length(tref)
        if t>=tref(i) && t<tref(i+1)
            tp = tref(i+1);
%             extrap3 = ref(i+1,1);
%             extrap4 = ref(i+1,2);
            extrap3 = ref(i+1,1);
            extrap4 = ref(i+1,2);
        end
    end
%     q3dr=(q3r-interp1(tref,ref(:,1),t,'linear'))/(tp-t);
%     q4dr=(q4r-interp1(tref,ref(:,2),t,'linear'))/(tp-t);
%     q3dr=(q3r - extrap3)/(t-tp);
%     q4dr=(q4r - extrap4)/(t-tp);
    q3dr=(extrap3 - q3r)/(tp-t);
    q4dr=(extrap4 - q4r)/(tp-t);
end




%Definition of the states
q = state(1:4);
qd = state(5:8);


%Control constants for phi3
Kpq3 = 250;
Kdq3 = 50;


%Control constants phi4
Kpq4 = 250;
Kdq4 = 35;


%Vector of references
qr = [q(1);
      q(2);
      q3r;
      q4r];
  
qdr = [0;
       0;
       q3dr;
       q4dr];

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
TC = u(3)
TD = u(4)