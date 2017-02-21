function body_animate(t,x,par)

%% Settings
colors = [0.7 0.5 0.2];

%% Create link shape
shape1 = linkshape(-par.a1);
shape2 = linkshape(-par.a2);
shape3 = linkshape(-par.a3);
shape4 = linkshape(-par.a4);

%% Objects
animationfig = figure;
AxesHandle = axes('Parent',animationfig,  'Position',[0 0 1 1]);
link1 = patch('Parent',AxesHandle, 'FaceColor',colors(1,:));
link2 = patch('Parent',AxesHandle, 'FaceColor',colors(1,:));
link3 = patch('Parent',AxesHandle, 'FaceColor',colors(1,:));
link4 = patch('Parent',AxesHandle, 'FaceColor',colors(1,:));
floor = line('Parent',AxesHandle, 'Color',[0 0 0], 'LineWidth',2);


%% Animation

for n=1:length(t) 
    tic
    % state vector
    p1 = x(n,1);
    p2 = x(n,2);
    p3 = x(n,3);
    p4 = x(n,4);
    
    % leg positions
    pos1 = move(R(p1)*shape1,[-par.a1*sin(p1);par.a1*cos(p1)]);
    set(link1,'Xdata',pos1(1,:),'Ydata',pos1(2,:));
    pos2 = move(R(p2)*shape2,[-par.a1*sin(p1);par.a1*cos(p1)]);
    set(link2,'Xdata',pos2(1,:),'Ydata',pos2(2,:));
    pos3 = move(R(p3)*shape3,[-par.a1*sin(p1)-par.a3*sin(p3);par.a1*cos(p1)+par.a3*cos(p3)]);
    set(link3,'Xdata',pos3(1,:),'Ydata',pos3(2,:));
    pos4 = move(R(p4)*shape4,[-par.a1*sin(p1)-par.a3*sin(p3);par.a1*cos(p1)+par.a3*cos(p3)]);
    set(link4,'Xdata',pos4(1,:),'Ydata',pos4(2,:));    
    
    
    
    
    % floor and axis
    axis([-2,2,-1,3]);
    floor_pos = [-2, 2; 0 0];
    set(floor,'Xdata',floor_pos(1,:),'Ydata',floor_pos(2,:));
    axis equal
    drawnow
    
    td = toc;
    if n < length(t)
        pause(t(n+1) - t(n) - td)
    end
end




function shape = linkshape(l)
link_width = 0.1;
n   = linspace(pi/2,-pi/2,20);
top_arc    = (link_width/2)*[sin(n);cos(n)];
bottom_arc = (link_width/2)*[-sin(n);-cos(n)];
if l<0
    bottom_arc(2,:) = bottom_arc(2,:)+l;
else
    top_arc(2,:) = top_arc(2,:)+l;
end
shape = [top_arc, bottom_arc];


function rot = R(phi)
rot = [cos(phi)  -sin(phi);
       sin(phi)   cos(phi)];
   
function c = move(a, b)
c(1,:) = a(1,:) + b(1);
c(2,:) = a(2,:) + b(2);

        
