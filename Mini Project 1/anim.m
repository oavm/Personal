function anim(q)

global q

disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

try
pause(1);
if (clientID>-1)
disp('Connected to remote API server');
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);
    disp('TEST')
    
[res1,jointHandle1]=vrep.simxGetObjectHandle(clientID,'joint1',vrep.simx_opmode_oneshot_wait);
[res2,jointHandle2]=vrep.simxGetObjectHandle(clientID,'joint2',vrep.simx_opmode_oneshot_wait);
[res3,jointHandle3]=vrep.simxGetObjectHandle(clientID,'joint3',vrep.simx_opmode_oneshot_wait);
[res4,jointHandle4]=vrep.simxGetObjectHandle(clientID,'joint4',vrep.simx_opmode_oneshot_wait);
[res5,jointHandle5]=vrep.simxGetObjectHandle(clientID,'joint5',vrep.simx_opmode_oneshot_wait);
[res6,jointHandle6]=vrep.simxGetObjectHandle(clientID,'joint6',vrep.simx_opmode_oneshot_wait);
[res7,jointHandle7]=vrep.simxGetObjectHandle(clientID,'joint7',vrep.simx_opmode_oneshot_wait);
[res8,jointHandle8]=vrep.simxGetObjectHandle(clientID,'joint8',vrep.simx_opmode_oneshot_wait);  
  
   
if (res1==vrep.simx_error_noerror)
     
while(vrep.simxGetConnectionId(clientID)>-1)
    
for i = 1:length(q)

    
% [err1]=vrep.simxSetJointPosition(clientID,jointHandle1,-45*pi/180,vrep.simx_opmode_streaming);
%     disp('TEST')
% 
% [err1]=vrep.simxSetJointPosition(clientID,jointHandle1,45*pi/180,vrep.simx_opmode_streaming);
% disp(err1)

[err1]=vrep.simxSetJointPosition(clientID,jointHandle1,q(i,1),vrep.simx_opmode_streaming);
[err2]=vrep.simxSetJointPosition(clientID,jointHandle2,q(i,2),vrep.simx_opmode_streaming);
[err3]=vrep.simxSetJointPosition(clientID,jointHandle3,q(i,3),vrep.simx_opmode_streaming);
[err4]=vrep.simxSetJointPosition(clientID,jointHandle4,q(i,4),vrep.simx_opmode_streaming);
[err5]=vrep.simxSetJointPosition(clientID,jointHandle5,q(i,5),vrep.simx_opmode_streaming);
[err6]=vrep.simxSetJointPosition(clientID,jointHandle6,q(i,6),vrep.simx_opmode_streaming);
[err7]=vrep.simxSetJointPosition(clientID,jointHandle7,q(i,7),vrep.simx_opmode_streaming);
[err8]=vrep.simxSetJointPosition(clientID,jointHandle8,q(i,8),vrep.simx_opmode_streaming);


pause(0.02)

% [err1]=vrep.simxSetJointPosition(clientID,jointHandle1,0,vrep.simx_opmode_streaming);
% [err2]=vrep.simxSetJointPosition(clientID,jointHandle2,0,vrep.simx_opmode_streaming);
% [err3]=vrep.simxSetJointPosition(clientID,jointHandle3,0,vrep.simx_opmode_streaming);
% [err4]=vrep.simxSetJointPosition(clientID,jointHandle4,0,vrep.simx_opmode_streaming);
% [err5]=vrep.simxSetJointPosition(clientID,jointHandle5,0,vrep.simx_opmode_streaming);
% [err6]=vrep.simxSetJointPosition(clientID,jointHandle6,0,vrep.simx_opmode_streaming);
% [err7]=vrep.simxSetJointPosition(clientID,jointHandle7,0,vrep.simx_opmode_streaming);
% [err8]=vrep.simxSetJointPosition(clientID,jointHandle8,0,vrep.simx_opmode_streaming);


end
pause(2)
disp(err8)



       
end
end


else
disp('Failed connecting to remote API server');
end 

catch err
vrep.simxFinish(clientID); % close the line if still open
vrep.delete(); % call the destructor!
end; 
disp('Program ended');
end