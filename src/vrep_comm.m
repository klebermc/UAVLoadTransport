function [sys,x0,str,ts] = vrep_comm_out(t,x,u,flag)
% Dispatch the flag. The switch function controls the calls to
% S-function routines at each simulation stage.
switch flag,
	case 0
		[sys,x0,str,ts] = mdlInitializeSizes; % Initialization
		
	case 3
		sys = mdlOutputs(t,x,u); % Calculate outputs
	
	case 9
		sys = mdlTerminate(t,x,u);
	
	case { 1, 2, 4 }
		sys = []; % Unused flags
	
	otherwise
	error(['Unhandled flag = ',num2str(flag)]); % Error handling
end;
% End of function vrep_comm.


%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0 = []; % No continuous states
str = []; % No state ordering
ts = [-1 0]; % Inherited sample time - sample time: [period, offset]

% speicfy that the simState for this s-function is same as the default
simStateCompliance = 'DefaultSimState';

global clientID;
global vrep;
global quad_base;
global targetObj;

if clientID==-1
	%Initiate the connection and simulation at Vrep
	vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
	vrep.simxFinish(-1); % just in case, close all opened connections
	clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
end

if (clientID>-1)
        disp('Connected to remote API server');	
		
		% enable the synchronous mode on the client:
		vrep.simxSynchronous(clientID,true);
		
        % Now try to retrieve data in a blocking fashion (i.e. a service call):
        [res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking);
        if (res==vrep.simx_return_ok)
            fprintf('Number of objects in the scene: %d\n',length(objs));
        else
            fprintf('Remote API function call returned with error code: %d\n',res);
		end
		
		%Get the required handles for the scene.
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		[~,Motor1]=vrep.simxGetObjectHandle(clientID,'Quadricopter_propeller_respondable1',vrep.simx_opmode_oneshot_wait);                 %
		[~,Motor2]=vrep.simxGetObjectHandle(clientID,'Quadricopter_propeller_respondable2',vrep.simx_opmode_oneshot_wait);                 %
		[~,Motor3]=vrep.simxGetObjectHandle(clientID,'Quadricopter_propeller_respondable3',vrep.simx_opmode_oneshot_wait);                 %
		[~,Motor4]=vrep.simxGetObjectHandle(clientID,'Quadricopter_propeller_respondable4',vrep.simx_opmode_oneshot_wait);                 %
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		
		%The manipulator sphere
		[~,targetObj] = vrep.simxGetObjectHandle(clientID,'Quadricopter_target',vrep.simx_opmode_oneshot_wait);
		[~] = vrep.simxSetObjectParent(clientID, targetObj, -1, true,vrep.simx_opmode_oneshot_wait);
		[~,targetPos] = vrep.simxGetObjectPosition(clientID,targetObj,-1, vrep.simx_opmode_streaming);
		[~,targetOrient] = vrep.simxGetObjectOrientation(clientID,targetObj,-1,vrep.simx_opmode_streaming);

		%The Quadrirotor base
		[~,quad_base] = vrep.simxGetObjectHandle(clientID,'Quadricopter_base',vrep.simx_opmode_oneshot_wait);
		[~,quadPos] = vrep.simxGetObjectPosition(clientID,quad_base,-1, vrep.simx_opmode_streaming);
		[~,quadLinearVel,quadAngularVel] = vrep.simxGetObjectVelocity(clientID,quad_base,vrep.simx_opmode_streaming);
		[~,quadOrient] = vrep.simxGetObjectOrientation(clientID,quad_base,-1,vrep.simx_opmode_streaming);
		[~,sp]=vrep.simxGetObjectPosition(clientID,targetObj,quad_base,vrep.simx_opmode_streaming);

		%Starting the simulation in V-REP.
		[errorCodeStart]=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);
        pause(2);
end
% end mdlInitializeSizes

%
%=============================================================================
% mdlOutputs
% Return the output vector for the S-function
%=============================================================================
%
function sys = mdlOutputs(t,x,u)
global clientID;
global vrep;
global quad_base;
global targetObj;

quadPos = [0,0,0];
quadOrient = [0,0,0];
targetPos = [0,0,0];
targetOrient = [0,0,0];

sys = [quadPos,quadOrient,targetPos,targetOrient];

if (clientID>-1)
	
	vrep.simxSynchronousTrigger(clientID);

% 	[returnCode,data]=vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_buffer); % Try to retrieve the streamed data
	[returnCode,targetPos]=vrep.simxGetObjectPosition(clientID,targetObj,-1, vrep.simx_opmode_buffer);
	if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
		%Position of the spherical target
		[~,targetPos]=vrep.simxGetObjectPosition(clientID,targetObj,-1, vrep.simx_opmode_buffer);

		%Orientation of the spherical target
		[~,targetOrient] = vrep.simxGetObjectOrientation(clientID,targetObj,-1,vrep.simx_opmode_buffer);

		%quadrotor base position and velocity
		[~,quadPos] = vrep.simxGetObjectPosition(clientID,quad_base,-1,vrep.simx_opmode_buffer);
% 		[~,quadLinearVel,quadAngularVel] = vrep.simxGetObjectVelocity(clientID,quad_base,vrep.simx_opmode_buffer);

		%Quadrirotor orientation relative to target
		[~,quadOrient] = vrep.simxGetObjectOrientation(clientID,quad_base,-1,vrep.simx_opmode_buffer);

		%Send to vrep the calculated velocities
		propellerVel = u;
		
		[~]=vrep.simxSetFloatSignal(clientID,'particleVelocity1',propellerVel(1),vrep.simx_opmode_oneshot_wait);
		[~]=vrep.simxSetFloatSignal(clientID,'particleVelocity2',propellerVel(2),vrep.simx_opmode_oneshot_wait);
		[~]=vrep.simxSetFloatSignal(clientID,'particleVelocity3',propellerVel(3),vrep.simx_opmode_oneshot_wait);
		[~]=vrep.simxSetFloatSignal(clientID,'particleVelocity4',propellerVel(4),vrep.simx_opmode_oneshot_wait);
	else
		pause(0.05);
		sys=zeros(1,12);
	end
end

sys = [double(quadPos),double(quadOrient),double(targetPos),double(targetOrient)];
% end mdlOutputs

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)
global clientID;
global vrep;

% Now send some data to V-REP in a non-blocking fashion:
vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot);

% Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID);

%stop the simulation at vrep
[errorCode]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
disp('Simulation terminated error Code');
disp(errorCode);

% Now close the connection to V-REP:    
vrep.simxFinish(clientID);

sys = [];

% end mdlTerminate