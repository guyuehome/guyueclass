% This small example illustrates how to use the remote API
% synchronous mode. The synchronous mode needs to be
% pre-enabled on the server side. You would do this by
% starting the server (e.g. in a child script) with:
%
% simRemoteApi.start(19999,1300,false,true)
%
% But in this example we try to connect on port
% 19997 where there should be a continuous remote API
% server service already running and pre-enabled for
% synchronous mode.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function simpleSynchronousTest()
    disp('Program started');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');

        % enable the synchronous mode on the client:
        sim.simxSynchronous(clientID,true);

        % start the simulation:
        sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);

        % Now step a few times:
        for i=0:10
            disp('Press a key to step the simulation!');
            pause;
            sim.simxSynchronousTrigger(clientID);
        end

        % stop the simulation:
        sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);

        % Now close the connection to CoppeliaSim:    
        sim.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    sim.delete(); % call the destructor!
    
    disp('Program ended');
end
