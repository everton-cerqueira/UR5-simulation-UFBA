function computedTrajectory = exampleHelperURGenerateTrajectory(robot,currentJointConfig, waypoints,orientations,vel,dt)
% Function to generate the trajectory for glue dispensing application. 

%   Copyright 2021 The MathWorks, Inc.

q_out = [];
qd_out = [];
qdd_out = [];
posTCP = [];

% Initialize IK solver
ik = inverseKinematics('RigidBodyTree',robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1, 1, 1, 1, 1, 1];

q_init = wrapToPi(currentJointConfig');
taskInit = getTransform(robot,q_init,'dispenserEdge');
T_totalSample = 0;

for m = 2:size(waypoints,1)
    % Calculate the desired task configuration
    taskFinal = trvec2tform(waypoints(m,:))*eul2tform(orientations(m,:));
    distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
    T = distance/vel;
    trajTimes = 0:dt:T;
    timeInterval = [trajTimes(1); trajTimes(end)];
    T_totalSample = T_totalSample + length(trajTimes);

    % Calculate intermediate task waypoints
    taskWaypoints = transformtraj(taskInit,taskFinal,timeInterval,trajTimes);

    % initialize joint array and current position array
    q = zeros(length(trajTimes),6);
    curPosTCP = zeros(length(trajTimes),3);

    % Calculate joint configuration for each task waypoints
    for i = 1:length(trajTimes)
        q(i,:) = wrapToPi(ik('dispenserEdge', taskWaypoints(:,:,i), weights, q_init));
        curPosTCP(i,:) = tform2trvec(getTransform(robot,q(i,:),'dispenserEdge'));
        q_init = q(i,:);
    end
    taskInit = getTransform(robot,q(end,:),'dispenserEdge');
    q_init = q(end,:);
    q_out = [q_out; q(1:end-1,:)];
    posTCP = [posTCP; curPosTCP];
end

% Generate a smooth trajectory
T_array = 0:dt:(size(q_out,1)-1)*dt;
[q_out,qd_out,qdd_out] = minjerkpolytraj(q_out',T_array,length(T_array));

computedTrajectory.position = q_out';
computedTrajectory.velocities = qd_out';
computedTrajectory.acceleration = qdd_out';
computedTrajectory.posTCP = posTCP;

end

% LocalWords:  IK waypoints
