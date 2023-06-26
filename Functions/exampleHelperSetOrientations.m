function exampleHelperSetOrientations(~,evnt,waypoints,viztree,ik,varargin)
%

%   Copyright 2021 The MathWorks, Inc.

persistent orient config currentWaypoint i cPressed_i

if length(varargin) == 1 && nargin > 4 && strcmp(varargin{1},'reset')
    clear orient config currentWaypoint i cPressed_i;
    disp("Press Up-Down arrow key to rotate about the X-axis. Press Left-Right arrow key to rotate about the Y-axis.")
else
    if isempty(currentWaypoint)
        i = 1;
        cPressed_i = 0;
        currentWaypoint = waypoints(1,:);
        config = viztree.Configuration';
    end

    trasnform  = getTransform(viztree.RigidBodyTree, viztree.Configuration, 'dispenserEdge');
    orient = tform2eul(trasnform);
    ikInitGuess = config;
    ikInitGuess = wrapToPi(ikInitGuess);
    ikWeights = [1 1 1 1 1 1];

    if strcmpi(evnt.Key,'leftarrow')
        orient = orient - [0 0.01 0];
        computeJointConfig();
    elseif strcmpi(evnt.Key,'rightarrow')
        orient = orient + [0 0.01 0];
        computeJointConfig();
    elseif strcmpi(evnt.Key,'uparrow')
        orient = orient + [0 0 0.01];
        computeJointConfig();
    elseif strcmpi(evnt.Key,'downarrow')
        orient = orient - [0 0 0.01];
        computeJointConfig();
    elseif strcmpi(evnt.Key,'n')
        if i < size(waypoints,1)
            disp("Moving to next waypoint...")
            disp("Press Up-Down arrow key to rotate about the X-axis. Press Left-Right arrow key to rotate about the Y-axis.")
            orient = [0 0 -pi/3];
            i = i + 1;
            currentWaypoint = waypoints(i,:);
            computeJointConfig();
        else
            disp("This is the last stored waypoints. Please move to the next section.");
        end

    elseif strcmpi(evnt.Key,'c')
        if i <= size(waypoints,1) && i ~= cPressed_i
            disp("Waypoint data saved.")
            plot3(currentWaypoint(1),currentWaypoint(2),currentWaypoint(3),'Marker','o','MarkerFaceColor',[1 0 0],'MarkerSize',3)
            hold on
            exampleHelperGetFinalWaypointData(currentWaypoint,orient);
            cPressed_i = i;
        elseif i == cPressed_i
            disp("Orientations for this waypoints is stored. Press N to continue to next waypoint.");
        else
            disp("Orientations for all the waypoints are stored. Please move to the next section.");
        end
    end
end

    function computeJointConfig()
        tgtPose = trvec2tform(currentWaypoint) * eul2tform(orient); %target pose
        config = ik('dispenserEdge',double(tgtPose),ikWeights',ikInitGuess);
        viztree.Configuration = config;
    end
end

% LocalWords:  leftarrow rightarrow uparrow downarrow
