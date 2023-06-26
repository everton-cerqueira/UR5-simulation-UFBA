function out = exampleHelperSelectWaypoints(~,event)
% Function to capture the waypoints using mouse button callback

%   Copyright 2021 The MathWorks, Inc.

persistent var
if nargout == 0
    cord = event.IntersectionPoint;
    var = [var; cord];
    plot3(cord(1),cord(2),cord(3),'Marker','o','MarkerFaceColor',[0 1 0],'MarkerSize',5)
    hold on
else
    out = var;
    clear var;
end
end

% LocalWords:  waypoints
