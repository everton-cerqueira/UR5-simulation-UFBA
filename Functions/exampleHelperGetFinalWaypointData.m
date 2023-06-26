function [outWaypoint, outOrient]= exampleHelperGetFinalWaypointData(dataWaypoint, dataOrient)
%

%   Copyright 2021 The MathWorks, Inc.

persistent finalOrient finalWaypoint
if nargout == 0
    finalOrient = [finalOrient; dataOrient];
    finalWaypoint = [finalWaypoint; dataWaypoint];
else
    outWaypoint = finalWaypoint;
    outOrient = finalOrient;
    clear finalOrient finalWaypoint;
end
end
