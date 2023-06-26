function generateAndTransferLaunchScript(device,WorkSpaceFolder)
%

%   Copyright 2021 The MathWorks, Inc.

% Open a file to write set of commands to launch interface with
% simulated UR5 in gazebo or URSim
fid=fopen(fullfile(tempdir,"launchURGlueDispensing.sh"),"w+");
fprintf(fid,"#!/bin/sh\n");
fprintf(fid,"export SVGA_VGPU10=0\n");
fprintf(fid,"export ROS_IP=%s\n",device.DeviceAddress);
fprintf(fid,"export ROS_MASTER_URI=http://$ROS_IP:11311\n");

fprintf(fid,"gnome-terminal --title=\42Simulated UR5 Robot\42 -- /bin/bash -c 'source %s/setup.bash; source %s/devel/setup.bash; roslaunch ur_glue_dispensing_gazebo ur_glue_dispensing_gazebo.launch'",device.ROSFolder,WorkSpaceFolder);

fclose(fid);

% Copy file into ROS device
putFile(device,fullfile(tempdir,'launchURGlueDispensing.sh'),'~/')

% Make the shell script executable
system(device,'chmod a+x ~/launchURGlueDispensing.sh');
end
