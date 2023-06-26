
% Disciplina sistemas roboticos - 2023
% Alunos: Everton Cerqueira e Juliana Santana
% A ideia do trabalho é usar o robo UR5 para aplicar adesivo em um para-brisa
% Esse codigo usa:
% Robotics System Toolbox Support Package for Universal Robots UR Series Manipulators
% ROS Toolbox
% Robotics System Toolbox
% ROS MELODIC
% GAZEBO 9
% Universal_Robots_ROS_Driver
% UR_GLUE_DISPENSING_GAZEBO ROS PKG

ur5_RBT = loadrobot("universalUR5");
ur5_RBT.DataFormat = 'row';

rotationX = @(t) [1 0 0; 0 cosd(t) -sind(t) ; 0 sind(t) cosd(t)] ;

glueDispenserBody = rigidBody('dispenser');
addVisual(glueDispenserBody,"Mesh",'glueDispenserMesh.STL')
glueDispenserBodyJoint = rigidBodyJoint('glueDispenserBodyJoint','fixed');
glueDispenserBody.Joint = glueDispenserBodyJoint;
transfForglueDispenserBody = rotm2tform(rotationX(-90));
setFixedTransform(glueDispenserBody.Joint, transfForglueDispenserBody)
curEndEffectorBodyName = ur5_RBT.BodyNames{10};
addBody(ur5_RBT,glueDispenserBody,curEndEffectorBodyName)

transfForNewEndEffectorBody = rotm2tform(rotationX(180));
transfForNewEndEffectorBody(:,4) = [0.04; -0.195; 0; 1];
newEndEffectorBody = rigidBody('dispenserEdge');
setFixedTransform(newEndEffectorBody.Joint, transfForNewEndEffectorBody);
glueDispenserBodyName = ur5_RBT.BodyNames{11};
addBody(ur5_RBT,newEndEffectorBody,glueDispenserBodyName);

% Feche a janela da figura anterior antes de executar o script novamente
close(findobj('type','figure','name','Interactive Visualization'));

% Visualize a árvore interativa do robo na posição inicial ('q_home')
disp("Carregando robo interativo na posicao home");

ur5_iRBT = interactiveRigidBodyTree(ur5_RBT);
q_home = [0 -90 0 -90 0 0]'*pi/180;
rotate3d off;
view(145,25)
lightangle(20,-160)
axis([-1 1 -1 1 -0.5 1])
hold on
zlim([-0.5 1.5])
ur5_iRBT.ShowMarker = false;
ur5_iRBT.Configuration = q_home;

pause(5);

disp("Carregando para-brisa");

% Carregar objeto STL e obter objeto de correção
windshield = stlread('windshieldv3.stl');
orintationOfWindshield = 55; % In degree
locationOfWindshield = [0 0.4 0.15]; % With respect to global frame of the world [x y z]
windshieldPatch = patch(gca,'Faces',windshield.ConnectivityList,'Vertices',(windshield.Points*rotationX(orintationOfWindshield)) ...
    + locationOfWindshield,'FaceColor',[0.8 0.8 0.9],'LineStyle','none');

pause(5);

disp("Carregando waypoints");

    selectedWaypoints =  [ -0.3186    0.4080    0.1514; -0.3350    0.4932    0.1720; -0.3514    0.5798    0.1928;
        -0.3653    0.6683    0.2139; -0.3817    0.7500    0.2338; -0.3948    0.8153    0.2496;-0.3158    0.8263    0.2447; 
        -0.2397    0.8329    0.2407; -0.1565    0.8346    0.2368;-0.0603    0.8419    0.2357;
        0.0311    0.8442    0.2358; 0.1307    0.8383    0.2366; 0.3935    0.8144    0.2492;
        0.3786    0.7400    0.2312; 0.3664    0.6682    0.2141; 0.3538    0.5981    0.1972;
        0.3207    0.4120    0.1525; 0.2180    0.4132    0.1473; 0.1021    0.4186    0.1449;
        -0.0568    0.4185    0.1441; -0.1718    0.4180    0.1466; -0.3128    0.4086    0.1512];

disp("Carregando cinematica inversa");

ik = inverseKinematics('RigidBodyTree',ur5_RBT);
% Desativar reinicializações aleatórias
ik.SolverParameters.AllowRandomRestart = false;
ikWeights = [1 1 1 1 1 1];

% Defina a posição do robô para o primeiro waypoint para configuração de orientação
firstWaypoint = selectedWaypoints(1,:);
orient = [0 0 -pi/3];
tgtPose = trvec2tform(firstWaypoint) * eul2tform(orient); %target pose
config = ik('dispenserEdge',double(tgtPose),ikWeights',q_home');
ur5_iRBT.Configuration = config;
ur5_iRBT.ShowMarker = true;
rotate3d off;

% Redefina a função auxiliar antes de usá-la para evitar possíveis problemas com as variáveis persistentes
exampleHelperSetOrientations([],[],[],[],[],'reset');

zoom reset;
pan off;
view(145,25);

% Mova o robo para posicao home
ur5_iRBT.Configuration = q_home;

% Desabilitar o marcador do corpo da ferramenta
ur5_iRBT.ShowMarker = false;

finalWaypoints = selectedWaypoints;
finalOrientations = repmat([0 0 -pi/3],size(finalWaypoints,1),1);

% Velocidade desejada do ponto central da ferramenta (TCP: Tool Center Point)
velOfTCPForTask = 0.15;

% Resolução de passo de tempo desejada
dtForTask = 0.02;

computedTrajForTask = exampleHelperURGenerateTrajectory(ur5_RBT,config',finalWaypoints,finalOrientations,velOfTCPForTask,dtForTask);
trajTimes = 0:dtForTask:(size(computedTrajForTask.position,1)-1)*dtForTask;

% Mova o cobot para o primeiro waypoint desejado
ur5_iRBT.Configuration = computedTrajForTask.position(1,:);

% Posição do TCP em coordenadas cartesianas
posTCPForTask = computedTrajForTask.posTCP;

disp("Loop de controle do robo");

r = rateControl(1/dtForTask); % Taxa para controlar a execução do loop for
for idx = 1:size(computedTrajForTask.position,1)
    config = computedTrajForTask.position(idx,:);
    ur5_iRBT.Configuration = config';
    plot3(posTCPForTask(idx,1),posTCPForTask(idx,2),posTCPForTask(idx,3),'color','b','Marker','.','MarkerSize',5)
    waitfor(r);
end
eeSpeed = zeros(length(computedTrajForTask.position),6);
for i=1:length(computedTrajForTask.position)
    % Calcular jacobiano geometrico
    jacobian = geometricJacobian(ur5_RBT,computedTrajForTask.position(i,:),'dispenserEdge');

    % Calcular velocidade do end-effector
    eeSpeed(i,:) = (jacobian*computedTrajForTask.velocities(i,:)')';
end

disp("Carregando posicao das juntas");
close(findobj('type','figure','name','Desired joint position'));
hFig1 = figure('Name','Desired joint position');
set(hFig1,'Visible','on')
sgtitle("Desired joint position")
for i=1:6    
    subplot(2,3,i);   
    plot(trajTimes,computedTrajForTask.position(:,i));
    titleString = "Actuator:" + num2str (i);
    title(titleString);
    ylabel("Angle(rad)");
    xlabel("Time(second)");
    grid on;
    axis auto;
end

pause(1);

disp("Carregando velocidade das juntas");
close(findobj('type','figure','name','Desired joint velocity'));
hFig2 = figure('Name','Desired joint velocity');
set(hFig2,'Visible','on')
sgtitle("Desired joint velocity")
for i=1:6   
    subplot(2,3,i);
    plot(trajTimes,computedTrajForTask.velocities(:,i));
    titleString = "Actuator:" + num2str (i);
    title(titleString);
    ylabel("Velocity(rad/s)");
    xlabel("Time(second)");
    grid on;
    axis auto;
end

pause(1);

disp("Carregando aceleracao das juntas");
close(findobj('type','figure','name','Desired joint acceleration'));
hFig3 = figure('Name','Desired joint acceleration');
set(hFig3,'Visible','on')
sgtitle("Desired joint acceleration")
for i=1:6   
    subplot(2,3,i);
    plot(trajTimes,computedTrajForTask.acceleration(:,i));
    titleString = "Actuator:" + num2str (i);
    title(titleString);
    ylabel("Acceleration(rad/s^2)");
    xlabel("Time(second)");
    grid on;
    axis auto;
end

pause(1);

disp("Carregando velocidade linear desejada do end-effector");
close(findobj('type','figure','name','Desired end-effector linear velocity'));
hFig4 = figure('Name','Desired end-effector linear velocity');
set(hFig4,'Visible','on')
plot(trajTimes,vecnorm(eeSpeed(:,4:6)'));
title("Desired end-effector linear velocity")
ylabel("Velocity(m/s)");
xlabel("Time(second)");
grid on;
axis auto;

pause(1);

% Conexao com o ROS e Gazebo
disp("Carregando conexao com ROS e Gazebo");
ROSDeviceAddress = '127.0.0.1';
username = 'everton';
password = '102030';
ROSFolder = '/opt/ros/melodic';
WorkSpaceFolder = '~/ur_ws';

device = rosdevice(ROSDeviceAddress,username,password);
device.ROSFolder = ROSFolder;

generateAndTransferLaunchScriptGlueDispensing(device,WorkSpaceFolder);

if ~isCoreRunning(device)
    w = strsplit(system(device,'who'));
    displayNum = cell2mat(w(2));

    system(device,['export SVGA_VGPU10=0; ' ...
        'export DISPLAY=' displayNum '.0; ' ...
        './launchURGlueDispensing.sh &']);
end

pause(10);

ur = universalrobot(ROSDeviceAddress,'RigidBodyTree',ur5_RBT);

pause(2);

disp("Robo aplicando adesivo no para-brisa");
sendJointConfigurationAndWait(ur,computedTrajForTask.position(1,:),'EndTime',5);
followTrajectory(ur,computedTrajForTask.position',computedTrajForTask.velocities',computedTrajForTask.acceleration',trajTimes);

[result,~] = getMotionStatus(ur);
while ~result
    [result,~] = getMotionStatus(ur);
end

disp("Robo indo para home");
jointWaypoints = [0 -90 0 -90 0 0]*pi/180;
sendJointConfigurationAndWait(ur,jointWaypoints,'EndTime',5);
