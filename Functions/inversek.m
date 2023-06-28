
clear;

ur5_RBT= loadrobot("universalUR5");
ur5_RBT.DataFormat = 'row';

rotationX = @(t) [1 0 0; 0 cosd(t) -sind(t) ; 0 sind(t) cosd(t)];

disp("Carregando detalhes do robo");
showdetails(ur5_RBT)

pause(15);

disp("Carregando imagem da posicao inicial do robo");
poseConfig = [5.5239   -2.8769    0.6034   -5.8904    0.1074    5.4453];
show(ur5_RBT, poseConfig);

pause(15);

disp("Gerando matriz");
tform = getTransform(ur5_RBT,poseConfig,'ee_link','base_link')

pause(15);

disp("Cinematica inversa");
ik = inverseKinematics('RigidBodyTree',ur5_RBT);
weights = [1 1 1 1 1 1];
initialguess = ur5_RBT.homeConfiguration;
[configSoln,solnInfo] = ik('ee_link',tform,weights,initialguess);

disp("Carregando robo com o end-effector na mesma posicao, mas com uma configuracao de juntas diferente");
show(ur5_RBT, configSoln);
