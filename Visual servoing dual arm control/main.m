clc;
clear all;
close all;
addpath('RemApi');

%----------------  parameters ----------------------------%
% task parameters
param.desiredPathScale = 0.1;
param.taskDuration = 30;
param.samplingGap = 0.05;
param.rollOutNum = 1;
% algorithm parameters
param.filterParameter = 1;
param.GNDConvergenceRate = 5;
% ESN parameters
param.reservoirSize = 400;
param.learningRate = 0.001;
param.randSeed = rand(1)*1000;
param.LINParameter = 2;
param.leakyRate = 0.6;
%----------------  parameters ----------------------------%

% Time step
steps = param.taskDuration / param.samplingGap * param.rollOutNum + 1;    
t = 0:param.samplingGap:param.taskDuration;
samplingNumber = steps;

%------------- create a instance of VREP -------------------%
vrepInstance = VREP(19999);%Port number
%Enable the synchronous mode (Blocking function call)
vrepInstance.vrep.simxSynchronous(vrepInstance.clientID,true);
vrepInstance.vrep.simxStartSimulation(vrepInstance.clientID,...
    vrepInstance.vrep.simx_opmode_oneshot); 
pause(1.0);
if vrepInstance.clientID > -1
    disp(['Connect to V-REP successfully',newline]);
else
    disp(['Fail to Connect to V-REP',newline]);
end
%------------- create a instance of VREP -------------------%

%------------- create instances of manipulators -------------------%
jaco = Jaco2(samplingNumber,vrepInstance);
Jaco2InitialJoint = jaco.GetJointAnglePosition(jaco, vrepInstance);
jaco.theta(:,1) = Jaco2InitialJoint;
% Initialize the Jacobian matrix
jaco = jaco.InitializeJacobian(jaco, Jaco2InitialJoint, vrepInstance, 1);
jaco.SetJointAnglePosition(jaco, vrepInstance, jaco.theta(:,1));

kuka = Kuka(samplingNumber,vrepInstance);
KukaInitialJoint = kuka.GetJointAnglePosition(kuka, vrepInstance);
kuka.theta(:,1) = KukaInitialJoint;
% Initialize the Jacobian matrix
kuka = kuka.InitializeJacobian(kuka, KukaInitialJoint, vrepInstance, 1);
kuka.SetJointAnglePosition(kuka, vrepInstance, kuka.theta(:,1));
%------------- create instances of manipulators -------------------%

%------------- create instances of cerebellum network ----------------------------------%
TCerebNetInputDimension = jaco.jointNumber + jaco.taskSpaceDimension;
TCerebNetOutputDimension = jaco.taskSpaceDimension;
TCerebNet = ESN(param.reservoirSize, param.learningRate, param.LINParameter,...
    param.leakyRate,param.randSeed, TCerebNetInputDimension, TCerebNetOutputDimension);

VCerebNetInputDimension = kuka.jointNumber + kuka.taskSpaceDimension;
VCerebNetOutputDimension = kuka.taskSpaceDimension;
VCerebNet = ESN(param.reservoirSize, param.learningRate, param.LINParameter,...
    param.leakyRate,param.randSeed, VCerebNetInputDimension, VCerebNetOutputDimension);
%------------- create instances of cerebellum network ----------------------------------%

% get initial position
% coneInitialPose = vrepInstance.Get_object_pose(vrepInstance, vrepInstance.cone_handle);
coneInitialPose_in_camera = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle, vrepInstance.vision_sensor_handle);
JacoPositionFilter(:,1) = coneInitialPose_in_camera;
kukaPositionFilter(:,1) = coneInitialPose_in_camera;

t0 = clock;
% desired path (Rhodonea)
% desiredPath(1,:) = param.desiredPathScale*cos(4*pi*sin(0.5*pi*t/param.taskDuration).^2).*cos(2*pi*sin(0.5*pi*t/param.taskDuration).^2)-param.desiredPathScale+coneInitialPose(1);
% desiredPath(2,:) = param.desiredPathScale*cos(pi/6)*cos(4*pi*sin(0.5*pi*t/param.taskDuration).^2).*sin(2*pi*sin(0.5*pi*t/param.taskDuration).^2)+coneInitialPose(2);
% desiredPath(3,:) = param.desiredPathScale*sin(pi/6)*cos(4*pi*sin(0.5*pi*t/param.taskDuration).^2).*sin(2*pi*sin(0.5*pi*t/param.taskDuration).^2)+coneInitialPose(3);

% desiredPose = vrepInstance.Get_object_pose(vrepInstance, vrepInstance.target_handle);
% desiredPath(1,:) = desiredPose(1)*ones(1,param.taskDuration/param.samplingGap+1);
% desiredPath(2,:) = desiredPose(2)*ones(1,param.taskDuration/param.samplingGap+1);
% desiredPath(3,:) = (desiredPose(3)+0.1)*ones(1,param.taskDuration/param.samplingGap+1);
% desiredPath(4,:) = desiredPose(4)*ones(1,param.taskDuration/param.samplingGap+1);
% desiredPath(5,:) = desiredPose(5)*ones(1,param.taskDuration/param.samplingGap+1);
% desiredPath(6,:) = desiredPose(6)*ones(1,param.taskDuration/param.samplingGap+1);
% targetPosition = desiredPath;

VDesiredPose = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle,  vrepInstance.vision_sensor_handle);
% VDesiredPose = [VDesiredPose2(1:3,1) + [0; 0; 0.05]; 0; 0; 0; -1];
VDesiredPath(1,:) = VDesiredPose(1)*ones(1,param.taskDuration/param.samplingGap+1);
VDesiredPath(2,:) = VDesiredPose(2)*ones(1,param.taskDuration/param.samplingGap+1);
VDesiredPath(3,:) = VDesiredPose(3)*ones(1,param.taskDuration/param.samplingGap+1);
VDesiredPath(4,:) = VDesiredPose(4)*ones(1,param.taskDuration/param.samplingGap+1);
VDesiredPath(5,:) = VDesiredPose(5)*ones(1,param.taskDuration/param.samplingGap+1);
VDesiredPath(6,:) = VDesiredPose(6)*ones(1,param.taskDuration/param.samplingGap+1);
% VDesiredPath(7,:) = VDesiredPose(7)*ones(1,param.taskDuration/param.samplingGap+1);
VTargetPosition = VDesiredPath;

% for r = 1:param.rollOutNum-1
%     targetPosition = [targetPosition,desiredPath(:,2:end)];
%     VTargetPosition = [VTargetPosition,VDesiredPath(:,2:end)];
% end

%%
stage = 1;
for iter = 1:param.rollOutNum*steps-1
        
    % Control the robot
    jaco.SetJointAnglePosition(jaco, vrepInstance, jaco.theta(:,iter));
    kuka.SetJointAnglePosition(kuka, vrepInstance, kuka.theta(:,iter));

    % Actual position of the cone
    conePose_in_camera = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle,  vrepInstance.vision_sensor_handle);
    jaco.actualPath(:,iter) = conePose_in_camera;  
    if stage == 1
        desiredPose = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cylinder_handle,  vrepInstance.vision_sensor_handle);
        targetPosition(:,iter) = desiredPose;
    else
        desiredPose = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cylinder_handle,  vrepInstance.vision_sensor_handle);
        desiredPose2 = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.target_handle,  vrepInstance.vision_sensor_handle);
        targetPosition(:,iter) = (desiredPose + desiredPose2)/2;
    end
    jaco.errors(:,iter) = targetPosition(:,iter)-jaco.actualPath(:,iter);
    
%     conePose_in_camera = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle,  vrepInstance.vision_sensor_handle);
    kuka.actualPath(:,iter) = conePose_in_camera;
    kuka.errors(:,iter) = VTargetPosition(:,iter)-kuka.actualPath(:,iter);
    
    % cerebellum networks forward
    % input
    uT = [jaco.errors(:,iter);jaco.theta(:,iter)]; % jaco
    uV = [kuka.errors(:,iter);kuka.theta(:,iter)]; % kuka
    % output
    [TCerebNet, YoutT] = TCerebNet.Forward(TCerebNet,uT);
    [VCerebNet, YoutV] = VCerebNet.Forward(VCerebNet,uV);
    
%     YoutT = zeros(7,1);
%     YoutV = zeros(7,1);
            
    Jp = jaco.JHat(1:3,:,iter);
    Jw = jaco.JHat(4:jaco.taskSpaceDimension,:,iter);
    Ja = Jp;
%     JJTe = Jp*Jp'*(jaco.errors(1:3,iter)+YoutT(1:3));
%     alpha_p = dot(jaco.errors(1:3,iter)+yout(1:3),JJTe)/(dot(JJTe,JJTe)+1e-7);
    alpha_p = param.samplingGap * 4;
    Jaco2DotTheta_p =  alpha_p*Jp'*(jaco.errors(1:3,iter)+YoutT(1:3));
    
    Pa = eye(jaco.jointNumber) - pinv(Ja)*Ja;
    JTilde = Jw * Pa;
%     JJTe = Jw*Jw'*(jaco.errors(4:end,iter)+YoutT(4:end));
%     alpha_w = dot(jaco.errors(4:6,iter)+yout(4:6),JJTe)/(dot(JJTe,JJTe)+1e-7);
    alpha_w = param.samplingGap * 4;
    Jaco2DotTheta_w = alpha_w * JTilde' * (jaco.errors(4:end,iter) + YoutT(4:end) - Jw*Jaco2DotTheta_p) ;

    %--------------- jaco control system ------------------%
%     Jp = jaco.JHat(1:3,:,iter);
%     Jw = jaco.JHat(4:7,:,iter);
%     Ja = Jw;
%     JJTe = Jw*Jw'*(jaco.errors(4:7,iter) + YoutT(4:7));
%     alpha_p = dot(jaco.errors(1:3,iter) + YoutT(1:3),JJTe)/(dot(JJTe,JJTe)+1e-7);
%     alpha_w = param.samplingGap * 5;
%     Jaco2DotTheta_w =  alpha_w*Jw'*(jaco.errors(4:7,iter) + YoutT(4:7));
    
%     Pa = eye(jaco.jointNumber) - pinv(Ja)*Ja;
%     JTilde = Jp * Pa;
%     JJTe = Jp*Jp'*(jaco.errors(1:3,iter) + YoutT(1:3));
%     alpha_w = dot(jaco.errors(4:6,iter) + YoutT(4:6),JJTe)/(dot(JJTe,JJTe)+1e-7);
%     alpha_p = param.samplingGap * 5;
%     Jaco2DotTheta_p =  alpha_p * JTilde' * (jaco.errors(1:3,iter) + YoutT(1:3) - Jp*Jaco2DotTheta_w) ;
%     Jaco2DotTheta_p =  alpha_p * Jp' * (jaco.errors(1:3,iter) + YoutT(1:3)) ;
%     Jaco2DotTheta_p = zeros(jaco.jointNumber,1);
    jaco.theta(:,iter+1) = jaco.theta(:,iter) + Jaco2DotTheta_p + Jaco2DotTheta_w ;
%     jaco.theta(:,iter+1) = jaco.theta(:,iter);
    jaco.dotTheta(:,iter) = (Jaco2DotTheta_p + Jaco2DotTheta_w) / param.samplingGap;
    %--------------- jaco control system ------------------%
 
    %--------------- kuka control system ------------------%
    kuka_Jp = kuka.JHat(1:3,:,iter);
    kuka_Jw = kuka.JHat(4:end,:,iter);
    
    kuka_Ja = kuka_Jp;
    alpha_p = param.samplingGap * 4;
    kuka_dotTheta_p =  alpha_p*kuka_Jp'*(kuka.errors(1:3,iter) + YoutV(1:3));
    
    kuka_Pa = eye(kuka.jointNumber) - pinv(kuka_Ja)*kuka_Ja;
    kuka_JTilde = kuka_Jw * kuka_Pa;
    alpha_w = param.samplingGap * 4;
    kuka_dotTheta_w =  alpha_w * kuka_JTilde' * (kuka.errors(4:end,iter) + YoutV(4:end) - kuka_Jw*kuka_dotTheta_p) ;
%     kuka_Jp = kuka.JHat(1:3,:,iter);
%     kuka_Jw = kuka.JHat(4:kuka.taskSpaceDimension,:,iter);
%     kuka_Ja = kuka_Jw;
% %     kuka_JJTe = kuka_Jw*kuka_Jw'*(kuka.errors(4:end,iter) + YoutV(4:end));
% %     alpha_p = dot(jaco.errors(1:3,iter) + YoutT(1:3),JJTe)/(dot(JJTe,JJTe)+1e-7);
%     alpha_w = param.samplingGap * 5;
%     kuka_dotTheta_w =  alpha_w*kuka_Jw'*(kuka.errors(4:end,iter) + YoutV(4:end));
%     
%     kuka_Pa = eye(kuka.jointNumber) - pinv(kuka_Ja)*kuka_Ja;
%     kuka_JTilde = kuka_Jp * kuka_Pa;
% %     kuka_JJTe = kuka_Jp*kuka_Jp'*(kuka.errors(1:3,iter) + YoutV(1:3));
% %     alpha_w = dot(jaco.errors(4:6,iter) + YoutT(4:6),JJTe)/(dot(JJTe,JJTe)+1e-7);
%     alpha_p = param.samplingGap * 5;
% %     kuka_dotTheta_p =  alpha_p * kuka_JTilde' * (kuka.errors(1:3,iter) + YoutV(1:3) - kuka_Jp*kuka_dotTheta_w) ;
%     kuka_dotTheta_p =  alpha_p * kuka_Jp' * (kuka.errors(1:3,iter) + YoutV(1:3)) ;

    kuka.theta(:,iter+1) = kuka.theta(:,iter) + kuka_dotTheta_p + kuka_dotTheta_w ;
%     kuka.theta(:,iter+1) = kuka.theta(:,iter);
    kuka.dotTheta(:,iter) = (kuka_dotTheta_p + kuka_dotTheta_w) / param.samplingGap;
    %--------------- kuka control system ------------------%
    
   
    % actual velocity of the end effector
    if iter > 1
        % position filter
        JacoPositionFilter(:,iter) = (1-param.filterParameter*param.samplingGap)*JacoPositionFilter(:,iter-1) + param.filterParameter*param.samplingGap*jaco.actualPath(:,iter-1);
        jaco.actualVelocity(:,iter) = (jaco.actualPath(:,iter) - JacoPositionFilter(:,iter))*param.filterParameter;
        
        kukaPositionFilter(:,iter) = (1-param.filterParameter*param.samplingGap)*kukaPositionFilter(:,iter-1) + param.filterParameter*param.samplingGap*kuka.actualPath(:,iter-1);
        kuka.actualVelocity(:,iter) = (kuka.actualPath(:,iter) - kukaPositionFilter(:,iter))*param.filterParameter;
    end
    % Equation 2
    %GNN2
    jaco.dotJHat(:,:,iter) = param.GNDConvergenceRate*(jaco.actualVelocity(:,iter)-jaco.JHat(:,:,iter)*jaco.dotTheta(:,iter))*jaco.dotTheta(:,iter)';
    jaco.JHat(:,:,iter+1) = jaco.JHat(:,:,iter) + param.samplingGap*jaco.dotJHat(:,:,iter);
    
    kuka.dotJHat(:,:,iter) = param.GNDConvergenceRate*(kuka.actualVelocity(:,iter)-kuka.JHat(:,:,iter)*kuka.dotTheta(:,iter))*kuka.dotTheta(:,iter)';
    kuka.JHat(:,:,iter+1) = kuka.JHat(:,:,iter) + param.samplingGap*kuka.dotJHat(:,:,iter);

    % update output weight of ESN
    TCerebNet = TCerebNet.Backward(TCerebNet,newTanh(jaco.errors(:,iter)));
    VCerebNet = VCerebNet.Backward(VCerebNet,newTanh(kuka.errors(:,iter)));
%     if(norm(jaco.errors(1:3,iter)) < 0.001 && norm(jaco.errors(4:6)) < 0.02) 
%         break;
%     end
    if stage ==1 
        positionErrorBound = 0.004; orienErrorBound = 0.02;
    else
        positionErrorBound = 0.002; orienErrorBound = 0.01;
    end
    if (norm(jaco.errors(1:3,iter)) < positionErrorBound && norm(jaco.errors(4:jaco.taskSpaceDimension,iter)) < orienErrorBound)
        stage = stage+1;
        if stage == 3
            break;
        end
        jaco = jaco.InitializeJacobian(jaco, jaco.theta(:,iter+1), vrepInstance, iter+1);
        kuka = kuka.InitializeJacobian(kuka, kuka.theta(:,iter+1), vrepInstance, iter+1);
    end
    disp(iter);
    if stage ==3
        break;
    end
end
%%
desiredPose = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cylinder_handle,  vrepInstance.vision_sensor_handle);
desiredPose2 = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.target_handle,  vrepInstance.vision_sensor_handle);
targetPosition(:,iter+1) = (desiredPose + desiredPose2)/2;
% Control the robot
jaco.SetJointAnglePosition(jaco, vrepInstance, jaco.theta(:,iter+1));
% Actual positin of the cone
conePose_in_camera = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle, vrepInstance.vision_sensor_handle);
jaco.actualPath(:,iter+1) = conePose_in_camera;
kuka.actualPath(:,iter+1) = conePose_in_camera;
jaco.errors(:,iter+1) = targetPosition(:,iter+1)-jaco.actualPath(:,iter+1);
kuka.errors(:,iter+1) = VTargetPosition(:,iter+1)-kuka.actualPath(:,iter+1);

TotalTime = etime(clock, t0);

% vrepInstance.vrep.simxSetObjectParent(vrepInstance.clientID,vrepInstance.cone_handle,-1,true, vrepInstance.vrep.simx_opmode_blocking);
% pause(2);
% Disconnect to V-REP
vrepInstance.Disconnect(vrepInstance);


% size(t)
% size(y)
%%
save (['Data/results_CerebDGND.mat'],  'jaco','targetPosition','kuka','VCerebNet','TCerebNet','param');

%%
load Data/results_CerebDGND.mat
figure;
interval = 1;
plot3(targetPosition(1,1:end),targetPosition(2,1:end),targetPosition(3,1:end),'-','color',[0.5,0.5,0.5], 'linewidth', 2);hold on;
plot3(jaco.actualPath(1,1:interval:end),jaco.actualPath(2,1:interval:end),jaco.actualPath(3,1:interval:end),'k-.*', 'linewidth', 2);hold off
legend('Desired path','Actual path','fontsize',30,'fontname','times new roman');
set(gca,'FontSize',25);
grid on;
axis equal;
% title('Roll-outs: 1~3','fontname','times new roman');
xlabel("$x$ (m)",'interpreter','latex','fontsize',30); ylabel("$y$ (m)",'interpreter','latex','fontsize',30); zlabel("$z$ (m)",'interpreter','latex','fontsize',30);

figure
xMin = 0; xMax = 360*param.samplingGap; yMin = -0.1; yMax = 0.2;
Te = 0:param.samplingGap:param.taskDuration*param.rollOutNum;
p1 = plot(Te(1:interval:360),jaco.errors(1,1:interval:360)','-', 'linewidth', 3);
hold on;
p2 = plot(Te(1:interval:360),jaco.errors(2,1:interval:360)','--', 'linewidth', 3);%grid on;
p3 = plot(Te(1:interval:360),jaco.errors(3,1:interval:360)',':', 'linewidth', 3);
plot([231*param.samplingGap,231*param.samplingGap],[yMin,yMax],'g--','linewidth',3);
hold off;
legend('$e_1$','$e_2$','$e_3$','fontsize',30,'interpreter','latex');
% ax = gca;
% ax.YAxis.Exponent = -3;
axis([xMin xMax yMin yMax]);
set(gca, 'xtick', xMin:2:xMax);
set(gca, 'ytick', yMin:0.1:yMax);
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('Position error (m)','fontname','times new roman','fontsize',30);

figure
p4 = plot(Te(1:interval:360),jaco.errors(4,1:interval:360)','-', 'linewidth', 3); hold on;
p5 = plot(Te(1:interval:360),jaco.errors(5,1:interval:360)',':', 'linewidth', 3);
p6 = plot(Te(1:interval:360),jaco.errors(6,1:interval:360)','--', 'linewidth', 3);
plot([231*param.samplingGap,231*param.samplingGap],[yMin,yMax],'g--','linewidth',3);hold off;
% title('Roll-outs: 1~3','fontname','times new roman');
legend('$e_4$','$e_5$','$e_6$','fontsize',30,'interpreter','latex');
% 修改坐标轴数量级
% ax = gca;
% ax.YAxis.Exponent = -3;
% 修改坐标轴范围及刻度
xMin = 0; xMax = 360*param.samplingGap; yMin = -0.3; yMax = 0.1;
axis([xMin xMax yMin yMax]);
set(gca, 'xtick', xMin:2:xMax);
set(gca, 'ytick', yMin:0.1:yMax);
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('Orientation error (rad)','fontname','times new roman','fontsize',30);


% 
% figure;
% interval = 1;
% plot3(VTargetPosition(1,1:end),VTargetPosition(2,1:end),VTargetPosition(3,1:end),'-','color',[0.5,0.5,0.5], 'linewidth', 2);hold on;
% plot3(kuka.actualPath(1,1:interval:end),kuka.actualPath(2,1:interval:end),kuka.actualPath(3,1:interval:end),'k-.*', 'linewidth', 2);hold off
% legend('Desired path','Actual path','fontsize',30,'fontname','times new roman');
% set(gca,'FontSize',25);
% grid on;
% axis equal;
% % title('Roll-outs: 1~3','fontname','times new roman');
% xlabel("$x$ (m)",'interpreter','latex','fontsize',30); ylabel("$y$ (m)",'interpreter','latex','fontsize',30); zlabel("$z$ (m)",'interpreter','latex','fontsize',30);

figure

Te = 0:param.samplingGap:param.taskDuration*param.rollOutNum;
p1 = plot(Te(1:interval:end),kuka.errors(1,1:interval:end)','-', 'linewidth', 3);
hold on;
p2 = plot(Te(1:interval:end),kuka.errors(2,1:interval:end)','--', 'linewidth', 3);%grid on;
p3 = plot(Te(1:interval:end),kuka.errors(3,1:interval:end)',':', 'linewidth', 3);hold off;
legend('$e_x$','$e_y$','$e_z$','fontsize',30,'interpreter','latex');
% ax = gca;
% ax.YAxis.Exponent = -3;
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('$e(t)$ (m)','interpreter','latex','fontsize',30);

figure
p4 = plot(Te(1:interval:end),kuka.errors(4,1:interval:end)','-', 'linewidth', 3); hold on;
p5 = plot(Te(1:interval:end),kuka.errors(5,1:interval:end)',':', 'linewidth', 3);
p6 = plot(Te(1:interval:end),kuka.errors(6,1:interval:end)','--', 'linewidth', 3);
% p6 = plot(Te(1:interval:end),kuka.errors(7,1:interval:end)','-.', 'linewidth', 3);hold off;
% title('Roll-outs: 1~3','fontname','times new roman');
legend('$w_x$','$w_y$','$w_z$','fontsize',30,'interpreter','latex');
% ax = gca;
% ax.YAxis.Exponent = -3;
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('$e(t)$ (m)','interpreter','latex','fontsize',30);

