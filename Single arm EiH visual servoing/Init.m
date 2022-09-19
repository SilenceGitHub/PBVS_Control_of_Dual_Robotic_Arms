% clc;
clear all;
close all;
addpath('RemApi');

global kuka vrepInstance param VCerebNet 

%% ----------------  parameters ----------------------------%
% task parameters
param.taskDuration = 10;
param.samplingGap = 0.05;
% algorithm parameters
param.filterParameter = 0.1;
param.GNDConvergenceRate = 1;
param.zeta = 1;
% Cerebellum parameters
param.reservoirSize = 400;
param.learningRate = 0.001 ;
param.randSeed = rand(1)*1000;
param.LINParameter = 0.5;
param.leakyRate = 0.6;
param.iter = 1;
%----------------  parameters ----------------------------%

% Time step
steps = param.taskDuration / param.samplingGap + 1;    
t = 0:param.samplingGap:param.taskDuration;
samplingNumber = steps;

%% ------------- create a instance of VREP -------------------%
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


%% ------------- create instances of manipulators -------------------%
kuka = Kuka(samplingNumber,vrepInstance);
KukaInitialJoint = kuka.GetJointAnglePosition(kuka, vrepInstance, vrepInstance.vrep.simx_opmode_blocking);
kuka.theta(:,1) = KukaInitialJoint;
coneInitialPose_in_camera = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle, vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_blocking);

% Initialize the Jacobian matrix
% kuka = kuka.InitializeJacobian(kuka, KukaInitialJoint, vrepInstance);
kuka.JHat(:,:,1) = [-0.893625020980835,5.48362731933594e-05,-0.894176959991455,-0.000113248825073242,-0.289708375930786,0.0127267837524414,-0.196194648742676;0.0304889678955078,-0.889079570770264,0.0304460525512695,0.962667465209961,-0.181150436401367,0.502709150314331,-0.191949605941772;0.171368122100830,0.0737559795379639,0.171422958374023,0.273997783660889,2.62260437011719e-05,0.250846147537231,-0.0360190868377686;-0.0581729412078857,0.0919139385223389,-0.0581872463226318,-0.0795757770538330,0.0279796123504639,-0.0768220424652100,0.0279748439788818;-0.169989392161369,-0.0616145879030228,-0.170063376426697,0.0610379874706268,0.480765551328659,0.0513422489166260,0.426080763339996;-0.466311573982239,0.0217148661613464,-0.466409921646118,-0.0230604410171509,-0.0851184129714966,-0.0207564234733582,-0.0748741626739502;0.0101953744888306,0.487531125545502,0.0102287530899048,-0.489092171192169,0.0593999028205872,-0.469529330730438,0.0736305117607117];
kuka.SetJointAnglePosition(kuka, vrepInstance, kuka.theta(:,1), vrepInstance.vrep.simx_opmode_blocking);
coneInitialPose_in_camera =vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle,  vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_streaming);
jointAngle = kuka.GetJointAnglePosition(kuka, vrepInstance, vrepInstance.vrep.simx_opmode_streaming);
pause(0.1);


%% ------------- create instances of cerebellum network ----------------------------------%
VCerebNetInputDimension = kuka.jointNumber + kuka.taskSpaceDimension;
VCerebNetOutputDimension = kuka.taskSpaceDimension;
VCerebNet = Cerebellum(param.reservoirSize, param.learningRate, param.LINParameter,...
    param.leakyRate,param.randSeed, VCerebNetInputDimension, VCerebNetOutputDimension);


%%  get initial position
% coneInitialPose = vrepInstance.Get_object_pose(vrepInstance, vrepInstance.cone_handle);
coneInitialPose_in_camera = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle, vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_buffer);

kuka.positionFilter(:,1) = coneInitialPose_in_camera;
param.jacoInitPos = coneInitialPose_in_camera;