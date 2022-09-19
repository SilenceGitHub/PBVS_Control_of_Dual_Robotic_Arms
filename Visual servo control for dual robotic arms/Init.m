% clc;
clear all;
close all;
addpath('RemApi');

global ur kuka vrepInstance param TCerebNet VCerebNet 

%% ----------------  parameters ----------------------------%
% task parameters
param.taskDuration = 30;
param.samplingGap = 0.02;
param.stage = 1;
% algorithm parameters
param.filterParameter = 0.1;
param.GNDConvergenceRate = 5;
param.zeta = 2;
% Cerebellum parameters
param.reservoirSize = 400;
param.learningRate = 0.0001 ;
param.randSeed = rand(1)*1000;
param.LINParameter = 0.5;
param.leakyRate = 0.6;
param.iter = 1;
param.stage1Iter = 0;
param.stage2Iter = 0;
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
ur = UR(samplingNumber,vrepInstance);
URInitialJoint = ur.GetJointAnglePosition(ur, vrepInstance, vrepInstance.vrep.simx_opmode_blocking);
ur.theta(:,1) = URInitialJoint;
% Initialize the urbian matrix
ur = ur.InitializeJacobian(ur, URInitialJoint, vrepInstance, 1);
% ur.JHat(:,:,1) = [-0.0269579887390137,-0.390567779541016,0.172719955444336,-0.0299358367919922,-0.0140166282653809,0.0107073783874512;0.357882976531982,-0.0450468063354492,-0.0690650939941406,-0.0419616699218750,0.0150656700134277,0.00525712966918945;0.172718763351440,-0.0213760137557983,0.208038687705994,-0.0932407379150391,0.00619292259216309,0.00733077526092529;-0.290946960449219,0.0227367877960205,-0.0133240222930908,0.184446573257446,-0.0732064247131348,-0.0784969329833984;0.0885760784149170,-0.111104249954224,0.107469558715820,0.0980752706527710,0.285858511924744,0.0986331701278687;0.0898501276969910,-0.420873910188675,0.433645844459534,-0.191662311553955,-0.0904314219951630,0.191936492919922;-0.345273017883301,-0.118739604949951,0.151463747024536,0.295510292053223,0.0493562221527100,-0.0102710723876953];

kuka = Kuka(samplingNumber,vrepInstance);
KukaInitialJoint = kuka.GetJointAnglePosition(kuka, vrepInstance, vrepInstance.vrep.simx_opmode_blocking);
kuka.theta(:,1) = KukaInitialJoint;
% Initialize the Jacobian matrix
kuka = kuka.InitializeJacobian(kuka, KukaInitialJoint, vrepInstance, 1);
% kuka.JHat(:,:,1) = [-0.0269579887390137,-0.390567779541016,0.172719955444336,-0.0299358367919922,-0.0140166282653809,0.0107073783874512;0.357882976531982,-0.0450468063354492,-0.0690650939941406,-0.0419616699218750,0.0150656700134277,0.00525712966918945;0.172718763351440,-0.0213760137557983,0.208038687705994,-0.0932407379150391,0.00619292259216309,0.00733077526092529;-0.290946960449219,0.0227367877960205,-0.0133240222930908,0.184446573257446,-0.0732064247131348,-0.0784969329833984;0.0885760784149170,-0.111104249954224,0.107469558715820,0.0980752706527710,0.285858511924744,0.0986331701278687;0.0898501276969910,-0.420873910188675,0.433645844459534,-0.191662311553955,-0.0904314219951630,0.191936492919922;-0.345273017883301,-0.118739604949951,0.151463747024536,0.295510292053223,0.0493562221527100,-0.0102710723876953];


kuka.SetJointAnglePosition(kuka, vrepInstance, kuka.theta(:,1), vrepInstance.vrep.simx_opmode_blocking);
ur.SetJointAnglePosition(ur, vrepInstance, ur.theta(:,1), vrepInstance.vrep.simx_opmode_blocking);

coneInitialPose_in_camera =vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle,  vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_streaming);
desiredPose1 = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cylinder_handle,  vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_streaming);
desiredPose2 = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.target_handle,  vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_streaming);
% awaiting the VREP buffer prepared
pause(0.1);


%% ------------- create instances of cerebellum network ----------------------------------%
TCerebNetInputDimension = ur.jointNumber + ur.taskSpaceDimension;
TCerebNetOutputDimension = ur.taskSpaceDimension;
TCerebNet = Cerebellum(param.reservoirSize, param.learningRate, param.LINParameter,...
    param.leakyRate,param.randSeed, TCerebNetInputDimension, TCerebNetOutputDimension);

VCerebNetInputDimension = kuka.jointNumber + kuka.taskSpaceDimension;
VCerebNetOutputDimension = kuka.taskSpaceDimension;
VCerebNet = Cerebellum(param.reservoirSize, param.learningRate, param.LINParameter,...
    param.leakyRate,param.randSeed, VCerebNetInputDimension, VCerebNetOutputDimension);


%%  get initial position
% coneInitialPose = vrepInstance.Get_object_pose(vrepInstance, vrepInstance.cone_handle);
coneInitialPose_in_camera = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle, vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_buffer);

ur.positionFilter(:,1) = coneInitialPose_in_camera;
kuka.positionFilter(:,1) = coneInitialPose_in_camera;
param.InitPos = coneInitialPose_in_camera;