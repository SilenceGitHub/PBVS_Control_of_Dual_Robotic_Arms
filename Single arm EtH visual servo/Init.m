% clc;
clear all;
close all;
addpath('RemApi');

global jaco vrepInstance param TCerebNet 

%% ----------------  parameters ----------------------------%
% task parameters
param.desiredPathScale = 0.08;
param.taskDuration = 20;
param.samplingGap = 0.01;
% algorithm parameters
param.filterParameter = 0.1;
param.GNDConvergenceRate = 1;
param.zeta = 100;
% Cerebellum parameters
param.reservoirSize = 200;
param.learningRate = 0.001 ;
param.randSeed = rand(1)*1000;
param.LINParameter = param.samplingGap;
param.leakyRate = 0.9;
param.iter = 1;

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
jaco = Jaco2(samplingNumber,vrepInstance);
Jaco2InitialJoint = jaco.GetJointAnglePosition(jaco, vrepInstance, vrepInstance.vrep.simx_opmode_blocking);
jaco.theta(:,1) = Jaco2InitialJoint;
% Initialize the Jacobian matrix
% jaco = jaco.InitializeJacobian(jaco, Jaco2InitialJoint, vrepInstance);
jaco.JHat(:,:,1) = [-0.0269579887390137,-0.390567779541016,0.172719955444336,-0.0299358367919922,-0.0140166282653809,0.0107073783874512;0.357882976531982,-0.0450468063354492,-0.0690650939941406,-0.0419616699218750,0.0150656700134277,0.00525712966918945;0.172718763351440,-0.0213760137557983,0.208038687705994,-0.0932407379150391,0.00619292259216309,0.00733077526092529;-0.290946960449219,0.0227367877960205,-0.0133240222930908,0.184446573257446,-0.0732064247131348,-0.0784969329833984;0.0885760784149170,-0.111104249954224,0.107469558715820,0.0980752706527710,0.285858511924744,0.0986331701278687;0.0898501276969910,-0.420873910188675,0.433645844459534,-0.191662311553955,-0.0904314219951630,0.191936492919922;-0.345273017883301,-0.118739604949951,0.151463747024536,0.295510292053223,0.0493562221527100,-0.0102710723876953];

jaco.SetJointAnglePosition(jaco, vrepInstance, jaco.theta(:,1), vrepInstance.vrep.simx_opmode_blocking);
coneInitialPose_in_camera =vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle,  vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_streaming);
jointAngle = jaco.GetJointAnglePosition(jaco, vrepInstance, vrepInstance.vrep.simx_opmode_streaming);
pause(0.1);


%% ------------- create instances of cerebellum network ----------------------------------%
TCerebNetInputDimension = jaco.jointNumber + jaco.taskSpaceDimension;
TCerebNetOutputDimension = jaco.taskSpaceDimension;
TCerebNet = Cerebellum(param.reservoirSize, param.learningRate, param.LINParameter,...
    param.leakyRate,param.randSeed, TCerebNetInputDimension, TCerebNetOutputDimension);
% load Data/results_CIDGND_T1.mat TCerebNet

%%  get initial position
coneInitialPose_in_camera = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle, vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_buffer);

jaco.positionFilter(:,1) = coneInitialPose_in_camera;
param.jacoInitPos = coneInitialPose_in_camera;