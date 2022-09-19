function output = VrepRead()
%#codegen

global kuka vrepInstance param

iter = param.iter;

vrepInstance.vrep.simxSynchronousTrigger(vrepInstance.clientID);

conePose_in_camera = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle,  vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_buffer);
% jointAngle = kuka.GetJointAnglePosition(kuka, vrepInstance, vrepInstance.vrep.simx_opmode_buffer);
kuka.actualPath(:,iter) = conePose_in_camera;
% kuka.theta(:,iter) = jointAngle;

output = [conePose_in_camera];