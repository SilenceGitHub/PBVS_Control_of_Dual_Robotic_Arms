function output = VrepRead(t)
%#codegen

global ur kuka vrepInstance param

iter = round(t/param.samplingGap) + 1;

vrepInstance.vrep.simxSynchronousTrigger(vrepInstance.clientID);

conePose_in_camera = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle,  vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_buffer);
cylinder_pose = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cylinder_handle,  vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_buffer);
target_pose = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.target_handle,  vrepInstance.vision_sensor_handle, vrepInstance.vrep.simx_opmode_buffer);

ur.actualPath(:,iter) = conePose_in_camera;
kuka.actualPath(:,iter) = conePose_in_camera;

output = [conePose_in_camera; cylinder_pose; target_pose];