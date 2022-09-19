%Function for control a VREP simulator for ball and plate task
%obj is the class object
%object_name is the name of the object in the VREP scene
%P_b: Position of the ball respect the world frame [m]
%O_b: Orientation of the ball respect the world frame (Quaternion)


function ret =  Get_object_pose_reference(obj, object_handle, reference_handle, opmode)
   
    % Get object position in the camera frame
    [res, P_b] = obj.vrep.simxGetObjectPosition(obj.clientID,object_handle, reference_handle, opmode);
   
    %Get object orientation
    [res, O_b] = obj.vrep.simxGetObjectQuaternion(obj.clientID, object_handle, reference_handle, opmode);
    
    ret = [P_b, O_b]';
end
       
