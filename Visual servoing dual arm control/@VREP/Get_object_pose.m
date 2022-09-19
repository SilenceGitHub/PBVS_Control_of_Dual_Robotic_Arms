%Function for control a VREP simulator for ball and plate task
%obj is the class object
%object_name is the name of the object in the VREP scene
%P_b: Position of the ball respect the world frame [m]
%O_b: Orientation of the ball respect the world frame (Euler angles [x y z])[rad]


function ret =  Get_object_pose(obj, object_handle)
   
    [res, P_b] = obj.vrep.simxGetObjectPosition(obj.clientID,object_handle, -1,obj.vrep.simx_opmode_blocking);
   
    %Get object orientation
    [res, O_b] = obj.vrep.simxGetObjectOrientation(obj.clientID, object_handle, -1, obj.vrep.simx_opmode_blocking);
    
    ret = [P_b, O_b]';
end
       
