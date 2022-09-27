function [] = SetJointVel(obj, vrepInstance, input, opmode)

    for i = 1:obj.jointNumber
        res = vrepInstance.vrep.simxSetJointTargetVelocity(vrepInstance.clientID, obj.joint_handle(i), input(i), opmode); 
    end
    
end
