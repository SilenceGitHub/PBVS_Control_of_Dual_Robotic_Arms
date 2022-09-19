function [] = SetJointAnglePosition(obj,vrepInstance,input, opmode)

    for i = 1:obj.jointNumber
        vrepInstance.vrep.simxSetJointTargetPosition(vrepInstance.clientID, obj.joint_handle(i), input(i), opmode);
    end
    
end