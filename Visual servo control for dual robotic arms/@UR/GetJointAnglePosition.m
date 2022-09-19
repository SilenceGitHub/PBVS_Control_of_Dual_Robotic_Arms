function ret = GetJointAnglePosition(obj, vrepInstance, opmode)
    for i = 1:obj.jointNumber
        [code, theta(i)] = vrepInstance.vrep.simxGetJointPosition(vrepInstance.clientID, obj.joint_handle(i), opmode);
    end
    ret = theta';
end