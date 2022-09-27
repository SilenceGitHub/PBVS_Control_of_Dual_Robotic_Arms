function ret = InitializeJacobian(obj, initialJointAngle, vrepInstance)
    incrementalAmount = 0.05;
    for i = 1:obj.jointNumber
        theta1 = initialJointAngle;
        theta2 = initialJointAngle;
        theta2(i) = theta2(i) + incrementalAmount;
        
        opmode = vrepInstance.vrep.simx_opmode_blocking;
        
        obj.SetJointAnglePosition(obj, vrepInstance, theta1, opmode);
        pose1 = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle, vrepInstance.vision_sensor_handle, opmode);
        
        obj.SetJointAnglePosition(obj, vrepInstance, theta2, opmode);
        pose2 = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle, vrepInstance.vision_sensor_handle, opmode);

        obj.JHat(:,i,1) = (pose2-pose1) / incrementalAmount; 
    end
    disp('The Jacobian matrx of Jaco2 has been initialized');
    ret = obj;
end