function ret = InitializeJacobian(obj, initialJointAngle, vrepInstance, index)
    incrementalAmount = 0.05;
    opmode = vrepInstance.vrep.simx_opmode_blocking;
    for i = 1:obj.jointNumber
        theta1 = initialJointAngle;
        theta2 = initialJointAngle;
        theta2(i) = theta2(i) + incrementalAmount;
        
        obj.SetJointAnglePosition(obj, vrepInstance, theta1, opmode);
        pose1 = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle, vrepInstance.vision_sensor_handle, opmode);
        
        obj.SetJointAnglePosition(obj, vrepInstance, theta2, opmode);
        pose2 = vrepInstance.Get_object_pose_reference(vrepInstance, vrepInstance.cone_handle, vrepInstance.vision_sensor_handle, opmode);

        obj.JHat(:,i,index) = (pose2-pose1) / incrementalAmount; 
    end
    obj.SetJointAnglePosition(obj, vrepInstance, initialJointAngle, opmode);
    disp('The Jacobian matrx of Kuka has been initialized');
    ret = obj;
end