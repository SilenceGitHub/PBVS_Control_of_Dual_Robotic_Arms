function VrepWrite(theta)

global jaco vrepInstance param
iter = param.iter;
% Save joint velocity trajectroy
jaco.theta(:,iter) = theta;

% Control the robot
% jaco.SetJointVel(jaco, vrepInstance, dotTheta, vrepInstance.vrep.simx_opmode_oneshot);
jaco.SetJointAnglePosition(jaco,vrepInstance,theta, vrepInstance.vrep.simx_opmode_oneshot)

