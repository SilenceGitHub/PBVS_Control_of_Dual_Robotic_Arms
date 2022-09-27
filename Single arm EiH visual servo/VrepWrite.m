function VrepWrite(theta)

global kuka vrepInstance param
iter = param.iter;
% Save joint velocity trajectroy
kuka.theta(:,iter) = theta;

% Control the robot
% kuka.SetJointVel(kuka, vrepInstance, dotTheta, vrepInstance.vrep.simx_opmode_oneshot);
kuka.SetJointAnglePosition(kuka,vrepInstance,theta, vrepInstance.vrep.simx_opmode_oneshot)

