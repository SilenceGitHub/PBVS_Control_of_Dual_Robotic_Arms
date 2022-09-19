function VrepWrite(theta_T, theta_V, t)

global ur kuka vrepInstance param
iter = round(t/param.samplingGap) + 1;
% Save joint velocity trajectroy
ur.theta(:,iter) = theta_T;
kuka.theta(:,iter) = theta_V;

% Control the robot
ur.SetJointAnglePosition(ur,vrepInstance,theta_T, vrepInstance.vrep.simx_opmode_oneshot)
kuka.SetJointAnglePosition(kuka,vrepInstance,theta_V, vrepInstance.vrep.simx_opmode_oneshot)

