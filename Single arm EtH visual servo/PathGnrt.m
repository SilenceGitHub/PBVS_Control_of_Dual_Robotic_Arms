function output = PathGnrt(t)

global param jaco

jacoInitPos = param.jacoInitPos;
iter = param.iter;


p_d(1) = param.desiredPathScale*cos(4*pi*sin(0.5*pi*t/param.taskDuration).^2).*cos(2*pi*sin(0.5*pi*t/param.taskDuration).^2)-param.desiredPathScale+jacoInitPos(1);
p_d(2) = param.desiredPathScale*cos(pi/6)*cos(4*pi*sin(0.5*pi*t/param.taskDuration).^2).*sin(2*pi*sin(0.5*pi*t/param.taskDuration).^2)+jacoInitPos(2);
p_d(3) = param.desiredPathScale*sin(pi/6)*cos(4*pi*sin(0.5*pi*t/param.taskDuration).^2).*sin(2*pi*sin(0.5*pi*t/param.taskDuration).^2)+jacoInitPos(3);

% desiredPose = vrepInstance.Get_object_pose(vrepInstance, vrepInstance.target_handle);
% desiredPath(1,:) = desiredPose(1)*ones(1,param.taskDuration/param.samplingGap+1);
% desiredPath(2,:) = desiredPose(2)*ones(1,param.taskDuration/param.samplingGap+1);
% desiredPath(3,:) = (desiredPose(3)+0.1)*ones(1,param.taskDuration/param.samplingGap+1);
p_d(4) = jacoInitPos(4);
p_d(5) = jacoInitPos(5);
p_d(6) = jacoInitPos(6);
p_d(7) = jacoInitPos(7);

jaco.desiredPath(:,iter) = p_d;

output = p_d;
          
