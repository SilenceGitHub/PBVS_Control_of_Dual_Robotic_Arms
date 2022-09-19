function output = PathGnrt(cylinder_pose, target_pose, t)

global param ur kuka

initPos = param.InitPos;
iter = round(t/param.samplingGap) + 1;
stage = param.stage;

p_dV = initPos;

if stage == 1
    p_dT = cylinder_pose;
else
    p_dT = (cylinder_pose + target_pose)/2;
end

ur.desiredPath(:,iter) = p_dT;
kuka.desiredPath(:,iter) = p_dV;

output = [p_dT; p_dV];
          
