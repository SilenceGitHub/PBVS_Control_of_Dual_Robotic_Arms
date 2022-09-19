function output = PathGnrt(t)

global param kuka

iter = param.iter;

VDesiredPose = [0;0;0.4;-1;0;0;0];


kuka.desiredPath(:,iter) = VDesiredPose;

output = VDesiredPose;
          
