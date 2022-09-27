function output  = DGND_T(p_d, z_T, p_a, t)
    global param ur 
    
    iter = round(t/param.samplingGap) + 1;
    zeta_s = param.zeta;
    zeta_w = param.zeta;
    stage = param.stage;
%     z_T = zeros(ur.taskSpaceDimension,1);
    
    ur.errors(:,iter) = p_d - p_a;
    
    Js = ur.JHat(1:3,:,iter);
    Jw = ur.JHat(4:ur.taskSpaceDimension,:,iter);
    Ja = Js;

    dotTheta_s =  zeta_s*Js'*(ur.errors(1:3,iter)+z_T(1:3));
    
    Pa = eye(ur.jointNumber) - pinv(Ja)*Ja;
    JBar = Jw * Pa;

    dotTheta_w = zeta_w * JBar' * (ur.errors(4:end,iter) + z_T(4:end) - param.samplingGap * Jw*dotTheta_s) ;

    
    dotTheta = dotTheta_s + dotTheta_w;

    % actual velocity of the end effector
    if iter > 1
        % position filter
        ur.positionFilter(:,iter) = (1-param.filterParameter*param.samplingGap)*ur.positionFilter(:,iter-1) + param.filterParameter*param.samplingGap*ur.actualPath(:,iter-1);
        ur.actualVelocity(:,iter) = (ur.actualPath(:,iter) - ur.positionFilter(:,iter))*param.filterParameter;
    end
    
    % Jacobian estiamtion
    dotJHat = param.GNDConvergenceRate*(ur.actualVelocity(:,iter)-ur.JHat(:,:,iter)*dotTheta)*dotTheta';
    
    ur.JHat(:,:,iter+1) = ur.JHat(:,:,iter) + param.samplingGap * dotJHat;
    
   if stage ==1 
        positionErrorBound = 0.006; orienErrorBound = 0.02;
    else
        positionErrorBound = 0.002; orienErrorBound = 0.01;
    end
    if (norm(ur.errors(1:3,iter)) < positionErrorBound && norm(ur.errors(4:ur.taskSpaceDimension,iter)) < orienErrorBound)
        if stage <= 3
            param.stage = stage+1;
        end
        if param.stage ==2
            param.stage1Iter = iter;
        elseif param.stage == 3
            param.stage2Iter = iter;
        end
    end

    param.iter = param.iter + 1;
    output = dotTheta;

end