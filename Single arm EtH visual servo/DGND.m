function output  = DGND(p_d, z_T, p_a, JHat)
    global param jaco
    iter = param.iter;
    zeta_s = param.zeta;
    zeta_w = param.zeta;
%     z_T = zeros(7,1);
    
    jaco.JHat(:,:,iter) = JHat;
    jaco.errors(:,iter) = p_d - p_a;
    
    Js = jaco.JHat(1:3,:,iter);
    Jw = jaco.JHat(4:jaco.taskSpaceDimension,:,iter);
    Ja = Js;

    Jaco2DotTheta_s =  zeta_s*Js'*(jaco.errors(1:3,iter)+z_T(1:3));
    
    Pa = eye(jaco.jointNumber) - pinv(Ja)*Ja;
    JBar = Jw * Pa;

    Jaco2DotTheta_w = zeta_w * JBar' * (jaco.errors(4:end,iter) + z_T(4:end) - param.samplingGap * Jw*Jaco2DotTheta_s) ;

    
    dotTheta = Jaco2DotTheta_s + Jaco2DotTheta_w;

    % actual velocity of the end effector
    if iter > 1
        % position filter
        jaco.positionFilter(:,iter) = (1-param.filterParameter*param.samplingGap)*jaco.positionFilter(:,iter-1) + param.filterParameter*param.samplingGap*jaco.actualPath(:,iter-1);
        jaco.actualVelocity(:,iter) = (jaco.actualPath(:,iter) - jaco.positionFilter(:,iter))*param.filterParameter;
    end
    
    % Jacobian estiamtion
    dotJHat = param.GNDConvergenceRate*(jaco.actualVelocity(:,iter)-JHat*dotTheta)*dotTheta';
    
    output = [dotTheta; reshape(dotJHat, [42,1])];
    param.iter = iter + 1;
end