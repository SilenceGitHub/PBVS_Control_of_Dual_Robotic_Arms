function output  = DGND_V(p_d, z_T, p_a, t)
    global param kuka
    
    iter = round(t/param.samplingGap) + 1;
    zeta_s = param.zeta;
    zeta_w = param.zeta;
%     z_T = zeros(kuka.taskSpaceDimension,1);
    
    kuka.errors(:,iter) = p_d - p_a;
    
    Js = kuka.JHat(1:3,:,iter);
    Jw = kuka.JHat(4:kuka.taskSpaceDimension,:,iter);
    Ja = Js;

    dotTheta_s =  zeta_s*Js'*(kuka.errors(1:3,iter)+z_T(1:3));
    
    Pa = eye(kuka.jointNumber) - pinv(Ja)*Ja;
    JBar = Jw * Pa;

    dotTheta_w = zeta_w * JBar' * (kuka.errors(4:end,iter) + z_T(4:end) - param.samplingGap * Jw*dotTheta_s) ;

    
    dotTheta = dotTheta_s + dotTheta_w;

    % actual velocity of the end effector
    if iter > 1
        % position filter
        kuka.positionFilter(:,iter) = (1-param.filterParameter*param.samplingGap)*kuka.positionFilter(:,iter-1) + param.filterParameter*param.samplingGap*kuka.actualPath(:,iter-1);
        kuka.actualVelocity(:,iter) = (kuka.actualPath(:,iter) - kuka.positionFilter(:,iter))*param.filterParameter;
    end
    
    % Jacobian estiamtion
    dotJHat = param.GNDConvergenceRate*(kuka.actualVelocity(:,iter)-kuka.JHat(:,:,iter)*dotTheta)*dotTheta';
    
    kuka.JHat(:,:,iter+1) = kuka.JHat(:,:,iter) + param.samplingGap * dotJHat;


    output = dotTheta;

end