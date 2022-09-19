function output  = DGND(p_d, z_V, p_a, JHat)
    global param kuka
    iter = param.iter;
    zeta_s = param.zeta;
    zeta_w = param.zeta ;
%     z_V = zeros(7,1);
    
    kuka.JHat(:,:,iter) = JHat;
    kuka.errors(:,iter) = p_d - p_a;
    
    Js = kuka.JHat(1:3,:,iter);
    Jw = kuka.JHat(4:kuka.taskSpaceDimension,:,iter);
    Ja = Js;

    kukaDotTheta_s =  zeta_s*Js'*(kuka.errors(1:3,iter)+z_V(1:3));
    
    Pa = eye(kuka.jointNumber) - pinv(Ja)*Ja;
    JBar = Jw * Pa;

    kukaDotTheta_w = zeta_w * JBar' * (kuka.errors(4:end,iter) + z_V(4:end) - param.samplingGap * Jw*kukaDotTheta_s) ;

    
    dotTheta = kukaDotTheta_s + kukaDotTheta_w;

    % actual velocity of the end effector
    if iter > 1
        % position filter
        kuka.positionFilter(:,iter) = (1-param.filterParameter*param.samplingGap)*kuka.positionFilter(:,iter-1) + param.filterParameter*param.samplingGap*kuka.actualPath(:,iter-1);
        kuka.actualVelocity(:,iter) = (kuka.actualPath(:,iter) - kuka.positionFilter(:,iter))*param.filterParameter;
    end
    
    % kukabian estiamtion
    dotJHat = param.GNDConvergenceRate*(kuka.actualVelocity(:,iter)-JHat*dotTheta)*dotTheta';
    
    output = [dotTheta; reshape(dotJHat, [49,1])];
    param.iter = iter + 1;
end