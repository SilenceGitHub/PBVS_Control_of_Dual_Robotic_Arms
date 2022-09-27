function output = CerebNet_T(u_T, e_T)
    global TCerebNet
    % Forward
    [TCerebNet, z_T] = TCerebNet.Forward(TCerebNet,u_T);
    
    % Backward
    % update output weight of cerebellum model
    TCerebNet = TCerebNet.Backward(TCerebNet,e_T);
    output = z_T;
end