function output = CerebNet(u_V, e_V)
    global VCerebNet
    % Forward
    [VCerebNet, z_V] = VCerebNet.Forward(VCerebNet,u_V);
    
    % Backward
    % update output weight of cerebellum model
    VCerebNet = VCerebNet.Backward(VCerebNet,e_V);
    output = z_V;
end