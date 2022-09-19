function ret = Backward(obj,error)
    % update output weight of ESN
    deltaWout = obj.learningRate * newTanh(error) * obj.cell';
    obj.Wout = obj.Wout + deltaWout;
    ret = obj;
end