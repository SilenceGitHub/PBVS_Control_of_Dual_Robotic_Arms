function [ret, output] = Forward(obj, input)
    % get the output
    global param
    obj.cell = (1 -  param.samplingGap*obj.leakyRate/obj.LINParameter) * obj.cell +   param.samplingGap/obj.LINParameter * obj.sigmoid(obj.Win * input + obj.W * obj.cell);
    output = tanh(obj.Wout * obj.cell);
    ret = obj;
end