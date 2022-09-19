function [ret, output] = Forward(obj, input)
    % get the output
    obj.cell = (1 -  1/obj.LINParameter*obj.leakyRate) * obj.cell +   1/obj.LINParameter * obj.sigmoid(obj.Win * input + obj.W * obj.cell);
    output = tanh(obj.Wout * obj.cell);
    ret = obj;
end