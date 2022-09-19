function ret = sigmoid(x)
    ret = 1 ./ (1 + exp(-x));
end