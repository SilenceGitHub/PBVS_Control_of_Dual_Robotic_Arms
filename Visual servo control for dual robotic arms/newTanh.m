function ret = newTanh(x)
    threshold = 0.0005;
    ret = tanh(x);
    ret(x<-threshold) = -threshold;
    ret(x>threshold) = threshold;
end