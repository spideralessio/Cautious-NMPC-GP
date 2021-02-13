function [ypred] = evaluate_gp(x,u)
    load('gp_model.mat');
    ypred = [x',u']*w + b;
    ypred = ypred';
end

