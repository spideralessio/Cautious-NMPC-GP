function [ypred] = evaluate_gp(x,u)
    if exist('lr_online_model.mat', 'file')
        load('lr_online_model.mat'); % or gp_model.mat
        ypred = [x',u']*w + b;
        ypred = ypred';
    else
        ypred = zeros(3, 1);
    end
end

