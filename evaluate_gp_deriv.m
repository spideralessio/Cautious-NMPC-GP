function [ypred_prime] = evaluate_gp_deriv(x,u)
    if exist('lr_online_model.mat', 'file')
        load('lr_online_model.mat'); % or gp_model.mat
        ypred_prime = w';
    else
        ypred_prime = zeros(3, 10);
    end
end

