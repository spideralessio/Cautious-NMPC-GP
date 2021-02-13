function [ypred_prime] = evaluate_gp(x,u)
    load('gp_model.mat');
    ypred_prime = w';
end

