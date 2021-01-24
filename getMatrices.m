function [A, b, Aeq, beq, lb, ub] = getMatrices(X, params)
Horizon = params.Horizon;

ModelParams=bycicle_params();
A = [];
b = [];



n = ModelParams.nx + ModelParams.nu;

%set x(0) = x0
Aeq = zeros(ModelParams.nx, length(X));
beq = zeros(ModelParams.nx,1);
for i=1:ModelParams.nx
    Aeq(i, i) = 1; 
    beq(i) = X(i);
end

% set bounds on states and inputs
lb = -inf*ones(length(X),1);
ub = inf*ones(length(X),1);
for i=1:Horizon
    lb(1 + n*(i-1):n + n*(i-1),1) = [ModelParams.Xmin, ModelParams.Ymin, -Inf,-Inf,-Inf, -Inf, -Inf, ModelParams.Dmin, ModelParams.deltamin, 0]';
    ub(1 + n*(i-1):n + n*(i-1),1) = [ModelParams.Xmax, ModelParams.Ymax, Inf, Inf, Inf, Inf, Inf, ModelParams.Dmax, ModelParams.deltamax, 2]';
end

