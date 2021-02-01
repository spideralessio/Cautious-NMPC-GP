function [ud,Sd] = evaluate_gp(x,u)
    load('gp_model.mat');
    [ux, sx] = predict(modelX,[x,u]);
    [uy, sy] = predict(modelY,[x,u]);
    [uphi, sphi] = predict(modelPhi,[x,u]);
    
    ud = [ux;uy;uphi];
    Sd = diag([sx^2, sy^2, sphi^2]);
end

