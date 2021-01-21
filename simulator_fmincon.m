clear
clc
load track.mat

Ts = 0.05;

startIdx = 20;
last_closestIdx = startIdx;

ModelParams = bycicle_params();

vx0 = 1;

trackWidth = norm(track.inner(:,1)-track.outer(:,1));

x0 = [track.center(1,startIdx),track.center(2,startIdx),... % point on centerline
      -pi+atan2(track.center(1,startIdx+1) - track.center(1,startIdx), track.center(2,startIdx+1) - track.center(2,startIdx)),... % aligned with centerline
      vx0 ,0, 0]';
  
u0 = [0;0];

Horizon = 20;
params = struct;
params.track = track;
params.trackWidth = trackWidth;
params.Horizon = Horizon;
params.Ts = Ts;



X_x = repmat(x0', Horizon, 1);
X_u = repmat(u0', Horizon, 1);
for i=2:Horizon
    X_x(i,:) = X_x(i-1,:) + Ts*bycicle_model(X_x(i-1,:)', X_u(i-1,:)', params)';
end


X = [X_x, X_u];
X_flat = reshape(X,1,[]);
options = optimoptions('fmincon','Display','none','Algorithm','sqp');


f = @(x)fmincon_cost_function(x, params);
nonlcon = @(x)fmincon_constraints(x, params);
for i=1:100
    [A, b, Aeq, beq, lb, ub] = getMatrices(X_flat, params);
    [X_flat_new, fval, exitflag, output] = fmincon(f,X_flat,A,b,Aeq,beq,lb,ub,nonlcon,options)
    X_new = reshape(X_flat_new, Horizon, []);
    
    X = X_new;
    
    X_x = X(:, 1:ModelParams.nx);
    X_u = X(:, ModelParams.nx+1: ModelParams.nx+ModelParams.nu);
    
    u = X_u(1,:)';
    x = X_x(1,:)';
    
    plot_car(X_x, track);
    x = bycicle_step(x, u, Ts);
    pause(0.2);
    X_x(1,:) = x';
    X_u(1,:) = X_u(2,:);
    for i=2:Horizon
        X_x(i,:) = X_x(i-1,:) + Ts*bycicle_model(X_x(i-1,:)', X_u(i-1,:)', params)';
    end
    X = [X_x, X_u];
    X_flat = reshape(X,1,[]);
    
end
