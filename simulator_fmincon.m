clear
clc
load track.mat

Ts = 0.03;

startIdx = 1;

ModelParams = bycicle_params();

vx0 = 1;

trackWidth = norm(track.inner(:,1)-track.outer(:,1));

x0 = [track.center(1,startIdx),track.center(2,startIdx),... % point on centerline
      atan2(track.center(2,startIdx+1) - track.center(2,startIdx), track.center(1,startIdx+1) - track.center(1,startIdx)),... % aligned with centerline
      vx0 ,0, 0, track.progress(1, startIdx)]';
  
u0 = [vx0;0;vx0];

Horizon = 10;
params = struct;
params.track = track;
params.trackWidth = trackWidth;
params.Horizon = Horizon;
params.Ts = Ts;
params.modified = false;
Bd = [zeros(3); eye(3); zeros(1,3)];


X_x = repmat(x0', Horizon, 1); %(horizon, nx)
X_u = repmat(u0', Horizon, 1); %(horizon, nu)
for i=2:Horizon
    X_x(i,:) = X_x(i-1,:) + Ts*bycicle_model(X_x(i-1,:)', X_u(i-1,:)', params)';% + (Bd*evaluate_gp(X_x(i-1,:)', X_u(i-1,:)'))';
end


X = [X_x, X_u]; %(horizon, n) = (horizon, nx+nu)
X_flat = reshape(X',[],1); %(horizon*n,1)
options = optimoptions('fmincon','Display','none','Algorithm','sqp', 'MaxIterations',50,'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true);


f = @(x)fmincon_cost_function(x, params); %create wrapper for cost function
nonlcon = @(x)fmincon_constraints(x, params); % create wrapper for constr func


data = struct;
data.u = [];
data.x = [];

T = 0;
lap = 1;
prev_idx = startIdx;
for i=1:3000
    [A, b, Aeq, beq, lb, ub] = getMatrices(X_flat, params);
    [X_flat_new, fval, exitflag, output] = fmincon(f,X_flat,A,b,Aeq,beq,lb,ub,nonlcon,options);
    X_new = reshape(X_flat_new, [], Horizon)';
    
    X = X_new;
    
    X_x = X(:, 1:ModelParams.nx);
    X_u = X(:, ModelParams.nx+1: ModelParams.nx+ModelParams.nu);
    
    u = X_u(1,:)';
    x = X_x(1,:)';
    %u
    %exitflag
    %fval
    
    
    plot_car(X_x, track, T, lap);
    data.x = [data.x,x];
    data.u = [data.u,u];
    x = bycicle_step(x, u, Ts);
    [c, idx] = get_c(x, track);
    x(ModelParams.stateindex_theta) = track.progress(1, idx);
    X_x(1,:) = x';
    X_u(1,:) = X_u(2,:);
    for j=2:Horizon
        X_x(j,:) = X_x(j-1,:) + Ts*bycicle_model(X_x(j-1,:)', X_u(j-1,:)', params)';% + (Bd*evaluate_gp(X_x(j-1,:)', X_u(j-1,:)'))';
    end
    T = T+Ts;
    X = [X_x, X_u];
    X_flat = reshape(X',[],1);
    if (prev_idx > idx)
        lap = lap+1;
    end
    prev_idx = idx;
    
    if lap == 2
        break
    end

end
